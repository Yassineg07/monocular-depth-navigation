#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

class GroundEdgeNode : public rclcpp::Node
{
public:
  GroundEdgeNode() : Node("ground_edge_node")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    this->declare_parameter("camera_topic", "/camera/image_raw");
    this->declare_parameter("camera_info_topic", "/camera/camera_info");
    this->declare_parameter("ground_edge.roi_ratio", 0.35);
    this->declare_parameter("ground_edge.min_edge_strength", 30);
    this->declare_parameter("ground_edge.smoothing_alpha", 0.3);
    this->declare_parameter("ground_edge.camera_height", 0.3);
    this->declare_parameter("ground_edge.depth_scale", 1.0);
    
    camera_topic_ = this->get_parameter("camera_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    roi_ratio_ = this->get_parameter("ground_edge.roi_ratio").as_double();
    min_edge_strength_ = this->get_parameter("ground_edge.min_edge_strength").as_int();
    smoothing_alpha_ = this->get_parameter("ground_edge.smoothing_alpha").as_double();
    camera_height_ = this->get_parameter("ground_edge.camera_height").as_double();
    depth_scale_ = this->get_parameter("ground_edge.depth_scale").as_double();
    
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, 10,
      std::bind(&GroundEdgeNode::imageCallback, this, std::placeholders::_1));
    
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      std::bind(&GroundEdgeNode::cameraInfoCallback, this, std::placeholders::_1));
    
    marker_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/ground_edge_scan", 10);
    depth_prior_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/ground_edge_depth", 10);
    
    RCLCPP_INFO(this->get_logger(), "Ground Edge Node initialized");
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (K_.empty()) {
      K_ = cv::Mat::eye(3, 3, CV_64F);
      K_.at<double>(0, 0) = msg->k[0];
      K_.at<double>(1, 1) = msg->k[4];
      K_.at<double>(0, 2) = msg->k[2];
      K_.at<double>(1, 2) = msg->k[5];
      RCLCPP_INFO(this->get_logger(), "Ground edge initialized with camera calibration");
    }
  }
  
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (K_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "Waiting for camera calibration");
      return;
    }
    
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    
    int img_height = gray.rows;
    int img_width = gray.cols;
    
    int roi_start_row = static_cast<int>(img_height * (1.0 - roi_ratio_));
    cv::Mat roi = gray(cv::Rect(0, roi_start_row, img_width, img_height - roi_start_row));
    
    depth_per_column_.resize(img_width, -1.0f);
    std::vector<cv::Point3f> edge_points_3d;
    
    double fx = K_.at<double>(0, 0);
    double fy = K_.at<double>(1, 1);
    double cx = K_.at<double>(0, 2);
    double cy = K_.at<double>(1, 2);
    
    for (int u = 0; u < img_width; u++) {
      // Get ground reference intensity from bottom row
      uchar ground_intensity = roi.at<uchar>(roi.rows - 1, u);
      
      int edge_row = -1;
      
      // Search from bottom up for significant intensity change
      for (int r = roi.rows - 1; r >= 0; r--) {
        uchar pixel_intensity = roi.at<uchar>(r, u);
        int diff = std::abs(static_cast<int>(pixel_intensity) - static_cast<int>(ground_intensity));
        
        if (diff > min_edge_strength_) {
          edge_row = r;
          break;
        }
      }
      
      if (edge_row < 0) continue;
      
      int v = roi_start_row + edge_row;
      
      double angle_from_optical = std::atan((v - cy) / fy);
      double ground_angle = angle_from_optical;
      
      if (ground_angle <= 0.01) continue;
      
      float depth = (camera_height_ / std::tan(ground_angle)) * depth_scale_;
      
      if (depth < 0.1 || depth > 10.0) continue;
      
      depth_per_column_[u] = depth;
      
      double x = (u - cx) * depth / fx;
      double y = (v - cy) * depth / fy;
      edge_points_3d.push_back(cv::Point3f(x, y, depth));
    }
    
    if (edge_points_3d.empty()) return;
    
    // Transform points to base_link
    geometry_msgs::msg::TransformStamped cam_to_base_tf;
    try {
      cam_to_base_tf = tf_buffer_->lookupTransform("base_link", "camera_link_optical", tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "TF lookup failed: %s", ex.what());
      return;
    }
    
    tf2::Transform tf_cam_to_base;
    tf2::fromMsg(cam_to_base_tf.transform, tf_cam_to_base);
    
    std::vector<cv::Point3f> edge_points_base;
    for (const auto& pt : edge_points_3d) {
      tf2::Vector3 pt_cam(pt.x, pt.y, pt.z);
      tf2::Vector3 pt_base = tf_cam_to_base * pt_cam;
      edge_points_base.push_back(cv::Point3f(pt_base.x(), pt_base.y(), pt_base.z()));
    }
    
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = msg->header.stamp;
    scan.header.frame_id = "base_link";
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = 0.0209;  // ~1.2 degrees, ~300 points
    scan.range_min = 0.1;
    scan.range_max = 10.0;
    
    int num_bins = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.resize(num_bins, std::numeric_limits<float>::infinity());
    
    std::vector<std::pair<int, float>> bin_ranges;
    for (const auto& pt : edge_points_base) {
      double bearing = std::atan2(pt.y, pt.x);
      double range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
      
      int bin = static_cast<int>((bearing - scan.angle_min) / scan.angle_increment);
      if (bin >= 0 && bin < num_bins) {
        bin_ranges.push_back({bin, static_cast<float>(range)});
      }
    }
    
    // Fill ranges and detect gaps
    for (size_t i = 0; i < bin_ranges.size(); i++) {
      scan.ranges[bin_ranges[i].first] = bin_ranges[i].second;
      
      // Add gap markers between disconnected segments
      if (i > 0) {
        int gap = bin_ranges[i].first - bin_ranges[i-1].first;
        if (gap > 10) {
          for (int j = bin_ranges[i-1].first + 1; j < bin_ranges[i].first; j++) {
            scan.ranges[j] = std::numeric_limits<float>::infinity();
          }
        }
      }
    }
    
    marker_pub_->publish(scan);
    
    static bool first_publish = true;
    if (first_publish) {
      RCLCPP_INFO(this->get_logger(), "Publishing ground edge data to /ground_edge_depth");
      first_publish = false;
    }
    
    std_msgs::msg::Float32MultiArray depth_msg;
    depth_msg.data = depth_per_column_;
    depth_prior_pub_->publish(depth_msg);
  }
  
  std::string camera_topic_, camera_info_topic_;
  double roi_ratio_, smoothing_alpha_, camera_height_, depth_scale_;
  int min_edge_strength_;
  cv::Mat K_;
  std::vector<float> depth_per_column_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr depth_prior_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundEdgeNode>());
  rclcpp::shutdown();
  return 0;
}
