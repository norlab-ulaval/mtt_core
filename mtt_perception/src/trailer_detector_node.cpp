// Trailer angle estimation from LiDAR point cloud.
// Uses the RoboSense Bpearl (rear-facing, hemispheric FoV) to detect the
// trailer body behind the robot and compute the articulation angle.
//
// Method: Extract points in the expected trailer region, fit a line/plane
// to the trailer body, compute the angle between the fitted direction
// and the robot's forward axis.

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TrailerDetectorNode : public rclcpp::Node {
public:
  TrailerDetectorNode() : Node("trailer_detector_node") {
    declare_parameter("lidar_topic", "/rsairy_ns/points");
    declare_parameter("output_topic", "trailer/angle");
    // Region of interest behind the robot (in lidar frame)
    // The trailer hitch is at roughly [-1.05, 0.21, 0.36] from base_link
    declare_parameter("roi_x_min", -2.5);   // behind the robot
    declare_parameter("roi_x_max", -0.5);
    declare_parameter("roi_y_min", -1.0);
    declare_parameter("roi_y_max", 1.0);
    declare_parameter("roi_z_min", -0.5);
    declare_parameter("roi_z_max", 1.0);
    declare_parameter("voxel_size", 0.05);
    declare_parameter("min_points", 20);
    declare_parameter("publish_rate", 10.0);

    roi_x_min_ = get_parameter("roi_x_min").as_double();
    roi_x_max_ = get_parameter("roi_x_max").as_double();
    roi_y_min_ = get_parameter("roi_y_min").as_double();
    roi_y_max_ = get_parameter("roi_y_max").as_double();
    roi_z_min_ = get_parameter("roi_z_min").as_double();
    roi_z_max_ = get_parameter("roi_z_max").as_double();
    voxel_size_ = get_parameter("voxel_size").as_double();
    min_points_ = get_parameter("min_points").as_int();

    auto lidar_topic = get_parameter("lidar_topic").as_string();
    auto output_topic = get_parameter("output_topic").as_string();

    angle_pub_ = create_publisher<std_msgs::msg::Float64>(output_topic, 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, rclcpp::SensorDataQoS(),
        std::bind(&TrailerDetectorNode::cloud_callback, this,
                  std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "Trailer detector started on %s", lidar_topic.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) return;

    // Filter: keep only points in the trailer region
    auto filtered = filter_roi(cloud);
    if (!filtered || static_cast<int>(filtered->size()) < min_points_) return;

    // Downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(filtered);
    vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vg.filter(*downsampled);

    if (static_cast<int>(downsampled->size()) < min_points_) return;

    // Fit a line to the trailer body (RANSAC)
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);  // 10cm tolerance
    seg.setMaxIterations(100);
    seg.setInputCloud(downsampled);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < static_cast<size_t>(min_points_)) return;

    // Line direction is coefficients[3:5] (dx, dy, dz)
    double dx = coefficients->values[3];
    double dy = coefficients->values[4];
    // Trailer angle in the XY plane relative to the LiDAR forward axis
    double angle = std::atan2(dy, dx);

    // Publish angle
    std_msgs::msg::Float64 angle_msg;
    angle_msg.data = angle;
    angle_pub_->publish(angle_msg);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_roi(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(roi_x_min_, roi_x_max_);
    pass.filter(*result);

    pass.setInputCloud(result);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(roi_y_min_, roi_y_max_);
    pass.filter(*result);

    pass.setInputCloud(result);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(roi_z_min_, roi_z_max_);
    pass.filter(*result);

    return result;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  double roi_x_min_, roi_x_max_;
  double roi_y_min_, roi_y_max_;
  double roi_z_min_, roi_z_max_;
  double voxel_size_;
  int min_points_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrailerDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
