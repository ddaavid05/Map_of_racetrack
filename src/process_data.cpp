#include <iostream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class LidarToWorldNode : public rclcpp::Node
{
public:
  LidarToWorldNode() : Node("lidar_to_world_node")
  {
    // Get static map path parameter
    this->declare_parameter<std::string>("static_map_path", "");
    this->get_parameter("static_map_path", static_map_path);

    // Subscribe to the lidar point cloud topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sensor/lidar_front/points", 10,
      std::bind(&LidarToWorldNode::handlePointCloud, this, std::placeholders::_1));

    // Create a publisher for the accumulated point cloud
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/world/transformed_point_cloud", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a timer to publish the accumulated point cloud periodically
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LidarToWorldNode::publishAccumulatedPointCloud, this));
  }

private:
  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Transform the point cloud
    if (tf_buffer_->canTransform("map", "lidar_front", msg->header.stamp, rclcpp::Duration::from_seconds(0.2)))
    {
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
        auto transform = tf_buffer_->lookupTransform("map", "lidar_front", msg->header.stamp);
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud_transformed;
        pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud_transformed, transform);

        // Append the transformed point cloud to the cumulative point cloud
        pcl_all_cloud += pcl_cloud_transformed;

        // Visualize the racetrack
        cloud_writer.write(static_map_path + "/racetrack.pcd", pcl_all_cloud);

        // Create a shared pointer for pcl_all_cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_all_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_all_cloud));
        // Voxel
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(pcl_all_cloud_ptr);
        voxel_filter.setLeafSize(1, 1, 1);
        voxel_filter.filter(voxel_pcl_all_cloud);

        // Visualize the voxelized racetrack
        cloud_writer_voxel.write(static_map_path + "/voxelized_racetrack.pcd", voxel_pcl_all_cloud);
    }
  }

  void publishAccumulatedPointCloud()
  {
    // Convert the accumulated PCL point cloud to ROS PointCloud2
    sensor_msgs::msg::PointCloud2 accumulated_msg;
    pcl::toROSMsg(voxel_pcl_all_cloud, accumulated_msg);
    accumulated_msg.header.stamp = this->get_clock()->now();
    accumulated_msg.header.frame_id = "map";

    // Publish the accumulated point cloud
    publisher_->publish(accumulated_msg);
  }

  std::string static_map_path;
  pcl::PCDWriter cloud_writer;
  pcl::PCDWriter cloud_writer_voxel;
  pcl::PointCloud<pcl::PointXYZI> pcl_all_cloud;
  pcl::PointCloud<pcl::PointXYZI> voxel_pcl_all_cloud;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarToWorldNode>());
    rclcpp::shutdown();
    return 0;
}
