#include "voyant-ros/bin_to_mcap.hpp"

Bin2Mcap::Bin2Mcap()
{
  // Optional: Initialize members here
  lidar_frame_id_ = "voyant_sensor";
  timestamp_mode_ = "TIME_FROM_ROS"; // CURRENT_TIME, TIME_FROM_SENSOR
  valid_only_filter_ = false;
}

Bin2Mcap::~Bin2Mcap() { std::cout << "Shutting down..." << std::endl; };

sensor_msgs::msg::PointCloud2 Bin2Mcap::pointDatatoRosMsg(const VoyantFrameWrapper frame)
{

  pcl::PointCloud<VoyantPoint> pcl_cloud;
  const auto &points = frame.points();

  // Reserve space for the point cloud
  const size_t point_count = points.size();
  pcl_cloud.resize(point_count);
  // An ordered pointcloud has height 1 and width equal to the number of points
  pcl_cloud.width = point_count;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;

  size_t index = 0;

  // convert points to pcl point cloud
  for(const auto &p : points)
  {
    // Only include valid points
    if(valid_only_filter_ && p.drop_reason() != DropReason::SUCCESS)
    {
      continue;
    }

    VoyantPoint &point = pcl_cloud.points[index]; // Overwrite the point, this is safe because we
                                                  // reserved the space and we don't have to use
                                                  // push_back
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    point.v = p.radial_vel();
    point.snr = p.snr_linear();
    point.drop_reason = static_cast<uint8_t>(p.drop_reason());
    point.timestamp_nsecs = p.timestamp_nanosecs();
    point.point_idx = p.point_index();

    index++; // Next valid slot
  }

  pcl_cloud.resize(index);

  // Convert pcl cloud to ros message
  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);

  // set header timestamps
  if(timestamp_mode_ == "TIME_FROM_SENSOR")
  {
    ros_cloud.header.stamp.sec = frame.header().timestampSeconds();
    ros_cloud.header.stamp.nanosec = frame.header().timestampNanoseconds();
  }
  else
  {
    // std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    rclcpp::Time now = rclcpp::Clock().now();
    ros_cloud.header.stamp = now;
  }
  ros_cloud.header.frame_id = lidar_frame_id_;
  return ros_cloud;
}
