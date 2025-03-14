#pragma once

#include <chrono>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <voyant_client.hpp>

struct EIGEN_ALIGN16 VoyantPoint
{
    PCL_ADD_POINT4D; // This adds x, y, z, and padding
    float v;
    float snr;
    uint8_t drop_reason;

    inline VoyantPoint()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f; // Set padding to 1 to prevent undefined behavior.
        v = 0.0f;
        snr = 0.0f;
        drop_reason = 0;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(VoyantPoint, (float, x, x)(float, y, y)(float, z, z)(float, v, v)(float, snr, snr)(
                                                   uint8_t, drop_reason, drop_reason))

class VoyantSensorDriver : public rclcpp::Node
{
  public:
    VoyantSensorDriver();
    virtual ~VoyantSensorDriver();

  private:
    // Methods
    void initialize();
    void getParams();
    void publishPointCloud();
    sensor_msgs::msg::PointCloud2 pointDatatoRosMsg(const VoyantFrameWrapper *frame);

    // Configuration parameters
    std::string binding_address_;
    std::string multicast_group_;
    std::string interface_address_;
    std::string timestamp_mode_;
    std::string lidar_frame_id_;
    bool spn_filter_;
    int reconnect_attempts_ = 5;

    // ROS components
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;

    // Voyant client
    std::shared_ptr<VoyantClient> client_;
};