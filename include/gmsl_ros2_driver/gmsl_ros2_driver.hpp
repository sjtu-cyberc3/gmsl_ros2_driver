#pragma once
#ifndef GMSL_ROS2_DRIVER_HPP
#define GMSL_ROS2_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>

namespace gmsl_ros2_driver {

class GmslDriver : public rclcpp::Node {
   public:
    GmslDriver(const rclcpp::NodeOptions &options);
    ~GmslDriver();

   private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;
};

}  // namespace gmsl_ros2_driver

#endif  // GMSL_ROS2_DRIVER_HPP