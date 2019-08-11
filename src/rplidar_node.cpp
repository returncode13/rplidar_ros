/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <chrono>
#include <cmath>
#include "rplidar_ros/rplidar_node.hpp"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x) * M_PI / 180.)

using namespace rp::standalone::rplidar;

namespace rplidar_ros
{

RPlidarNode::RPlidarNode(const std::string & name, const rclcpp::NodeOptions & options)
: rclcpp::Node(name, options),
  driver_(nullptr),
  clock_(RCL_ROS_TIME),
  channel_type_("serial"),
  tcp_ip_("192.168.0.7"),
  serial_port_("/dev/ttyUSB0"),
  tcp_port_(20108),
  serial_baudrate_(115200),  // ros2 run for A1 A2, change to 256000 if A3
  frame_id_("laser_frame"),
  inverted_(false),
  angle_compensate_(true),
  max_distance_(8.0),
  scan_mode_("")
{
  declare_parameter<std::string>("channel_type", channel_type_);
  declare_parameter<std::string>("tcp_ip", tcp_ip_);
  declare_parameter<int>("tcp_port", tcp_port_);
  declare_parameter<std::string>("serial_port", serial_port_);
  declare_parameter<int>("serial_baudrate", serial_baudrate_);
  declare_parameter<std::string>("frame_id", frame_id_);
  declare_parameter<bool>("inverted", inverted_);
  declare_parameter<bool>("angle_compensate", angle_compensate_);
  declare_parameter<double>("max_distance", max_distance_);
  declare_parameter<std::string>("scan_mode", scan_mode_);

  float max_distance = 8.0;
  int angle_compensate_multiple = 1;  // it stand of angle compensate at per 1 degree

  RCLCPP_INFO(get_logger(),
    "RPLIDAR running on ROS 2 package rplidar_ros. SDK Version:" RPLIDAR_SDK_VERSION "");

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(182ms, std::bind(&RPlidarNode::spin, this));  // 5.5Hz
}

void RPlidarNode::connect_driver()
{
  if (channel_type_ == "tcp") {
    driver_.reset(rp::standalone::rplidar::RPlidarDriver::CreateDriver(
        rp::standalone::rplidar::DRIVER_TYPE_TCP));
  } else {
    driver_.reset(rp::standalone::rplidar::RPlidarDriver::CreateDriver(
        rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT));
  }

  if (!driver_) {
    RCLCPP_ERROR(get_logger(), "Create Driver fail, exit");
    throw std::runtime_error("runtime_error");
  }

  if (channel_type_ == "tcp") {
    if (IS_FAIL(driver_->connect(tcp_ip_.c_str(), (_u32)tcp_port_))) {
      RCLCPP_ERROR(get_logger(), "Error, cannot bind to the specified serial port %s.",
        serial_port_.c_str());
      RPlidarDriver::DisposeDriver(driver_.get());
      throw std::runtime_error("runtime_error");
    }

  } else {
    if (IS_FAIL(driver_->connect(serial_port_.c_str(), (_u32)serial_baudrate_))) {
      RCLCPP_ERROR(get_logger(), "Error, cannot bind to the specified serial port %s.",
        serial_port_.c_str());
      RPlidarDriver::DisposeDriver(driver_.get());
      throw std::runtime_error("runtime_error");
    }
  }

  if (!get_device_info()) {
    throw std::runtime_error("runtime_error");
  }

  if (!check_health()) {
    throw std::runtime_error("runtime_error");
  }
}

bool RPlidarNode::get_device_info()
{
  u_result result;
  rplidar_response_device_info_t dev_info;

  result = driver_->getDeviceInfo(dev_info);
  if (IS_FAIL(result)) {
    if (result == RESULT_OPERATION_TIMEOUT) {
      RCLCPP_ERROR(get_logger(), "Operation time out. RESULT_OPERATION_TIMEOUT!");
    } else {
      RCLCPP_ERROR(get_logger(), "Unexpected error, code: %x", result);
    }
    return false;
  }

  RCLCPP_INFO(get_logger(), "Firmware Ver: %d.%02d",
    dev_info.firmware_version >> 8, dev_info.firmware_version & 0xFF);
  RCLCPP_INFO(get_logger(), "Hardware Rev: %d",
    (int)dev_info.hardware_version);
  return true;
}

bool RPlidarNode::check_health()
{
  u_result result;
  rplidar_response_device_health_t health_info;

  result = driver_->getHealth(health_info);
  if (IS_OK(result)) {
    RCLCPP_INFO(get_logger(), "RPlidar health status: %d", health_info.status);
    if (health_info.status == RPLIDAR_STATUS_ERROR) {
      RCLCPP_ERROR(
        get_logger(), "RPlidar internal error detected. Please reboot the device to retry.");
      return false;
    } else {
      return true;
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Cannot retrieve RPlidar health code: %x", result);
    return false;
  }
}

void RPlidarNode::start_motor(
  const std::shared_ptr<rmw_request_id_t> request_header,
  std_srvs::srv::Empty::Request::SharedPtr request,
  std_srvs::srv::Empty::Response::SharedPtr response)
{
  (void)request_header;
  (void)request;
  (void)response;

  RCLCPP_INFO(get_logger(), "Start motor");
  driver_->startMotor();
  driver_->startScan(false, true);
}

void RPlidarNode::stop_motor(
  const std::shared_ptr<rmw_request_id_t> request_header,
  std_srvs::srv::Empty::Request::SharedPtr request,
  std_srvs::srv::Empty::Response::SharedPtr response)
{
  (void)request_header;
  (void)request;
  (void)response;

  RCLCPP_INFO(get_logger(), "Stop motor");
  driver_->stop();
  driver_->stopMotor();

}

float RPlidarNode::get_angle(
  const rplidar_response_measurement_node_hq_t & node)
{
  return node.angle_z_q14 * 90.f / 16384.f;
}

void RPlidarNode::spin()
{
}

void RPlidarNode::publish_scan(
  rplidar_response_measurement_node_hq_t * nodes,
  size_t node_count)
{
  float angle_min = DEG2RAD(0.0f);
  float angle_max = DEG2RAD(359.0f);

  static int scan_count = 0;
  sensor_msgs::msg::LaserScan scan_msg;

  rclcpp::Time now = clock_.now();
  scan_msg.header.stamp = now;
  scan_msg.header.frame_id = frame_id_;
  scan_count++;

  bool reversed = (angle_max > angle_min);
  if (reversed) {
    scan_msg.angle_min = M_PI - angle_max;
    scan_msg.angle_max = M_PI - angle_min;
  } else {
    scan_msg.angle_min = M_PI - angle_min;
    scan_msg.angle_max = M_PI - angle_max;
  }
  scan_msg.angle_increment =
    (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count - 1);

  double scan_time = clock_.now().seconds();
  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (double)(node_count - 1);
  scan_msg.range_min = 0.15;
  scan_msg.range_max = max_distance_;

  scan_msg.intensities.resize(node_count);
  scan_msg.ranges.resize(node_count);
  bool reverse_data = (!inverted_ && reversed) || (inverted_ && !reversed);
  if (!reverse_data) {
    for (size_t i = 0; i < node_count; i++) {
      float read_value = (float) nodes[i].dist_mm_q2 / 4.0f / 1000;
      if (read_value == 0.0) {
        scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
      } else {
        scan_msg.ranges[i] = read_value;
      }
      scan_msg.intensities[i] = (float) (nodes[i].quality >> 2);
    }
  } else {
    for (size_t i = 0; i < node_count; i++) {
      float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
      if (read_value == 0.0) {
        scan_msg.ranges[node_count - 1 - i] = std::numeric_limits<float>::infinity();
      } else {
        scan_msg.ranges[node_count - 1 - i] = read_value;
      }
      scan_msg.intensities[node_count - 1 - i] = (float) (nodes[i].quality >> 2);
    }
  }

  scan_publisher_->publish(scan_msg);
}

}  // namespace rplidar_ros

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rplidar_ros::RPlidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
