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

#include "rplidar_ros/rplidar_node.hpp"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

namespace rplidar_ros
{

RPlidarNode::RPlidarNode(const std::string & name, const rclcpp::NodeOptions & options)
: rclcpp::Node(name, options), driver_(nullptr)
{
  std::string channel_type = "serial";
  std::string tcp_ip = "192.168.0.7";
  std::string serial_port = "/dev/ttyUSB0";
  int tcp_port = 20108;
  int serial_baudrate = 115200;  // ros2 run for A1 A2, change to 256000 if A3
  std::string frame_id = "laser_frame";
  bool inverted = false;
  bool angle_compensate = true;
  std::string scan_mode;

  declare_parameter<std::string>("channel_type", channel_type);
  declare_parameter<std::string>("tcp_ip", tcp_ip);
  declare_parameter<int>("tcp_port", tcp_port);
  declare_parameter<std::string>("serial_port", serial_port);
  declare_parameter<int>("serial_baudrate", serial_baudrate);
  declare_parameter<std::string>("frame_id", frame_id);
  declare_parameter<bool>("inverted", inverted);
  declare_parameter<bool>("angle_compensate", angle_compensate);
  declare_parameter<std::string>("scan_mode", scan_mode);

  float max_distance = 8.0;
  int angle_compensate_multiple = 1;  // it stand of angle compensate at per 1 degree
}

void RPlidarNode::start_motor(
    const std::shared_ptr<rmw_request_id_t> request_header,
    std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response) {
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
    std_srvs::srv::Empty::Response::SharedPtr response) {
  (void)request_header;
  (void)request;
  (void)response;

  RCLCPP_INFO(get_logger(), "Stop motor");
  driver_->stop();
  driver_->stopMotor();

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
