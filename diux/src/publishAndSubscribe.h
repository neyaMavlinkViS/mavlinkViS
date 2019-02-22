#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "diux_msgs/msg/geomagnetic_property_request_type.hpp"
#include "diux_msgs/msg/geomagnetic_property_command_request_type.hpp"
#include "diux_msgs/msg/global_pose_request_type.hpp"
#include "diux_msgs/msg/global_pose_command_request_type.hpp"
#include "diux_msgs/msg/geomagnetic_property_command_type.hpp"
#include "diux_msgs/msg/global_pose_command_type.hpp"

#include "diux_msgs/msg/geomagnetic_property_status_type.hpp"
#include "diux_msgs/msg/geomagnetic_property_command_report_type.hpp"
#include "diux_msgs/msg/global_pose_status_type.hpp"
#include "diux_msgs/msg/global_pose_command_report_type.hpp"
#include "diux_msgs/msg/global_pose_command_status_type.hpp"
#include "diux_msgs/msg/geomagnetic_property_command_status_type.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "port.h"
#include <unistd.h>
#include <string>
#include <thread>

#include <chrono>
#include <cstdio>
#include <memory>

#include "rcutils/cmdline_parser.h"

//#include "std_msgs/msg/string.hpp"
mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
#define MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_EXTERNAL_RX_STATUS  // Single m_mavlink_status instance is in QGCApplication.cc
#include <stddef.h>                 // Hack workaround for Mav 2.0 header problem with respect to offsetof usage
#include "mavlink/v2.0/mavlink_types.h"
#include "mavlink/v2.0/common/mavlink.h"



class publishAndSubscribe
{
public:
    publishAndSubscribe(std::string name);
    rclcpp::Node::SharedPtr m_node;
    void GeomagneticPropertyRequestTypeCallback(const diux_msgs::msg::GeomagneticPropertyRequestType::SharedPtr msg);

protected:
  rclcpp::Publisher<diux_msgs::msg::GeomagneticPropertyRequestType>::SharedPtr m_geomagneticPropertyRequestTypePub;
  rclcpp::Subscription<diux_msgs::msg::GeomagneticPropertyRequestType>::SharedPtr m_geomagneticPropertyRequestTypeSub;



}
