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
#include "diux_msgs/msg/wrench_effort_command_type.hpp"
//#include <manualControlPublish.h>
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


#define MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_EXTERNAL_RX_STATUS  // Single m_mavlink_status instance is in QGCApplication.cc
#include <stddef.h>                 // Hack workaround for Mav 2.0 header problem with respect to offsetof usage
#include "mavlink/v2.0/mavlink_types.h"
mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
#include "mavlink/v2.0/common/mavlink.h"

#define BUFLEN 2048
#define BUFSIZE 2048
//#define MSGS 5


bool hbCont = true;
bool recieving = true;
bool sending = true;
struct sockaddr_in myaddr, remaddr;
int fd, i, slen=sizeof(remaddr);

using namespace std;
using namespace std::chrono_literals;


class manualControlPublish
{

public:
    manualControlPublish(std::string name);

    rclcpp::Node::SharedPtr m_node;

    void TestSendMessage();
    void sendLoop();


protected:


    rclcpp::Publisher<diux_msgs::msg::WrenchEffortCommandType>::SharedPtr m_wrenchEffortCommandTypePub;


};



manualControlPublish::manualControlPublish(std::string name)
{


    m_node = std::make_shared<rclcpp::Node>(name);


    m_wrenchEffortCommandTypePub= m_node->create_publisher<diux_msgs::msg::WrenchEffortCommandType>("set_wrench_effort_command_type");



};


void manualControlPublish::TestSendMessage()
{
    diux_msgs::msg::WrenchEffortCommandType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::WrenchEffortCommandType>();

    ros2_msg->propulsive_linear_effort.x_axis= 0.9;
    ros2_msg->propulsive_linear_effort.y_axis= 0.0;
    ros2_msg->propulsive_linear_effort.z_axis= 0.9;
    ros2_msg->propulsive_rotational_effort.pitch_effort = 0.0;
    ros2_msg->propulsive_rotational_effort.roll_effort = 0.0;
    ros2_msg->propulsive_rotational_effort.yaw_effort= 0.0;
    ros2_msg->resistive_linear_effort.x_axis = 0.0;
    ros2_msg->resistive_linear_effort.y_axis = 0.0;
    ros2_msg->resistive_linear_effort.z_axis = 0.0;
    ros2_msg->resistive_rotational_effort.pitch_effort = 0.0;
    ros2_msg->resistive_rotational_effort.roll_effort = 0.0;
    ros2_msg->resistive_rotational_effort.yaw_effort = 0.0;
    ros2_msg->session_id = " ";
    ros2_msg->source_subsystem_id = " ";
    ros2_msg->source_system_id = " ";
    ros2_msg->target_subsystem_id = " ";
    ros2_msg->target_system_id = " ";
    ros2_msg->time_stamp = 0.0;
    m_wrenchEffortCommandTypePub->publish(ros2_msg);
}

void manualControlPublish::sendLoop()
{
  int i = 0;
  while(i < 100)
  {
    TestSendMessage();
    usleep(100000);
    i++;
  }
}


int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);

  manualControlPublish * translationLayer = new manualControlPublish("task");

  translationLayer->sendLoop();

  rclcpp::spin(translationLayer->m_node);

  exit(1);
  return 0;
}
