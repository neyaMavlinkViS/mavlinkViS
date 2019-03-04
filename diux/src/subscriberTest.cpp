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
//#include <subscriberTest.h>
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
#include<unistd.h>

#include <chrono>
#include <cstdio>
#include <memory>

#include "rcutils/cmdline_parser.h"

//#include "std_msgs/msg/string.hpp"

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


using namespace std;
using namespace std::chrono_literals;







class subscriberTest
{

public:
    subscriberTest(std::string name);

    rclcpp::Node::SharedPtr m_node;

    void GlobalPoseStatusTypeCallback(const diux_msgs::msg::GlobalPoseStatusType::SharedPtr msg);

protected:

    rclcpp::Subscription<diux_msgs::msg::GlobalPoseStatusType>::SharedPtr m_globalPoseStatusTypeSub;

};


//---------------------------------------------subscriberTest CONSTRUCTOR---------------------------------------------//

//-------------------------------------------------------------------------------------------------------------------------//

subscriberTest::subscriberTest(std::string name)
{
    m_node = std::make_shared<rclcpp::Node>(name);

    m_globalPoseStatusTypeSub = m_node->create_subscription<diux_msgs::msg::GlobalPoseStatusType>( "set_global_pose_status_type", [this]( const diux_msgs::msg::GlobalPoseStatusType::SharedPtr msg) -> void
     {
       this->GlobalPoseStatusTypeCallback(msg);
     } );
};


  //---------------------------------------------SUBSCRIBING TO DDS MESSAGES---------------------------------------------//
                  //subscribes to a DDS thread, recieves a message, and calls a mavlink send function
  //---------------------------------------------------------------------------------------------------------------------//

void subscriberTest::GlobalPoseStatusTypeCallback(const diux_msgs::msg::GlobalPoseStatusType::SharedPtr msg)
{
  cout<<"Latitude: \t";
  cout << msg->position.geodetic_position.geodetic_latitude.latitude << std::endl;
  cout<<"Longitude: \t";
  cout<<msg->position.geodetic_position.geodetic_longitude.longitude << std::endl; 

}



//---------------------------------------------MAIN---------------------------------------------//

                //creates UDP socket connections, creates multiple threads for sending/recieving,
                //UDP messages, and initializes and spins ROS messages
//----------------------------------------------------------------------------------------------//

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  subscriberTest* translationLayer = new subscriberTest("task");

  rclcpp::spin(translationLayer->m_node);


  return 0;
}
