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
//#include <publishAndSubscribe.h>
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
struct sockaddr_in myaddr, remaddr;
int fd, i, slen=sizeof(remaddr);

using namespace std;
using namespace std::chrono_literals;


class publishAndSubscribe
{

public:
    publishAndSubscribe(std::string name);

    rclcpp::Node::SharedPtr m_node;

    //void MavlinkGeomagneticPropertyCommandStatusCallback( mavlink_geomagnetic_type_t mavlink_msg );
    //void MavlinkHeartbeatCallback(mavlink_heartbeat_t mavlink_msg);
    //void MavlinkRadioStatusCallback(mavlink_radio_status_t mavlink_msg);
    //void MavlinkRCChannelsCallback(mavlink_message_t mavlink_msg);
    //void MavlinkRCChannelsRawCallback(mavlink_message_t mavlink_msg);
    //void MavlinkBatteryStatusCallback(mavlink_battery_status_t mavlink_msg);
    //void MavlinkSysStatusCallback(mavlink_sys_status_t mavlink_msg);
    //void MavlinkVibrationCallback(mavlink_vibration_t mavlink_msg);
    //void MavlinkExtendedSysStateCallback(mavlink_extended_sys_state_t mavlink_msg);
    //void MavlinkCommandAckCallback(mavlink_command_ack_t mavlink_msg);
    //void MavlinkCommandLongCallback(mavlink_command_long_t mavlink_msg);
    //void MavlinkWindCovCallback(mavlink_wind_cov_t mavlink_msg);
    //void MavlinkHilActuatorControlsCallback(mavlink_hil_actuator_controls_t mavlink_msg);
    //void MavlinkLoggingDataCallback(mavlink_logging_data_t mavlink_msg);
    //void MavlinkLoggingDataAckedCallback(mavlink_logging_data_acked_t mavlink_msg);
    void MavlinkGpsRawIntCallback(mavlink_gps_raw_int_t mavlink_msg);
    //void MavlinkGlobalPositionIntCallback(mavlink_global_position_int_t mavlink_msg);
    //void MavlinkAltitudeCallback(mavlink_altitude_t mavlink_msg);
    //void MavlinkVfrHudCallback(mavlink_vfr_hud_t mavlink_msg);
    //void MavlinkScaledPressureCallback(mavlink_scaled_pressure_t mavlink_msg);
    //void MavlinkScaledPressure2Callback(mavlink_scaled_pressure2_t mavlink_msg);
    //void MavlinkScaledPressure3Callback(mavlink_scaled_pressure3_t mavlink_msg);
    //void MavlinkCameraImageCapturedCallback(mavlink_camera_image_captured_t mavlink_msg);
    //void MavlinkADSBVehicleCallback(mavlink_adsb_vehicle_t mavlink_msg);
    //void MavlinkHighLatency2Callback(mavlink_high_latency2_t mavlink_msg);
    //void MavlinkAttitudeCallback(mavlink_attitude_t mavlink_msg);
    //void MavlinkAttitudeQuaternionCallback(mavlink_attitude_quaternion_t mavlink_msg);
    //void MavlinkAttitudeTargetCallback(mavlink_attitude_target_t mavlink_msg);
    //void MavlinkDistanceSensorCallback(mavlink_distance_sensor_t mavlink_msg);
    //void MavlinkEstimatorStatusCallback(mavlink_estimator_status_t mavlink_msg);
    //void MavlinkStatusTextCallback(mavlink_statustext_t mavlink_msg);
    //void MavlinkOrbitExecutionStatusCallback(mavlink_orbit_execution_status_t mavlink_msg);

    //void GeomagneticPropertyRequestTypeCallback(const diux_msgs::msg::GeomagneticPropertyRequestType::SharedPtr msg);

    void _handleGpsRawInt(mavlink_message_t message);
    void _handleHeartbeat(mavlink_message_t& message);
    void recieveMessages(int slen, int fd, struct sockaddr_in remaddr);
    void sendHeartbeat(int slen, int fd, struct sockaddr_in remaddr);
    void sendManualControl(float newPitchCommand, float newRollCommand, float newThrustCommand, float newYawCommand, int buttons);
    void sendArmVehicle();

    void WrenchEffortCommandTypeCallback(const diux_msgs::msg::WrenchEffortCommandType::SharedPtr msg);


protected:

    //PUBLISHERS//
      //globalPose
    //rclcpp::Publisher<diux_msgs::msg::GeomagneticPropertyRequestType>::SharedPtr m_geomagneticPropertyRequestTypePub;
    /*rclcpp::Publisher<diux_msgs::msg::GeomagneticPropertyCommandRequestType>::SharedPtr m_geomagneticPropertyCommandRequestTypePub;
    rclcpp::Publisher<diux_msgs::msg::GlobalPoseRequestType>::SharedPtr m_globalPoseRequestTypePub;
    rclcpp::Publisher<diux_msgs::msg::GlobalPoseCommandRequestType>::SharedPtr m_globalPoseCommandRequestTypePub;
    rclcpp::Publisher<diux_msgs::msg::GeomagneticPropertyCommandType>::SharedPtr m_geomagneticPropertyCommandTypePub;
    rclcpp::Publisher<diux_msgs::msg::GlobalPoseCommandType>::SharedPtr m_globalPoseCommandTypePub;
    rclcpp::Publisher<diux_msgs::msg::GeomagneticPropertyStatusType>::SharedPtr m_geomagneticPropertyStatusTypePub;
    rclcpp::Publisher<diux_msgs::msg::GeomagneticPropertyCommandReportType>::SharedPtr m_geomagneticPropertyCommandReportTypePub;
    */
    rclcpp::Publisher<diux_msgs::msg::GlobalPoseStatusType>::SharedPtr m_globalPoseStatusTypePub;
    /*
    rclcpp::Publisher<diux_msgs::msg::GlobalPoseCommandReportType>::SharedPtr m_globalPoseCommandReportTypePub;
    rclcpp::Publisher<diux_msgs::msg::GlobalPoseCommandStatusType>::SharedPtr m_globalPoseCommandStatusTypePub;
    rclcpp::Publisher<diux_msgs::msg::GeomagneticPropertyCommandStatusType>::SharedPtr m_geomagneticPropertyCommandStatusTypePub;
      //primitiveDrivers
    rclcpp::Publisher<diux_msgs::msg::WrenchEffortRequestType>::SharedPtr m_wrenchEffortRequestTypePub;
    rclcpp::Publisher<diux_msgs::msg::WrenchEffortCommandRequestType>::SharedPtr m_wrenchEffortCommandRequestTypePub;
    rclcpp::Publisher<diux_msgs::msg::WrenchEffortCommandType>::SharedPtr m_wrenchEffortCommandTypePub;
    rclcpp::Publisher<diux_msgs::msg::WrenchEffortStatusType>::SharedPts m_wrenchEffortStatusTypePub;
    rclcpp::Publisher<diux_msgs::msg::WrenchEffortCommandReportType>::SharedPtr m_wrenchEffortCommandReportTypePub;
    rclcpp::Publisher<diux_msgs::msg::WrenchEffortCommandStatusType>::SharedPtr m_wrenchEffortCommandStatusTypePub;
*/
    //SUBSCRIPTIONS
      //globalPose
    //rclcpp::Subscription<diux_msgs::msg::GeomagneticPropertyRequestType>::SharedPtr m_geomagneticPropertyRequestTypeSub;
    /*rclcpp::Subscription<diux_msgs::msg::GeomagneticPropertyCommandRequestType>::SharedPtr m_geomagneticPropertyCommandRequestTypeSub;
    rclcpp::Subscription<diux_msgs::msg::GlobalPoseRequestType>::SharedPtr m_globalPoseRequestTypeSub;
    rclcpp::Subscription<diux_msgs::msg::GlobalPoseCommandRequestType>::SharedPtr m_globalPoseCommandRequestTypeSub;
    rclcpp::Subscription<diux_msgs::msg::GeomagneticPropertyCommandType>::SharedPtr m_geomagneticPropertyCommandTypeSub;
    rclcpp::Subscription<diux_msgs::msg::GlobalPoseCommandType>::SharedPtr m_globalPoseCommandTypeSub;
    rclcpp::Subscription<diux_msgs::msg::GeomagneticPropertyStatusType>::SharedPtr m_geomagneticPropertyStatusTypeSub;
    rclcpp::Subscription<diux_msgs::msg::GeomagneticPropertyCommandReportType>::SharedPtr m_geomagneticPropertyCommandReportTypeSub;
    rclcpp::Subscription<diux_msgs::msg::GlobalPoseStatusType>::SharedPtr m_globalPoseStatusTypeSub;
    rclcpp::Subscription<diux_msgs::msg::GlobalPoseCommandReportType>::SharedPtr m_globalPoseCommandReportTypeSub;
    rclcpp::Subscription<diux_msgs::msg::GlobalPoseCommandStatusType>::SharedPtr m_globalPoseCommandStatusTypeSub;
    rclcpp::Subscription<diux_msgs::msg::GeomagneticPropertyCommandStatusType>::SharedPtr m_geomagneticPropertyCommandStatusTypeSub;
      //primitiveDrivers
    rclcpp::Subscription<diux_msgs::msg::WrenchEffortRequestType>::SharedPtr m_wrenchEffortRequestTypeSub;
    rclcpp::Subscription<diux_msgs::msg::WrenchEffortCommandRequestType>::SharedPtr m_wrenchEffortCommandRequestTypeSub;
    */
    rclcpp::Subscription<diux_msgs::msg::WrenchEffortCommandType>::SharedPtr m_wrenchEffortCommandTypeSub;
/*
    rclcpp::Subscription<diux_msgs::msg::WrenchEffortStatusType>::SharedPts m_wrenchEffortStatusTypeSub;
    rclcpp::Subscription<diux_msgs::msg::WrenchEffortCommandReportType>::SharedPtr m_wrenchEffortCommandReportTypeSub;
    rclcpp::Subscription<diux_msgs::msg::WrenchEffortCommandStatusType>::SharedPtr m_wrenchEffortCommandStatusTypeSub;
*/

};


//---------------------------------------------publishAndSubscribe CONSTRUCTOR---------------------------------------------//



//---------------------------------------------------------------------------------------------------------//

publishAndSubscribe::publishAndSubscribe(std::string name)
{


    m_node = std::make_shared<rclcpp::Node>(name);

    //PUBLISHERS//
      //globalPose
    //m_geomagneticPropertyRequestTypePub = m_node->create_publisher<diux_msgs::msg::GeomagneticPropertyRequestType>("set_geomagnetic_property_request_type");
    /*m_geomagneticPropertyCommandRequestTypePub = m_node->create_publisher<diux_msgs::msg::GeomagneticPropertyCommandRequestType>("set_geomagnetic_property_command_request_type");
    m_globalPoseRequestTypePub = m_node->create_publisher<diux_msgs::msg::GlobalPoseRequestType>("set_global_pose_request_type_status");
    m_globalPoseCommandRequestTypePub = m_node->create_publisher<diux_msgs::msg::GlobalPoseCommandRequestType>("set_global_pose_command_request_type");
    m_geomagneticPropertyCommandTypePub = m_node->create_publisher<diux_msgs::msg::GeomagneticPropertyCommandType>("set_geomagnetic_property_command_type");
    m_globalPoseCommandTypePub = m_node->create_publisher<diux_msgs::msg::GlobalPoseCommandType>("set_global_pose_command_type");
    m_geomagneticPropertyStatusTypePub = m_node->create_publisher<diux_msgs::msg::GeomagneticPropertyStatusType>("set_geomagnetic_property_status_type");
    m_geomagneticPropertyCommandReportTypePub = m_node->create_publisher<diux_msgs::msg::GeomagneticPropertyCommandReportType>("set_geomagnetic_property_command_report_type");
    */
    m_globalPoseStatusTypePub = m_node->create_publisher<diux_msgs::msg::GlobalPoseStatusType>("set_global_pose_status_type");
    /*
    m_globalPoseCommandReportTypePub = m_node->create_publisher<diux_msgs::msg::GlobalPoseCommandReportTypePub>("set_global_pose_command_report_type");
    m_globalPoseCommandStatusTypePub = m_node->create_publisher<diux_msgs::msg::GlobalPoseCommandStatusType>("set_global_pose_command_status_type");
    m_geomagneticPropertyCommandStatusTypePub = m_node->create_publisher<diux_msgs::msg::GeomagneticPropertyCommandStatusType>("set_geomagnetic_property_command_status_type");
      //primitiveDrivers
    m_wrenchEffortRequestTypePub= m_node->create_publisher<diux_msgs::msg::WrenchEffortRequestType>("set_wrench_effort_request_type");
    m_wrenchEffortCommandRequestTypePub= m_node->create_publisher<diux_msgs::msg::WrenchEffortCommandRequestType>("set_wrench_effort_command_request_type");
    m_wrenchEffortCommandTypePub= m_node->create_publisher<diux_msgs::msg::WrenchEffortCommandType>("set_wrench_effort_command_type");
    m_wrenchEffortStatusTypePub= m_node->create_publisher<diux_msgs::msg::WrenchEffortStatusType>("set_wrench_effort_status_type");
    m_wrenchEffortCommandReportTypePub= m_node->create_publisher<diux_msgs::msg::WrenchEffortCommandReportType>("set_wrench_effort_command_report_type");
    m_wrenchEffortCommandStatusTypePub= m_node->create_publisher<diux_msgs::msg::WrenchEffortCommandStatusType>("set_wrench_effort_commandn_status_type");
*/
    //SUBSCRIPTIONS
      //globalPose
  /*  m_geomagneticPropertyRequestTypeSub = m_node->create_subscription<diux_msgs::msg::GeomagneticPropertyRequestType>( "set_geomagnetic_property_request_type", [this]( const diux_msgs::msg::GeomagneticPropertyRequestType::SharedPtr msg) -> void
    {
      this->GeomagneticPropertyRequestTypeCallback(msg);
    } );*/
      /*m_geomagneticPropertyCommandRequestTypeSub= m_node->create_subscription<diux_msgs::msg::GeomagneticPropertyCommandRequestType>("set_geomagnetic_property_command_request_type");
    m_globalPoseRequestTypeSub= m_node->create_subscription<diux_msgs::msg::GlobalPoseRequestType>("set_global_pose_request_type_status");
    m_globalPoseCommandRequestTypeSub= m_node->create_subscription<diux_msgs::msg::GlobalPoseCommandRequestType>("set_global_pose_command_request_type");
    m_geomagneticPropertyCommandTypeSub= m_node->create_subscription<diux_msgs::msg::GeomagneticPropertyCommandType>("set_geomagnetic_property_command_type");
    m_globalPoseCommandTypeSub= m_node->create_subscription<diux_msgs::msg::GlobalPoseCommandType>("set_global_pose_command_type");
    m_geomagneticPropertyStatusTypeSub= m_node->create_subscription<diux_msgs::msg::GeomagneticPropertyStatusType>("set_geomagnetic_property_status_type");
    m_geomagneticPropertyCommandReportTypeSub= m_node->create_subscription<diux_msgs::msg::GeomagneticPropertyCommandReportType>("set_geomagnetic_property_command_report_type");
    m_globalPoseStatusTypeSub= m_node->create_subscription<diux_msgs::msg::GlobalPoseStatusType>("set_global_pose_status_type");
    m_globalPoseCommandReportTypeSub= m_node->create_subscription<diux_msgs::msg::GlobalPoseCommandReportTypePub>("set_global_pose_command_report_type");
    m_globalPoseCommandStatusTypeSub= m_node->create_subscription<diux_msgs::msg::GlobalPoseCommandStatusType>("set_global_pose_command_status_type");
    m_geomagneticPropertyCommandStatusTypeSub= m_node->create_subscription<diux_msgs::msg::GeomagneticPropertyCommandStatusType>("set_geomagnetic_property_command_status_type");
      //primitiveDrivers
    m_wrenchEffortRequestTypeSub= m_node->create_subscription<diux_msgs::msg::WrenchEffortRequestType>("set_wrench_effort_request_type");
    m_wrenchEffortCommandRequestTypeSub= m_node->create_subscription<diux_msgs::msg::WrenchEffortCommandRequestType>("set_wrench_effort_command_request_type");
    */

    m_wrenchEffortCommandTypeSub = m_node->create_subscription<diux_msgs::msg::WrenchEffortCommandType>( "set_wrench_effort_command_type", [this]( const diux_msgs::msg::WrenchEffortCommandType::SharedPtr msg) -> void
     {
       this->WrenchEffortCommandTypeCallback(msg);

     } );

    /*m_wrenchEffortStatusTypeSub= m_node->create_subscription<diux_msgs::msg::WrenchEffortStatusType>("set_wrench_effort_status_type");
    m_wrenchEffortCommandReportTypeSub= m_node->create_subscription<diux_msgs::msg::WrenchEffortCommandReportType>("set_wrench_effort_command_report_type");
    m_wrenchEffortCommandStatusTypeSub= m_node->create_subscription<diux_msgs::msg::WrenchEffortCommandStatusType>("set_wrench_effort_commandn_status_type");
*/
};


//------------------------------------------------------------------------------------------------------------------//

                                      //~SENDING MAVLINK COMMANDS~//

//------------------------------------------------------------------------------------------------------------------//


//---------------------------------------------MAVLINK HEARBEAT---------------------------------------------//

                //Sends Mavlink Heartbeat at 10Hz

//----------------------------------------------------------------------------------------------------------//
void publishAndSubscribe::sendHeartbeat(int slen, int fd, struct sockaddr_in remaddr )
{
  cout<<"HeartBeat Spinning Up \n";

	while(hbCont)
	{
		char buf[BUFLEN];
		uint8_t _vehicleSystemId = 1;
		uint8_t _vehicleComponentId = 1;
		uint8_t _mavlinkChannel = 2;
		uint8_t _vehicleType = 6;
		uint8_t _firmwareType = 8;
		uint8_t _mavBaseMode = 192;
		uint32_t _mavCustomMode = 0;
		uint8_t _mavState = 4;

		mavlink_message_t   msg;
		mavlink_msg_heartbeat_pack_chan(_vehicleSystemId,_vehicleComponentId,_mavlinkChannel,&msg,_vehicleType,_firmwareType,_mavBaseMode,_mavCustomMode,_mavState);

		int len = mavlink_msg_to_send_buffer(buf, &msg);

		//SENDING//
		if (sendto(fd, buf, len, 0, (struct sockaddr *)&remaddr, slen)==-1) {
			perror("sendto");
			exit(1);
		}
		usleep(100000);
	}
}


//---------------------------------------------SEND MANUAL CONTROL---------------------------------------------//

                //Sends Mavlink Heartbeat at 10Hz

//-------------------------------------------------------------------------------------------------------------//
void publishAndSubscribe::sendManualControl(float newPitchCommand, float newRollCommand, float newThrustCommand, float newYawCommand, int buttons)
{
	char buf[BUFLEN];
	uint8_t _vehicleSystemId = 255;
	uint8_t _vehicleComponentId = 0;
	uint8_t _mavlinkChannel = 2;
	uint8_t uasId = 1;

	mavlink_message_t msg;
	mavlink_msg_manual_control_pack_chan(_vehicleSystemId,_vehicleComponentId,_mavlinkChannel,&msg,uasId,newPitchCommand, newRollCommand, newThrustCommand, newYawCommand, buttons);

	int len = mavlink_msg_to_send_buffer(buf, &msg);

	if (sendto(fd, buf, len, 0, (struct sockaddr *)&remaddr, slen)==-1) {
			perror("sendto");
			exit(1);
		}
}



  //---------------------------------------------DDS MESSAGE SUBSCRIPTION CALLBACKS---------------------------------------------//

                  //subscribes to a DDS thread, recieves a message, and calls a mavlink send function

  //----------------------------------------------------------------------------------------------------------------------------//


void publishAndSubscribe::WrenchEffortCommandTypeCallback(const diux_msgs::msg::WrenchEffortCommandType::SharedPtr msg)
{
    //cout<<"ManualControlMessage Recieved\n";
    float newPitchCommand,newYawCommand,newRollCommand, newThrustCommand;
    newPitchCommand = msg->propulsive_linear_effort.x_axis;
    newYawCommand = msg->propulsive_linear_effort.y_axis;
    newRollCommand = msg->propulsive_rotational_effort.roll_effort;
    newThrustCommand = msg->propulsive_linear_effort.z_axis;
    newPitchCommand = newPitchCommand*1000;
    newYawCommand = newYawCommand * 1000;
    newRollCommand = newRollCommand*1000;
    newThrustCommand = newThrustCommand*1000;
    //cout<<newPitchCommand<<"\n";
    //cout<<newYawCommand<<"\n";
    //cout<<newRollCommand<<"\n";
    //cout<<newThrustCommand<<"\n";
    sendManualControl(newPitchCommand,newRollCommand,newThrustCommand, newYawCommand,0);
    //sendManualControl(1000,1000,0, -7.8,0);
}






//---------------------------------------------PUBLISHING DDS MESSAGES---------------------------------------------//

                //takes a mavlink decoded message, translates to a DDS message, and publishes the DDS message

//-----------------------------------------------------------------------------------------------------------------//
/*void publishAndSubscribe::MavlinkGeomagneticPropertyCommandStatusCallback( mavlink_geomagnetic_type_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/

/*
void publishAndSubscribe::MavlinkHeartbeatCallback( mavlink_heartbeat_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkRCChannelsCallback( mavlink_heartbeat_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkRCChannelsRawCallback( mavlink_heartbeat_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkBatteryStatusCallback(mavlink_battery_status_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkSysStatusCallback( mavlink_sys_status_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkVibrationCallback( mavlink_vibration_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*

void publishAndSubscribe::MavlinkExtendedSysStateCallback( mavlink_extended_sys_state_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkCommandAckCallback( mavlink_command_ack_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkCommandLongCallback( mavlink_command_long_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkAutopilotVersionCallback( mavlink_autopilot_version_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkWindCovCallback( mavlink_wind_cov_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkHilActuatorControlsCallback( mavlink_hil_actuator_controls_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*

void publishAndSubscribe::MavlinkLoggingDataCallback( mavlink_logging_data_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkLoggingDataAckedCallback( mavlink_loggging_data_acked mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/

void publishAndSubscribe::MavlinkGpsRawIntCallback( mavlink_gps_raw_int_t mavlink_msg )
{   //cout<<"\t Callback";
    diux_msgs::msg::GlobalPoseStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GlobalPoseStatusType>();

    ros2_msg->altitude.altitude = mavlink_msg.alt;
    ros2_msg->altitude_agl.altitude = 0;
    ros2_msg->altitude_asf = 0;
    ros2_msg->attitude.pitch_y = 0;
    ros2_msg->attitude.roll_x = 0;
    ros2_msg->attitude.yaw_z = 0;
    ros2_msg->attitude_rms.orientation_error = 0;
    ros2_msg->depth = 0;
    ros2_msg->heading = 0;
    ros2_msg->navigation_solution.value = " ";
    ros2_msg->position.height_above_ellipsoid.altitude = 0;
    ros2_msg->position.geodetic_position.geodetic_latitude.latitude = mavlink_msg.lat;
    ros2_msg->position.geodetic_position.geodetic_longitude.longitude = mavlink_msg.lon;
    ros2_msg->session_id = " ";
    ros2_msg->source_subsystem_id = " ";
    ros2_msg->source_system_id = " ";
    ros2_msg->target_subsystem_id = " ";
    ros2_msg->target_system_id = " ";
    ros2_msg->time_stamp = 0;
    ros2_msg->xy_position_rms.position_error = 0;
    ros2_msg->z_position_rms.distance_error = 0;

    //cout<<"\tpublishing\n";
    m_globalPoseStatusTypePub->publish(ros2_msg);
}

/*
void publishAndSubscribe::MavlinkGlobalPositionIntCallback( mavlink_global_position_int_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkAltitudeCallback( mavlink_attitude_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkVfrHudCallback( mavlink_vfr_hud_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkScaledPressureCallback( mavlink_scaled_pressure_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkScaledPressure2Callback( mavlink_scaled_pressure2_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkScaledPressure3Callback( mavlink_scaled_pressure3_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkCameraImageCapturedCallback( mavlink_camera_image_captured_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkADSBVehicleCallback( mavlink_adsb_vehicle_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkHighLatency2Callback( mavlink_high_latency2_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkAttitudeCallback( mavlink_attitude_t mavlink_msg )
{
    diux_msgs::msg::GlobalPoseCommandType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GlobalPoseCommandType>();

    ros2_msg.x = mavlink_msg.pitch;
    ros2_msg.y = mavlink_msg.roll;
    ros2_msg.z = mavlink_msg.yaw;
    ros2_msg.a = mavlink_msg.time_boot_ms;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkAttitudeQuaternionCallback( mavlink_attitude_quaternion_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkAttitudeTargetCallback( mavlink_attitude_target_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkEstimatorStatusCallback( mavlink_estimator_status_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkStatusTextCallback( mavlink_statustext_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkOrbitExecutionStatusCallback( mavlink_orbit_execution_status_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkDistanceSensorCallback( mavlink_distance_sensor_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkHomePositionCallback( mavlink_home_position_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/
/*
void publishAndSubscribe::MavlinkRadioStatusCallback( mavlink_radio_status_t mavlink_msg )
{
    diux_msgs::msg::GeomagneticPropertyCommandStatusType::SharedPtr ros2_msg = std::make_shared<diux_msgs::msg::GeomagneticPropertyCommandStatusType>();

    ros2_msg.x = mavlink_msg.x;
    ros2_msg.y = mavlink_msg.y;
    ros2_msg.z = mavlink_msg.z;

    m_geomagneticPropertyCommandStatusTypePub->publish(ros2_msg);
}
*/

//---------------------------------------------HANDLING MAVLINK MESSAGES---------------------------------------------//

                //takes a recieved mavlink message, decodes it, and calls a MavlinkCallback function

//-------------------------------------------------------------------------------------------------------------------//

void publishAndSubscribe::_handleHeartbeat(mavlink_message_t& message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    int a = heartbeat.base_mode;
    std::cout<<"BaseMode\t"<<(a)<<"\n";
}

/*
void _handleRCChannels(mavlink_message_t& message)
{

}
*/
/*
void _handleRCChannelsRaw(mavlink_message_t& message)
{

}
*/
/*
void _handleBatteryStatus(mavlink_message_t& message)
{
	mavlink_battery_status_t battery;
	mavlink_msg_battery_status_decode(&message, &battery);
  MavlinkBatteryStatusCallback(battery);
}
*/
/*
void _handleSysStatus(mavlink_message_t& message)
{
	mavlink_sys_status_t sysStatus;
	mavlink_msg_sys_status_decode(&message, &sysStatus);
  MavlinkSysStatusCallback(sysStatus);
}
*/
/*
void _handleVibration(mavlink_message_t& message)
{
	mavlink_vibration_t vibration;
	mavlink_msg_vibration_decode(&message, &vibration);
  MavlinkVibrationCallback(vibration);
}
*/
/*
void _handleExtendedSysState(mavlink_message_t& message)
{
	mavlink_extended_sys_state_t extendedSysState;
	mavlink_msg_extended_sys_state_decode(&message, &extendedSysState);
  MavlinkVibrationCallback(extendedSysState);
}
*/
/*
void _handleCommandAck(mavlink_message_t& message)
{
	mavlink_command_ack_t commandAck;
	mavlink_msg_command_ack_decode(&message, &commandAck);
  MavlinkCommandAckCallback(commandAck);
}
*/
/*
void _handleCommandLong(mavlink_message_t& message)
{
	mavlink_command_long_t commandLong;
	mavlink_msg_command_long_decode(&message, &commandLong);
  MavlinkCommandLongCallback(commandLong)
}
*/
/*
void _handleAutopilotVersion(mavlink_message_t& message)
{
	mavlink_autopilot_version_t autopilotVersion;
	mavlink_msg_autopilot_version_decode(&message, &autopilotVersion);
  MavlinkAutopilotVersionCallback(autopilotVersion);
}
*/
/*
void _handleWindCov(mavlink_message_t& message)
{
	mavlink_wind_cov_t windCov;
	mavlink_msg_wind_cov_decode(&message, &windCov);
  MavlinkWindCovCallback(windCov);
}
*/
/*
void _handleHilActuatorControls(mavlink_message_t& message)
{
	mavlink_hil_actuator_controls_t hilActuatorControls;
	mavlink_msg_hil_actuator_controls_decode(&message, &hilActuatorControls);
  MavlinkHilActuatorControlsCallback(hilActuatorControls);
}
*/
/*
_handleMavlinkLoggingData(mavlink_message_t& message)
{
	mavlink_logging_data_t loggingData;
	mavlink_msg_logging_data_decode(&message, &loggingData);
  MavlinkLoggingDataCallback(loggingData);
}
*/
/*
_handleMavlinkLoggingDataAcked(mavlink_message_t& message)
{
	mavlink_logging_data_acked_t loggingDataAcked;
	mavlink_msg_logging_data_acked_decode(&message, &loggingDataAcked);
  MavlinkLoggingDataAckedCallback(loggingDataAcked);
}
*/

void publishAndSubscribe::_handleGpsRawInt(mavlink_message_t message)
{
	mavlink_gps_raw_int_t gpsRawInt;
	mavlink_msg_gps_raw_int_decode(&message, &gpsRawInt);
  //cout<<"handlingGpsRawInt";
  MavlinkGpsRawIntCallback(gpsRawInt);
}

/*
void _handleGlobalPositionInt(mavlink_message_t& message)
{
	mavlink_global_position_int_t globalPositionInt;
	mavlink_msg_global_position_int_decode(&message, &globalPositionInt);
  MavlinkGlobalPositionIntCallback(globalPositionInt);
}
*/
/*
void _handleAltitude(mavlink_message_t& message)
{
	mavlink_altitude_t altitude;
	mavlink_msg_altitude_decode(&message, &altitude);
  MavlinkAltitudeCallback(altitude);
}
*/
/*
void _handleVfrHud(mavlink_message_t& message)
{
	mavlink_vfr_hud_t vfrHud;
	mavlink_msg_vfr_hud_decode(&message, &vfrHud);
  MavlinkVfrHudCallback(vfrHud);
}
*/
/*
void _handleScaledPressure(mavlink_message_t& message)
{
	mavlink_scaled_pressure_t scaledPressure;
	mavlink_msg_scaled_pressure_decode(&message, &scaledPressure);
  MavlinkScaledPressureCallback(scaledPressure);
}
*/
/*
void _handleScaledPressure2(mavlink_message_t& message)
{
	mavlink_scaled_pressure2_t scaledPressure2;
	mavlink_msg_scaled_pressure2_decode(&message, &scaledPressure2);
  MavlinkScaledPressure2Callback(scaledPressure2);
}
*/
/*
void _handleScaledPressure3(mavlink_message_t& message)
{
	mavlink_scaled_pressure3_t scaledPressure3;
	mavlink_msg_scaled_pressure3_decode(&message, &scaledPressure3);
  MavlinkHandleScaledPressure3Callback(scaledPressure3);
}
*/
/*
void _handleCameraImageCaptured(mavlink_message_t& message)
{
	mavlink_camera_image_captured_t cameraImageCaptured;
	mavlink_msg_camera_image_captured_decode(&message, &cameraImageCaptured);
  MavlinkCameraImageCapturedCallback(cameraImageCaptured);
}
*/
/*
void _handleADSBVehicle(mavlink_message_t& message)
{
	mavlink_adsb_vehicle_t adsbVehicle;
	mavlink_msg_adsb_vehicle_decode(&message, &adsbVehicle);
  MavlinkADSBVehicleCallback(adsbVehicle);
}
*/
/*
void _handleHighLatency2(mavlink_message_t& message)
{
	mavlink_high_latency2_t highLatency2;
	mavlink_msg_high_latency2_decode(&message, &highLatency2);
  MavlinkHighLatency2Callback(highLatency2);
}
*/
/*
void _handleAttitude(mavlink_message_t& message)
{
	mavlink_attitude_t attitude;
	mavlink_msg_attitude_decode(&message, &attitude);
  MavlinkAttitudeCallback(attitude);
}
*/
/*
void _handleAttitudeQuaternion(mavlink_message_t& message)
{
	mavlink_attitude_quaternion_t attitudeQuaternion;
	mavlink_msg_attitude_quaternion_decode(&message, &attitudeQuaternion);
  MavlinkAttitudeQuaternionCallback(attitudeQuaternion);
}
*/
/*
void _handleAttitudeTarget(mavlink_message_t& message)
{
	mavlink_attitude_target_t attitudeTarget;
	mavlink_msg_attitude_target_decode(&message, &attitudeTarget);
  MavlinkAttitudeTargetCallback(attitudeTarget);
}
*/
/*
void _handleEstimatorStatus(mavlink_message_t& message)
{
	mavlink_estimator_status_t estimatorStatus;
	mavlink_msg_estimator_status_decode(&message, &estimatorStatus);
  MavlinkEstimatorStatusCallback(estimatorStatus);
}
*/
/*
void _handleStatusText(mavlink_message_t& message)
{
	mavlink_statustext_t statusText;
	mavlink_msg_statustext_decode(&message, &statusText);
  MavlinkStatusTextCallback(statusText);
}
*/
/*
void _handleOrbitExecutionStatus(mavlink_message_t& message)
{
	mavlink_orbit_execution_status_t orbitExecutionStatus;
	mavlink_msg_orbit_execution_status_decode(&message, &orbitExecutionStatus);
  MavlinkOrbitExecutionStatusCallback(orbitExecutionStatus);
}
*/
/*void _handlePing(mavlink_message_t& message)
{

}
*/
/*
void _handleDistanceSensor(mavlink_message_t& message)
{
	mavlink_distance_sensor_t distanceSensor;
	mavlink_msg_distance_sensor_decode(&message, &distanceSensor);
  MavlinkDistanceSensorCallback(distanceSensor);
}
*/
/*
void _handleHomePosition(mavlink_message_t& message)
{
	mavlink_home_position_t homePosition;
	mavlink_msg_home_position_decode(&message, &homePosition);
  MavlinkHomePositionCallback(homePosition);
}
*/
/*
void _handleRadioStatus(mavlink_message_t& message)
{
	mavlink_radio_status_t radioStatus;
	mavlink_msg_radio_status_decode(&message, &radioStatus);
  MavlinkRadioStatusCallback(radioStatus);
}
*/


//---------------------------------------------RECIEVING MAVLINK MESSAGES---------------------------------------------//

                //recieves mavlink messages, and places a call to a handling function

//--------------------------------------------------------------------------------------------------------------------//

void publishAndSubscribe::recieveMessages(int slen, int fd, struct sockaddr_in remaddr)
{	char buf[BUFLEN];
	int recvlen;

  cout<<"Messaging Recieving Spinning up \n";
	while(recieving)
	{
		recvlen = recvfrom(fd, buf, BUFLEN, 0, (struct sockaddr *)&remaddr, &slen);
		    if (recvlen >= 0)
		    {
		        buf[recvlen] = 0;
			int channel = 0;
			mavlink_message_t message;
			mavlink_status_t status;
			memset( &message, 0, sizeof(mavlink_message_t) );
			memset( &status, 0, sizeof(mavlink_status_t) );
			for( int i = 0; i < BUFSIZE; i++ )
			{
	 		  if( mavlink_parse_char( channel, buf[i], &message, &status ) )
	  		 {
				switch (message.msgid)
				{
				    case MAVLINK_MSG_ID_HOME_POSITION:
            //cout<<"0";
					//_handleHomePosition(message);
					break;
				    case MAVLINK_MSG_ID_HEARTBEAT:
            //cout<<"1\n";
          _handleHeartbeat(message);
					break;
				    case MAVLINK_MSG_ID_RADIO_STATUS:
            //cout<<"2\n";
          //_handleRadioStatus(message);
					break;
				    case MAVLINK_MSG_ID_RC_CHANNELS:
            //cout<<"3\n";
          //_handleRCChannels(message);
					break;
				    case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
            //cout<<"4\n";
          //_handleRCChannelsRaw(message);
					break;
				    case MAVLINK_MSG_ID_BATTERY_STATUS:
            //cout<<"5\n";
          //_handleBatteryStatus(message);
					break;
				    case MAVLINK_MSG_ID_SYS_STATUS:
            //cout<<"6\n";
          //_handleSysStatus(message);
					break;
				    //case MAVLINK_MSG_ID_RAW_IMU:
					//emit mavlinkRawImu(message);
					//break;
				    //case MAVLINK_MSG_ID_SCALED_IMU:
					//emit mavlinkScaledImu1(message);
					//break;
				   // case MAVLINK_MSG_ID_SCALED_IMU2:
					//emit mavlinkScaledImu2(message);
					//break;
				    //case MAVLINK_MSG_ID_SCALED_IMU3:
					//emit mavlinkScaledImu3(message);
					//break;
				    case MAVLINK_MSG_ID_VIBRATION:
            //cout<<"7\n";
          //_handleVibration(message);
					break;
				    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
            //cout<<"8\n";
          //_handleExtendedSysState(message);
					break;
				    case MAVLINK_MSG_ID_COMMAND_ACK:
            //cout<<"9\n";
          //_handleCommandAck(message);
					break;
				    case MAVLINK_MSG_ID_COMMAND_LONG:
            //cout<<"10\n";
          //_handleCommandLong(message);
					break;
				    case MAVLINK_MSG_ID_WIND_COV:
            //cout<<"11\n";
          //_handleWindCov(message);
					break;
				    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
            //cout<<"12\n";
          //_handleHilActuatorControls(message);
					break;
				    case MAVLINK_MSG_ID_LOGGING_DATA:
            //cout<<"13\n";
          //_handleMavlinkLoggingData(message);
					break;
				    case MAVLINK_MSG_ID_LOGGING_DATA_ACKED:
            //cout<<"14\n";
          //_handleMavlinkLoggingDataAcked(message);
					break;
				    case MAVLINK_MSG_ID_GPS_RAW_INT:
            //cout<<"15\n";
          _handleGpsRawInt(message);
					break;
				    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            //cout<<"16\n";
          //_handleGlobalPositionInt(message);
					break;
				    case MAVLINK_MSG_ID_ALTITUDE:
            //cout<<"17\n";
          //_handleAltitude(message);
					break;
				    case MAVLINK_MSG_ID_VFR_HUD:
            //cout<<"18\n";
          //_handleVfrHud(message);
					break;
				    case MAVLINK_MSG_ID_SCALED_PRESSURE:
            //cout<<"19\n";
          //_handleScaledPressure(message);
					break;
				    case MAVLINK_MSG_ID_SCALED_PRESSURE2:
            //cout<<"20\n";
          //_handleScaledPressure2(message);
					break;
				    case MAVLINK_MSG_ID_SCALED_PRESSURE3:
            //cout<<"21\n";
          //_handleScaledPressure3(message);
					break;
				    case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
            //cout<<"22\n";
          //_handleCameraImageCaptured(message);
					break;
				    case MAVLINK_MSG_ID_ADSB_VEHICLE:
            //cout<<"23\n";
          //_handleADSBVehicle(message);
					break;
				    case MAVLINK_MSG_ID_HIGH_LATENCY2:
            //cout<<"24\n";
          //_handleHighLatency2(message);
					break;
				    case MAVLINK_MSG_ID_ATTITUDE:
            //cout<<"25\n";
          //_handleAttitude(message);
					break;
				    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            //cout<<"26\n";
          //_handleAttitudeQuaternion(message);
					break;
				    case MAVLINK_MSG_ID_ATTITUDE_TARGET:
            //cout<<"27\n";
          //_handleAttitudeTarget(message);
					break;
				    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
            //cout<<"28\n";
          //_handleDistanceSensor(message);
					break;
				    case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
            //cout<<"29\n";
          //_handleEstimatorStatus(message);
					break;
				    case MAVLINK_MSG_ID_STATUSTEXT:
            //cout<<"30\n";
          //_handleStatusText(message);
					break;
				    case MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS:
            //cout<<"31\n";
          //_handleOrbitExecutionStatus(message);
					break;
				}

	     		 memset( &message, 0, sizeof(mavlink_message_t) );
	     		 memset( &status, 0, sizeof(mavlink_status_t) );
	  		 }
			}
		     }
	}

}

void publishAndSubscribe::sendArmVehicle()
{

	char buf2[BUFLEN];
	uint8_t _vehicleSystemId = 255;
	uint8_t _vehicleComponentId = 1;
	uint8_t _mavlinkChannel = 2;
	uint8_t _vehicleType = 135;
	uint8_t _firmwareType = 0;
	uint8_t _mavBaseMode = 0;
	uint32_t _mavCustomMode = 0;
	uint8_t _mavState = 4;

	usleep(100000);

	mavlink_message_t   msg2;
	mavlink_msg_command_long_pack(255, _vehicleComponentId, &msg2, 1, 0, MAV_CMD_COMPONENT_ARM_DISARM, true, 1, 0,0, 0,0,0,0);

	int len = mavlink_msg_to_send_buffer(buf2, &msg2);

	//SENDING//
	if (sendto(fd, buf2, len, 0, (struct sockaddr *)&remaddr, slen)==-1) {
		perror("sendto");
		exit(1);
	}

}

//---------------------------------------------MAIN---------------------------------------------//

                //creates UDP socket connections, creates multiple threads for sending/recieving,
                //UDP messages, and initializes and spins ROS messages
//----------------------------------------------------------------------------------------------//

int main(int argc, char * argv[])
{
    //BUILDING UDP CONNECTION//
/*  struct sockaddr_in myaddr, remaddr;
  int fd, i, slen=sizeof(remaddr);
*/
  int recvlen;
  const char *server = "127.0.0.1";	// change this to use a different server //

  // create a socket //
  if ((fd=socket(AF_INET, SOCK_DGRAM, 0))==-1)
    printf("socket created\n");

  // bind it to all local addresses and pick any port number //
  memset((char *)&myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  myaddr.sin_port = htons(0);

  if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
    perror("bind failed");
    return 0;
  }

  // now define remaddr, the address to whom we want to send messages //
  // For convenience, the host address is expressed as a numeric IP address //
  // that we will convert to a binary format via inet_aton //
  memset((char *) &remaddr, 0, sizeof(remaddr));
  remaddr.sin_family = AF_INET;
  remaddr.sin_port = htons(SERVICE_PORT);
  if (inet_aton(server, &remaddr.sin_addr)==0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }

  cout<<"Socket Created \n";


  rclcpp::init(argc, argv);
  publishAndSubscribe * translationLayer = new publishAndSubscribe("task");
  //WHERE TO PUT THE INIT, DOES SPIN LOOP, OR WILL IT ALLOW NEXT CODE TO BE EXECUTED//

  std::thread t1 (&publishAndSubscribe::sendHeartbeat, translationLayer, slen, fd, remaddr);
  std::thread t2 (&publishAndSubscribe::recieveMessages,translationLayer, slen, fd, remaddr);
  translationLayer->sendArmVehicle();

  rclcpp::spin(translationLayer->m_node);

  t1.join();
  t2.join();
  close(fd);
  exit(1);
  return 0;
}
