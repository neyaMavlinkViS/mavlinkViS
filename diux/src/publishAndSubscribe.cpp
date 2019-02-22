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


    ////MAVLINLK MESSAGE RECEIVE PROTOTYPES////


    void _handleGpsRawInt(mavlink_message_t message);
    void _handleHeartbeat(mavlink_message_t& message);
    /*
    void _handleHomePosition(mavlink_message_t message);
    void _handleRadioStatus(mavlink_message_t message);
    void _handleRCChannelsRaw(mavlink_message_t message);
    void _handleBatteryStatus(mavlink_messave_t message);
    void _handleSysStatus(mavlink_message_t message);
    void _handleVibration(mavlink_message_t message);
    void _handleExtendedSysState(malvink_message_t message);
    void _handleCommandAck(mavlink_message_t message);
    void _handleCommandLong(mavlink_message_t message);
    void _handleWindCov(mavlink_message_t message);
    void _handleHilActuatorControls(mavlink_message_t message);
    void _handleMavlinkLoggingData(mavlink_message_t message);
    void _handleMavlinkLoggingDataAcked(mavlink_message_t message);
    void _handleGlobalPositionInt(mavlink_message_t message);
    void _handleAltitude(mavlink_message_t message);
    void _handleVfrHud(mavlink_message_t message);
    void _handleScaledPressure(mavlink_message_t message);
    void _handleScaledPressure2(mavlink_message_t message);
    void _handleScaledPressure3(mavlink_message_t message);
    void _handleCameraImageCaptured(mavlink_message_t message);
    void _handleADSBVehicle(mavlink_message_t message);
    void _handleHighLatency2(mavlink_message_t message);
    void _handleAttitude(mavlink_message_t message);
    void _handleAttitudeQuaternion(mavlink_message_t message);
    void _handleAttitudeTarget(mavlink_message_t message);
    void _handleDistanceSensor(mavlink_message_t message);
    void _handleEstimatorStatus(mavlink_message_t message);
    void _handleStatusText(mavlink_message_t message);
    void _handleOrbitExecutionStatus(mavlink_message_t message);
    void _handleSystemTime(mavlink_message_t message);
    void _handlePing(mavlink_message_t message);
    void _handleChangeOperatorControl(mavlink_message_t message);
    void _handleAuthKey(mavlink_message_t message);
    void _handleSetMode(mavlink_message_t message);
    void _handleParamRequested(mavlink_message_t message);
    void _handleParamRequestList(mavlink_message_t message);
    void _handleGPSStatus(mavlink_message_t message);
    void _handleRawPressure(mavlink_message_t message);
    void _handleLocalPositionNed(mavlink_message_t message);
    void _handleServoOutputRaw(mavlink_message_t message);
    void _handleMissionRequestPartialList(mavlink_message_t message);
    void _handleWritePartialList(mavlink_message_t message);
    void _handleMissionItem(mavlink_message_t message);
    void _handleMissionRequest(mavlink_message_t message);
    void _handleSetCurrent(mavlink_message_t message);
    void _handleRequestList(mavlink_message_t message);
    void _handleMissionCount(mavlink_message_t message);
    void _handleClearAll(mavlink_message_t message);
    void _handleItemReached(mavlink_message_t message);
    void _handleMissionAcked(mavlink_message_t message);
    void _handleSetGPSGlobalOrigin(mavlink_message_t message);
    void _handleGPSGlobalOrigin(mavlink_message_t message);
    void _handleParamMapRC(mavlink_message_t message);
    void _handleMissionRequestInt(mavlink_message_t message);
    void _handleSafetySetAllowedArea(mavlink_message_t message);
    void _handleSafetyAllowedArea(mavlink_message_t message);
    void _handleAttitudeQuaternionCov(mavlink_message_t message);
    void _handleNavControllerOoutput(mavlink_message_t message);
    void _handleGlobalPositionNedCov(mavlink_message_t message);
    void _handleRCChannels(mavlink_message_t message);
    void _handleRequestDataStream(mavlink_message_t message);
    void _handleDataStream(mavlink_message_t message);
    void _handleManualControl(mavlink_message_t message);
    void _handleRCChannelsOverride(mavlink_message_t message);
    void _handleMissionItemInt(mavlink_message_t message);
    void _handleCommandInt(mavlink_message_t message);
    void _handleManualSetpoint(mavlink_message_t message);
    void _handleSetAttitudeTarget(mavlink_message_t message);
    void _handleSetPositionTargetLocalNed(mavlink_message_t message);
    void _handlePositionTargetLocalNed(mavlink_message_t message);
    void _handleSetPositionTargetGlobalInt(mavlink_message_t message);
    void _handlePositionTargetGlobalInt(mavlink_message_t message);
    void _handleLocalPositionNedSystemGlobalOffset(mavlink_message_t message);
    void _handleHilState(mavlink_message_t message);
    void _handleHilControls(mavlink_message_t message);
    void _handleRCInputsRaw(mavlink_message_t message);
    void _handleOpticalFlow(mavlink_message_t message);
    void _handleGlobalVisionPositionEstimate(mavlink_message_t message);
    void _handleVisionPositionEstimate(mavlink_message_t message);
    void _handleVisionSpeedEstimate(mavlink_message_t message);
    void _handleViconPositionEstimate(mavlink_message_t message);
    void _handleGighresIMU(mavlink_message_t message);
    void _handleOpticalFlowRad(mavlink_message_t message);
    void _handleHilSensor(mavlink_message_t message);
    void _handleSimstate(mavlink_message_t message);
    void _handleSim_State(mavlink_message_t message);
    void _handleFileTransferProtocol(mavlink_message_t message);
    void _handletimeSync(mavlink_message_t message);
    void _handleCameraTrigger(mavlink_message_t message);
    void _handleHilGPS(mavlink_message_t message);
    void _handleHilOpticalFlow(mavlink_message_t message);
    void _handleHilStateQuaternion(mavlink_message_t message);
    void _handleLogRequestList(mavlink_message_t message);
    void _handleLogEntry(mavlink_message_t message);
    void _handleLogRequestData(mavlink_message_t message);
    void _handleLogData(mavlink_message_t message);
    void _handleLogErase(mavlink_message_t message);
    void _handleLogRequestEnd(mavlink_message_t message);
    void _handleGPSInjectData(mavlink_message_t message);
    void _handleGPS2Raw(mavlink_message_t message);
    void _handlePowerStatus(mavlink_message_t message);
    void _handleSerialControl(mavlink_message_t message);
    void _handleGPSRtk(mavlink_message_t message);
    void _handleGPS2Rtk(mavlink_message_t message);
    void _handleDataTransmissionHandshake(mavlink_message_t message);
    void _handleEncapsulatedData(mavlink_message_t message);
    void _handleTerrainRequest(mavlink_message_t message);
    void _handleTerrainData(mavlink_message_t message);
    void _handleTerrainCheck(mavlink_message_t message);
    void _handleTerrainReport(mavlink_message_t message);
    void _handleAttPosMocap(mavlink_message_t message);
    void _handleSetActuatorControlTarget(mavlink_message_t message);
    void _handleActuatorControlTarget(mavlink_message_t message);
    void _handleResourceRequest(mavlink_message_t message);
    void _handleFollowTarget(mavlink_message_t message);
    void _handleControlSystemState(mavlink_message_t message);
    void _handleLandingTarget(mavlink_message_t message);
    void _handleGPSInput(mavlink_message_t message);
    void _handleGPSRtcmData(mavlink_message_t message);
    void _handleHighLatency(mavlink_message_t message);
    void _handleSetHomePosition(mavlink_message_t message);
    void _handleMessageInterval(mavlink_message_t message);
    void _handleCollision(mavlink_message_t message);
    void _handleV2Extension(mavlink_message_t message);
    void _handleMemoryVect(mavlink_message_t message);
    void _handleDebugVect(mavlink_message_t message);
    void _handleNamedValueFloat(mavlink_message_t message);
    void _handleNamedValueInt(mavlink_message_t message);
    void _handleDebug(mavlink_message_t message);
    void _handleSetupSigning(mavlink_message_t message);
    void _handleButtonChange(mavlink_message_t message);
    void _handlePlayTune(mavlink_message_t message);
    void _handleCameraInformation(mavlink_message_t message);
    void _handleCameraSettings(mavlink_message_t message);
    void _handleStorageInformation(mavlink_message_t message);
    void _handleCameraCaptureStatus(mavlink_message_t message);
    void _handleFlightInformation(mavlink_message_t message);
    void _handleMountOrientation(mavlink_message_t message);
    void _handleLoggingAck(mavlink_message_t message);
    void _handleVideoStreamInformation(mavlink_message_t message);
    void _handleSetVideoStreamInformation(mavlink_message_t message);
    void _handleWIFIConfigAp(mavlink_message_t message);
    void _handleProtocolVersion(mavlink_message_t message);
    void _handleVAVCANNNodeStatus(mavlink_message_t message);
    void _handleUAVCANNodeInfo(mavlink_message_t message);
    void _handleParamExtRequestRead(mavlink_message_t message);
    void _handleParamExtRequestList(mavlink_message_t message);
    void _handleParamExtValue(mavlink_message_t message);
    void _handleParamExtSet(mavlink_message_t message);
    void _handleParamExtAck(mavlink_message_t message);
    void _handleObstacleDistance(mavlink_message_t message);
    void _handleOdometry(mavlink_message_t message);
    void _handleTrajectoryRepresentationWaypoints(mavlink_message_t message);
    void _handleTrajectoryRepresenationBezier(mavlink_message_t message);
    void _handleUTMGlobalPosition(mavlink_message_t message);
    void _handleDebugFloatArray(mavlink_message_t message);
    */

    ////PROTOTYPE FOR MAVLINK SEND////

    void recieveMessages(int slen, int fd, struct sockaddr_in remaddr);
    void sendHeartbeat(int slen, int fd, struct sockaddr_in remaddr);
    void sendManualControl(float newPitchCommand, float newRollCommand, float newThrustCommand, float newYawCommand, int buttons);
    void sendArmVehicle();

    ////PROTOTYPE FOR ROS2 MSG CALLBACKS////

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



//-------------------------------------------------------------------------------------------------------------------------//

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
}
*/
/*
void _handleSysStatus(mavlink_message_t& message)
{
	mavlink_sys_status_t sysStatus;
	mavlink_msg_sys_status_decode(&message, &sysStatus);
}
*/
/*
void _handleVibration(mavlink_message_t& message)
{
	mavlink_vibration_t vibration;
	mavlink_msg_vibration_decode(&message, &vibration);
}
*/
/*
void _handleExtendedSysState(mavlink_message_t& message)
{
	mavlink_extended_sys_state_t extendedSysState;
	mavlink_msg_extended_sys_state_decode(&message, &extendedSysState);
}
*/
/*
void _handleCommandAck(mavlink_message_t& message)
{
	mavlink_command_ack_t commandAck;
	mavlink_msg_command_ack_decode(&message, &commandAck);
}
*/
/*
void _handleCommandLong(mavlink_message_t& message)
{
	mavlink_command_long_t commandLong;
	mavlink_msg_command_long_decode(&message, &commandLong);
}
*/
/*
void _handleAutopilotVersion(mavlink_message_t& message)
{
	mavlink_autopilot_version_t autopilotVersion;
	mavlink_msg_autopilot_version_decode(&message, &autopilotVersion);
}
*/
/*
void _handleWindCov(mavlink_message_t& message)
{
	mavlink_wind_cov_t windCov;
	mavlink_msg_wind_cov_decode(&message, &windCov);
}
*/
/*
void _handleHilActuatorControls(mavlink_message_t& message)
{
	mavlink_hil_actuator_controls_t hilActuatorControls;
	mavlink_msg_hil_actuator_controls_decode(&message, &hilActuatorControls);
}
*/
/*
_handleMavlinkLoggingData(mavlink_message_t& message)
{
	mavlink_logging_data_t loggingData;
	mavlink_msg_logging_data_decode(&message, &loggingData);
}
*/
/*
_handleMavlinkLoggingDataAcked(mavlink_message_t& message)
{
	mavlink_logging_data_acked_t loggingDataAcked;
	mavlink_msg_logging_data_acked_decode(&message, &loggingDataAcked);
}
*/

void publishAndSubscribe::_handleGpsRawInt(mavlink_message_t message)
{
	mavlink_gps_raw_int_t mavlink_msg;
	mavlink_msg_gps_raw_int_decode(&message, &mavlink_msg);
  //cout<<"handlingGpsRawInt";
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
void _handleGlobalPositionInt(mavlink_message_t& message)
{
	mavlink_global_position_int_t globalPositionInt;
	mavlink_msg_global_position_int_decode(&message, &globalPositionInt);
}
*/
/*
void _handleAltitude(mavlink_message_t& message)
{
	mavlink_altitude_t altitude;
	mavlink_msg_altitude_decode(&message, &altitude);
}
*/
/*
void _handleVfrHud(mavlink_message_t& message)
{
	mavlink_vfr_hud_t vfrHud;
	mavlink_msg_vfr_hud_decode(&message, &vfrHud);
}
*/
/*
void _handleScaledPressure(mavlink_message_t& message)
{
	mavlink_scaled_pressure_t scaledPressure;
	mavlink_msg_scaled_pressure_decode(&message, &scaledPressure);
}
*/
/*
void _handleScaledPressure2(mavlink_message_t& message)
{
	mavlink_scaled_pressure2_t scaledPressure2;
	mavlink_msg_scaled_pressure2_decode(&message, &scaledPressure2);
}
*/
/*
void _handleScaledPressure3(mavlink_message_t& message)
{
	mavlink_scaled_pressure3_t scaledPressure3;
	mavlink_msg_scaled_pressure3_decode(&message, &scaledPressure3);
}
*/
/*
void _handleCameraImageCaptured(mavlink_message_t& message)
{
	mavlink_camera_image_captured_t cameraImageCaptured;
	mavlink_msg_camera_image_captured_decode(&message, &cameraImageCaptured);
}
*/
/*
void _handleADSBVehicle(mavlink_message_t& message)
{
	mavlink_adsb_vehicle_t adsbVehicle;
	mavlink_msg_adsb_vehicle_decode(&message, &adsbVehicle);
}
*/
/*
void _handleHighLatency2(mavlink_message_t& message)
{
	mavlink_high_latency2_t highLatency2;
	mavlink_msg_high_latency2_decode(&message, &highLatency2);
}
*/
/*
void _handleAttitude(mavlink_message_t& message)
{
	mavlink_attitude_t attitude;
	mavlink_msg_attitude_decode(&message, &attitude);
}
*/
/*
void _handleAttitudeQuaternion(mavlink_message_t& message)
{
	mavlink_attitude_quaternion_t attitudeQuaternion;
	mavlink_msg_attitude_quaternion_decode(&message, &attitudeQuaternion);
}
*/
/*
void _handleAttitudeTarget(mavlink_message_t& message)
{
	mavlink_attitude_target_t attitudeTarget;
	mavlink_msg_attitude_target_decode(&message, &attitudeTarget);
}
*/
/*
void _handleEstimatorStatus(mavlink_message_t& message)
{
	mavlink_estimator_status_t estimatorStatus;
	mavlink_msg_estimator_status_decode(&message, &estimatorStatus);
}
*/
/*
void _handleStatusText(mavlink_message_t& message)
{
	mavlink_statustext_t statusText;
	mavlink_msg_statustext_decode(&message, &statusText);
}
*/
/*
void _handleOrbitExecutionStatus(mavlink_message_t& message)
{
	mavlink_orbit_execution_status_t orbitExecutionStatus;
	mavlink_msg_orbit_execution_status_decode(&message, &orbitExecutionStatus);
}
*/
/*void _handlePing(mavlink_message_t& message)
{
  mavlink_ping_t ping;
  mavlink_msg_ping_decode(&message, &ping);
}
*/
/*
void _handleDistanceSensor(mavlink_message_t& message)
{
	mavlink_distance_sensor_t distanceSensor;
	mavlink_msg_distance_sensor_decode(&message, &distanceSensor);
}
*/
/*
void _handleHomePosition(mavlink_message_t& message)
{
	mavlink_home_position_t homePosition;
	mavlink_msg_home_position_decode(&message, &homePosition);
}
*/
/*
void _handleRadioStatus(mavlink_message_t& message)
{
	mavlink_radio_status_t radioStatus;
	mavlink_msg_radio_status_decode(&message, &radioStatus);
}

void _handleHomePosition(mavlink_message_t message)
{
  mavlink_home_position_t homePosition;
  mavlink_msg_home_position_decode(&message, &homePosition);
}
void _handleRadioStatus(mavlink_message_t message);
{
  mavlink_radio_status_t radioStatus;
  mavlink_msg_radio_status_decode(&message, &radioStatus);
}
void _handleRCChannelsRaw(mavlink_message_t message);
{
  mavlink_rc_channels_raw_t RCChannels;
  mavlink_msg_rc_channels_raw_decode(&message, &RCChannels);
}
void _handleBatteryStatus(mavlink_messave_t message);
{
  mavlink_battery_status_t batteryStatus;
  mavlink_msg_battery_status_decode(&message, &batteryStatus);
}
void _handleSysStatus(mavlink_message_t message);
{
  mavlink_sys_status_t sysStatus;
  mavlink_msg_sys_status_decode(&message, &sysStatus);
}
void _handleVibration(mavlink_message_t message);
{
  mavlink_vibration_t vibration;
  mavlink_msg_vibration_decode(&message, &vibration);
}
void _handleExtendedSysState(malvink_message_t message);
{
  mavlink_extended_sys_state_t extendedSysState;
  mavlink_msg_extended_sys_state_decode(&message, &extendedSysState);
}
void _handleCommandAck(mavlink_message_t message);
{
  mavlink_command_ack_t commandAck;
  malvink_msg_command_ack_decode(&message, &commandAck);
}
void _handleCommandLong(mavlink_message_t message);
{
  mavlink_command_long_t commandLong;
  mavlink_msg_command_long_decode(&message, &commandLong);
}
void _handleWindCov(mavlink_message_t message);
{
  mavlink_wind_cov_t windCov;
  mavlink_msg_wind_cov_decode(&message, &windCov);
}
void _handleHilActuatorControls(mavlink_message_t message);
{
  mavlink_hil_actuator_controls_t hilActuatorControls;
  mavlink_msg_hil_actuator_controls_decode(&message, &hilActuatorControls);
}
void _handleMavlinkLoggingData(mavlink_message_t message);
{
  mavlink_logging_data_t loggingData;
  mavlink_msg_logging_data_decode(&message, &loggingData);
}
void _handleMavlinkLoggingDataAcked(mavlink_message_t message);
{
  mavlink_logging_data_acked_t loggingDataAcked;
  mavlink_msg_logging_data_acked_decode(&message, &loggingDataAcked);
}
void _handleGlobalPositionInt(mavlink_message_t message);
{
  mavlink_global_position_int_t globalPositionInt;
  mavlink_msg_global_position_int_decode(&message, &globalPositionInt);
}
void _handleAltitude(mavlink_message_t message);
{
  mavlink_altitude_t altitude;
  mavlink_msg_altitude_decode(&message, &altitude);
}
void _handleVfrHud(mavlink_message_t message);
{
  mavlink_vfr_hud_t vfrHud;
  mavlink_msg_vfr_hud_decode(&message, &vfrHud);
}
void _handleScaledPressure(mavlink_message_t message);
{
  mavlink_scaled_pressure_t scaledPressure;
  mavlink_message_scaled_pressure_decode(&message, &scaledPressure);
}
void _handleScaledPressure2(mavlink_message_t message);
{
  mavlink_scaled_pressure_t scaledPressure2;
  mavlink_message_scaled_pressure2_decode(&message, &scaledPressure2);
}
void _handleScaledPressure3(mavlink_message_t message);
{
  mavlink_scaled_pressure_t scaledPressure3;
  mavlink_message_scaled_pressure3_decode(&message, &scaledPressure3);
}
void _handleCameraImageCaptured(mavlink_message_t message);
{
    mavlink_camera_image_captured_t cameraImageCaptured;
    mavlink_msg_camera_image_captured_decode(&message, &cameraImageCaptured);
}
void _handleADSBVehicle(mavlink_message_t message);
{
  mavlink_adsb_vehicle_t adsbVehicle;
  mavlink_msg_adsb_vehicle_decode(&message, &adsbVehicle);
}
void _handleHighLatency2(mavlink_message_t message);
{
  mavlink_high_latency2_t highLatency2;
  mavlink_msg_high_latency2_decode(&message, &highLatency2);
}
void _handleAttitude(mavlink_message_t message);
{
  mavlink_attitude_t attitude;
  mavlink_msg_attitude_decode(&message, &attitude);
}
void _handleAttitudeQuaternion(mavlink_message_t message);
{
  mavlink_attitude_quaternion_t attitudeQuaternion;
  mavlink_msg_attitude_quaternion_decode(&message, &attitudeQuaternion);
}
void _handleAttitudeTarget(mavlink_message_t message)
{
  mavlink_attitude_target_t attitudeTarget;
  mavlink_msg_attitude_target_decode(&message, &attitudeTarget);
}
void _handleDistanceSensor(mavlink_message_t message)
{
  mavlink_distance_sensor_t distanceSensor;
  mavlink_msg_distance_sensor_decode(&message, &distanceSensor);
}
void _handleEstimatorStatus(mavlink_message_t message)
{
  mavlink_estimator_status_t estimatorStatus;
  mavlink_msg_estimator_status_decode(&message, &estimatorStatus);
}
void _handleStatusText(mavlink_message_t message)
{
  mavlink_statustext_t statusText;
  mavlink_msg_statustext_decode(&message, &statusText);
}
void _handleOrbitExecutionStatus(mavlink_message_t message)
{
  mavlink_orbit_execution_status_t orbitExecutionStatus;
  mavlink_msg_orbit_execution_status_decode(&message, &orbitExecutionStatus);
}
void _handleSystemTime(mavlink_message_t message)
{
  mavlink_system_time_t systemTime;
  mavlink_msg_system_time_decode(&message, &systemTime);
}
void _handleChangeOperatorControl(mavlink_message_t message)
{
  mavlink_change_operator_control_t changeOperatorControl;
  mavlink_msg_change_operator_control_decode(&message, &changeOperatorControl);
}
void _handleAuthKey(mavlink_message_t message)
{
  mavlink_auth_key_t authKey;
  mavlink_msg_auth_key_decode(&message, &authKey);
}
void _handleSetMode(mavlink_message_t message)
{
  mavlink_set_mode_t setMode;
  mavlink_msg_set_mode_decode(&message, &setMode);
}
void _handleParamRequested(mavlink_message_t message)
{
  mavlink_param_requested_t paramRequested;
  mavlink_msg_param_requested_decode(&message, &paramRequested);
}
void _handleParamRequestList(mavlink_message_t message)
{
  mavlink_param_request_list_t paramRequestList;
  mavlink_msg_param_request_list_decode(&message, & paramRequestList);
}
void _handleGPSStatus(mavlink_message_t message)
{
  mavlink_gps_status_t gpsStatus;
  mavlink_msg_gps_status_decode(&message, &gpsStatus);
}
void _handleRawPressure(mavlink_message_t message)
{
  mavlink_raw_pressure_t rawPressure;
  mavlink_msg_raw_pressure_decode(&message, &rawPressure);
}
void _handleLocalPositionNed(mavlink_message_t message)
{
  mavlink_local_position_ned_t localPositionNed;
  mavlink_msg_local_position_ned_decode(&message, &localPositionNed);
}
void _handleServoOutputRaw(mavlink_message_t message)
{
  mavlink_servo_output_raw_t servoOutputRaw;
  mavlink_msg_servo_output_raw_decode(&message, &servoOutputRaw);
}
void _handleMissionRequestPartialList(mavlink_message_t message)
{
  mavlink_mission_request_partial_list_t missionRequestPartialList;
  mavlink_msg_mission_request_partial_list_decode(&message, &missionRequestPartialList);
}
void _handleWritePartialList(mavlink_message_t message)
{
  mavlink_write_partial_list_t writePartialList;
  mavlink_msg_write_partial_list_decode(&message, &writePartialList);
}
void _handleMissionItem(mavlink_message_t message)
{
  mavlink_mission_item_t missionItem;
  mavlink_msg_mission_item_decode(&message, &missionItem);
}
void _handleMissionRequest(mavlink_message_t message)
{
  mavlink_mission_request_t missionRequest;
  mavlink_msg_mission_request_decode(&message, &missionRequest);
}
void _handleSetCurrent(mavlink_message_t message)
{
  mavlink_set_current_t setCurrent;
  mavlink_msg_set_current_decode(&message, &setCurrent);
}
void _handleRequestList(mavlink_message_t message)
{
  mavlink_request_list_t requestList;
  mavlink_msg_request_list_decode(&message, &requestList);
}
void _handleMissionCount(mavlink_message_t message)
{
  mavlink_mission_count_t missionCount;
  mavlink_msg__mission_count_decode(&message, &missionCount);
}
void _handleClearAll(mavlink_message_t message)
{
  mavlink_clear_all_t clearAll;
  mavlink_msg_clear_all_decode(&message, &clearAll);
}
void _handleItemReached(mavlink_message_t message)
{
  mavlink_item_reached_t itemReached;
  mavlink_msg_item_reached_decode(&message, &itemReached);
}
void _handleMissionAck(mavlink_message_t message)
{
  mavlink_mission_ack_t missionAck;
  mavlink_msg_mission_ack_decode(&message, &missionAck);
}
void _handleSetGPSGlobalOrigin(mavlink_message_t message)
{
  mavlink_set_gps_global_origin_t setGpsGlobalOrigin;
  mavlink_msg_set_gps_global_origin_decode(setGpsGlobalOrigin);
}
void _handleGPSGlobalOrigin(mavlink_message_t message)
{
  mavlink_gps_global_origin_t gpsGlobalOrigin;
  mavlink_msg_gps_global_origin_decode(&message, &gpsGlobalOrigin);
}
void _handleParamMapRC(mavlink_message_t message)
{
  mavlink_param_map_rc_t paramMapRC;
  mavlink_param_map_rc_decode(&message, &paramMapRC);
}
void _handleMissionRequestInt(mavlink_message_t message)
{
  mavlink_mission_request_int_t missionRequestInt;
  mavlink_msg_mission_request_int_decode(&message, &missionRequestInt);
}
void _handleSafetySetAllowedArea(mavlink_message_t message)
{
  mavlink_safety_set_allowed_area_t safetySetAllowedArea;
  mavlink_msg_safety_set_allowed_area_decode(&message, &safetySetAllowedArea);
}
void _handleSafetyAllowedArea(mavlink_message_t message)
{
  mavlink_safety_allowed_area_t safetyAllowedArea;
  mavlink_msg_safety_allowed_area_decode(&message, &safetyAllowedArea);
}
void _handleAttitudeQuaternionCov(mavlink_message_t message)
{
  mavlink_attitude_quaternion_cov_t attitudeQuaternionCov;
  mavlink_msg_attitude_quaternion_cov_decode(&message, &attitudeQuaternionCov);
}
void _handleNavControllerOutput(mavlink_message_t message)
{
  mavlink_nav_controller_output_t navControllerOutput;
  mavlink_msg_nav_controller_output_decode(&message, &navControllerOutput);
}
void _handleGlobalPositionNedCov(mavlink_message_t message)
{
  mavlink_global_position_ned_cov_t globalPositionNedCov;
  mavlink_msg_global_position_ned_cov_decode(&message, &globalPositionNedCov);
}
void _handleRCChannels(mavlink_message_t message)
{
  mavlink_rc_channels_t RCChannels;
  mavlink_msg_rc_channels_decode(&message, &RCChannels);
}
void _handleRequestDataStream(mavlink_message_t message)
{
  mavlink_request_data_stream_t requestDataStream;
  mavlink_msg_request_data_stream_decode(&message, &requestDataStream);
}
void _handleDataStream(mavlink_message_t message)
{
  mavlink_data_stream_t dataStream;
  mavlink_msg_data_stream_decode(&message, &dataStream);
}
void _handleManualControl(mavlink_message_t message)
{
  mavlink_manual_control_t manualControl;
  mavlink_msg_manual_control_decode(&message, &manualControl);
}
void _handleRCChannelsOverride(mavlink_message_t message)
{
  mavlink_rc_channels_override_t RCChannelsOverride;
  mavlink_msg_rc_channels_override_decode(&message, &RCChannelsOverride);
}
void _handleMissionItemInt(mavlink_message_t message)
{
  mavlink_mission_item_int_t missionItemInt;
  mavlink_msg_mission_item_int_decode(&message, &missionItemInt);
}
void _handleCommandInt(mavlink_message_t message)
{
  mavlink_command_int_t commandInt;
  mavlink_msg_command_int_decode(&message, &commandInt);
}
void _handleManualSetpoint(mavlink_message_t message)
{
  mavlink_manual_setpoint_t manualSetpoint;
  mavlink_msg_manual_setpoint_decode(&message, &manualSetpoint);
}
void _handleSetAttitudeTarget(mavlink_message_t message)
{
  mavlink_set_attitude_target_t setAttitudeTarget;
  mavlink_msg_set_attitude_target_decode(&message, &setAttitudeTarget);
}
void _handleSetPositionTargetLocalNed(mavlink_message_t message)
{
  mavlink_set_position_target_local_ned_t setPositionTargetLocalNed;
  mavlink_msg_set_position_target_local_ned_decode(&message, &setPositionTargetLocalNed);
}
void _handlePositionTargetLocalNed(mavlink_message_t message)
{
  mavlink_position_target_local_ned_t positionTargetLocalNed;
  mavlink_msg_position_target_local_ned_decode(&message, &positionTargetLocalNed);
}
void _handleSetPositionTargetGlobalInt(mavlink_message_t message)
{
  mavlink_set_position_target_global_int_t setPositionTargetGlobalInt;
  mavlink_msg_set_position_target_global_int(&message, &setPositionTargetGlobalInt);
}
void _handlePositionTargetGlobalInt(mavlink_message_t message)
{
  mavlink_position_target_global_int_t positionTargetGlobalInt;
  mavlink_msg_position_target_global_int_decode(&message, &positionTargetGlobalInt);
}
void _handleLocalPositionNedSystemGlobalOffset(mavlink_message_t message)
{
  mavlink_local_position_ned_system_global_offset_t localPositionNedSystemGlobalOffset;
  mavlink_msg_local_position_ned_system_global_offset_decode(&message, &localPositionNedSystemGlobalOffset);
}
void _handleHilState(mavlink_message_t message)
{
  mavlink_hil_state_t hilState;
  mavlink_msg_hil_state_decode(&message, &hilState);
}
void _handleHilControls(mavlink_message_t message)
{
  mavlink_hil_controls_t hilControls;
  mavlink_msg_hil_controls_decode(&message, &hilControls);
}
void _handleRCInputsRaw(mavlink_message_t message)
{
  mavlink_rc_inputs_raw_t RCInputsRaw;
  mavlink_msg_rc_inputs_raw_decode(&message, &RCInputsRaw);
}
void _handleOpticalFlow(mavlink_message_t message)
{
  mavlink_optical_flow_t opticalFlow;
  mavlink_msg_optical_flow_decode(&message, &opticalFlow);
}
void _handleGlobalVisionPositionEstimate(mavlink_message_t message)
{
  mavlink_global_vision_position_estimate_t globalVisionPositionEstimate;
  mavlink_msg_global_vision_position_estimate_decode(&message, &globalVisionPositionEstimate);
}
void _handleVisionPositionEstimate(mavlink_message_t message)
{
  mavlink_vision_position_estimate_t visionPositionEstimate;
  mavlink_msg_vision_position_estimate_decode(&message, &visionPositionEstimate);
}
void _handleVisionSpeedEstimate(mavlink_message_t message)
{
  mavlink_vision_speed_estimate_t visionSpeedEstimate;
  mavlink_msg_vision_speed_estimate_decode(&message, &visionSpeedEstimate);
}
void _handleViconPositionEstimate(mavlink_message_t message)
{
  mavlink_vicon_position_estimate_t viconPositionEstimate;
  mavlink_msg_vicon_position_estimate_decode(&message, &viconPositionEstimate);
}
void _handleHighresIMU(mavlink_message_t message)
{
  mavlink_highres_imu_t HighresIMU;
  mavlink_msg_highres_imu_decode(&message, &HighresIMU);
}
void _handleOpticalFlowRad(mavlink_message_t message)
{
  mavlink_optical_flow_rad_t opticalFlowRad;
  mavlink_msg_optical_flow_rad_decode(&message, &opticalFlowRad);
}
void _handleHilSensor(mavlink_message_t message)
{
  mavlink_hil_sensor_t hilSensor;
  mavlink_msg_hil_sensor_decode(&message, &hilSensor);
}
void _handleSimstate(mavlink_message_t message)
{
  mavlink_simstate_t simstate;
  mavlink_msg_simstate_decode(&message, &simstate);
}
void _handleSim_State(mavlink_message_t message)
{
  mavlink_sim_state_t simState;
  mavlink_msg_sim_state_decode(&message, &simState);
}
void _handleFileTransferProtocol(mavlink_message_t message)
{
  mavlink_file_transfer_protocol_t fileTransferProtocol;
  mavlink_msg_file_transfer_protocol_decode(&message, &fileTransferProtocol);
}
void _handletimeSync(mavlink_message_t message)
{
  mavlink_timesync_t timeSync;
  mavlink_msg_timesync_decode(&message, &timeSync);
}
void _handleCameraTrigger(mavlink_message_t message)
{
  mavlink_camera_trigger_t cameraTrigger;
  mavlink_msg_camera_trigger_decode(&message, &cameraTrigger);
}
void _handleHilGPS(mavlink_message_t message);
{
  mavlink_hil_gps_t hilGPS;
  mavlink_msg_hil_gps_decode(&message, &hilGPS);
}
void _handleHilOpticalFlow(mavlink_message_t message)
{
  mavlink_hil_optical_flow_t hilOpticalFlow;
  mavlink_msg_hil_optical_flow_decode(&message, &hilOpticalFlow);
}
void _handleHilStateQuaternion(mavlink_message_t message)
{
  mavlink_hil_state_quaternion_t hilStateQuaternion;
  mavlink_msg_hil_state_quaternion_decode(&message, &hilStateQuaternion);
}
void _handleLogRequestList(mavlink_message_t message)
{
  mavlink_log_request_list_t logRequestList;
  mavlink_msg_log_request_list_decode(&message, &logRequestList);
}
void _handleLogEntry(mavlink_message_t message)
{
  mavlink_log_entry_t logEntry;
  mavlink_msg_log_entry_decode(&message, &logEntry);
}
void _handleLogRequestData(mavlink_message_t message)
{
  mavlink_log_request_data_t logRequestData;
  mavlink_msg_log_request_data_decode(&message, &logRequestData);
}
void _handleLogData(mavlink_message_t message)
{
  mavlink_log_data_t logData;
  mavlink_msg_log_data_decode(&message, &logData);
}
void _handleLogErase(mavlink_message_t message)
{
  mavlink_log_erase_t logErase;
  mavlink_msg_log_erase_decode(&message, &logErase);
}
void _handleLogRequestEnd(mavlink_message_t message)
{
  mavlink_log_request_end_t logRequestEnd;
  mavlink_msg_log_request_end_decode(&message, &logRequestEnd);
}
void _handleGPSInjectData(mavlink_message_t message)
{
  mavlink_gps_inject_data_t GPSInjectData;
  mavlink_msg_gps_inject_data_decode(&message, &GPSInjectData)
}
void _handleGPS2Raw(mavlink_message_t message)
{
  mavlink_gps2_raw_t GPS2Raw;
  mavlink_msg_gps2_raw_decode(&message, &GPS2Raw);
}
void _handlePowerStatus(mavlink_message_t message)
{
  mavlink_power_status_t powerStatus;
  mavlink_msg_power_status_decode(&message, &powerStatus);
}
void _handleSerialControl(mavlink_message_t message)
{
  mavlink_serial_control_t serialControl;
  mavlink_msg_serial_control_decode(&message, &serialControl);
}
void _handleGPSRtk(mavlink_message_t message)
{
  mavlink_gps_rtk_t GPSRtk;
  mavlink_msg_gps_rtk_decode(&message, &GPSRtk);
}
void _handleGPS2Rtk(mavlink_message_t message)
{
  mavlink_gps2_rtk_t GPS2Rtk;
  mavlink_msg_gps2_rtk_decode(&message, &GPS2Rtk);
}
void _handleDataTransmissionHandshake(mavlink_message_t message)
{
  mavlink_data_transmission_handshake_t dataTransmissionHandshake;
  mavlink_msg_data_transmission_handshake_decode(&message, &dataTransmissionHandshake);
}
void _handleEncapsulatedData(mavlink_message_t message)
{
  mavlink_encapsulated_data_t encapsulatedData;
  mavlink_msg_encapsulated_data_decode(&message, &encapsulatedData);
}
void _handleTerrainRequest(mavlink_message_t message)
{
  mavlink_terrain_request_t terrainRequest;
  mavlink_msg_terrain_request_decode(&message, &terrainRequest);
}
void _handleTerrainData(mavlink_message_t message)
{
  mavlink_terrain_data_t terrainData;
  mavlink_msg_terrain_data_decode(&message, &terrainData);
}
void _handleTerrainCheck(mavlink_message_t message)
{
  mavlink_terrain_check_t terrainCheck;
  mavlink_msg_terrain_check_decode(&message, &terrainCheck);
}
void _handleTerrainReport(mavlink_message_t message)
{
  mavlink_terrain_report_t terrainReport;
  mavlink_msg_terrain_report_decode(&message, &terrainReport);
}
void _handleAttPosMocap(mavlink_message_t message)
{
  mavlink_att_pos_mocap_t attPosMocap;
  mavlink_msg_att_pos_mocap_decode(&message, &attPosMocap);
}
void _handleSetActuatorControlTarget(mavlink_message_t message)
{
  mavlink_set_actuator_control_target_t setActuatorControlTarget;
  mavlink_msg_set_actuator_control_target_decode(&message, &setActuatorControlTarget);
}
void _handleActuatorControlTarget(mavlink_message_t message)
{
  mavlink_actuator_control_target_t actuatorControlTarget;
  mavlink_msg_actuator_control_target_decode(&message, &actuatorControlTarget)
}
void _handleResourceRequest(mavlink_message_t message)
{
  mavlink_resource_request_t resourceRequest;
  mavlink_msg_resource_request_decode(&message, &resourceRequest);
}
void _handleFollowTarget(mavlink_message_t message)
{
  mavlink_follow_target_t followTarget;
  mavlink_msg_follow_target_decode(&message, &followTarget);
}
void _handleControlSystemState(mavlink_message_t message)
{
  mavlink_control_system_state_t controlSystemState;
  mavlink_msg_control_system_state_decode(&message, &controlSystemState);
}
void _handleLandingTarget(mavlink_message_t message)
{
  mavlink_landing_target_t landingTarget;
  mavlink_msg_landing_target_decode(&message, &landingTarget);
}
void _handleGPSInput(mavlink_message_t message)
{
  mavlink_gps_input_t GPSInput;
  mavlink_msg_gps_input_decode(&message, &GPSInput);
}
void _handleGPSRtcmData(mavlink_message_t message)
{
  mavlink_gps_rtcm_data_t GPSRTCMData;
  mavlink_msg_gps_rtcm_data_decode(&message, &GPSRTCMData);
}
void _handleHighLatency(mavlink_message_t message)
{
  mavlink_high_latency_t highLatency;
  mavlink_msg_high_latency_decode(&message, &highLatency);
}
void _handleSetHomePosition(mavlink_message_t message)
{
  mavlink_set_home_position_t setHomePosition;
  mavlink_msg_set_home_position_decode(&message, &setHomePosition);
}
void _handleMessageInterval(mavlink_message_t message)
{
  mavlink_message_interval_t messageInterval;
  mavlink_msg_message_interval_decode(&message, &messageInterval);
}
void _handleCollision(mavlink_message_t message)
{
  mavlink_collision_t collision;
  mavlink_msg_collision_decode(&message, &collision)
}
void _handleV2Extension(mavlink_message_t message)
{
  mavlink_v2_extension_t V2Extension;
  mavlink_msg_v2_extension_decode(&message, &V2Extension);
}
void _handleMemoryVect(mavlink_message_t message)
{
  mavlink_memory_vect_t memoryVect;
  mavlink_msg_memory_vect_decode(&message, &memoryVect);
}
void _handleDebugVect(mavlink_message_t message)
{
  mavlink_debug_vect_t debugVect;
  mavlink_msg_debug_vect_decode(&message, &debugVect);
}
void _handleNamedValueFloat(mavlink_message_t message)
{
  mavlink_named_value_float_t namedValueFloat;
  mavlink_msg_named_value_float_decode(&message, &namedValueFloat);
}
void _handleNamedValueInt(mavlink_message_t message)
{
  mavlink_named_value_int_t namedValueInt;
  mavlink_msg_named_value_int_decode(&message, &namedValueInt);
}
void _handleDebug(mavlink_message_t message)
{
  mavlink_debug_t debug;
  mavlink_msg_debug_decode(&message, &debug);
}
void _handleSetupSigning(mavlink_message_t message)
{
  mavlink_setup_signing_t setupSigning;
  mavlink_msg_setup_signing_decode(&message, &setupSigning);
}
void _handleButtonChange(mavlink_message_t message)
{
  mavlink_button_change_t buttonChange;
  mavlink_msg_button_change_decode(&message, &buttonChange);
}
void _handlePlayTune(mavlink_message_t message)
{
  mavlink_play_tune_t playTune;
  mavlink_msg_play_tune_decode(&message, &playTune);
}
void _handleCameraInformation(mavlink_message_t message)
{
  mavlink_camera_information_t cameraInformation;
  mavlink_msg_camera_information_decode(&message, &cameraInformation);
}
void _handleCameraSettings(mavlink_message_t message)
{
  mavlink_camera_settings_t cameraSettings;
  mavlink_msg_camera_settings_decode(&message, &cameraSettings);
}
void _handleStorageInformation(mavlink_message_t message)
{
  mavlink_storage_information_t storageInformation;
  mavlink_msg_storage_information_decode(&message, &storageInformation);
}
void _handleCameraCaptureStatus(mavlink_message_t message)
{
  mavlink_camera_capture_status_t cameraCaptureStatus;
  mavlink_msg_camera_capture_status_decode(&message, &cameraCaptureStatus);
}
void _handleFlightInformation(mavlink_message_t message)
{
  mavlink_flight_information_t flightInformation;
  mavlink_msg_flight_information_decode(&message, &flightInformation);
}
void _handleMountOrientation(mavlink_message_t message)
{
  mavlink_mount_orientation_t mountOrientation;
  mavlink_msg_mount_orientation_decode(&message_ &mountOrientation);
}
void _handleLoggingAck(mavlink_message_t message)
{
  mavlink_logging_ack_t loggingAck;
  mavlink_msg_logging_ack_decode(&message, &loggingAck);
}
void _handleVideoStreamInformation(mavlink_message_t message)
{
  mavlink_video_stream_information_t videoStreamInformation;
  mavlink_msg_video_stream_information_decode(&message, &videoStreamInformation);
}
void _handleSetVideoStreamInformation(mavlink_message_t message)
{
  mavlink_set_video_stream_information_t setVideoStreamInformation;
  mavlink_msg_set_video_stream_information_decode(setVideoStreamInformation);
}
void _handleWIFIConfigAp(mavlink_message_t message)
{
  mavlink_wifi_config_ap_t WIFIConfigAp;
  mavlink_msg_wifi_config_ap_decode(&message, &WIFIConfigAp);
}
void _handleProtocolVersion(mavlink_message_t message)
{
  mavlink_protocol_version_t protocolVersion;
  mavlink_msg_protocol_version_decode(&message, &protocolVersion);
}
void _handleUAVCANNNodeStatus(mavlink_message_t message)
{
  mavlink_uavcan_node_status_t UAVCANNodeStatus;
  mavlink_msg_uavcan_node_status_decode(&message, &UAVCANNodeStatus);
}
void _handleUAVCANNodeInfo(mavlink_message_t message)
{
  mavlink_uavcan_node_info_t UAVCANNodeInfo;
  mavlink_msg_uavcan_node_info_decode(&message, &UAVCANNodeInfo);
}
void _handleParamExtRequestRead(mavlink_message_t message)
{
  mavlink_param_ext_request_read_t paramExtRequestRead;
  mavlink_msg_param_ext_request_read_decode(&message, &paramExtRequestRead);
}
void _handleParamExtRequestList(mavlink_message_t message)
{
  mavlink_param_ext_request_list_t paramExtRequestList;
  mavlink_msg_param_ext_request_list_decode(&message, &paramExtRequestList);
}
void _handleParamExtValue(mavlink_message_t message)
{
  mavlink_param_ext_value_t paramExtValue;
  mavlink_msg_param_ext_value_decode(&message, &paramExtValue);
}
void _handleParamExtSet(mavlink_message_t message)
{
  mavlink_param_ext_set_t paramExtSet;
  mavlink_msg_param_ext_set_decode(&message, &paramExtSet);
}
void _handleParamExtAck(mavlink_message_t message)
{
  mavlink_param_ext_ack_t paramExtAck;
  mavlink_msg_param_ext_ack_decode(&message, &paramExtAck);
}
void _handleObstacleDistance(mavlink_message_t message)
{
  mavlink_obstacle_distance_t obstacleDistance;
  mavlink_msg_obstacle_distance_decode(&message, &obstacleDistance);
}
void _handleOdometry(mavlink_message_t message)
{
  mavlink_odometry_t odometry;
  mavlink_msg_odometry_decode(&message, &odometry);
}
void _handleTrajectoryRepresentationWaypoints(mavlink_message_t message)
{
  mavlink_trajectory_representation_waypoints_t trajectoryRepresentationWaypoints;
  mavlink_msg_trajectory_representation_waypoints_decode(&message, &trajectoryRepresentationWaypoints);
}
void _handleTrajectoryRepresenationBezier(mavlink_message_t message)
{
  mavlink_trajectory_representation_bezier_t trajectoryRepresentationBezier;
  mavlink_msg_trajectory_representation_bezier_decode(&message, &trajectoryRepresentationBezier);
}
void _handleUTMGlobalPosition(mavlink_message_t message)
{
  mavlink_utm_global_position_t UTMGlobalPosition;
  mavlink_msg_utm_global_position_decode(&message, &UTMGlobalPosition);
}
void _handleDebugFloatArray(mavlink_message_t message)
{
  mavlink_debug_float_array_t debugFloatArray;
  mavlink_msg_debug_float_array_decode(&message, &debugFloatArray);
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
            case MAVLINK_MSG_ID_SYSTEM_TIME:
            //_handleSystemTime(message);
            break;
            case MAVLINK_MSG_ID_PING:
            //_handlePing(message);
            break;
            case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
            //_handleChangeOperatorControl(message);
            break;
            case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:
            //_handleChangeOperatorControlAck(message);
            break;
            case MAVLINK_MSG_ID_AUTH_KEY:
            //_handleAuthKey(message);
            break;
            case MAVLINK_MSG_ID_SET_MODE:
            //_handleSetMode(message);
            break;
            //case MAVLINK_MSG_ID_PARAM_REQUESTED:
            //_handleParamRequested(message);
            //break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            //_handleParamRequestList(message);
            break;
            case MAVLINK_MSG_ID_GPS_STATUS:
            //_handleGPSStatus(message);
            break;
            case MAVLINK_MSG_ID_RAW_PRESSURE:
            //_handleRawPressure(message);
            break;
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            //_handleLocalPositionNed(message);
            break;
            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            //_handleServoOutputRaw(message);
            break;
            case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
            //_handleMissionRequestPartialList(message);
            break;
            //case MAVLINK_MSG_ID_WRITE_PARTIAL_LIST:
            //_handleWritePartialList(message);
            //break;
            case MAVLINK_MSG_ID_MISSION_ITEM:
            //_handleMissionItem(message);
            break;
            case MAVLINK_MSG_ID_MISSION_REQUEST:
            //_handleMissionRequest(message);
            break;
            //case MAVLINK_MSG_ID_SET_CURRENT:
            //_handleSetCurrent(message);
            //break;
            case MAVLINK_MSG_ID_MISSION_CURRENT:
            //_handleMissionCurrent(message);
            break;
            //case MAVLINK_MSG_ID_REQUEST_LIST:
            //_handleRequestList(message);
            //break;
            case MAVLINK_MSG_ID_MISSION_COUNT:
            //_handleMissionCount(message);
            break;
            //case MAVLINK_MSG_ID_CLEAR_ALL:
            //_handleClearAll(message);
            //break;
            //case MAVLINK_MSG_ID_ITEM_REACHED:
            //_handleItemReached(message);
            //break;
            case MAVLINK_MSG_ID_MISSION_ACK:
            //_handleMissionAcked(message);
            break;
            case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
            //_handleSetGPSGlobalOrigin(message);
            break;
            case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
            //_handleGPSGlobalOrigin(message);
            break;
            case MAVLINK_MSG_ID_PARAM_MAP_RC:
            //_handleParamMapRC(message);
            break;
            case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
            //_handleMissionRequestInt(message);
            break;
            case MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA:
            //_handleSafetySetAllowedArea(message);
            break;
            case MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA:
            //_handleSafetyAllowedArea(message);
            break;
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV:
            //_handleAttitudeQuaternionCov(message);
            break;
            case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
            //_handleNavControllerOutput(message);
            break;
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV:
            //_handleGlobalPositionIntCov(message);
            break;
            //case MAVLINK_MSG_ID_GLOBAL_POSITION_NED_COV:
            //_handleGlobalPositionNedCov(message);
            //break;
            case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
            //_handleRequestDataStream(message);
            break;
            case MAVLINK_MSG_ID_DATA_STREAM:
            //_handleDataStream(message);
            break;
            case MAVLINK_MSG_ID_MANUAL_CONTROL:
            //_handleManualControl(message);
            break;
            case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            //_handleRCChannelsOverride(message);
            break;
            case MAVLINK_MSG_ID_MISSION_ITEM_INT:
            //_handleMissionItemInt(message);
            break;
            case MAVLINK_MSG_ID_COMMAND_INT:
            //_handleCommandInt(message);
            break;
            case MAVLINK_MSG_ID_MANUAL_SETPOINT:
            //_handleManualSetpoint(message);
            break;
            case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
            //_handleSetAttitudeTarget(message);
            break;
            case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
            //_handleSetPositionTargetLocalNed(message);
            break;
            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
            //_handlePositionTargetLocalNed(message);
            break;
            case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
            //_handleSetPositionTargetGlobalInt(message);
            break;
            case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
            //_handlePositionTargetGlobalInt(message);
            break;
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET:
            //_handleLocalPositionNedSystemGlobalOffset(message);
            break;
            case MAVLINK_MSG_ID_HIL_STATE:
            //_handleHilState(message);
            break;
            case MAVLINK_MSG_ID_HIL_CONTROLS:
            //_handleHilControls(message);
            break;
            //case MAVLINK_MSG_ID_RC_INPUTS_RAW:
            //_handleRCInputsRaw(message);
            //break;
            case MAVLINK_MSG_ID_OPTICAL_FLOW:
            //_handleOpticalFlow(message);
            break;
            case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
            //_handleGlobalVisionPositionEstimate(message);
            break;
            case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
            //_handleVisionPositionEstimate(message);
            break;
            case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
            //_handleVisionSpeedEstimate(message);
            break;
            case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
            //_handleViconPositionEstimate(message);
            break;
            case MAVLINK_MSG_ID_HIGHRES_IMU:
            //_handleHighresIMU(message);
            break;
            case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
            //_handleOpticalFlowRad(message);
            break;
            case MAVLINK_MSG_ID_HIL_SENSOR:
            //_handleHilSensor(message);
            break;
            case MAVLINK_MSG_ID_SIM_STATE:
            //_handleSim_State(message);
            break;
            case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
            //_handleFileTransferProtocol(messages);
            break;
            //case MAVLINK_MSG_ID_TIMESYNC_DECODE:
            //_handletimeSync(message);
            //break;
            case MAVLINK_MSG_ID_CAMERA_TRIGGER:
            //_handleCameraTrigger(message);
            break;
            case MAVLINK_MSG_ID_HIL_GPS:
            //_handleHilGPS(message);
            break;
            case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
            //_handleHilOpticalFlow(messages);
            break;
            case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
            //_handleHilStateQuaternion(messages);
            break;
            case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
            //_handleLogRequestList(message);
            break;
            case MAVLINK_MSG_ID_LOG_ENTRY:
            //_handleLogEntry(message);
            break;
            case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
            //_handleLogRequestData(message);
            break;
            case MAVLINK_MSG_ID_LOG_DATA:
            //_handleLogData(messages);
            break;
            case MAVLINK_MSG_ID_LOG_ERASE:
            //_handleLogErase(message);
            break;
            case MAVLINK_MSG_ID_LOG_REQUEST_END:
            //_handleLogRequestEnd(message);
            break;
            case MAVLINK_MSG_ID_GPS_INJECT_DATA:
            //_handleGPSInjectData(message);
            break;
            case MAVLINK_MSG_ID_GPS2_RAW:
            //_handleGPS2Raw(message);
            break;
            case MAVLINK_MSG_ID_POWER_STATUS:
            //_handlePowerStatus(message);
            break;
            case MAVLINK_MSG_ID_SERIAL_CONTROL:
            //_handleSerialControl(message);
            break;
            case MAVLINK_MSG_ID_GPS_RTK:
            //_handleGPSRtk(message);
            break;
            case MAVLINK_MSG_ID_GPS2_RTK:
            //_handleGPS2Rtk(message);
            break;
            case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
            //_handleDataTransmissionHandshake(message);
            break;
            case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
            //_handleEncapsulatedData(message);
            break;
            case MAVLINK_MSG_ID_TERRAIN_REQUEST:
            //_handleTerrainRequest(message);
            break;
            case MAVLINK_MSG_ID_TERRAIN_DATA:
            //_handleTerrainData(message);
            break;
            case MAVLINK_MSG_ID_TERRAIN_CHECK:
            //_handleTerrainCheck(message);
            break;
            case MAVLINK_MSG_ID_TERRAIN_REPORT:
            //_handleTerrainReport(message);
            break;
            case MAVLINK_MSG_ID_ATT_POS_MOCAP:
            //_handleAttPosMocap(message);
            break;
            case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
            //_handleSetActuatorControlTarget(message);
            break;
            case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:
            //_handleActuatorControlTarget(message);
            break;
            case MAVLINK_MSG_ID_RESOURCE_REQUEST:
            //_handleResourceRequest(messages);
            break;
            case MAVLINK_MSG_ID_FOLLOW_TARGET:
            //_handleFollowTarget(message);
            break;
            case MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE:
            //_handleControlSystemState(message);
            break;
            case MAVLINK_MSG_ID_LANDING_TARGET:
            //_handleLandingTarget(message);
            break;
            case MAVLINK_MSG_ID_GPS_INPUT:
            //_handleGPSInput(message);
            break;
            case MAVLINK_MSG_ID_GPS_RTCM_DATA:
            //_handleGPSRtcmData(message);
            break;
            case MAVLINK_MSG_ID_HIGH_LATENCY:
            //_handleHighLatency(messages);
            break;
            case MAVLINK_MSG_ID_SET_HOME_POSITION:
            //_handleSetHomePosition(message);
            break;
            case MAVLINK_MSG_ID_MESSAGE_INTERVAL:
            //_handleMessageInterval(message);
            break;
            case MAVLINK_MSG_ID_COLLISION:
            //_handleCollision(message);
            break;
            case MAVLINK_MSG_ID_V2_EXTENSION:
            //_handleV2Extension(message);
            break;
            case MAVLINK_MSG_ID_MEMORY_VECT:
            //_handleMemoryVect(message);
            break;
            case MAVLINK_MSG_ID_DEBUG_VECT:
            //_handleDebugVect(message);
            break;
            case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
            //_handleNamedValueFloat(message);
            break;
            case MAVLINK_MSG_ID_NAMED_VALUE_INT:
            //_handleNamedValueInt(message);
            break;
            case MAVLINK_MSG_ID_DEBUG:
            //_handleDebug(message);
            break;
            case MAVLINK_MSG_ID_SETUP_SIGNING:
            //_handleSetupSigning(message);
            break;
            case MAVLINK_MSG_ID_BUTTON_CHANGE:
            //_handleButtonChange(message);
            break;
            case MAVLINK_MSG_ID_PLAY_TUNE:
            //_handlePlayTune(message);
            break;
            case MAVLINK_MSG_ID_CAMERA_INFORMATION:
            //_handleCameraInformation(message);
            break;
            case MAVLINK_MSG_ID_CAMERA_SETTINGS:
            //_handleCameraSettings(message);
            break;
            case MAVLINK_MSG_ID_STORAGE_INFORMATION:
            //_handleStorageInformation(message);
            break;
            case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
            //_handleCameraCaptureStatus(message);
            break;
            case MAVLINK_MSG_ID_FLIGHT_INFORMATION:
            //_handleFlightInformation(message);
            break;
            case MAVLINK_MSG_ID_MOUNT_ORIENTATION:
            //_handleMountOrientation(message);
            break;
            case MAVLINK_MSG_ID_LOGGING_ACK:
            //_handleLoggingAck(message);
            break;
            case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION:
            //_handleVideoStreamInformation(message);
            break;
            //case MAVLINK_MSG_ID_SET_VIDEO_STREAM_INFORMATION:
            //_handleSetVideoStreamInformation(message);
            //break;
            case MAVLINK_MSG_ID_WIFI_CONFIG_AP:
            //_handleWIFIConfigAP(message);
            break;
            case MAVLINK_MSG_ID_PROTOCOL_VERSION:
            //_handleProtocolVersion(message);
            break;
            case MAVLINK_MSG_ID_UAVCAN_NODE_STATUS:
            //_handleUAVCANNodeStatus(message);
            break;
            case MAVLINK_MSG_ID_UAVCAN_NODE_INFO:
            //_handleUAVCANNodeInfo(message);
            break;
            case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
            //_handleParamExtRequestRead(message);
            break;
            case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
            //_handleParamExtRequestList(message);
            break;
            case MAVLINK_MSG_ID_PARAM_EXT_VALUE:
            //_handleParamExtValue(message);
            break;
            case MAVLINK_MSG_ID_PARAM_EXT_SET:
            //_handleParamExtSet(message);
            break;
            case MAVLINK_MSG_ID_PARAM_EXT_ACK:
            //_handleParamExtAck(message);
            break;
            case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
            //_handleObstacleDistance(message);
            break;
            case MAVLINK_MSG_ID_ODOMETRY:
            //_handleOdometry(message);
            break;
            case MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS:
            //_handleTrajectoryRepresentationWaypoints(message);
            break;
            case MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER:
            //_handleTrajectoryRepresentationBezier(message);
            break;
            case MAVLINK_MSG_ID_UTM_GLOBAL_POSITION:
            //_handleUTMGlobalPosition(message);
            break;
            case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
            //_handleDebugFloatArray(message);
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
