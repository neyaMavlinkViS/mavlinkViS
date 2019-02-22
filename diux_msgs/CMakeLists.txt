cmake_minimum_required(VERSION 3.5)
project(diux_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions don't comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AltitudeAGLTuple.msg"
  "msg/AltitudeHAETuple.msg"
  "msg/AltitudeMSLTuple.msg"
  "msg/CommandStatusEnumTypeLDM.msg"
  "msg/CommandStatusTypeAll.msg"
  "msg/GeodeticLatitudeTuple.msg"
  "msg/GeodeticLongitudeTuple.msg"
  "msg/GeomagneticPropertyCommandReportType.msg"
  "msg/GeomagneticPropertyCommandRequestType.msg"
  "msg/GeomagneticPropertyCommandStatusType.msg"
  "msg/GeomagneticPropertyCommandType.msg"
  "msg/GeomagneticPropertyRequestType.msg"
  "msg/GeomagneticPropertyStatusType.msg"
  "msg/GlobalPoseCommandReportType.msg"
  "msg/GlobalPoseCommandRequestType.msg"
  "msg/GlobalPoseCommandStatusType.msg"
  "msg/GlobalPoseCommandType.msg"
  "msg/GlobalPoseRequestType.msg"
  "msg/GlobalPoseStatusType.msg"
  "msg/LinearEffortMeasurement.msg"
  "msg/NavigationSolutionEnumTypeLDM.msg"
  "msg/Orientation3DMeasurement.msg"
  "msg/Orientation3DPlatformXYZMeasurement.msg"
  "msg/Position2DTuple.msg"
  "msg/Position3DWGS84Tuple.msg"
  "msg/RMSDistanceErrorType.msg"
  "msg/RMSOrientationErrorType.msg"
  "msg/RMSPositionErrorType.msg"
  "msg/RotationalEffortMeasurement.msg"
  "msg/WrenchEffortCommandReportType.msg"
  "msg/WrenchEffortCommandRequestType.msg"
  "msg/WrenchEffortCommandStatusType.msg"
  "msg/WrenchEffortCommandType.msg"
  "msg/WrenchEffortRequestType.msg"
  "msg/WrenchEffortStatusType.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
