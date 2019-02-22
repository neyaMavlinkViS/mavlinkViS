cmake_minimum_required(VERSION 3.5)
project(diux)

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
find_package(diux_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)


SET(CMAKE_CXX_FLAGS "-fpermissive -pthread")

# Start of PrimitiveDriver

add_executable(PrimitiveDriver
  src/PrimitiveDriver.cpp
)

ament_target_dependencies(PrimitiveDriver
  "rclcpp"
  "diux_msgs"
)

install(TARGETS PrimitiveDriver
    DESTINATION bin)

# End of PrimitiveDriver

# Start of ViS

add_executable(ViS
  src/ViS.cpp
)

ament_target_dependencies(ViS
  "rclcpp"
  "diux_msgs"
)

install(TARGETS ViS
    DESTINATION bin)

# End of ViS


add_executable(subscriberTest
  src/subscriberTest.cpp
)

ament_target_dependencies(subscriberTest
  "rclcpp"
  "diux_msgs"
)

install(TARGETS subscriberTest
    DESTINATION bin)

#manualControl

add_executable(manualControlPublish
  src/manualControlPublish.cpp
)

ament_target_dependencies(manualControlPublish
  "rclcpp"
  "diux_msgs"
)

install(TARGETS manualControlPublish
    DESTINATION bin)
#end




# Start of GlobalPoseSensor
#include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/src ) 
#set(HEADER_FILES  pubishAndSubscribe.h
#)



add_executable(publishAndSubscribe
  src/publishAndSubscribe.cpp
)



ament_target_dependencies(publishAndSubscribe
  "rclcpp"
  "diux_msgs"
)

install(TARGETS publishAndSubscribe
    DESTINATION bin)

# End of GlobalPoseSensor


ament_package()
