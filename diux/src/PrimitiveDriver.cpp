#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "diux_msgs/msg/wrench_effort_request_type.hpp"
#include "diux_msgs/msg/wrench_effort_command_request_type.hpp"
#include "diux_msgs/msg/wrench_effort_command_type.hpp"

#include "diux_msgs/msg/wrench_effort_status_type.hpp"
#include "diux_msgs/msg/wrench_effort_command_report_type.hpp"
#include "diux_msgs/msg/wrench_effort_command_status_type.hpp"

using namespace std;
using namespace std::chrono_literals;


/*************************************************************
 *
 *      SERVICE REQUEST CLASSES
 *
 *************************************************************/



/*

Class: PrimitiveDriverQueryWrenchEffortPublisher

Service: PrimitiveDriver
ROS Node: primitive_driver_query_wrench_effort_publisher
ROS Topic: primitive_driver_query_wrench_effort

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverQueryWrenchEffortPublisher : public rclcpp::Node
{
public:
  PrimitiveDriverQueryWrenchEffortPublisher()
  : Node("primitive_driver_query_wrench_effort_publisher")
  {
    string topic = "primitive_driver_query_wrench_effort";
    wrench_effort_request_type_publisher_ =
      this->create_publisher<diux_msgs::msg::WrenchEffortRequestType>(topic);

    auto publish_msg = [this]() -> void {
        auto msg = std::make_shared<diux_msgs::msg::WrenchEffortRequestType>();

        // TODO: Insert code to set the values in 'msg'

        std::cout << "Publishing WrenchEffortRequestType:" << std::endl;

        wrench_effort_request_type_publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<diux_msgs::msg::WrenchEffortRequestType>::SharedPtr wrench_effort_request_type_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


/*

Class: PrimitiveDriverQueryWrenchEffortSubscriber

Service: PrimitiveDriver
ROS Node: primitive_driver_query_wrench_effort_subscriber
ROS Topic: primitive_driver_query_wrench_effort

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverQueryWrenchEffortSubscriber : public rclcpp::Node
{
public:
  PrimitiveDriverQueryWrenchEffortSubscriber()
  : Node("primitive_driver_query_wrench_effort_subscriber")
  {
    wrench_effort_request_type_subscriber_ = this->create_subscription<diux_msgs::msg::WrenchEffortRequestType>(
      "primitive_driver_query_wrench_effort",
      [this](diux_msgs::msg::WrenchEffortRequestType::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received WrenchEffortRequestType on topic: primitive_driver_query_wrench_effort.");
        // TODO: Insert code to read the values in 'msg'
    });
  }

private:
  rclcpp::Subscription<diux_msgs::msg::WrenchEffortRequestType>::SharedPtr wrench_effort_request_type_subscriber_;
};



/*

Class: PrimitiveDriverQueryWrenchEffortCmdPublisher

Service: PrimitiveDriver
ROS Node: primitive_driver_query_wrench_effort_cmd_publisher
ROS Topic: primitive_driver_query_wrench_effort_cmd

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverQueryWrenchEffortCmdPublisher : public rclcpp::Node
{
public:
  PrimitiveDriverQueryWrenchEffortCmdPublisher()
  : Node("primitive_driver_query_wrench_effort_cmd_publisher")
  {
    string topic = "primitive_driver_query_wrench_effort_cmd";
    wrench_effort_command_request_type_publisher_ =
      this->create_publisher<diux_msgs::msg::WrenchEffortCommandRequestType>(topic);

    auto publish_msg = [this]() -> void {
        auto msg = std::make_shared<diux_msgs::msg::WrenchEffortCommandRequestType>();

        // TODO: Insert code to set the values in 'msg'

        std::cout << "Publishing WrenchEffortCommandRequestType:" << std::endl;

        wrench_effort_command_request_type_publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<diux_msgs::msg::WrenchEffortCommandRequestType>::SharedPtr wrench_effort_command_request_type_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


/*

Class: PrimitiveDriverQueryWrenchEffortCmdSubscriber

Service: PrimitiveDriver
ROS Node: primitive_driver_query_wrench_effort_cmd_subscriber
ROS Topic: primitive_driver_query_wrench_effort_cmd

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverQueryWrenchEffortCmdSubscriber : public rclcpp::Node
{
public:
  PrimitiveDriverQueryWrenchEffortCmdSubscriber()
  : Node("primitive_driver_query_wrench_effort_cmd_subscriber")
  {
    wrench_effort_command_request_type_subscriber_ = this->create_subscription<diux_msgs::msg::WrenchEffortCommandRequestType>(
      "primitive_driver_query_wrench_effort_cmd",
      [this](diux_msgs::msg::WrenchEffortCommandRequestType::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received WrenchEffortCommandRequestType on topic: primitive_driver_query_wrench_effort_cmd.");
        // TODO: Insert code to read the values in 'msg'
    });
  }

private:
  rclcpp::Subscription<diux_msgs::msg::WrenchEffortCommandRequestType>::SharedPtr wrench_effort_command_request_type_subscriber_;
};



/*

Class: PrimitiveDriverSetWrenchEffortPublisher

Service: PrimitiveDriver
ROS Node: primitive_driver_set_wrench_effort_publisher
ROS Topic: primitive_driver_set_wrench_effort

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverSetWrenchEffortPublisher : public rclcpp::Node
{
public:
  PrimitiveDriverSetWrenchEffortPublisher()
  : Node("primitive_driver_set_wrench_effort_publisher")
  {
    string topic = "primitive_driver_set_wrench_effort";
    wrench_effort_command_type_publisher_ =
      this->create_publisher<diux_msgs::msg::WrenchEffortCommandType>(topic);

    auto publish_msg = [this]() -> void {
        auto msg = std::make_shared<diux_msgs::msg::WrenchEffortCommandType>();

        // TODO: Insert code to set the values in 'msg'

        std::cout << "Publishing WrenchEffortCommandType:" << std::endl;

        wrench_effort_command_type_publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<diux_msgs::msg::WrenchEffortCommandType>::SharedPtr wrench_effort_command_type_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


/*

Class: PrimitiveDriverSetWrenchEffortSubscriber

Service: PrimitiveDriver
ROS Node: primitive_driver_set_wrench_effort_subscriber
ROS Topic: primitive_driver_set_wrench_effort

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverSetWrenchEffortSubscriber : public rclcpp::Node
{
public:
  PrimitiveDriverSetWrenchEffortSubscriber()
  : Node("primitive_driver_set_wrench_effort_subscriber")
  {
    wrench_effort_command_type_subscriber_ = this->create_subscription<diux_msgs::msg::WrenchEffortCommandType>(
      "primitive_driver_set_wrench_effort",
      [this](diux_msgs::msg::WrenchEffortCommandType::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received WrenchEffortCommandType on topic: primitive_driver_set_wrench_effort.");
        // TODO: Insert code to read the values in 'msg'
    });
  }

private:
  rclcpp::Subscription<diux_msgs::msg::WrenchEffortCommandType>::SharedPtr wrench_effort_command_type_subscriber_;
};


/*************************************************************
 *
 *      RESPONSE CLASSES
 *
 *************************************************************/


/*

Class: PrimitiveDriverReportWrenchEffortPublisher

Service: PrimitiveDriver
ROS Node: primitive_driver_report_wrench_effort_publisher
ROS Topic: primitive_driver_report_wrench_effort

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverReportWrenchEffortPublisher : public rclcpp::Node
{
public:
  PrimitiveDriverReportWrenchEffortPublisher()
  : Node("primitive_driver_report_wrench_effort_publisher")
  {
    string topic = "primitive_driver_report_wrench_effort";
    wrench_effort_status_type_publisher_ =
      this->create_publisher<diux_msgs::msg::WrenchEffortStatusType>(topic);

    auto publish_msg = [this]() -> void {
        auto msg = std::make_shared<diux_msgs::msg::WrenchEffortStatusType>();

        // TODO: Insert code to set the values in 'msg'

        std::cout << "Publishing WrenchEffortStatusType:" << std::endl;

        wrench_effort_status_type_publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<diux_msgs::msg::WrenchEffortStatusType>::SharedPtr wrench_effort_status_type_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/*


Class: PrimitiveDriverReportWrenchEffortSubscriber

Service: PrimitiveDriver
ROS Node: primitive_driver_report_wrench_effort_subscriber
ROS Topic: primitive_driver_report_wrench_effort

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverReportWrenchEffortSubscriber : public rclcpp::Node
{
public:
  PrimitiveDriverReportWrenchEffortSubscriber()
  : Node("primitive_driver_report_wrench_effort_subscriber")
  {
    string topic = "primitive_driver_report_wrench_effort";
    wrench_effort_status_type_subscriber_ = this->create_subscription<diux_msgs::msg::WrenchEffortStatusType>(
      "primitive_driver_report_wrench_effort",
      [this](diux_msgs::msg::WrenchEffortStatusType::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received WrenchEffortStatusType on topic: primitive_driver_report_wrench_effort.");
        // TODO: Insert code to read the values in 'msg'
    });
  }

private:
  rclcpp::Subscription<diux_msgs::msg::WrenchEffortStatusType>::SharedPtr wrench_effort_status_type_subscriber_;
};


/*

Class: PrimitiveDriverReportWrenchEffortCmdPublisher

Service: PrimitiveDriver
ROS Node: primitive_driver_report_wrench_effort_cmd_publisher
ROS Topic: primitive_driver_report_wrench_effort_cmd

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverReportWrenchEffortCmdPublisher : public rclcpp::Node
{
public:
  PrimitiveDriverReportWrenchEffortCmdPublisher()
  : Node("primitive_driver_report_wrench_effort_cmd_publisher")
  {
    string topic = "primitive_driver_report_wrench_effort_cmd";
    wrench_effort_command_report_type_publisher_ =
      this->create_publisher<diux_msgs::msg::WrenchEffortCommandReportType>(topic);

    auto publish_msg = [this]() -> void {
        auto msg = std::make_shared<diux_msgs::msg::WrenchEffortCommandReportType>();

        // TODO: Insert code to set the values in 'msg'

        std::cout << "Publishing WrenchEffortCommandReportType:" << std::endl;

        wrench_effort_command_report_type_publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<diux_msgs::msg::WrenchEffortCommandReportType>::SharedPtr wrench_effort_command_report_type_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/*


Class: PrimitiveDriverReportWrenchEffortCmdSubscriber

Service: PrimitiveDriver
ROS Node: primitive_driver_report_wrench_effort_cmd_subscriber
ROS Topic: primitive_driver_report_wrench_effort_cmd

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverReportWrenchEffortCmdSubscriber : public rclcpp::Node
{
public:
  PrimitiveDriverReportWrenchEffortCmdSubscriber()
  : Node("primitive_driver_report_wrench_effort_cmd_subscriber")
  {
    string topic = "primitive_driver_report_wrench_effort_cmd";
    wrench_effort_command_report_type_subscriber_ = this->create_subscription<diux_msgs::msg::WrenchEffortCommandReportType>(
      "primitive_driver_report_wrench_effort_cmd",
      [this](diux_msgs::msg::WrenchEffortCommandReportType::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received WrenchEffortCommandReportType on topic: primitive_driver_report_wrench_effort_cmd.");
        // TODO: Insert code to read the values in 'msg'
    });
  }

private:
  rclcpp::Subscription<diux_msgs::msg::WrenchEffortCommandReportType>::SharedPtr wrench_effort_command_report_type_subscriber_;
};


/*

Class: PrimitiveDriverReportWrenchEffortCmdStatusPublisher

Service: PrimitiveDriver
ROS Node: primitive_driver_report_wrench_effort_cmd_status_publisher
ROS Topic: primitive_driver_report_wrench_effort_cmd_status

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverReportWrenchEffortCmdStatusPublisher : public rclcpp::Node
{
public:
  PrimitiveDriverReportWrenchEffortCmdStatusPublisher()
  : Node("primitive_driver_report_wrench_effort_cmd_status_publisher")
  {
    string topic = "primitive_driver_report_wrench_effort_cmd_status";
    wrench_effort_command_status_type_publisher_ =
      this->create_publisher<diux_msgs::msg::WrenchEffortCommandStatusType>(topic);

    auto publish_msg = [this]() -> void {
        auto msg = std::make_shared<diux_msgs::msg::WrenchEffortCommandStatusType>();

        // TODO: Insert code to set the values in 'msg'

        std::cout << "Publishing WrenchEffortCommandStatusType:" << std::endl;

        wrench_effort_command_status_type_publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<diux_msgs::msg::WrenchEffortCommandStatusType>::SharedPtr wrench_effort_command_status_type_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/*


Class: PrimitiveDriverReportWrenchEffortCmdStatusSubscriber

Service: PrimitiveDriver
ROS Node: primitive_driver_report_wrench_effort_cmd_status_subscriber
ROS Topic: primitive_driver_report_wrench_effort_cmd_status

Description: The purpose of this service is to provide basic platform mobility in six degrees of freedom using a percent of available effort in each direction.  

Notes:  
1.  It is not required to implement the operations and/or the entire set of interface messages.
2.  Interfaces are defined for both requests and responses. In a data centric environment, implementing the request interfaces such as RequestType interfaces or implementing the response interfaces such as RequestType interfaces or implementing the response interfaces such as CommandReportType or CommandStatusType interfaces are not required.

*/

class PrimitiveDriverReportWrenchEffortCmdStatusSubscriber : public rclcpp::Node
{
public:
  PrimitiveDriverReportWrenchEffortCmdStatusSubscriber()
  : Node("primitive_driver_report_wrench_effort_cmd_status_subscriber")
  {
    string topic = "primitive_driver_report_wrench_effort_cmd_status";
    wrench_effort_command_status_type_subscriber_ = this->create_subscription<diux_msgs::msg::WrenchEffortCommandStatusType>(
      "primitive_driver_report_wrench_effort_cmd_status",
      [this](diux_msgs::msg::WrenchEffortCommandStatusType::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received WrenchEffortCommandStatusType on topic: primitive_driver_report_wrench_effort_cmd_status.");
        // TODO: Insert code to read the values in 'msg'
    });
  }

private:
  rclcpp::Subscription<diux_msgs::msg::WrenchEffortCommandStatusType>::SharedPtr wrench_effort_command_status_type_subscriber_;
};


int main(int argc, char * argv[])
{

  if (argc < 2) {
    cout << "Please specificy whether to 'publish' or 'subscribe'." << endl;
    return -1;
  }

  cout << "Running the diux package PrimitiveDriver service." << endl;
  rclcpp::init(argc, argv);

  //For Publishing
  if (strstr(argv[1],"publish")) {
    auto publisher_node = std::make_shared<PrimitiveDriverQueryWrenchEffortPublisher>();
    //auto publisher_node = std::make_shared<PrimitiveDriverQueryWrenchEffortCmdPublisher>();
    //auto publisher_node = std::make_shared<PrimitiveDriverSetWrenchEffortPublisher>();
    //auto publisher_node = std::make_shared<PrimitiveDriverReportWrenchEffortPublisher>();
    //auto publisher_node = std::make_shared<PrimitiveDriverReportWrenchEffortCmdPublisher>();
    //auto publisher_node = std::make_shared<PrimitiveDriverReportWrenchEffortCmdStatusPublisher>();
    cout << "Publishing" << endl;
    rclcpp::spin(publisher_node);
  }

  //For Subscribing
  if (strstr(argv[1],"subscribe")) {
    auto subscriber_node = std::make_shared<PrimitiveDriverQueryWrenchEffortSubscriber>();
    //auto subscriber_node = std::make_shared<PrimitiveDriverQueryWrenchEffortCmdSubscriber>();
    //auto subscriber_node = std::make_shared<PrimitiveDriverSetWrenchEffortSubscriber>();
    //auto subscriber_node = std::make_shared<PrimitiveDriverReportWrenchEffortSubscriber>();
    //auto subscriber_node = std::make_shared<PrimitiveDriverReportWrenchEffortCmdSubscriber>();
    //auto subscriber_node = std::make_shared<PrimitiveDriverReportWrenchEffortCmdStatusSubscriber>();
    cout << "Subscribing" << endl;
    rclcpp::spin(subscriber_node);
  }

  return 0;
}
