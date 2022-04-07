#include<sensor_msgs/msg/battery_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include<iostream>
#include<chrono>
#include<example_interfaces/srv/set_bool.hpp>
#include<memory>
#include<string>

using namespace std;

class FakeBattery:public
  rclcpp::Node
{
public:
  FakeBattery ():Node ("fakebattery")
  {
    message.power_supply_status = 2;
    chg = 100;
    first = this->get_clock ()->now ();
    serviceptr = this->create_service<example_interfaces::srv::SetBool>("set_batterystate", batservice);
    created_publish = this->create_publisher<sensor_msgs::msg::BatteryState>("mybattery",
								 10);
    timer_ = this->create_wall_timer (std::chrono::milliseconds (500),
			       std::bind (&FakeBattery::topublish, this));
  }
  void
  topublish ()
  {
    message.header.stamp = this->get_clock ()->now ();
    auto
      timediff = (this->get_clock ()->now () - first);
    message.percentage =
      exp (-(timediff.nanoseconds () / pow (10, 11))) * chg;
    created_publish->publish (message);
  }
// service to reset fake battery charging condition
  std::function <void (std::shared_ptr <example_interfaces::srv::SetBool::Request>,
			std::shared_ptr <example_interfaces::srv::SetBool::Response>)>
    batservice =
    [this] (const std::shared_ptr <example_interfaces::srv::SetBool::Request> request,
	    std::shared_ptr <example_interfaces::srv::SetBool::Response>
	    response)
  {
    message.power_supply_status = (request->data == true) ? 1 : 2;
    chg = (request->data == true) ? 0 : 100;
    response->success = true;
    response->message = "current set to " + to_string (message.power_supply_status);
  };

private:
  rclcpp::Publisher <sensor_msgs::msg::BatteryState>::SharedPtr created_publish;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time first;
  rclcpp::Service <example_interfaces::srv::SetBool>::SharedPtr serviceptr;
  sensor_msgs::msg::BatteryState message;
  int
    chg;
};


int
main (int argc, char **argv)
{
  rclcpp::init (argc, argv);
  rclcpp::spin (std::make_shared < FakeBattery > ());
  rclcpp::shutdown ();
  return 0;

}
