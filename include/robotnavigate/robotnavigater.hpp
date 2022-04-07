#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <string>
#include <future>
#include <memory>
#include <chrono>
#include <iostream>

using namespace std;


class Posenavcli : public rclcpp::Node
{
 public:
	using Posenav = nav2_msgs::action::NavigateToPose;
	using GoalHandlePose = rclcpp_action::ClientGoalHandle<Posenav>;
    using Quaternion = tf2::Quaternion;   
	explicit Posenavcli(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
	bool is_goal_done() const;
	void send_goal();
 private:
    rclcpp_action::Client<Posenav>::SharedPtr client_ptr_;
	rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;
    void goal_response_callback(std::shared_future<GoalHandlePose::SharedPtr> future);
    void feedback_callback(
	    GoalHandlePose::SharedPtr,
	    const std::shared_ptr<const Posenav::Feedback> feedback);
	void result_callback(const GoalHandlePose::WrappedResult & result);
	   
};


