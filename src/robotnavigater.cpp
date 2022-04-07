#include<robotnavigate/robotnavigater.hpp>

using namespace std;


 Posenavcli::Posenavcli(const rclcpp::NodeOptions &options)
		:Node("pose_navclient",options),goal_done_(false)
	{
		this->client_ptr_ = rclcpp_action::create_client<Posenav>(this,"navigate_to_pose");
		this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
				std::bind(&Posenavcli::send_goal,this));
	}
	
bool Posenavcli::is_goal_done() const
    {
       return this->goal_done_;
    }
void Posenavcli::send_goal()
	{
	using namespace std::placeholders;
	this->timer_->cancel();
    this->goal_done_ = false;
	if(!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))){
              RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
	      this->goal_done_ = true;
	      return;
	}
   	
	auto goal_msg = Posenav::Goal();
	//cout << this->get_clock()->now().nanoseconds() <<endl;
	goal_msg.pose.header.stamp = this->get_clock()->now();
	goal_msg.pose.header.frame_id = "map";
	Quaternion q;
	float z = 0/180.0*3.14159; //to radian
	q.setRPY(0,0,z);
	goal_msg.pose.pose.position.x = 0.454 ;
	goal_msg.pose.pose.position.y = 1.25 ;
	goal_msg.pose.pose.position.z = 0 ;
	goal_msg.pose.pose.orientation.x= q.x();
	goal_msg.pose.pose.orientation.y= q.y();
	goal_msg.pose.pose.orientation.z= q.z();
	goal_msg.pose.pose.orientation.w= q.w();

	
	RCLCPP_INFO(this->get_logger(),"Sending Goal");
	auto send_goal_options = rclcpp_action::Client<Posenav>::SendGoalOptions();
	send_goal_options.goal_response_callback =
      std::bind(&Posenavcli::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&Posenavcli::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&Posenavcli::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
	}
       



void Posenavcli::goal_response_callback(std::shared_future<GoalHandlePose::SharedPtr> future)
	  {
	    auto goal_handle = future.get();
	    if (!goal_handle) {
	      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
	    } else {
	      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
	    }
	  }

void Posenavcli::feedback_callback(
	    GoalHandlePose::SharedPtr,
	    const std::shared_ptr<const Posenav::Feedback> feedback)
	  {
	    std::stringstream ss;
	    ss << "current pose : \n ";
	    ss <<"x " <<  feedback->current_pose.pose.position.x << "\n";
	    ss<< "y " << feedback->current_pose.pose.position.y << "\n";
	    ss <<"time remaining" << feedback->estimated_time_remaining.sec << "\n";
	    ss << "distamce remaining" << feedback->distance_remaining;	    
	    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
	  }

void Posenavcli::result_callback(const GoalHandlePose::WrappedResult & result)
	  {
	    this->goal_done_ = true;
	    switch (result.code) {
	      case rclcpp_action::ResultCode::SUCCEEDED:
		break;
	      case rclcpp_action::ResultCode::ABORTED:
		RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
		return;
	      case rclcpp_action::ResultCode::CANCELED:
		RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
		return;
	      default:
		RCLCPP_ERROR(this->get_logger(), "Unknown result code");
		return;
	    }
	    
	    /*
	    std::stringstream ss;
	    ss << "Result received: ";
	    for (auto number : result.result->sequence) {
	      ss << number << " ";
	    } // empty result shown
	    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
	    */
	  }


/*
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<Posenavcli>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}
*/
