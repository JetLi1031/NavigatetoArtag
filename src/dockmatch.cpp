#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <robotnavigate/robotnavigater.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

using namespace std;
using std::placeholders::_1;


// -------------------Pose container ---------------------------------------//
struct subPose{
subPose(string name):subject{name}{cout << "initialize " <<subject <<endl;}
double x,y,z,yaw,pitch,roll; string subject;
bool checked {false};
void fitxyz(const geometry_msgs::msg::Vector3 &pose){
    x=pose.x;
    y=pose.y;
    z=pose.z;
   // cout << "[Posesub]:("<< subject << ") fit x: " <<x <<",y: " << y <<",z: " << z<<endl;
}
void fitYPR(const geometry_msgs::msg::Quaternion &q){
   //cout << "qx " << q.x << " ,q.y " << q.y << " ,q.z "<< q.z<<" ,q.w " << q.w <<endl;
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
   // cout << "[Posesub]:("<< subject << ") fit yaw: " << yaw << " ,pitch: " << y << " ,row: "<< z<<endl;
}
};

// -------------------Battery subscription ---------------------------------------//
//seperate classs into ddifferent thread to run parallel
class BatterySubscriber : public rclcpp::Node {

public:

BatterySubscriber():Node("batterysub"){
batterysub_ = this->create_subscription<sensor_msgs::msg::BatteryState>("mybattery",10,std::bind(&BatterySubscriber::batsubcb,this,_1));
}
static uint8_t power_status;
private:
void batsubcb(sensor_msgs::msg::BatteryState::SharedPtr msg){

 RCLCPP_INFO(this->get_logger(), "%d seconds , at percentage: %f",msg->header.stamp.sec ,msg->percentage);
 power_status = msg->power_supply_status;
}

rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batterysub_;
};
// initinilize static power status
uint8_t BatterySubscriber::power_status = 2; //later publish fake battery

//------------------------------ Pose subcription for current and goal ---------------//
class Posesubcript : public rclcpp::Node {
public:
Posesubcript(std::shared_ptr<subPose> curpos,std::shared_ptr<subPose> golpos):Node("posesub"),curposse{curpos},goalposse{golpos}{
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      

        
    timer_ = this->create_wall_timer(
      0.25s, std::bind(&Posesubcript::on_timer, this));
  
  }
    
private:
 void on_timer()
  {
  string fromFrameRel = "base_link";
  string toFrameRel = "map";
  geometry_msgs::msg::TransformStamped transformStamped;
  try{

    transformStamped = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
  } catch(tf2::TransformException & ex){
    RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          goto tflookup;
  }
    curposse->fitxyz(transformStamped.transform.translation);
    curposse->fitYPR(transformStamped.transform.rotation);
    
   tflookup: //jump if one not found
    fromFrameRel = "Ar_tagframe";  toFrameRel = "map";
    try{

    transformStamped = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
 
  } catch(tf2::TransformException & ex){
    RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            goalposse->checked = false;
          return;
  }
    goalposse->checked = true;
    goalposse->fitxyz(transformStamped.transform.translation);
    goalposse->fitYPR(transformStamped.transform.rotation);  
  }
  // artag to map

  // first tf listen to current pose
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;   

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<subPose>curposse{nullptr};
  // second tf listern to goal pose

  std::shared_ptr<subPose> goalposse{nullptr};
};

// -------------------docking operation----------------------------------------------------//
class dockmat : public rclcpp::Node
{
public:
    using Posenav = nav2_msgs::action::NavigateToPose;
	dockmat(std::shared_ptr<subPose> mypose,std::shared_ptr<subPose> goalpose):Node("Posenavi"),currentx{mypose->x},currenty{mypose->y},currentyaw{mypose->yaw},goalx{goalpose->x},goaly{goalpose->y},goalyaw{goalpose->yaw},goalcheck{goalpose->checked}{
	
	cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
	timerone_ = this->create_wall_timer(
      0.5s, std::bind(&dockmat::start_docking, this));
	
	}
private:
float getRadiantoGoal(){  //yaw different
    //return yawlist.at(currernt_goal_idx) - currentyaw; for mutiple parallel point align
    float diffinradian = 1.5708 - currentyaw;
    RCLCPP_INFO(this->get_logger(),"Different in Radian: %f ",diffinradian );
    return diffinradian;
}

float getdisToGoal(){
    return sqrt(pow((goalx- currentx),2)+pow((goaly- currenty),2));
} 

float get_heading_error(){
  float delta_x = goalx - currentx; // current x are to update by subsribtion node
 // RCLCPP_INFO(this->get_logger(),"condition one angular goal x: %f current x: %f",goalx,currentx);
  float delta_y = goaly - currenty ;
 // RCLCPP_INFO(this->get_logger(),"condition one angular goal y : %f  current y: %f",goaly,currenty);
  float desired_heading = atan2(delta_y,delta_x);
  float heading_err = desired_heading - currentyaw;
 //  RCLCPP_INFO(this->get_logger(),"condition one desired head y : %f  current yaw: %f",desired_heading,currentyaw);
  if(heading_err > M_PI)
      heading_err -= 2*M_PI;
  if(heading_err < -M_PI)
     heading_err += 2*M_PI;
   return heading_err;
}

// to alignemnt with (in priority) distancetogoal -> headingerr -> lastly, yawgoal err
void alignment (){
    //RCLCPP_INFO(this->get_logger(),"Aligning !!!!!!!!!!!!!!!!!!!!!!!!!");
    float distance_to_goal = getdisToGoal();
    RCLCPP_INFO(this->get_logger(),"Distance to Goal : %f",distance_to_goal);
    float heading_err    = get_heading_error();
    float yaw_goal_err =getRadiantoGoal();
    geometry_msgs::msg::Twist twist;
    //twsit.linear.x = 
    if (fabs(distance_to_goal) > distance_goal_tolerance && reach_disgoal == false){
        //RCLCPP_INFO(this->get_logger(),"condition one");
		if( fabs(heading_err) >head_tol ){
		RCLCPP_INFO(this->get_logger(),"condition one angular %f",heading_err);
			twist.angular.z = heading_err>0?adj_ang_vel:-adj_ang_vel;
		}else{
		RCLCPP_INFO(this->get_logger(),"condition one linear %f",heading_err);
			twist.linear.x = adj_lin_vel;
		}	
    } else if(fabs(yaw_goal_err)>yawgoaltol )  {
        RCLCPP_INFO(this->get_logger(),"condition two %f",yaw_goal_err);
		twist.angular.z= yaw_goal_err>0?adj_ang_vel:-adj_ang_vel;
		reach_disgoal =true;
		
    } else{
        RCLCPP_INFO(this->get_logger(),"condition three");
		currernt_goal_idx = 2;
		RCLCPP_INFO(this->get_logger(),"Aligned at perpendicular line");
		twist.linear.x =0;
	    twist.angular.z=0;	
	}
    cmd_publisher_->publish(twist);
}

void start_docking(){
    this->timerone_->cancel();
    auto action_client = std::make_shared<Posenavcli>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }
  RCLCPP_INFO(this->get_logger(),"Finished navigating nearby docking zone");
  

  while ( BatterySubscriber::power_status == 2) {
    // rotate to center if AR tag was found 
    if( currernt_goal_idx == 0 ){
        if( !goalcheck){
         geometry_msgs::msg::Twist twist;    
         twist.angular.z= 0.25;
         cmd_publisher_->publish(twist);
        } else { 
        RCLCPP_INFO(this->get_logger(),"AR Tag found proceed to alignement");
        currernt_goal_idx = 1;}
    } else if ( currernt_goal_idx == 1){
        alignment();
    } else {
      BatterySubscriber::power_status = 1; //should be calling the charged service  
      }
   }
}


//for alignment purpose
double &currentx,&currenty,&currentyaw,&goalx,&goaly,&goalyaw;
bool &goalcheck;
int currernt_goal_idx {0};
float adj_lin_vel {0.12};
float adj_ang_vel { 0.07};
float distance_goal_tolerance {0.7};
bool reach_disgoal {false};
float head_tol {0.05};
float yawgoaltol {0.005};
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
rclcpp::TimerBase::SharedPtr timerone_{nullptr};
};



int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto poseptr = make_shared<subPose>("currentpose");  
    auto goalptr = make_shared<subPose>("goal pose"); 
    auto batsubnode = std::make_shared<BatterySubscriber>(); 
    auto posesubnode = std::make_shared<Posesubcript>(poseptr,goalptr);   
    auto docking = std::make_shared<dockmat>(poseptr,goalptr);
    executor.add_node(docking);
    executor.add_node(batsubnode);
    executor.add_node(posesubnode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
