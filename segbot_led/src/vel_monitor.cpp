/*******************************************************
*                    ROS Headers                       *
********************************************************/
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <tf/tf.h>

/*******************************************************
*                   segbot_led Headers                 *
********************************************************/
#include "bwi_msgs/LEDActionResult.h"
#include "bwi_msgs/LEDAnimations.h"
#include "bwi_msgs/LEDClear.h"
#include "bwi_msgs/LEDControlAction.h"

/*******************************************************
*                   Service Headers                    *
********************************************************/
#include "bwi_msgs/QuestionDialog.h"
#include "bwi_services/SpeakMessage.h"

/*******************************************************
*                 Global Variables                     *
********************************************************/
ros::Publisher signal_pub;
ros::Subscriber global_path;  
ros::Subscriber robot_vel;
geometry_msgs::Twist vel_msg;
bool heard_vel = false;
bool heard_path = false;
bool heard_pose = false;
nav_msgs::Path current_path;
geometry_msgs::Pose current_pose;
ros::Subscriber robot_pose;  

/*******************************************************
*                 Callback Functions                   *
********************************************************/
// Updates the current velocity global instance variable
void vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
   vel_msg = *msg;
   heard_vel = true;
}

// Updates the current path
void path_cb(const nav_msgs::Path::ConstPtr& msg) {
  current_path = *msg; 
  heard_path = true; 
}

// Updates the current pose
void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    geometry_msgs::PoseWithCovarianceStamped new_pose = *msg; 
    current_pose = new_pose.pose.pose; 
    heard_pose = true; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_monitor");
    ros::NodeHandle n;

    ros::Rate loop_rate(30);

    // Sets up subscribers
    robot_vel = n.subscribe("/cmd_vel", 1, vel_cb);



    // Sets up subscriber for turning
    global_path = n.subscribe("/move_base/EBandPlannerROS/global_plan", 1, path_cb);
    robot_pose = n.subscribe("/amcl_pose", 1, pose_cb);

    // Sets up action client
    actionlib::SimpleActionClient<bwi_msgs::LEDControlAction> ac("led_control_server", true);
    // Wait for initialization of the action client
    ac.waitForServer();
    // Initialization of the goal to send with the newly created action client
    bwi_msgs::LEDControlGoal goal;

    // Waits for current path and pose to update
    while(!heard_vel) 
    {
        ros::spinOnce();
        ros::Duration(3).sleep(); //jivko said we need to sleep
    }

	while(ros::ok()) {
		// Updates current path and pose
		ros::spinOnce();
		int vel = vel_msg.linear.x;
		ROS_INFO("%d\n", vel);

	// current orientation
	// Project deals with angles in Yaw rather than Quaternions
        double current_yaw = tf::getYaw(current_pose.orientation); 

	//Conditional to Check current velocity status of the robot
        if(vel_msg.linear.x > 0)
        {
	    //Trouble shooting message, kept for future use
            ROS_INFO("POSITIVE SPEED");
 	    // Initialization of the LED message with current status
            goal.type.led_animations = bwi_msgs::LEDAnimations::FORWARD;

            //CODE FOR TURN FUNCTIONALITY FROM PREVIOUS FRI PROJECT
  	    // Modified for further accuracy of the algorithm
            // We only traverse the first quarter of the Global Plan Array
            for(int i = 0; i < current_path.poses.size() / 4; i++) {
              double yaw = tf::getYaw(current_path.poses[i].pose.orientation);
              //Check for a treshold breaching of orientation values
              if(abs(current_yaw - yaw) > 0.2) {
                  // Right turn - Negative difference 
                  if(current_yaw - yaw < 0) {
		    // Troubleshooting message
                    ROS_INFO("RIGHT TURN");
		    // Update of LED Message because of turn detection
                    goal.type.led_animations = bwi_msgs::LEDAnimations::LEFT_TURN;    
                  }
                  // Left turn - Positive difference
                  else {
		    // Troubleshooting message
                    ROS_INFO("LEFT TURN");
		    // Update of LED Message because of turn detection
                    goal.type.led_animations = bwi_msgs::LEDAnimations::RIGHT_TURN;
                  } 
              }
            }
        }
	// Negative velocity detection
        else if(vel_msg.linear.x < 0)
        {
		        // Troubleshooting message
			ROS_INFO("NEGATIVE SPEED");
		        // Update of LED Message because of velocity detection
			goal.type.led_animations = bwi_msgs::LEDAnimations::BACKWARD;
	    }
		else
		{
			ROS_INFO("STOPPED");
			goal.type.led_animations = bwi_msgs::LEDAnimations::STOPPED;
      // if the linear is 0 but the angular is not 0
      // means that the robot is using the rotating mechanism
      if(vel_msg.angular.z != 0) 
      {
        // Troubleshooting message
        ROS_INFO("ROTATING");
        // Set of the LED Message in Sample case
        goal.type.led_animations = bwi_msgs::LEDAnimations::ROTATING;
      }  
		}
        goal.timeout = ros::Duration(2);
        //Action client sends message     
		ac.sendGoal(goal);
        //Wait for Action Client to come back for an allotted amount of time
		ac.waitForResult(ros::Duration(0.2, 0));
		loop_rate.sleep();        
	}
    return 0;
}
