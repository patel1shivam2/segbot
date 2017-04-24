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

ros::Subscriber robot_vel;

geometry_msgs::Twist vel_msg;

bool heard_vel = false;

/*******************************************************
*                 Callback Functions                   *
********************************************************/
// Updates the current path
void vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
   vel_msg = *msg;
   heard_vel = true;
}

int main(int argc, char **argv)
{
    ROS_INFO("MADE IT");

    ros::init(argc, argv, "vel_monitor");
    ros::NodeHandle n;

    ros::Rate loop_rate(30);

    // Sets up subscribers
    robot_vel = n.subscribe("/cmd_vel", 1, vel_cb);

    // Sets up action client
    actionlib::SimpleActionClient<bwi_msgs::LEDControlAction> ac("led_control_server", true);
    ac.waitForServer();

    bwi_msgs::LEDControlGoal goal;

    // Waits for current path and pose to update
    while(!heard_vel) 
    {
        ros::spinOnce();
    }

	while(ros::ok()) {
		// Updates current path and pose
		ros::spinOnce();
		
        if(vel_msg.linear.x > 0)
        {
            ROS_INFO("POSITIVE SPEED");
            goal.type.led_animations = bwi_msgs::LEDAnimations::UP;
            goal.timeout = ros::Duration(7);
            ac.sendGoal(goal);
        }
        else if(vel_msg.linear.x < 0)
        {
            ROS_INFO("NEGATIVE SPEED");
        }
        
        loop_rate.sleep();
	}
    return 0;
}