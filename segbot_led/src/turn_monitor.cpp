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
ros::Subscriber robot_pose;

nav_msgs::Path current_path;
geometry_msgs::Pose current_pose;

bool heard_path = false;
bool heard_pose = false;

/*******************************************************
*                 Callback Functions                   *
********************************************************/
// Updates the current path
void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
    current_path = *msg;
    heard_path = true;
}

// Updates the current pose
void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStamped new_pose = *msg;
    current_pose = new_pose.pose.pose;
    heard_pose = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_monitor");
    ros::NodeHandle n;

    ros::Rate loop_rate(30);

    // Sets up service clients
    ros::ServiceClient speak_message_client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message"); 
    bwi_services::SpeakMessage speak_srv;

    ros::ServiceClient gui_client = n.serviceClient<bwi_msgs::QuestionDialog>("question_dialog"); 
    bwi_msgs::QuestionDialog gui_srv;

    // Sets up subscribers
    global_path = n.subscribe("/move_base/GlobalPlanner/plan", 1, path_cb);
    robot_pose = n.subscribe("/amcl_pose", 1, pose_cb);

    // Sets up action client
    actionlib::SimpleActionClient<bwi_msgs::LEDControlAction> ac("led_control_server", true);
    ac.waitForServer();

    bwi_msgs::LEDControlGoal goal;

    // Waits for current path and pose to update
    while(!heard_path || !heard_pose) 
    {
        ros::spinOnce();
    }

    double stop_yaw = 0;
    bool turn_signal = false;

    while(ros::ok())
    {
        // Updates current path and pose
        ros::spinOnce();
        double current_yaw = tf::getYaw(current_pose.orientation);

        // Turns turn signal off if turn is complete
        if(turn_signal && (abs(current_yaw - stop_yaw) < .1))
        {
            gui_srv.request.type = 0; 
            gui_srv.request.message = ""; 
            gui_client.call(gui_srv);
            turn_signal = false;
        }

        // TODO: Choose max dist rather then halving plan
        // Iterates through the first half of points in the global path and determines if any major
        // changes in orientation are coming.
        for(int i = 0; i < current_path.poses.size() / 2; i++)
        {
            double yaw = tf::getYaw(current_path.poses[i].pose.orientation);
            if((abs(current_yaw - yaw) > 0.81) && !turn_signal)
            {
                turn_signal = true;
                // Right turn
                if(current_yaw - yaw < 0)
                {
                    goal.type.led_animations = bwi_msgs::LEDAnimations::RIGHT_TURN;
                    goal.timeout = ros::Duration(7);
                    ac.sendGoal(goal);
                    
                    gui_srv.request.type = 0; 
                    gui_srv.request.message = "Turning Right"; 
                    gui_client.call(gui_srv);

                    speak_srv.request.message = "Turning Right";
                    speak_message_client.call(speak_srv);  
                }
                // Left turn
                else
                {
                    goal.type.led_animations = bwi_msgs::LEDAnimations::LEFT_TURN;
                    goal.timeout = ros::Duration(7);
                    ac.sendGoal(goal);

                    gui_srv.request.type = 0; 
                    gui_srv.request.message = "Turning Left"; 
                    gui_client.call(gui_srv);

                    speak_srv.request.message = "Turning Left";
                    speak_message_client.call(speak_srv);  
                }
                stop_yaw = yaw;
            }
        }
        loop_rate.sleep();
    }

    return 0;
}
