#include <stdlib.h>
#include <time.h>

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
#include <move_base_msgs/MoveBaseAction.h>
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
#include "actionlib_msgs/GoalStatus.h"

/*******************************************************
*                 Global Variables                     *
********************************************************/
ros::Publisher signal_pub;

ros::Subscriber global_path;
ros::Subscriber robot_pose;
ros::Subscriber status;
ros::Subscriber robot_goal;

nav_msgs::Path current_path;
geometry_msgs::Pose current_pose;

actionlib_msgs::GoalStatusArray r_goal;

bool heard_path = false;
bool heard_pose = false;
bool heard_goal = false;
bool block_detected = false;

/*******************************************************
*                 Callback Functions                   *
********************************************************/
void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
    current_path = *msg;
    heard_path = true;
}

// status code 1 moving in progress
// status code 4 is failed to get a plan
// status code 3 is code succeded
void status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg_goal)
{
    r_goal = *msg_goal;
    heard_goal = true;
}

void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStamped new_pose = *msg;
    current_pose = new_pose.pose.pose;
    heard_pose = true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "blocked_monitor");
    ros::NodeHandle n;

    ros::Rate loop_rate(30);
    ros::Rate inner_rate(7);

    srand(time(NULL));

    // Sets up service clients
    ros::ServiceClient speak_message_client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");
    bwi_services::SpeakMessage speak_srv;

    ros::ServiceClient gui_client = n.serviceClient<bwi_msgs::QuestionDialog>("question_dialog");
    bwi_msgs::QuestionDialog gui_srv;

    // Sets up subscribers
    global_path = n.subscribe("/move_base/GlobalPlanner/plan", 1, path_cb);
    robot_pose = n.subscribe("/amcl_pose", 1, pose_cb);
    robot_goal = n.subscribe("/move_base/status", 1, status_cb);

    // Sets up action client
    actionlib::SimpleActionClient<bwi_msgs::LEDControlAction> ac("led_control_server", true);
    ac.waitForServer();

    bwi_msgs::LEDControlGoal goal;

    // Waits for current path and pose to update
    while(!heard_path || !heard_pose)
    {
        ros::spinOnce();
    }

    while(ros::ok())
    {
        // Updates current path and pose
        ros::spinOnce();

        //ROS_INFO_STREAM(r_goal.status_list[0].status);
        if(heard_goal == true)
        {
            int randLED = rand()%2;

            while(current_path.poses.size() == 0 && r_goal.status_list[0].status == 4 )
            {

                // TODO: Add logging for study
                // Time in state
                // Times recovery behavior is used
                // Times replanning of path is done
                // Optional: Record scene
                if(randLED == 1)
                {
                    goal.type.led_animations = bwi_msgs::LEDAnimations::BLOCKED;
                    goal.timeout = ros::Duration(7);
                    ac.sendGoal(goal);

                    gui_srv.request.type = 0;
                    gui_srv.request.message = "Blocked";
                    gui_client.call(gui_srv);

                    speak_srv.request.message = "My path is Blocked, please clear a path for me";
                    speak_message_client.call(speak_srv);
                }
                else
                {
                    // Log without led
                }
                ROS_INFO("blocked");

                block_detected = true;
                inner_rate.sleep();
                ros::spinOnce();
            }
            if (block_detected)
            {
                // End log
                ac.cancelAllGoals();
                block_detected = false;
            }
            heard_goal = false;
        }
        loop_rate.sleep();
    }
    return 0;
}
