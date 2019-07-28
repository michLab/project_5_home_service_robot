#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // set up goal locations: pick_up and drop_off
  move_base_msgs::MoveBaseGoal pick_up_goal;
  move_base_msgs::MoveBaseGoal drop_off_goal;

  // set up the frame parameters
  pick_up_goal.target_pose.header.frame_id = "map";
  drop_off_goal.target_pose.header.frame_id = "map";

  // Define a position and orientation for the robot to reach
  pick_up_goal.target_pose.pose.position.x = -3.0;
  pick_up_goal.target_pose.pose.position.y = 2.0;
  pick_up_goal.target_pose.pose.orientation.w = 1.0;
  
  drop_off_goal.target_pose.pose.position.x = -4.0;
  drop_off_goal.target_pose.pose.position.y = -4.0;
  drop_off_goal.target_pose.pose.orientation.w = 1.0;

  while (true) {
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending pick-up goal");
    pick_up_goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(pick_up_goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    	ROS_INFO("Success, robot reached pick-up zone, picking-up object");
        ros::Duration(5.0).sleep();
      	ROS_INFO("Object picked");
    }
    else {
    	ROS_INFO("Failure, robot failed to get pick-up zone");
    }
    
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending drop-off goal");
    drop_off_goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(drop_off_goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    	ROS_INFO("Succes, robot reached drop-off zone, dropping object");
    	ROS_INFO("Object dropped");
    }
    else
      ROS_INFO("Failure, robot failed to get drop-off zone");
  }
  return 0;
}
