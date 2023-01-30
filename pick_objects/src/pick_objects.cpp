#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal pickup_goal;
    move_base_msgs::MoveBaseGoal dropoff_goal;

    // set up the frame parameters for pickup
    pickup_goal.target_pose.header.frame_id = "map";
    pickup_goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach pickup position
    pickup_goal.target_pose.pose.position.x = -6.9;
    pickup_goal.target_pose.pose.position.y = 5.73;
    pickup_goal.target_pose.pose.orientation.w = 1.0;

    // Send the pickup position and orientation for the robot to reach
    ROS_INFO("Sending pickup goal");
    ac.sendGoal(pickup_goal);

    // Wait for infinite time for the results
    ac.waitForResult();

    // Check if the robot reached the pickup goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The robot reached the pickup position successfully!");
    else
      ROS_INFO("The robot failed to reach the pickup position");

    // Wait for 5 sec
    ROS_INFO("Waiting for 5 seconds.");
    sleep(5);

    // set up the frame parameters for dropoff
    dropoff_goal.target_pose.header.frame_id = "map";
    dropoff_goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach pickup position
    dropoff_goal.target_pose.pose.position.x = -3.2;
    dropoff_goal.target_pose.pose.position.y = -3.77;
    dropoff_goal.target_pose.pose.orientation.w = 1.0;

    // Send the dropoff position and orientation for the robot to reach
    ROS_INFO("Sending drop-off goal");
    ac.sendGoal(dropoff_goal);

    // Wait for infinite time for the results
    ac.waitForResult();

    // Check if the robot reached the dropoff goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The robot reached the drop-off position successfully");
    else
      ROS_INFO("The robot failed to reach the drop-off position");

    return 0;
}
