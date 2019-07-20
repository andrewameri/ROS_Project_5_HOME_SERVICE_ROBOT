#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

//robot status
enum robot_operation_status{START, PICKUP, DROPOFF, ERROR};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  std_msgs::UInt8 msg;
  
  //define a handle
  ros::NodeHandle n;

  //publisher for status of robot
  ros::Publisher robot_status_pub = n.advertise<std_msgs::UInt8>("/robot_status", 1);
  msg.data = START;
  robot_status_pub.publish(msg);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_zone;
  move_base_msgs::MoveBaseGoal dropoff_zone;

  // set up the frame parameters
  pickup_zone.target_pose.header.frame_id = "map";
  pickup_zone.target_pose.header.stamp = ros::Time::now();

  dropoff_zone.target_pose.header.frame_id = "map";
  dropoff_zone.target_pose.header.stamp = ros::Time::now();

  // Define pick up zone
  pickup_zone.target_pose.pose.position.x = 4.0;
  pickup_zone.target_pose.pose.position.y = 0.0;
  pickup_zone.target_pose.pose.orientation.w = 1.0;

  // Define drop off zone
  dropoff_zone.target_pose.pose.position.x = 0.0;
  dropoff_zone.target_pose.pose.position.y = -4.0;
  dropoff_zone.target_pose.pose.orientation.w = 1.0;

  // Send Pick Up Goal
  ROS_INFO("Sending pickup zone coordinates to robot ...");
  ac.sendGoal(pickup_zone);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot has reached pick up zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Robot just reached at pickup zone!");
    msg.data = PICKUP;
    robot_status_pub.publish(msg);
  }
  else{
    ROS_INFO("Robot failed to reach pickup zone!");
    msg.data = ERROR;
    robot_status_pub.publish(msg);
  }

  // Wait 5 Sec
  ros::Duration(5.0).sleep();

  // Send Drop Off Goal
  ROS_INFO("Sending dropoff zone coordinates to robot ...");
  ac.sendGoal(dropoff_zone);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot just reached at dropoff zone!");
    msg.data = DROPOFF;
    robot_status_pub.publish(msg);
  }
  else{
    ROS_INFO("Robot failed to reach dropoff zone!");
    msg.data = ERROR;
    robot_status_pub.publish(msg);
  }

  // Wait 1 Sec
  ros::Duration(3.0).sleep();
  return 0;
}
