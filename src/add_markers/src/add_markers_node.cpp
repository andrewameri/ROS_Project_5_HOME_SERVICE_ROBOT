#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

//robot status
enum robot_operation_status{START, PICKUP, DROPOFF, ERROR};
uint8_t current_robot_status = START;
bool item_dropped_off = false;


// Define robot status callback
void robot_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
   ROS_INFO("Received message : %d ", msg->data);
   current_robot_status = msg->data;
   return;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber robot_status_sub = n.subscribe("/robot_status", 1, robot_status_callback);
  
  while (ros::ok)
  {
    ros::spinOnce();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "object_marker";
    marker.id = 0;
    // defining a CUBE as marker
    marker.type = visualization_msgs::Marker::CUBE;

    switch(current_robot_status)
    {
      case 0: //START
        marker.action = visualization_msgs::Marker::ADD;
        // Set the pose of the marker for pickup coordinate
        marker.pose.position.x = 4;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        break;
      case 1: //PICKUP
        marker.action = visualization_msgs::Marker::DELETE;
        break;
      case 2: //DROPOFF
        marker.action = visualization_msgs::Marker::ADD;
        // Set the pose of the marker for pickup coordinate
        marker.pose.position.x = 0;
        marker.pose.position.y = -4;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        item_dropped_off = true;
        break;
    }

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    if (item_dropped_off)
      return 0;

    r.sleep();
  }

  return 0;
}

