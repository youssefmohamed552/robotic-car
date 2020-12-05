#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

int
main(int argc, char* argv[]){
  ros::init(argc, argv, "odom_publisher");
  ros::NodeHandle node_handle;
  ros::Publisher odom_publisher = node_handle.advertise< nav_msgs::Odometry >( "odom", 1, true );
  sleep(1);

  std::cout << "creating message" << std::endl;
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = 0;
  msg.pose.pose.position.y = 0;
  msg.pose.pose.position.z = 0;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.7071;
  msg.pose.pose.orientation.w = 0.7071;
  std::cout << "message" << std::endl << msg;

  std::cout << "public messages" << std::endl;
  odom_publisher.publish(msg);
  sleep(1);
  std::cout << "done" << std::endl;
  return 0;
}
