#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"


int
main(int argc, char* argv[]){
  ros::init(argc, argv, "gui_goal_publisher");
  ros::NodeHandle node_handle;
  ros::Publisher goal_publisher = node_handle.advertise< geometry_msgs::Pose >( "goal", 1, true);
  sleep(1);

  std::cout << "creating message" << std::endl;
  geometry_msgs::Pose msg;
  msg.position.x = 5.0;
  msg.position.y = 3.0;
  msg.position.z = 0.0;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 1.0;
  std::cout << "message" << std::endl << msg;

  std::cout << "publishing message" << std::endl;
  goal_publisher.publish(msg);
  sleep(1);
  std::cout << "done" << std::endl;
  return 0;
}
