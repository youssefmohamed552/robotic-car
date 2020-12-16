#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "planner/planner.h"


int
main( int argc, char* argv[] ){
  Planner planner;
  ros::init( argc, argv, "planner_node" );
  ros::NodeHandle node_handle;
  ros::Subscriber odom_subscriber = node_handle.subscribe( "odom", 1, &Planner::handle_odom, &planner );
  ros::Subscriber goal_subscriber = node_handle.subscribe( "goal", 1, &Planner::handle_goal, &planner );
  planner.openlistsize_publisher = node_handle.advertise< std_msgs::UInt32 >( "openlistsize", 1 );
  planner.closedlistsize_publisher = node_handle.advertise< std_msgs::UInt32 >( "closedlistsize", 1 );
  planner.path_publisher = node_handle.advertise< nav_msgs::Path >( "path", 1 );
  ros::spin();
  return 0;
}
