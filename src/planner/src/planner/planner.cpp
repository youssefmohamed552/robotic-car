#include <iostream>
#include "planner/planner.h"
#include "std_msgs/UInt32.h"


Planner::
Planner(){}


Planner::
~Planner(){}


void
Planner::
handle_odom( const nav_msgs::Odometry::ConstPtr& msg ){
  std::cout << "in handle odom" << std::endl;
  odometry = *msg;
  return;
}


void
Planner::
handle_goal( const geometry_msgs::Pose::ConstPtr& msg ){
  std::cout << "in handle goal" << std::endl;
  search( odometry.pose.pose, *msg );
  return;
}


void
Planner::
search( const geometry_msgs::Pose& start, const geometry_msgs::Pose& end ){
  std::cout << "in search" << std::endl;
  path_publisher.publish( a_star.solve( start, end ) );
  std_msgs::UInt32 openlistsize;
  std_msgs::UInt32 closedlistsize;
  openlistsize.data = a_star.open_list.size();
  closedlistsize.data = a_star.closed_list.size();
  openlistsize_publisher.publish( openlistsize );
  closedlistsize_publisher.publish( closedlistsize );
  return;
}
