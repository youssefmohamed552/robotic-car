#include <iostream>
#include "ros/ros.h"
#include "sim/sim.h"

int
main( int argc, char* argv[] ){
  Sim sim;

  ros::init( argc, argv, "sim_node" );
  ros::NodeHandle node_handle;
  ros::Subscriber command_subscriber = node_handle.subscribe( "cmd_vel_mux/input/navi", 1, &Sim::handle_command, &sim );
  ros::Publisher odometry_publisher = node_handle.advertise< nav_msgs::Odometry >( "odom", 1, true );

  double frequency = 10.0;
  ros::Rate timer( frequency );
  while( ros::ok() ){
    sim.step( 1.0 / frequency );
    odometry_publisher.publish( sim.odometry_msg() );
    ros::spinOnce();
    timer.sleep();
  }
  return 0;
}
