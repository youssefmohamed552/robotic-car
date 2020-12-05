 #include <QtWidgets/QApplication>
#include "gui/gui.h"

int
main( int argc, char* argv[] ){
  QApplication app( argc, argv );
  ros::init( argc, argv, "gui" );
  ros::NodeHandle node_handle;
  GUI gui;
  ros::Subscriber subscriber_reset_odometry = node_handle.subscribe( "laserscan", 1, &GUI::handleLaserScan, &gui );
  ros::Subscriber subscriber_odom = node_handle.subscribe( "odom", 1, &GUI::handleOdom, &gui );
  ros::Subscriber subscriber_goal = node_handle.subscribe( "goal", 1, &GUI::handleGoal, &gui );
  ros::Subscriber subscriber_path = node_handle.subscribe( "path", 1, &GUI::handlePath, &gui );
  gui.show();
  return app.exec();
}
