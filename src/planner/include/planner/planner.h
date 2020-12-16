#ifndef PLANNER_H
#define PLANNER_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "planner/a_star.h"

class Planner{
  public:
    Planner();
    virtual ~Planner();

    void search( const geometry_msgs::Pose& start, const geometry_msgs::Pose& end );
    
    void handle_odom( const nav_msgs::Odometry::ConstPtr& msg );
    void handle_goal( const geometry_msgs::Pose::ConstPtr& msg );

    AStarSolver a_star;
    ros::Publisher path_publisher;
    ros::Publisher openlistsize_publisher;
    ros::Publisher closedlistsize_publisher;
    nav_msgs::Odometry odometry;
};


#endif /* PLANNER_H */
