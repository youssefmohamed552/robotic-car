#ifndef SIM_H
#define SIM_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class Sim{
  public:
    Sim();
    virtual ~Sim();

    void step( const double& dt );
    void handle_command( const geometry_msgs::Twist::ConstPtr& msg );
    nav_msgs::Odometry odometry_msg(void) const;
    sensor_msgs::LaserScan scna_msg(void) const;

  protected:
    Eigen::Vector3d _x;
    Eigen::Vector3d _u;
    double _alpha1;
    double _alpha2;
    double _alpha3;
    double _alpha4;
    double _alpha5;
    double _alpha6;
    double _t;
    unsigned int _num_scan_angles;
    unsigned int _num_scan_distances;
};

#endif /* SIM_H */
