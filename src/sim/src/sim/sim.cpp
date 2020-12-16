#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "sim/sim.h"

/**
 * x(0) = x position
 * x(1) = y position
 * x(2) = heading
 * u(0) = linear_velocity (v)
 * u(1) = angular_velocity (w)
 */

geometry_msgs::Quaternion
yaw_to_quaternion( const double& yaw ){
  geometry_msgs::Quaternion quaternion;
  quaternion.w = cos( yaw / 2.0 );
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin( yaw / 2.0 );
  return quaternion;
}

double
sample( const double& bsquared ){
  double tmp = 0.0;
  double b = sqrt(bsquared);
  for( unsigned int i = 0; i < 12; i++ ){
    tmp += (-b + 2.0 * b * ((double)(rand() % 1000) / (double)(1000)));
  }
  return 0.5 * tmp;
}

double
sgn( const double& x ){
  if( x >= 0 )
    return 1.0;
  else
    return -1.0;
}


Sim::
Sim()
  : _x( 0.0, 0.0, 0.0 ), _u( 0.0, 0.0, 0.0 ), _alpha1( 0.005 ), _alpha2( 0.05 ), _alpha3( 0.005 ), _alpha4( 0.05 ), _alpha5( 0.005 ), _alpha6( 0.05 ), _t( 0.0 ), _num_scan_angles( 128 ), _num_scan_distances( 256 ) {
  srand( time( nullptr ) );
}

Sim::
~Sim(){}


void
Sim::
step( const double& dt ){
  Eigen::Vector3d uhat( _u );

  uhat( 0 ) += (_alpha1 * pow( _u( 0 ), 2 )) + ( _alpha2 * pow( _u( 1 ), 2 ) );
  uhat( 1 ) += (_alpha3 * pow( _u( 0 ), 2 )) + ( _alpha4 * pow( _u( 1 ), 2 ) );

  Eigen::Vector3d dx( 0.0, 0.0, 0.0 );

  if( fabs( uhat( 1 ) ) < 0.0000001 )
    uhat( 1 ) = sgn( uhat( 1 ) ) * 0.0000001;
  dx( 0 ) = (( -(uhat( 0 ) / uhat( 1 )) * sin(_x( 2 ))) + ( (uhat( 0 )/uhat( 1 )) * sin( _x(2) + ( uhat(1) * dt ) ) ));
  dx( 1 ) = (( (uhat( 0 ) / uhat( 1 )) * cos(_x( 2 ))) - ( (uhat( 0 )/uhat( 1 )) * cos( _x(2) + ( uhat(1) * dt ) ) ));
  dx( 2 ) = ( uhat( 1 ) * dt );

  _x += dx;
  _t += dt;

  std::cout << "_u[3]:{" << _u( 0 ) << "," << _u( 1 ) << "," << _u( 2 ) << "}" << std::endl;
  std::cout << "uhat[3]:{" << uhat( 0 ) << "," << uhat( 1 ) << "," << uhat( 2 ) << "}" << std::endl;
  std::cout << "_x[3]:{" << _x( 0 ) << "," << _x( 1 ) << "," << _x( 2 ) << "}" << std::endl;
  std::cout << "_t:" << _t << std::endl;
  return;
}

void
Sim::
handle_command( const geometry_msgs::Twist::ConstPtr& msg ){
  _u( 0 ) = msg->linear.x;
  _u( 1 ) = msg->angular.z;
  return;
}

nav_msgs::Odometry
Sim::
odometry_msg( void ) const{
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = _x( 0 );
  msg.pose.pose.position.y = _x( 1 );
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = yaw_to_quaternion( _x( 2 ) );
  return msg;
}
