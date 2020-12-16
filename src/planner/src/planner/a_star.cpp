#include <iostream>
#include <stdio.h>
#include <algorithm>
#include "planner/a_star.h"
#include "geometry_msgs/PoseStamped.h"


double
compute_distance( const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2 ){
  double x = p1.position.x - p2.position.x;
  double y = p1.position.y - p2.position.y;
  return sqrt( x * x + y * y );
}


AStarSolver::
AStarSolver(){}


AStarSolver::
~AStarSolver(){}


std::vector< geometry_msgs::Pose >
explore( const geometry_msgs::Pose& curr ){
  std::vector< geometry_msgs::Pose > explore_poses(8);
  double dxs[] = { 0.5, 0.5, 0.0, -0.5, -0.5, -0.5, 0.0, 0.5 };
  double dys[] = { 0.0, -0.5, -0.5, -0.5, 0.0, 0.5, 0.5, 0.5 };
  for( unsigned int i = 0; i < 8; i++){
    explored_poses[i].position.x = curr.position.x + dxs[i];
    explored_poses[i].position.y = curr.position.y + dys[i];
  }
  return explored_poses;
}


bool
comp_sort( const Pose* p1, const Pose* p2 ){
  return p1->f > p2->f;
}


bool
equal( const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2 ){
  return p1.position.x == p2.position.x && p1.position.y == p2.position.y;
}


std::vector< Pose* >::iterator
linear_search( std::vector< Pose* >& v, const Pose& val ){
  for( auto p = v.begin(); p!= v.end(); p++ ){
    if( equal( (*p)->pose, val.pose ) ) return p;
  }
  return v.end();
}



