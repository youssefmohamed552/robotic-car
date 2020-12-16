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
  std::cout << "in explore" << std::endl;
  std::vector< geometry_msgs::Pose > explored_poses(8);
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


void
AStarSolver::
update_open_list( const std::vector< geometry_msgs::Pose >& explored_poses, Pose* curr ){
  std::cout << "in update_open_list" << std::endl;
  std::vector< Pose* > available_poses;
  for( auto p = explored_poses.begin(); p != explored_poses.end(); p++ ){
    Pose* test_pose = new Pose(*p, goal, curr);
    auto closed_list_element = linear_search( closed_list, * test_pose );
    if( closed_list_element != closed_list.end() )
      continue;
    auto open_list_element = linear_search( open_list, *test_pose );
    if( open_list_element == open_list.end() ){
      available_poses.push_back(test_pose);
    }
    else {
      (*open_list_element)->update(goal, curr);
    }
  }
  open_list.insert( open_list.end(), available_poses.begin(), available_poses.end() );
  sort( open_list.begin(), open_list.end(), comp_sort );
}

nav_msgs::Path
get_path( std::vector< Pose* >& v ){
  std::cout << "in get_path" << std::endl;
  nav_msgs::Path path;
  Pose* p = v.back();
  while( p != nullptr ){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = p->pose;
    path.poses.push_back( pose_stamped );    
    p = p->prev;
  }
  std::reverse( path.poses.begin(), path.poses.end() );
  return path;
}


nav_msgs::Path
AStarSolver::
solve( const geometry_msgs::Pose& start, const geometry_msgs::Pose& end ){
  std::cout << "in solve" << std::endl;
  goal = end;
  Pose* start_pose = new Pose( start, end, nullptr );
  open_list.push_back( start_pose );
  Pose* curr = nullptr;
  while( curr == nullptr || !equal( curr->pose, goal ) ){
    Pose* visited = open_list.back();
    closed_list.push_back( visited );
    open_list.pop_back();
    curr = closed_list.back();
    std::vector< geometry_msgs::Pose > explored_poses  = explore( curr->pose );
    update_open_list( explored_poses, curr );
  }
  return get_path( closed_list );
}


Pose::
Pose( const geometry_msgs::Pose& curr, const geometry_msgs::Pose& goal, Pose* prv )
  : pose( curr ){
  h = compute_distance(curr, goal);
  prev = prv;
  if( !prev ){
    g = 0.0;
    f = h;
  }
  else{
    g = prev->g + compute_distance( prev->pose, pose );
    f = g + h;
  }
}


Pose::
Pose( const Pose& other ){
  pose = other.pose;
  g = other.g;
  h = other.h;
  f = other.f;
  prev = other.prev;
}


Pose::
Pose(){}


Pose::
~Pose(){}



void
Pose::
update( const geometry_msgs::Pose& goal, Pose* new_prev ){
  if( new_prev == nullptr ) return;
  double new_g = new_prev->g + compute_distance( new_prev->pose, pose );
  if( new_g >= g ) return;
  g = new_g;
  f = g + h;
  prev = new_prev;
}
