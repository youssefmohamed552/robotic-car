#ifndef A_STAR_H
#define A_STAR_H

#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include <vector>

class Pose{
  public:
    Pose( const geometry_msgs::Pose& curr, const geometry_msgs::Pose& goal, Pose* prv );
    Pose( const Pose& other );
    Pose();
    virtual ~Pose();
    void update( const geometry_msgs::Pose& goal, Pose* new_prev );

    geometry_msgs::Pose pose;
    double h;
    double f;
    double g;
    Pose* prev;
};


class AStarSolver{
  public:
    AStarSolver();
    virtual ~AStarSolver();

    nav_msgs::Path solve( const geometry_msgs::Pose& start, const geometry_msgs::Pose& end );
    void update_open_list( const std::vector< geometry_msgs::Pose >& explored_poses, Pose* curr );

    geometry_msgs::Pose goal;
    std::vector< Pose* > open_list;
    std::vector< Pose* > closed_list;
};


#endif /* A_STAR_H */
