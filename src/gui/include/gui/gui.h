#ifndef GUI_H
#define GUI_H

#include <iostream>
#include "ros/ros.h"

#include <QtOpenGL/QGLWidget>
#include <QtGui/QKeyEvent>
#include <QtCore/QTimer>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

class GUI: public QGLWidget {
  Q_OBJECT
  public:
    GUI( QWidget* parent = NULL );
    virtual ~GUI();

    void handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg );
    void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
    void handleGoal( const geometry_msgs::Pose::ConstPtr& msg );
    void handlePath( const nav_msgs::Path::ConstPtr& msg );

  protected slots:
    void timer_callback( void );

  protected:
    virtual void initializeGL();
    virtual void resizeGL( int width, int height );
    virtual void paintGL();
    void drawCoordinateSystem();
    void drawGrid();
    void drawLaserScan( const sensor_msgs::LaserScan& laserscan, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0 );
    void drawRobot( const geometry_msgs::Pose& pose, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0, const double& radius = 0.0 );
    void drawPath( const nav_msgs::Path& path, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0, const double& width = 0.0 );
    virtual void keyPressEvent( QKeyEvent* event );

    QTimer _timer;

    double _zoom;
    std::pair< double, double > _center;

    sensor_msgs::LaserScan _laserscan;
    nav_msgs::Odometry _odom;
    geometry_msgs::Pose _goal;
    nav_msgs::Path _path;
    
};

#endif /* GUI_H */
