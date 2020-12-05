#include <iostream>

#include <GL/gl.h>
#include <GL/glu.h>


#include "gui/gui.h"


double
quaternion_to_yaw( const geometry_msgs::Quaternion& quaternion ){
  return atan2( 2.0 * ( quaternion.w * quaternion.z + quaternion.x * quaternion.y ), 1.0 - 2.0 * ( quaternion.y * quaternion.y + quaternion.z * quaternion.z ) );
}


GUI::
GUI( QWidget * parent ) 
  : QGLWidget( parent ), _timer(), _zoom( 5.0 ), _center( 0.0, 0.0 ), _laserscan(), _odom(), _goal(), _path()
{
  setMinimumSize ( 600, 600 );
  setFocusPolicy( Qt::StrongFocus );

  connect( &_timer, SIGNAL( timeout() ), this, SLOT( timer_callback() ) );
  _timer.start( 10 );
}


GUI::
~GUI(){}


void
GUI::
handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg ){
  std::cout << "in handleLaserScan" << std::endl;
  _laserscan = *msg;
  updateGL();
  return;
}


void
GUI::
handleOdom( const nav_msgs::Odometry::ConstPtr& msg ){
  std::cout << "in handleOdom" << std::endl;
  _odom = *msg;
  updateGL();
  return;
}


void
GUI::
handleGoal( const geometry_msgs::Pose::ConstPtr& msg ){
  std::cout << "in handleGoal" << std::endl;
  _goal = *msg;
  updateGL();
  return;
}


void
GUI::
handlePath( const nav_msgs::Path::ConstPtr& msg ){
  std::cout << "in handlePath" << std::endl;
  _path = *msg;
  updateGL();
  return;
}


void
GUI::
timer_callback( void ){
  ros::spinOnce();
  return;
}


void
GUI::
initializeGL(){
  glClearColor( 1.0, 1.0, 1.0, 1.0 );
  glEnable( GL_LINE_SMOOTH );
  glEnable( GL_BLEND );
  return;
}


void
GUI::
resizeGL( int width, int height ){
  glViewport( 0, 0, width, height );
  return;
}


void
GUI::
paintGL(){
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  double ratio = (double) ( size().width() ) / (double)( size().height() );
  gluOrtho2D( -_zoom * ratio + _center.first, _zoom * ratio + _center.first, -_zoom + _center.second, _zoom + _center.second );
  glMatrixMode(GL_MODELVIEW);

  glLoadIdentity();
  drawGrid();
  drawCoordinateSystem();
  drawLaserScan( _laserscan, 0.0, 0.0, 1.0 );
  drawRobot( _odom.pose.pose, 0.0, 0.0, 0.0, 0.1225 );
  drawRobot( _goal, 0.0, 1.0, 0.0, 0.1225 );
  drawPath( _path, 1.0, 0.0, 0.0, 1.0 );
  return;
}


void
GUI::
drawCoordinateSystem( void ){
  glBegin( GL_LINES );
  glColor4f( 1.0, 0.0, 0.0, 1.0 );
  glVertex3f( 0.0, 0.0, 0.0 );
  glVertex3f( 1.0, 0.0, 0.0 );
  glColor4f( 0.0, 1.0, 0.0, 1.0 );
  glVertex3f( 0.0, 0.0, 0.0 );
  glVertex3f( 0.0, 1.0, 0.0 );
  glColor4f( 0.0, 0.0, 1.0, 1.0 );
  glVertex3f( 0.0, 0.0, 0.0 );
  glVertex3f( 0.0, 0.0, 1.0 );
  glEnd();
  return;
}


void
GUI::
drawGrid( void ){
  glColor4f( 0.8, 0.8, 0.8, 1.0 );
  glLineWidth( 2.0 );
  glBegin( GL_LINES );
  for( int i = -10; i <= 10; i++ ){
    glVertex3f( -10.0, (double)(i), 0.0 );
    glVertex3f( 10.0, (double)(i), 0.0 );
    glVertex3f( (double)(i), -10.0, 0.0 );
    glVertex3f( (double)(i), 10.0, 0.0 );
  }
  glEnd();
  glLineWidth( 1.0 );
}


void
GUI::
drawLaserScan( const sensor_msgs::LaserScan& laserscan, const double& red, const double& green, const double& blue ){
  return;
}


void
GUI::
drawRobot( const geometry_msgs::Pose& pose, const double& red, const double& green, const double& blue, const double& radius){
  glPushMatrix();
  glTranslated( pose.position.x, pose.position.y, 0.0);
  glRotated( quaternion_to_yaw( pose.orientation ) * 180.0 / M_PI, 0.0, 0.0, 1.0 );
  unsigned int discretization = 33;
  glColor4f(red, green, blue, 1.0);
  glLineWidth( 5.0 );
  glBegin( GL_LINE_STRIP );
  for( unsigned int i = 0; i < discretization; i++ ){
    double angle = 2.0 * M_PI * (double)(i) / (double)( discretization - 1 );
    glVertex3f( radius * cos( angle ), radius * sin( angle ), 0.0 );
  }
  glEnd();
  glBegin( GL_LINES );
  glVertex3f( radius, 0.0, 0.0 );
  glVertex3f( -radius, 0.0, 0.0 );
  glEnd();
  glBegin( GL_TRIANGLES );
  glVertex3f( radius, 0.0, 0.0 );
  glVertex3f( radius/4.0, radius/2.0, 0.0 );
  glVertex3f( radius/4.0, -radius/2.0, 0.0 );
  glEnd();
  glLineWidth( 1.0 );
  glPopMatrix();
  return;
}


void
GUI::
drawPath( const nav_msgs::Path& path, const double& red, const double& green, const double& blue, const double& width ){
  glLineWidth(width);
  glColor4f( red, green, blue, 1.0 );
  glBegin( GL_LINE_STRIP );
  for( unsigned int i = 0; i < path.poses.size(); i++ ){
    glVertex3f( path.poses[i].pose.position.x, path.poses[i].pose.position.y, 0.0 );
  }
  glEnd();
  glLineWidth( 1.0 );
  return;
}


void
GUI::
keyPressEvent( QKeyEvent* event ){
  if( event->matches( QKeySequence::Copy ) ){
    close();
    return;
  }
  else{
    switch(event->key()){
      case Qt::Key_Left:
        _center.first -= 0.5;
        break;
      case Qt::Key_Right:
        _center.first += 0.5;
        break;
      case Qt::Key_Down:
        _center.second -= 0.5;
        break;
      case Qt::Key_Up:
        _center.second += 0.5;
        break;
      case Qt::Key_I:
        if( _zoom > 0.5 ){
          _zoom -= 0.5;
        }
        break;
      case Qt::Key_O:
        _zoom += 0.5;
        break;
      default:
        std::cout << "couldn't handle key " << event->key() << std::endl;
        break;
    }
    updateGL();
  }
  return;
}
