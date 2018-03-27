// Transformation between the facedown camera frame and the drone frame
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <sensor_msgs/Range.h>

const double ALPHA = 31.8244;
const double BETA  = 40.4497;
const double PIXLES[2] = {320, 240};
const double PI = 3.14159;
nav_msgs::Odometry current_pose;
std_msgs::Float64 current_heading;
using namespace std;
geometry_msgs::PoseStamped roombaPose;
sensor_msgs::Range rngfnd;
void pixel2metric_facedown(double x_e, double y_e, double alt, double theta, double theta_0, vector<double> obj_pix, vector<double> &O_m)
{
  // find vector from middle of camera
  double r_p[2] = {0, 0};
  r_p[0] = obj_pix[0] - PIXLES[0];
  r_p[1] = obj_pix[1] - PIXLES[1];

  double O_mx = r_p[0]/3779.527;
  double O_my = (-1.0)*r_p[1]/3779.527;

  // convert degrees to radians
  theta = theta * (PI/180.0);
  theta_0 = theta_0 * (PI/180.0);

  // compute coefficients
  double c11 = cos(theta_0)*cos(theta) + sin(theta)*sin(theta_0);
  double c12 = sin(theta)*cos(theta_0) - sin(theta_0)*cos(theta);
  double c21 = sin(theta_0)*cos(theta) - sin(theta)*cos(theta_0);
  double c22 = sin(theta)*sin(theta_0) + cos(theta)*cos(theta_0);

  // Transform drone pos to ENU
  double x_d = x_e*cos(theta_0) - y_e*sin(theta_0);
  double y_d = x_e*sin(theta_0) + y_e*cos(theta_0);

  // transform roomba position from drone to gym frame
  double r_x = O_mx*c11 + O_my*c12 + x_d;
  double r_y = O_mx*c21 + O_my*c22 + y_d;

  // O_m[0] = O_mx;
  // O_m[1] = O_my;
  ROS_INFO("transformed to x: %f y: %f z: %f meters \n", r_x, r_y, rngfnd.range); 
  roombaPose.pose.position.x = O_my;
  roombaPose.pose.position.y = O_mx;
  roombaPose.pose.position.z = rngfnd.range;
}

void chatterCallback(const std_msgs::Int8::ConstPtr& msg)
{
  int num = msg->data;
  //ROS_INFO("%d objects found", num);
}
void rng_cb(const sensor_msgs::Range::ConstPtr& msg){
    rngfnd = *msg;
    //ROS_INFO("Range: %f", rngfnd.range); 
}

void centerPoint(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msgBox)
{
  darknet_ros_msgs::BoundingBoxes boxesFound;
  darknet_ros_msgs::BoundingBox objectBounds;
  boxesFound = *msgBox;
  double xCenter;
  double yCenter;
  for(int i=0; i < boxesFound.boundingBoxes.size(); i++)
  {
    objectBounds = boxesFound.boundingBoxes[i];
    xCenter = (objectBounds.xmax + objectBounds.xmin)/2;
    yCenter = (objectBounds.ymax + objectBounds.ymin)/2;
    std::string objectType = objectBounds.Class;
    ROS_INFO("%s found x: %f y: %f pixels", objectType.c_str(), xCenter, yCenter); 
  }
  double x_e = current_pose.pose.pose.position.x;
  double y_e = current_pose.pose.pose.position.y;
  double alt = rngfnd.range;
  double theta = current_heading.data;
  double theta_0 = 0;
  vector<double> obj_pix;
  obj_pix.push_back(xCenter);
  obj_pix.push_back(yCenter);
  vector<double> O_m;
  pixel2metric_facedown(x_e, y_e, alt, theta, theta_0, obj_pix, O_m);
}
//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg) 
{
  current_pose = *msg;
  //ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}
//get compass heading 
void heading_cb(const std_msgs::Float64::ConstPtr& msg)
{
  current_heading = *msg;
  //ROS_INFO("current heading: %f", current_heading.data);
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "transformations");
  
  ros::NodeHandle n;
  ros::Subscriber rng_sub = n.subscribe<sensor_msgs::Range>("mavros/rangefinder/rangefinder",1,rng_cb);
  ros::Subscriber sub2 = n.subscribe("/darknet_ros/bounding_boxes",1 ,centerPoint);
  ros::Subscriber sub = n.subscribe("/darknet_ros/found_object", 1, chatterCallback);
  ros::Subscriber currentPos = n.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
  ros::Subscriber currentHeading = n.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, heading_cb);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("roombaPose", 1000);

  

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    chatter_pub.publish(roombaPose);

    ros::spinOnce();

    loop_rate.sleep();

  }







  return 0;
}



