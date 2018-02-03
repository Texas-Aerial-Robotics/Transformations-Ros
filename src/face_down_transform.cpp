// Transformation between the facedown camera frame and the drone frame
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

const double ALPHA = 31.8244;
const double BETA  = 40.4497;
const double PIXLES[2] = {320, 240};
const double PI = 3.14159;

using namespace std;
geometry_msgs::PoseStamped roombaPose;
void pixel2metric_facedown(double alt, vector<double> obj_pix, vector<double> &O_m)
{
  // find vector from middle of camera
  double r_p[2] = {0, 0};
  r_p[0] = obj_pix[0] - PIXLES[0];
  r_p[1] = obj_pix[1] - PIXLES[1];

  double O_mx = r_p[0]/3779.527;
  double O_my = (-1.0)*r_p[1]/3779.527;

  // O_m[0] = O_mx;
  // O_m[1] = O_my;
  ROS_INFO("transformed to x: %f y: %f meters \n", O_mx, O_my); 
  roombaPose.pose.position.x = O_my;
  roombaPose.pose.position.y = O_mx;
  roombaPose.pose.position.z = 1;
}

void chatterCallback(const std_msgs::Int8::ConstPtr& msg)
{
  int num = msg->data;
  //ROS_INFO("%d objects found", num);
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
  double alt = 1;
  vector<double> obj_pix;
  obj_pix.push_back(xCenter);
  obj_pix.push_back(yCenter);
  vector<double> O_m;
  pixel2metric_facedown(alt, obj_pix, O_m);
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "transformations");
  
  ros::NodeHandle n;
  
  ros::Subscriber sub2 = n.subscribe("/darknet_ros/bounding_boxes",1 ,centerPoint);
  ros::Subscriber sub = n.subscribe("/darknet_ros/found_object", 1, chatterCallback);
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



