// Transformation between the facedown camera frame and the drone frame
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
const double PI = 3.14159;
//field of view y
const double PHI = 31.8244*PI/180;
//fiield of view x
const double PHI_X=40*PI/180;
//rotation angle for y direction
double  THETA = 0;
//rotation angle in x direction
const double THETA_X=0;
const double PIXELS[2] = {640, 480};


using namespace std;
geometry_msgs::PoseStamped roombaPose;
nav_msgs::Odometry current_pose;
//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg) 
{
  current_pose = *msg;
  //ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}

void pixel2metric_facedown(double alt, vector<double> obj_pix, vector<double> &O_m)
{	//find puxel of interest in x direction
	double T_x=obj_pix[0];
	double psi_x;
	double O_mx;
	double O_my;
	psi_x=2*abs(T_x/PIXELS[0]*PHI_X-PHI_X/2);
	if(T_x>PIXELS[0]/2){
		 O_mx=alt*tan(THETA_X+psi_x/2);
	}else{ O_mx=alt*tan(THETA_X-psi_x/2);
	}
	
	
	
	
	
	
	
  // find pixel of interest in y direction
  double T_y=obj_pix[1];

 double psi;
  //calculate slice of field of interest
  psi=2*abs(T_y/PIXELS[1]*PHI-PHI/2);
  

  //double O_mx = r_p[0]/3779.527;
  if(T_y<PIXELS[1]/2){
   O_my = alt*tan(THETA+psi/2);
  }else{ O_my=alt*tan(THETA-psi/2);
  }
  
  // O_m[0] = O_mx;
  // O_m[1] = O_my;
  ROS_INFO("transformed to x: %f y: %f meters \n", O_mx, O_my); 
  roombaPose.pose.position.x = O_my;
  roombaPose.pose.position.y = O_mx;
  roombaPose.pose.position.z = 1;
  roombaPose.pose.position.x = O_my +  current_pose.pose.pose.position.x;
  roombaPose.pose.position.y = O_mx +  current_pose.pose.pose.position.y;
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
  double alt = current_pose.pose.pose.position.z;
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
  ros::Subscriber currentPos = n.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
  ros::Subscriber sub2 = n.subscribe("/darknet_ros/bounding_boxes",1 ,centerPoint);
  ros::Subscriber sub = n.subscribe("/darknet_ros/found_object", 1, chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("roombaPose", 1000);


  float camRoll;
  float camPitch;
  float camYaw;

  ros::param::get("/camera_params/cameras/cam2/orientation/roll", camRoll);
  ros::param::get("/camera_params/cameras/cam2/orientation/pitch", camPitch);
  ros::param::get("/camera_params/cameras/cam2/orientation/roll", camYaw);

  ROS_INFO("Parameters loaded");
  ROS_INFO("Camera orientation roll: %f pitch: %f yaw: %f", camRoll, camPitch, camYaw);
  ROS_INFO("Node Started");

  THETA = camPitch;
  

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    chatter_pub.publish(roombaPose);

    ros::spinOnce();

    loop_rate.sleep();

  }







  return 0;
}



