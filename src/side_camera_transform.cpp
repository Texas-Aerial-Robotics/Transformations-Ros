// Transformation between the facedown camera frame and the drone frame
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include "opencv2/core/version.hpp"
#include "transformations_ros/roombaPoses.h"
#include "transformations_ros/roombaPose.h"

int W = 1000; // width of gui
int gridM = 20; // size of grid in meters

const double PI = 3.14159;
//field of view y
const double PHI = 31.8244*PI/180;
//fiield of view x
const double PHI_X=40*PI/180;
//rotation angle for y direction
double  THETA_Y = 0;
//rotation angle in x direction
double THETA_X=0;
const double PIXELS[2] = {640, 480};


using namespace std;
geometry_msgs::PoseStamped roombaPose;
nav_msgs::Odometry current_pose;
std_msgs::Float64 gymOffset;
//transformations_ros::roombaPoses roombaPositions;

struct orientation
{
  float roll;
  float pitch;
  float yaw;
  string frame_id;
};
orientation CAMPARAMS[5];
// void grid(Mat img)
// {
//   img = Mat::zeros(W, W, CV_8UC3);
//   int pixPM = W / gridM;
//   for (int i = 1; i < gridM; i++)
//   {
//     line(img, Point(1, i*(W / gridM)), Point(W, i*(W / gridM)), Scalar(0, 255, 255), 2, 8);
//     line(img, Point(i*(W / gridM), 1), Point(i*(W / gridM), W), Scalar(0, 255, 255), 2, 8);
//   }

  
// }
void enu_2_gym(nav_msgs::Odometry current_pose_enu)
{
  float GYM_OFFSET = gymOffset.data;
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  float X = x*cos(GYM_OFFSET*deg2rad) - y*sin(GYM_OFFSET*deg2rad);
  float Y = x*sin(GYM_OFFSET*deg2rad) + y*cos(GYM_OFFSET*deg2rad);
  float Z = z;
  current_pose.pose.pose.position.x = X;
  current_pose.pose.pose.position.y = Y;
  current_pose.pose.pose.position.z = Z;
  //ROS_INFO("pose gym x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
 
}
//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg) 
{
  nav_msgs::Odometry current_pose_enu = *msg;
  enu_2_gym(current_pose_enu);
  //ROS_INFO("pose enu x: %f y: %f z: %f", current_pose_enu.pose.pose.position.x, current_pose_enu.pose.pose.position.y, current_pose.pose.pose.position.z);
}
void gym_cb(const std_msgs::Float64::ConstPtr& msg)
{
  gymOffset = *msg;
  //ROS_INFO("current heading: %f", current_heading.data);
}
geometry_msgs::PoseStamped pixel2metric_facedown(double alt, vector<double> obj_pix, vector<double> &O_m, orientation camparamOfFrame)
{ //find puxel of interest in x direction
  THETA_Y = camparamOfFrame.pitch*(M_PI/180);
  THETA_Y = camparamOfFrame.roll*(M_PI/180);
  double T_x=obj_pix[0];
  double psi_x;
  double O_mx;
  double O_my;
  psi_x=2*abs(T_x/PIXELS[0]*PHI_X-PHI_X/2);
  if(T_x>PIXELS[0]/2){
     O_mx=alt*tan(THETA_X+psi_x/2);
  }else{ 
    O_mx=alt*tan(THETA_X-psi_x/2);
  }
  
  // find pixel of interest in y direction
  double T_y=obj_pix[1];

  double psi;
  //calculate slice of field of interest
  psi=2*abs(T_y/PIXELS[1]*PHI-PHI/2);
  

  //double O_mx = r_p[0]/3779.527;
  if(T_y<PIXELS[1]/2){
    O_my = alt*tan(THETA_Y+psi/2);
  }else{ 
    O_my=alt*tan(THETA_Y-psi/2);  
  }
  
  // O_m[0] = O_mx;
  // O_m[1] = O_my;
  ROS_INFO("transformed to x: %f y: %f meters \n", O_mx, O_my); 
  roombaPose.pose.position.x = O_mx + current_pose.pose.pose.position.x;
  roombaPose.pose.position.y = O_my + current_pose.pose.pose.position.y;
  roombaPose.pose.position.z = alt;
  //ROS_INFO("roombaPose gym x: %f y: %f z: %f", roombaPose.pose.position.x, roombaPose.pose.position.y, roombaPose.pose.position.z);
  return roombaPose;
 
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
  cout << boxesFound << endl;
  // get camera mounting angles 
  orientation frameParam;
  for(int i=0; i<=5; i++)
  {
    if(boxesFound.header.frame_id == CAMPARAMS[i].frame_id)
    {
      frameParam = CAMPARAMS[i];
      break;
    }
  } 

  ROS_INFO("camera ID %s", frameParam.frame_id.c_str());

  double xCenter;
  double yCenter;
  double alt = current_pose.pose.pose.position.z;
  int numDetections = boxesFound.boundingBoxes.size();
  transformations_ros::roombaPoses roombaPositions;
  transformations_ros::roombaPose roombaPoseMsg;
  for(int i=0; i < boxesFound.boundingBoxes.size(); i++)
  {
    //safety incase there are too many detections
    if(i>10)
    {
      break;
    }
    objectBounds = boxesFound.boundingBoxes[i];
    xCenter = (objectBounds.xmax + objectBounds.xmin)/2;
    yCenter = (objectBounds.ymax + objectBounds.ymin)/2;
    std::string objectType = objectBounds.Class;
    ROS_INFO("%s found x: %f y: %f pixels", objectType.c_str(), xCenter, yCenter); 

    vector<double> obj_pix;
    obj_pix.push_back(xCenter);
    obj_pix.push_back(yCenter);
    vector<double> O_m;
    
    //roombaPositions.roombaPoses.push_back();
    //[i].header.stamp = boxesFound.header.stamp;

    roombaPoseMsg.roombaPose = pixel2metric_facedown(alt, obj_pix, O_m, frameParam);
    roombaPose.header.stamp = boxesFound.header.stamp;
    roombaPositions.roombaPoses.push_back(roombaPoseMsg);
  }
  //boundingBoxesResults_.boundingBoxes.push_back(boundingBox);
  //chatter_pub.publish(roombaPositions);
  
  cout << roombaPositions << endl;

  
  
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "transformations");
  
  ros::NodeHandle n;
  ros::Subscriber currentPos = n.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
  ros::Subscriber sub2 = n.subscribe("/darknet_ros/bounding_boxes",1 ,centerPoint);
  ros::Subscriber sub = n.subscribe("/darknet_ros/found_object", 1, chatterCallback);
  ros::Subscriber gym_offset_sub = n.subscribe("/gymOffset", 1, gym_cb);
  ros::Publisher chatter_pub = n.advertise<transformations_ros::roombaPoses>("roombaPoses", 1000);


  orientation cam1, cam2, cam3, cam4, cam5;

  // get params
  if (!n.hasParam("/camera_params/cameras/cam1/orientation/roll"))
  {
    ROS_INFO("No param named 'cam1'");
  }else{
    ros::param::get("/camera_params/cameras/cam1/orientation/roll", cam1.roll);
    ros::param::get("/camera_params/cameras/cam1/orientation/pitch", cam1.pitch);
    ros::param::get("/camera_params/cameras/cam1/orientation/roll", cam1.yaw);
    ros::param::get("/camera_params/cameras/cam1/frame_id", cam1.frame_id);
  }
   if (!n.hasParam("/camera_params/cameras/cam2/orientation/roll"))
  {
    ROS_INFO("No param named 'cam2'");
  }else{
    ros::param::get("/camera_params/cameras/cam2/orientation/roll", cam2.roll);
    ros::param::get("/camera_params/cameras/cam2/orientation/pitch", cam2.pitch);
    ros::param::get("/camera_params/cameras/cam2/orientation/roll", cam2.yaw);
    ros::param::get("/camera_params/cameras/cam2/frame_id", cam2.frame_id);
  }
   if (!n.hasParam("/camera_params/cameras/cam3/orientation/roll"))
  {
    ROS_INFO("No param named 'cam3'");
  }else{
    ros::param::get("/camera_params/cameras/cam3/orientation/roll", cam3.roll);
    ros::param::get("/camera_params/cameras/cam3/orientation/pitch", cam3.pitch);
    ros::param::get("/camera_params/cameras/cam3/orientation/roll", cam3.yaw);
    ros::param::get("/camera_params/cameras/cam3/frame_id", cam3.frame_id);
  }
  cout << cam3.pitch << endl;
   if (!n.hasParam("/camera_params/cameras/cam4/orientation/roll"))
  {
    ROS_INFO("No param named 'cam4'");
  }else{
    ros::param::get("/camera_params/cameras/cam4/orientation/roll", cam4.roll);
    ros::param::get("/camera_params/cameras/cam4/orientation/pitch", cam4.pitch);
    ros::param::get("/camera_params/cameras/cam4/orientation/roll", cam4.yaw);
    ros::param::get("/camera_params/cameras/cam4/frame_id", cam4.frame_id);
  }
   if (!n.hasParam("/camera_params/cameras/cam5/orientation/roll"))
  {
    ROS_INFO("No param named 'cam5'");
  }else{
    ros::param::get("/camera_params/cameras/cam5/orientation/roll", cam5.roll);
    ros::param::get("/camera_params/cameras/cam5/orientation/pitch", cam5.pitch);
    ros::param::get("/camera_params/cameras/cam5/orientation/roll", cam5.yaw);
    ros::param::get("/camera_params/cameras/cam5/frame_id", cam5.frame_id);
  }
  
  CAMPARAMS[0] = cam1; 
  CAMPARAMS[1] = cam2;
  CAMPARAMS[2] = cam3;
  CAMPARAMS[3] = cam4;
  CAMPARAMS[4] = cam5;

  ROS_INFO("Parameters loaded");
  cout << "cam 1 id \n" << CAMPARAMS[1].frame_id.c_str() << endl;
  //ROS_INFO("Camera orientation roll: %f pitch: %f yaw: %f", camRoll, camPitch, camYaw);
  ROS_INFO("Node Started");

// Mat grid = Mat::zeros(W, W, CV_8UC3);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    
    chatter_pub.publish(roombaPose);

    ros::spinOnce();

    loop_rate.sleep();
    // grid(grid);
    // imshow("positions", grid);

  }







  return 0;
}



