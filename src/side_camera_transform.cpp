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
#include "sensor_msgs/Imu.h"

int W = 1000; // width of gui
int gridM = 20; // size of grid in meters


//field of view y
const double PHI_Y = (32.751868*M_PI/180);
//field of view in x
const double PHI_X = (43.08195*M_PI/180);

const double PIXELS[2] = {640, 480};


using namespace std;
nav_msgs::Odometry current_pose;
std_msgs::Float64 gymOffset;
transformations_ros::roombaPoses roombaPositions1;
sensor_msgs::Imu IMU;
std_msgs::Float64 current_heading;

struct orientation
{
  float roll;
  float pitch;
  float yaw;
  string frame_id;
};
orientation CAMPARAMS[5];
orientation current_orientation;


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
void heading_cb(const std_msgs::Float64::ConstPtr& msg)
{
  current_heading = *msg;
  //ROS_INFO("current heading: %f", current_heading.data);
}
//get current orientation of drone
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  IMU = *msg;
  //cout << IMU << endl;

  float q0 = IMU.orientation.w; //w
  float q1 = IMU.orientation.x; // x
  float q2 = IMU.orientation.y; // y
  float q3 = IMU.orientation.z; // z

  float phi = atan2((2*(q0*q1 + q2*q3)), (1 - 2*(pow(q1,2) + pow(q2,2))) ); // roll
  float theta = asin( 2*(q0*q2 - q3*q1) ); // pitch
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

  current_orientation.roll=phi;
  current_orientation.pitch=theta;
  current_orientation.yaw=psi;


  cout<<"roll :" << phi*(180/M_PI) << " pitch : " << theta*(180/M_PI) << " yaw : " << psi*(180/M_PI) << endl;

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
geometry_msgs::PoseStamped pixel2metric_facedown(double alt, vector<double> obj_pix, orientation camparamOfFrame)
{
float theta_x_offset;
float theta_y_offset;

  //use drone attitude to modify camera orientation angles
  if(camparamOfFrame.frame_id=="camera_link"){theta_x_offset=-current_orientation.roll;
                                            theta_y_offset=-current_orientation.pitch;

  }else if(camparamOfFrame.frame_id=="camera_link2"){theta_x_offset=-current_orientation.pitch;
                                                    theta_y_offset=current_orientation.roll;
  
  }else if(camparamOfFrame.frame_id== "camera_link3"){theta_x_offset=current_orientation.roll;
                                                      theta_y_offset=current_orientation.pitch;                                                
  }else if(camparamOfFrame.frame_id== "camera_link4"){theta_x_offset=current_orientation.pitch;
                                                      theta_y_offset=-current_orientation.roll;
  }else if(camparamOfFrame.frame_id=="camera_link5"){theta_x_offset=-current_orientation.roll;
                                                    theta_y_offset=-current_orientation.pitch;}
    

  
  //find pixel of interest in x direction
	double THETA_Y = camparamOfFrame.pitch*(M_PI/180)+theta_y_offset;
	double THETA_X = camparamOfFrame.roll*(M_PI/180)+theta_x_offset;
	double T_x=obj_pix[0];
	double psi_x;
	double O_mx;

	psi_x=2*abs(T_x/PIXELS[0]*PHI_X-PHI_X/2);
    // find pixel of interest in y direction
  double T_y=obj_pix[1];
  double O_my;
  double psi_y;
  double hyp_x;
  double hyp_y;
  //calculate slice of field of interest
  psi_y=2*abs(T_y/PIXELS[1]*PHI_Y-PHI_Y/2);
  double gamma_y;
  double gamma_x;
  //double O_mx = r_p[0]/3779.527;
  if(T_y<PIXELS[1]/2){
  gamma_y= THETA_Y+psi_y/2; 
  }else{ 
  gamma_y= THETA_Y-psi_y/2;  
  }
  hyp_x=alt/cos(gamma_y);
  if(T_x>PIXELS[0]/2){
  gamma_x=THETA_X+psi_x/2;
  }else{ 
  gamma_x=THETA_X-psi_x/2;
  }
  hyp_y=alt/cos(gamma_x);
  
  O_mx=hyp_x*tan(gamma_x);
  O_my = alt*tan(gamma_y);


	ROS_INFO("transformed to x: %f y: %f meters \n", O_mx, O_my); 
	

	//put transformed point in drone reference frame
	float GYM_OFFSET = gymOffset.data;
	float deg2rad = (M_PI/180);
	float O_mx_drone = O_mx*cos(camparamOfFrame.yaw*deg2rad) - O_my*sin(camparamOfFrame.yaw*deg2rad);
	float O_my_drone = O_mx*sin(camparamOfFrame.yaw*deg2rad) + O_my*cos(camparamOfFrame.yaw*deg2rad);

  //put point in gym reference frame
  float X = O_mx_drone*cos(-(GYM_OFFSET+current_heading.data)*deg2rad) - O_my_drone*sin(-(GYM_OFFSET+current_heading.data)*deg2rad);
  float Y = O_mx_drone*sin(-(GYM_OFFSET+current_heading.data)*deg2rad) + O_my_drone*cos(-(GYM_OFFSET+current_heading.data)*deg2rad);

	geometry_msgs::PoseStamped roombaPose;
	roombaPose.pose.position.x = X + current_pose.pose.pose.position.x;
	roombaPose.pose.position.y = Y + current_pose.pose.pose.position.y;
	roombaPose.pose.position.z = 0;
	ROS_INFO("roombaPose gym x: %f y: %f z: %f", roombaPose.pose.position.x, roombaPose.pose.position.y, roombaPose.pose.position.z);
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
  //cout << boxesFound << endl;
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

  //apply transformation to each detection and repackage for stratnode
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
    

    roombaPoseMsg.roombaPose = pixel2metric_facedown(alt, obj_pix, frameParam); 
    roombaPoseMsg.roombaPose.header.stamp = boxesFound.header.stamp;
    roombaPoseMsg.roombaPose.header.frame_id = frameParam.frame_id;
    roombaPositions.roombaPoses.push_back(roombaPoseMsg);
  }
  //cout << roombaPositions << endl;
  roombaPositions1 = roombaPositions;

  
  
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "transformations");
  
  ros::NodeHandle n;
  ros::Subscriber currentPos = n.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 1, pose_cb);
  ros::Subscriber currentHeading = n.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg", 1, heading_cb);
  ros::Subscriber currentIMU = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, imu_cb);
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
    ros::param::get("/camera_params/cameras/cam1/orientation/yaw", cam1.yaw);
    ros::param::get("/camera_params/cameras/cam1/frame_id", cam1.frame_id);
    ROS_INFO("Camera 1 Parameters loaded");
  }
   if (!n.hasParam("/camera_params/cameras/cam2/orientation/roll"))
  {
    ROS_INFO("No param named 'cam2'");
  }else{
    ros::param::get("/camera_params/cameras/cam2/orientation/roll", cam2.roll);
    ros::param::get("/camera_params/cameras/cam2/orientation/pitch", cam2.pitch);
    ros::param::get("/camera_params/cameras/cam2/orientation/yaw", cam2.yaw);
    ros::param::get("/camera_params/cameras/cam2/frame_id", cam2.frame_id);
    ROS_INFO("Camera 2 Parameters loaded");
  }
   if (!n.hasParam("/camera_params/cameras/cam3/orientation/roll"))
  {
    ROS_INFO("No param named 'cam3'");
  }else{
    ros::param::get("/camera_params/cameras/cam3/orientation/roll", cam3.roll);
    ros::param::get("/camera_params/cameras/cam3/orientation/pitch", cam3.pitch);
    ros::param::get("/camera_params/cameras/cam3/orientation/yaw", cam3.yaw);
    ros::param::get("/camera_params/cameras/cam3/frame_id", cam3.frame_id);
    ROS_INFO("Camera 3 Parameters loaded");
  }
   if (!n.hasParam("/camera_params/cameras/cam4/orientation/roll"))
  {
    ROS_INFO("No param named 'cam4'");
  }else{
    ros::param::get("/camera_params/cameras/cam4/orientation/roll", cam4.roll);
    ros::param::get("/camera_params/cameras/cam4/orientation/pitch", cam4.pitch);
    ros::param::get("/camera_params/cameras/cam4/orientation/yaw", cam4.yaw);
    ros::param::get("/camera_params/cameras/cam4/frame_id", cam4.frame_id);
    ROS_INFO("Camera 4 Parameters loaded");
  }
   if (!n.hasParam("/camera_params/cameras/cam5/orientation/roll"))
  {
    ROS_INFO("No param named 'cam5'");
  }else{
    ros::param::get("/camera_params/cameras/cam5/orientation/roll", cam5.roll);
    ros::param::get("/camera_params/cameras/cam5/orientation/pitch", cam5.pitch);
    ros::param::get("/camera_params/cameras/cam5/orientation/yaw", cam5.yaw);
    ros::param::get("/camera_params/cameras/cam5/frame_id", cam5.frame_id);
    ROS_INFO("Camera 5 Parameters loaded");
  }
  
  CAMPARAMS[0] = cam1; 
  CAMPARAMS[1] = cam2;
  CAMPARAMS[2] = cam3;
  CAMPARAMS[3] = cam4;
  CAMPARAMS[4] = cam5;

  ROS_INFO("Parameters loaded");
  ROS_INFO("Node Started");

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    
    chatter_pub.publish(roombaPositions1);

    ros::spinOnce();

    loop_rate.sleep();

  }







  return 0;
}



