// Transformation between the facedown camera frame and the drone frame
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <sensor_msgs/Image.h>

#include "transformations_ros/objects.h"
#include "transformations_ros/object.h"
#include "sensor_msgs/Imu.h"


//field of view y
const double PHI_Y = (32.751868*M_PI/180);
//field of view in x
const double PHI_X = (43.08195*M_PI/180);

double PIXELS[2] = {640, 480};


using namespace std;
nav_msgs::Odometry current_pose_g;
std_msgs::Float64 local_offset_msg;
transformations_ros::objects roombaPositions1;
sensor_msgs::Imu IMU;
std_msgs::Float64 current_heading;
float current_heading_g;
float local_offset_g = 0;

struct orientation
{
  float roll;
  float pitch;
  float yaw;
  string frame_id;
};
std::vector<orientation> cam_params_g;
orientation current_orientation;

void image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
 sensor_msgs::Image detection_image;
 detection_image = *msg;
 PIXELS[0] = detection_image.height;
 PIXELS[1] = detection_image.width;
}
void enu_2_local(nav_msgs::Odometry current_pose_enu)
{
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  float X = x*cos(local_offset_g*deg2rad) - y*sin(local_offset_g*deg2rad);
  float Y = x*sin(local_offset_g*deg2rad) + y*cos(local_offset_g*deg2rad);
  float Z = z;
  //ROS_INFO("Local position %f %f %f",X, Y, Z);
}
void enu_2_gym(nav_msgs::Odometry current_pose_enu)
{
  float local_offset = local_offset_msg.data;
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  float X = x*cos(local_offset*deg2rad) - y*sin(local_offset*deg2rad);
  float Y = x*sin(local_offset*deg2rad) + y*cos(local_offset*deg2rad);
  float Z = z;
  current_pose_g.pose.pose.position.x = X;
  current_pose_g.pose.pose.position.y = Y;
  current_pose_g.pose.pose.position.z = Z;


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

}
//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  nav_msgs::Odometry current_pose_enu = *msg;
  enu_2_gym(current_pose_enu);

  float q0 = current_pose_enu.pose.pose.orientation.w;
  float q1 = current_pose_enu.pose.pose.orientation.x;
  float q2 = current_pose_enu.pose.pose.orientation.y;
  float q3 = current_pose_enu.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
  //Heading is in ENU
  current_heading_g = psi*(180/M_PI) - local_offset_g;

  //ROS_INFO("pose enu x: %f y: %f z: %f", current_pose_enu.pose.pose.position.x, current_pose_enu.pose.pose.position.y, current_pose_g.pose.pose.position.z);
}
void local_offset_cb(const std_msgs::Float64::ConstPtr& msg)
{
  local_offset_msg = *msg;
  //ROS_INFO("current heading: %f", current_heading.data);
}
geometry_msgs::PointStamped pixel2metric(double alt, vector<double> obj_pix, orientation camparamOfFrame)
{
  float theta_x_offset;
  float theta_y_offset;

  //TODO: make this mathematic
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
  //END TODO

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
	float local_offset = local_offset_msg.data;
	float deg2rad = (M_PI/180);
	float O_mx_drone = O_mx*cos(camparamOfFrame.yaw*deg2rad) - O_my*sin(camparamOfFrame.yaw*deg2rad);
	float O_my_drone = O_mx*sin(camparamOfFrame.yaw*deg2rad) + O_my*cos(camparamOfFrame.yaw*deg2rad);
  //ROS_INFO("cam params to x: %f y: %f meters \n", O_mx_drone, O_my_drone);
  //put point in gym reference frame
  float X = O_mx_drone*cos(-(current_heading_g-local_offset )*deg2rad) - O_my_drone*sin(-(current_heading_g-local_offset)*deg2rad);
  float Y = O_mx_drone*sin(-(current_heading_g-local_offset )*deg2rad) + O_my_drone*cos(-(current_heading_g-local_offset)*deg2rad);

	geometry_msgs::PointStamped roombaPose;
	roombaPose.point.x = X + current_pose_g.pose.pose.position.x;
	roombaPose.point.y = Y + current_pose_g.pose.pose.position.y;
	roombaPose.point.z = 0;
  //ROS_INFO("pose gym x: %f y: %f z: %f", current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, current_pose_g.pose.pose.position.z);
	ROS_INFO("roombaPose gym x: %f y: %f z: %f", roombaPose.point.x, roombaPose.point.y, roombaPose.point.z);
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
  // get camera mounting angles
  orientation frameParam;
  for(int i=0; i<cam_params_g.size(); i++)
  {
    if(boxesFound.image_header.frame_id == cam_params_g[i].frame_id)
    {
      frameParam = cam_params_g[i];
      break;
    }
  }

  ROS_INFO("camera ID %s", frameParam.frame_id.c_str());

  double xCenter;
  double yCenter;
  double alt = current_pose_g.pose.pose.position.z;
  int numDetections = boxesFound.bounding_boxes.size();
  transformations_ros::objects roombaPositions;
  transformations_ros::object roombaPoseMsg;

  //apply transformation to each detection and repackage for stratnode
  for(int i=0; i < boxesFound.bounding_boxes.size(); i++)
  {
    //safety incase there are too many detections
    if(i>100)
    {
      break;
    }
    objectBounds = boxesFound.bounding_boxes[i];
    xCenter = (objectBounds.xmax + objectBounds.xmin)/2;
    yCenter = (objectBounds.ymax + objectBounds.ymin)/2;
    std::string objectType = objectBounds.Class;
    ROS_INFO("%s found x: %f y: %f pixels", objectType.c_str(), xCenter, yCenter);

    vector<double> obj_pix;
    obj_pix.push_back(xCenter);
    obj_pix.push_back(yCenter);

    roombaPoseMsg.Class = boxesFound.bounding_boxes[i].Class;
    roombaPoseMsg.position = pixel2metric(alt, obj_pix, frameParam);
    roombaPoseMsg.position.header.stamp = boxesFound.header.stamp;
    roombaPoseMsg.position.header.frame_id = frameParam.frame_id;
    roombaPositions.objects.push_back(roombaPoseMsg);
  }
  roombaPositions1 = roombaPositions;



}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "transformations");

  ros::NodeHandle n;
  ros::Subscriber currentPos = n.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, pose_cb);
  ros::Subscriber currentHeading = n.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg", 1, heading_cb);
  ros::Subscriber currentIMU = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, imu_cb);
  ros::Subscriber detectionImage = n.subscribe("/darknet_ros/detection_image", 1, image_cb);
  ros::Subscriber sub2 = n.subscribe("/darknet_ros/bounding_boxes",1 ,centerPoint);
  ros::Subscriber sub = n.subscribe("/darknet_ros/found_object", 1, chatterCallback);
  ros::Subscriber local_offset_sub = n.subscribe("/controlnode/local_offset", 1, local_offset_cb);
  ros::Publisher chatter_pub = n.advertise<transformations_ros::objects>("objects", 10);

  int numOfCams = 0;
  //figure out how many cameras are on aircraft
  string prefix = "/camera_params/cameras/";
  if (!n.hasParam((prefix + "numOfCams").c_str()))
  {
    ROS_INFO("No param named numOfCams");
  }else{
    ros::param::get((prefix + "numOfCams").c_str(), numOfCams);
    ROS_INFO("%d cameras on aircraft", numOfCams);
  }

  //load the camera intrinsics 
  string param_path;
  orientation cam;
  for( int i=1; i<=numOfCams; i++)
  {
    param_path = "";
    param_path = prefix + "cam" + std::to_string(i);
    // get params
    if (!n.hasParam( (param_path ).c_str() ) )
    {
      ROS_ERROR("No param named %s", param_path.c_str());
      return -1;
    }else{
      ros::param::get((param_path + "/orientation/roll").c_str() , cam.roll);
      ros::param::get((param_path + "/orientation/pitch").c_str() , cam.pitch);
      ros::param::get((param_path + "/orientation/yaw").c_str() , cam.yaw);
      ros::param::get((param_path + "/frame_id").c_str() , cam.frame_id);
      cam_params_g.push_back(cam);

      ROS_INFO("%s Parameters loaded", param_path.c_str());
    }
  }
  
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



