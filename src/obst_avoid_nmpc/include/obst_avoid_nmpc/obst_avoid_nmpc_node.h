/*
  \file     obst_avoid_nmpc_v2_node.h
  \brief    Header file for Obstacle Avoidance NMPC ROS wrapper.

  \author   Stephen Geiger <sag0020@tigermail.auburn.edu>
  \date     May 9, 2017
*/

#ifndef OBST_AVOID_NODE_H
#define OBST_AVOID_NODE_H

// ROS Includes
#include <ros/ros.h>
#include <ros/time.h>

// C++ Includes
#include <math.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <array>

// Eigen Package include
#include <eigen3/Eigen/Dense>

// Message Includes
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <loosely_coupled_ekf/LooselyCoupledEstimate.h>
#include <wgs_conversions/WgsConversion.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>

// Other Includes
#include "obst_avoid_nmpc.h"

// Define PI and gpsPI
#ifndef PI
#define PI 3.14159265358979
#endif


class ObstAvoidNode
{
public:
	ObstAvoidNode();
	~ObstAvoidNode(){};

	void odom_gazebo_callback(const nav_msgs::Odometry& msg);
	void refLLA_callback(const geometry_msgs::Vector3Stamped& msg);
	void ekf_callback(const loosely_coupled_ekf::LooselyCoupledEstimate& msg);
	void steer_callback(const std_msgs::Float64& msg);
	void obstacle_callback(const geometry_msgs::PoseArray& msg);
	void Run();

	wgs_conversions::WgsConversion wgs_conv;
	ros::ServiceClient lla2enu_client;
	ros::Subscriber odomSub;
	ros::Subscriber refLLASub;
	ros::Subscriber ekfSub;
	ros::Subscriber steerSub;
	ros::Subscriber obstSub;
	ros::Publisher controlPub;
	ObstAvoidNMPC NMPC;
	bool PROVIDE_TARGET_LLA;

private:
	Eigen::Vector2d pos;
	Eigen::Vector2d vel_n;
	Eigen::Vector2d vel_b;
	double yaw;
	double yaw_rate;
	double roll;
	double roll_rate;
	double pitch;

	Eigen::VectorXd X;
	Eigen::VectorXd u;
	Eigen::VectorXd u_last;
	Eigen::VectorXd dist_obst;
	double dist_tar;
	double err_yaw;


};











#endif
