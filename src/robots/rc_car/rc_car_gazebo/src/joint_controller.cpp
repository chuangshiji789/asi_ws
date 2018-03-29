#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <unistd.h>

// ROS headers
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// Control headers
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

using namespace std;

// Variables
double rosRate = 20;

// Joint States msg
    sensor_msgs::JointState joint_states_msg;

// Get Joint States message
    void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg){
        // Copy message
        joint_states_msg = *msg;

    }

int main(int argc, char **argv)
{
  // Initialize ROS node
      ros::init(argc, argv, "odom node");
      ros::NodeHandle nh;

  // Subscribers and publishers
      // Joint States subscriber
        ros::Subscriber joint_states_sub = nh.subscribe<sensor_msgs::JointState>
                ("joint_states", 1, joint_states_cb);

      // Odom publisher
        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>
                ("odom", 1);

  // Odometry variables
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

  // Timing
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

  // Loop timing
      ros::Rate rate(rosRate);   // 20Hz

  // Loop
    while (nh.ok())
    {
      // Check for messages
        ros::spinOnce();

      // Reset time
        current_time = ros::Time::now();

      // Update odom
        double dt = (current_time = last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;



    }


}
