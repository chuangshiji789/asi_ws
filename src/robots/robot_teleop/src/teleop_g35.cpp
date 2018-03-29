/** Notes: ros::Rate depends on ROS time. When Gazebo is launched with sim_time = true, ROS time
           becomes Gazebo time. When Gazebo is not running, ROS time is undefined, so ros::Rate will
           not behave as expected.
**/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <unistd.h>

// ROS headers
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/Joy.h>

// ROS message containers
  // Joy
  sensor_msgs::Joy joy_msg;

  // Ackermann
  ackermann_msgs::AckermannDrive ackermann_msg;

  // Callbacks
  void joy_cb(const sensor_msgs::Joy::ConstPtr& msg)
  {
    joy_msg = *msg;
    ackermann_msg.speed = joy_msg.axes[1] * 30.0;
    ackermann_msg.steering_angle = joy_msg.axes[2] * 45.0;
    if (joy_msg.buttons[7] > 0.1)
    {
      ackermann_msg.acceleration = -1.0;
    }
    else
    {
      ackermann_msg.acceleration = 0.0;
    }

  }

  int main(int argc, char **argv)
  {
    // Initialize ROS node
    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle nh;

    // Subscribers and publishers
      // Joy Subscriber
        ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>
          ("joy", 1, joy_cb);

      // Ackermann publisher
        ros::Publisher ackermann_pub = nh.advertise<ackermann_msgs::AckermannDrive>
          ("ackermann_cmd", 1);

      // Loop timing
        ros::Rate rate(10.0);

      // Main Loop
        while(ros::ok())
        {
          ros::spinOnce();
          ackermann_pub.publish(ackermann_msg);
          // printf("message: %.6f  %.6f \n", ackermann_msg.speed, ackermann_msg.steering_angle);

          rate.sleep();
        }
  }
