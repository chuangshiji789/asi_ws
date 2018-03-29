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
#include <std_msgs/Float32.h>

// ROS message containers
  // Joy
  sensor_msgs::Joy joy_msg;

  // Ackermann
  ackermann_msgs::AckermannDrive ackermann_msg;
  // Elevator
  std_msgs::Float32 elevator_msg;
  // Rail
  std_msgs::Float32 rail_msg;
  // Swivel
  std_msgs::Float32 swivel_msg;

  // Callbacks
  void joy_cb(const sensor_msgs::Joy::ConstPtr& msg)
  {
    joy_msg = *msg;
    ackermann_msg.speed = joy_msg.axes[3] * 25.4; // right stick up down
    ackermann_msg.steering_angle = joy_msg.axes[2] * 90.0;  // right stick left right

    // Check command direction for elevator
    elevator_msg.data = joy_msg.axes[1] * 0.5;

    // Rail command
    if (joy_msg.buttons[5] && !joy_msg.buttons[4])
    {
      rail_msg.data = -1;
    }
    else if (!joy_msg.buttons[5] && joy_msg.buttons[4])
    {
      rail_msg.data = 1;
    }
    else
    {
      rail_msg.data = 0;  // stop elevator if both or no commands issued
    }

    // Swivel command
    swivel_msg.data = joy_msg.axes[0];

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
      // Elevator publisher
        ros::Publisher elevator_pub = nh.advertise<std_msgs::Float32>
          ("elevator_cmd", 1);
      // Rail publisher
        ros::Publisher rail_pub = nh.advertise<std_msgs::Float32>
          ("rail_cmd", 1);
      // Swivel publisher
        ros::Publisher swivel_pub = nh.advertise<std_msgs::Float32>
          ("swivel_cmd", 1);
      // Loop timing
        ros::Rate rate(20.0);

      // Main Loop
        while(ros::ok())
        {
          ros::spinOnce();
          ackermann_pub.publish(ackermann_msg);
          elevator_pub.publish(elevator_msg);
          rail_pub.publish(rail_msg);
          swivel_pub.publish(swivel_msg);
          // printf("message: %.6f  %.6f \n", ackermann_msg.speed, ackermann_msg.steering_angle);

          rate.sleep();
        }
  }
