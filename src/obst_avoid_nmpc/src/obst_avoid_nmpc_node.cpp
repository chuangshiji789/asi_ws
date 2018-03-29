/*
  \file     obst_avoid_nmpc_node_v2.cpp
  \brief    C++ file for Obstacle Avoidance NMPC ROS wrapper.

  \author   Stephen Geiger <sag0020@tigermail.auburn.edu>
  \date     May 9, 2017
*/

#include "obst_avoid_nmpc_node.h"
#include <tf/transform_datatypes.h>

void HandleInfoMessages(const std::string &msg)
{
    ROS_INFO("%s", msg.c_str());
};

void HandleWarningMessages(const std::string &msg)
{
  ROS_WARN("%s", msg.c_str());
};

void HandleErrorMessages(const std::string &msg)
{
  ROS_ERROR("%s", msg.c_str());
};

void HandleDebugMessages(const std::string &msg)
{
  ROS_DEBUG("%s", msg.c_str());
};


ObstAvoidNode::ObstAvoidNode(): X(5), u(4), u_last(4)
{
	// Create Private Node Handle
  ros::NodeHandle nh;
	ros::NodeHandle nhPvt("~");

	// Parameter Assignment from Launch file
  if (
      !nhPvt.getParam("target_POS", NMPC.target_POS) ||
      !nhPvt.getParam("prediction_horizon", NMPC.t_h) ||
      !nhPvt.getParam("max_velocity", NMPC.vel_max) ||
      !nhPvt.getParam("avoidance_radius", NMPC.r_avoid) ||
      !nhPvt.getParam("VEHICLE_ID", NMPC.VEHICLE_ID) ||
      !nhPvt.getParam("gamma", NMPC.gamma) ||
      !nhPvt.getParam("zeta", NMPC.zeta) ||
      !nhPvt.getParam("kappa", NMPC.kappa) ||
      !nhPvt.getParam("epsilon", NMPC.epsilon) ||
      !nhPvt.getParam("ARE_OBSTACLES_COST_FUNCTION", NMPC.ARE_OBSTACLES_COST_FUNCTION) ||
      !nhPvt.getParam("NO_OBSTACLES_COST_FUNCTION", NMPC.NO_OBSTACLES_COST_FUNCTION) ||
      !nhPvt.getParam("PROVIDE_TARGET_LLA", PROVIDE_TARGET_LLA))
  {
    ROS_ERROR("Could not get all NMPC parameters");
  }

  odomSub = nh.subscribe("odom_truth_asdf", 10, &ObstAvoidNode::odom_gazebo_callback, this);
  if (PROVIDE_TARGET_LLA == true) {
    NMPC.initTarget == false;
    refLLASub = nh.subscribe("EKF/reflla", 10, &ObstAvoidNode::refLLA_callback, this);
  } else {
    NMPC.initTarget == true;
    NMPC.target(0) = NMPC.target_POS[0];
    NMPC.target(1) = NMPC.target_POS[1];
    NMPC.target(2) = NMPC.target_POS[2];
  }
  ekfSub = nh.subscribe("EKF/estimate", 10, &ObstAvoidNode::ekf_callback, this);
  steerSub = nh.subscribe("current_steering_angle", 10, &ObstAvoidNode::steer_callback, this);
  obstSub = nh.subscribe("object_estimated_poses", 10, &ObstAvoidNode::obstacle_callback, this);
  controlPub = nh.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd",10);
  // controlPub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("NMPC/desired",10);
  lla2enu_client = nh.serviceClient<wgs_conversions::WgsConversion>("lla2enu");

  u << 0, 0, 0, 0;
  u_last << 0, 0, 0, 0;
  NMPC.obstacle.ARE_OBSTACLES = false;

}

void ObstAvoidNode::odom_gazebo_callback(const nav_msgs::Odometry& msg)
{
  pos << -msg.pose.pose.position.y, msg.pose.pose.position.x;
  vel_n << -msg.twist.twist.linear.y, msg.twist.twist.linear.x;

  tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  m.getRPY(roll,pitch,yaw);

  yaw = yaw + PI/2;

  yaw_rate = msg.twist.twist.angular.z;

  // Transform Velocity to body frame
  vel_b(0) =  vel_n(0)*cos(yaw) + vel_n(1)*sin(yaw);
  vel_b(1) = -vel_n(0)*sin(yaw) + vel_n(1)*cos(yaw);

  u_last(1) = vel_n(0);
  u_last(2) = vel_n(1);
  u_last(3) = vel_b(0);
  X << pos(0), pos(1), yaw, yaw_rate, vel_b(1);
  NMPC.initState = true;
}

void ObstAvoidNode::ekf_callback(const loosely_coupled_ekf::LooselyCoupledEstimate& msg)
{
  // Total Roll Angle
  roll = msg.state.roll;
  // Unbiased Roll Rate
  roll_rate = msg.inputs.gx;
  // Vehicle heading
  yaw = msg.state.yaw;
  // Unbiased Yaw Rate
  yaw_rate = msg.inputs.gz;

  if (msg.header.frame_id == "ENU")
  {
    // Position and Velocity in ENU
    pos << msg.state.x, msg.state.y;
    vel_n << msg.state.vx, msg.state.vy;
  } else if (msg.header.frame_id == "NED") {
    // Position and Velocity changed to ENU
    pos << msg.state.y, msg.state.x;
    vel_n << msg.state.vy, msg.state.vx;
    // Vehicle heading changed to ENU
    if (yaw >= 0 && yaw < PI/2)
    {
      yaw = PI/2 - yaw;
    } else {
      yaw = 5*PI/2 - yaw;
    }
    // Unbiased Yaw Rate
    yaw_rate = -yaw_rate;
  }
  // Transform Velocity to body frame
  vel_b(0) = vel_n(0)*cos(yaw) + vel_n(1)*sin(yaw);
  vel_b(1) = vel_n(1)*cos(yaw) - vel_n(0)*sin(yaw);

  u_last(1) = vel_n(0);
  u_last(2) = vel_n(1);
  u_last(3) = vel_b(0);
  X << pos(0), pos(1), yaw, yaw_rate, vel_b(1);
  NMPC.initState = true;

}

void ObstAvoidNode::refLLA_callback(const geometry_msgs::Vector3Stamped& msg)
{
  NMPC.init_lla = {msg.vector.x, msg.vector.y, msg.vector.z};
  // Server Requested Information
  wgs_conv.request.lla[0] = NMPC.target_POS[0];
  wgs_conv.request.lla[1] = NMPC.target_POS[1];
  wgs_conv.request.lla[2] = NMPC.target_POS[2];
  wgs_conv.request.ref_lla[0] = NMPC.init_lla[0];
  wgs_conv.request.ref_lla[1] = NMPC.init_lla[1];
  wgs_conv.request.ref_lla[2] = NMPC.init_lla[2];
  // Server Response
  if (lla2enu_client.call(wgs_conv)){
    NMPC.target_n[0] = wgs_conv.response.enu[0];
    NMPC.target_n[1] = wgs_conv.response.enu[1];
    NMPC.target_n[2] = wgs_conv.response.enu[2];
    NMPC.target(0) = NMPC.target_n[0];
    NMPC.target(1) = NMPC.target_n[1];
    NMPC.target(2) = NMPC.target_n[2];
    NMPC.initTarget = true;
  } else {
    ROS_ERROR("Failed to call service lla2enu for target location");
  }
}

void ObstAvoidNode::steer_callback(const  std_msgs::Float64& msg)
{
  u_last(0) = msg.data;
}

void ObstAvoidNode::obstacle_callback(const geometry_msgs::PoseArray& msg)
{
  // There are Obstacles
  NMPC.obstacle.NUM_OBSTACLES = msg.poses.size();
  if (NMPC.obstacle.NUM_OBSTACLES == 0)
  {
    NMPC.obstacle.ARE_OBSTACLES = false;
  } else {
    NMPC.obstacle.ARE_OBSTACLES = true;
  }

  Eigen::MatrixXd obst_pos(NMPC.obstacle.NUM_OBSTACLES,2);
  Eigen::MatrixXd obst_vel(NMPC.obstacle.NUM_OBSTACLES,2);

  for (int i=0; i<NMPC.obstacle.NUM_OBSTACLES; i++)
  {
    obst_pos(i,0) = msg.poses[i].position.x;
    obst_pos(i,1) = msg.poses[i].position.y;
    obst_vel(i,0) = msg.poses[i].orientation.x;
    obst_vel(i,1) = msg.poses[i].orientation.y;
  }
  NMPC.obstacle.pos = obst_pos;
  NMPC.obstacle.vel = obst_vel;
}

void ObstAvoidNode::Run()
{
  ros::spinOnce();
  // If the state is initialized (via the EKF or the Odom msg) and the target position
  // is initialized (via the reference LLA or by sending the ENU position), then perform the following
  if (NMPC.initTarget && NMPC.initState)
  {
    ackermann_msgs::AckermannDrive desired_control;

    // Calculate distance to target and the yaw error in the body frame
    dist_tar = pow(pow(NMPC.target(0)-X(0),2)+pow(NMPC.target(1)-X(1),2),0.5);
		err_yaw = atan2((NMPC.target(1)-X(1))*cos(X(2))-(NMPC.target(0)-X(0))*sin(X(2)),(NMPC.target(1)-X(1))*sin(X(2))+(NMPC.target(0)-X(0))*cos(X(2)));

    // If the distance to the target is less than 20m, the prediction horizon is reduced based on the max allowed velocity (vel_max).
    if (dist_tar < 20)
    {
      NMPC.t_h = dist_tar/NMPC.vel_max;
    }

    // If the vehicle is more than 4m away from the target, the optimization routine is performed.
    if (dist_tar > 4.0)
    {
      u = NMPC.optimization(X, u, u_last, NMPC.t_h, 0.1, NMPC.r_avoid, NMPC.target, NMPC.obstacle);
    } else {
    // Otherwise, set steer and velocity to zero.
      u << 0, 0, 0, 0;
    }
    // If the distance to the target is between 4m and 20m and the yaw error between the vehicle
    // and the target is less than 90, the vehicle speed is set to 4m/s with zero steer.
    if (dist_tar > 4.0 && dist_tar < 20.0 && fabs(err_yaw) < PI/2)
    {
      u << 0, 0, 0, 4;
    }

    // ****This is the emergency brake/throttle stuff****
    // if (NMPC.obstacle.ARE_OBSTACLES)
    // {
    //   // std::cout << "Distances to Obstacles: " << std::endl;
    //   for (int i=0; i < NMPC.obstacle.NUM_OBSTACLES; i++)
    //   {
    //     NMPC.dist_obst(i) = pow(pow(NMPC.obstacle.pos(i,0),2) + pow(NMPC.obstacle.pos(i,1),2), 0.5);
    //       // std::cout << "Calculated distance to obstacles" << std::endl;
    //       // std::cout << "Obs #" << i+1 << ": " << NMPC.dist_obst(i) << std::endl;
    //     if ((NMPC.dist_obst(i) < NMPC.r_avoid) && (NMPC.obstacle.pos(i,0) > 0))
    //     {
    //       std::cout << "HIT BRAKE!!!!" << std::endl;
    //       u(0) = 0;
    //       // desired_control.drive.acceleration = -1;
    //       desired_control.acceleration = -1;
    //     } else if ((NMPC.dist_obst(i) < NMPC.r_avoid) && (NMPC.obstacle.pos(i,0) < 0)) {
    //     std::cout << "MAX THROTTLE!!!!" << std::endl;
    //       u(0) = 0;
    //       // desired_control.drive.acceleration = 1;
    //       desired_control.acceleration = 1;
    //     }
    //   }
    // }

    // Assign to the output variable and publish.
    desired_control.steering_angle = u(0)*180/PI;
    desired_control.speed = u(3);
    controlPub.publish(desired_control);
    NMPC.obstacle.ARE_OBSTACLES = false;
  }

}

int main(int argc, char **argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "NMPC_node");

  ObstAvoidNode node;
  // Set controller update rate
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    node.Run();
    loop_rate.sleep();
  }
	return 0;
}
