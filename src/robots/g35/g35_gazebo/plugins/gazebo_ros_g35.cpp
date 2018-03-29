#ifndef _CONTROL_PLUGIN_HH_
#define _CONTROL_PLUGIN_HH_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <unistd.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>


// ROS
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ackermann_msgs/AckermannDrive.h"
#include <gazebo/msgs/msgs.hh>
#include <std_msgs/Float64.h>

// TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Odometry
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <g35_gazebo/SteeringParamsConfig.h>
#include <g35_gazebo/DriveParamsConfig.h>
#include <g35_gazebo/SuspensionParamsConfig.h>
#include <g35_gazebo/DriveBrakeParamsConfig.h>

// Ignition
#include <ignition/math/math.hh>

namespace gazebo
{

  enum {
      FRONT_LEFT,
      FRONT_RIGHT,
      REAR_LEFT,
      REAR_RIGHT,
  };

  enum OdomSource
  {
      ENCODER = 0,
      WORLD = 1,
  };

  /// \brief A plugin to control a Velodyne sensor.
  class ControlPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ControlPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Visuals
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(_model->GetWorld()->GetName());
        visPub = this->node->Advertise<msgs::Visual>("~/visual", 10);
        // Set the visual's name. This should be unique.
        // Set the visual's parent. This visual will be attached to the parent
        // std::string visual_link_name = "g35_robot1::visual_link";
        // Create a cylinder
        msgs::Geometry *geomMsg = visualMsg.mutable_geometry();
        geomMsg->set_type(msgs::Geometry::CYLINDER);
        geomMsg->mutable_cylinder()->set_radius(0.5);
        geomMsg->mutable_cylinder()->set_length(0.5);
        visualMsg.set_cast_shadows(false);
        this->visual_link = _model->GetLink("base_link");

        visualMsg.set_parent_name("g35_robot1");
        visualMsg.set_name("red_cylinder");
        msgs::Set(visualMsg.mutable_pose(), ignition::math::Pose3d(0, 0, 4, 0, 0, 0));
        msgs::Set(visualMsg.mutable_material()->mutable_diffuse(), common::Color(0.0, 0, 0, 1.0));
        msgs::Set(visualMsg.mutable_material()->mutable_emissive(), common::Color(0.0, 0.0, 0.0, 0.0));
        msgs::Set(visualMsg.mutable_material()->mutable_specular(), common::Color(0.1, 0.1, 0.1, 1.0));
        msgs::Set(visualMsg.mutable_material()->mutable_ambient(), common::Color(0.0, 0.0, 0.0, 1.0));
        bool state_on = true;
        if(state_on)
        visualMsg.set_visible(true);
        else
        visualMsg.set_visible(false);
        visPub->Publish(visualMsg);



        // gazebo::common::Color newColor(0.5, 0.5, 0.5, 1.0);
        // gazebo::msgs::Color *colorMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
        // gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);
        //
        // this->visual_link = _model->GetLink("left_rear_wheel");
        // std::cout << "LINK NAME " << visual_link << std::endl;
        // sdf::ElementPtr linkSDF = link_test->GetSDF();

        // if (!linkSDF)
        // {
        //   // std::cout << "Link had NULL SDF" << std::endl;
        //   // gzerr << "Link had NULL SDF" << std::endl;
        //   // return;
        // }
        // if (linkSDF->HasElement("visual"))
        // {
          // for (sdf::ElementPtr visualSDF = linkSDF->GetElement("visual");
          //      visualSDF; visualSDF = linkSDF->GetNextElement("visual"))
          // {
          //   GZ_ASSERT(visualSDF->HasAttribute("name"), "Malformed visual element!");
          //   std::string visualName = visualSDF->Get<std::string>("name");
          //   gazebo::msgs::Visual visMsg;
          //   visMsg = visual_link->GetVisualMessage(visualName);
          //   if ((!visMsg.has_material()) || visMsg.mutable_material() == NULL)
          //   {
          //     gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;
          //     visMsg.set_allocated_material(materialMsg);
          //   }
          //   gazebo::msgs::Material *materialMsg = visMsg.mutable_material();
          //   if (materialMsg->has_ambient())
          //   {
          //     materialMsg->clear_ambient();
          //   }
          //   materialMsg->set_allocated_ambient(colorMsg);
          //   if (materialMsg->has_diffuse())
          //   {
          //     materialMsg->clear_diffuse();
          //   }
          //   visMsg.set_name(visual_link->GetScopedName());
          //   visMsg.set_parent_name(_model->GetScopedName());
          //   materialMsg->set_allocated_diffuse(diffuseMsg);
          //   visPub->Publish(visMsg);
          // }
        // }





      // Just output a message for now
      std::cerr << "\nThe control plugin is attach to model[" <<
        _model->GetName() << "]\n";

      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Control plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the wheel links
      this->drive_wheel_left = model->GetLink("left_rear_wheel");
      this->drive_wheel_right = model->GetLink("right_rear_wheel");
      this->free_wheel_left = model->GetLink("left_front_wheel");
      this->free_wheel_right = model->GetLink("right_front_wheel");


      // Get the wheel joints
      this->joint_drive_left = _model->GetJoint("left_rear_axle");
      this->joint_drive_right = _model->GetJoint("right_rear_axle");
      this->joint_free_left = _model->GetJoint("left_front_axle");
      this->joint_free_right = _model->GetJoint("right_front_axle");

      // Get the steering links
      this->link_steer_left = _model->GetLink("steer_link_front_left");
      this->link_steer_right = _model->GetLink("steer_link_front_right");

      // Get the steering joints
      this->joint_steer_left = _model->GetJoint("left_steering_joint");
      this->joint_steer_right = _model->GetJoint("right_steering_joint");

      // Get the suspension joints
      joint_shock.resize(4);

      joint_shock[FRONT_LEFT] = _model->GetJoint("left_front_shock");
      joint_shock[FRONT_RIGHT] = _model->GetJoint("right_front_shock");
      joint_shock[REAR_LEFT] = _model->GetJoint("left_rear_shock");
      joint_shock[REAR_RIGHT] = _model->GetJoint("right_rear_shock");

      // Add all joints to single vector and prepare for TF
      joints_all.resize(10);
      joints_all[0] = joint_drive_left;
      joints_all[1] = joint_drive_right;
      joints_all[2] = joint_free_left;
      joints_all[3] = joint_free_right;
      joints_all[4] = joint_steer_left;
      joints_all[5] = joint_steer_right;
      joints_all[6] = _model->GetJoint("left_front_shock");
      joints_all[7] = joint_shock[FRONT_RIGHT];
      joints_all[8] = joint_shock[REAR_LEFT];
      joints_all[9] = joint_shock[REAR_RIGHT];


      // Get PID parameters from URDF
      // GetPIDParams(_sdf);
      InitParams(_sdf);

       // Initialize ros, if it has not already bee initialized.
       if (!ros::isInitialized())
       {
         int argc = 0;
         char **argv = NULL;
         ros::init(argc, argv, "gazebo_client",
             ros::init_options::NoSigintHandler);
       }

       // Create our ROS node. This acts in a similar manner to
       // the Gazebo node
       this->rosNode.reset(new ros::NodeHandle(robot_namespace));
       this->rosNode_steering.reset(new ros::NodeHandle(robot_namespace + "/steering"));
       this->rosNode_drive.reset(new ros::NodeHandle(robot_namespace + "/drive"));
       this->rosNode_drive_brake.reset(new ros::NodeHandle(robot_namespace + "/drive_brake"));
       this->rosNode_suspension.reset(new ros::NodeHandle(robot_namespace + "/suspension"));


       // Create a named topic, and subscribe to it.
      //  ros::SubscribeOptions so =
      //    ros::SubscribeOptions::create<ackermann_msgs::AckermannDrive>(
      //        "/" + this->model->GetName() + "/ackermann_cmd",
      //        1,
      //        boost::bind(&ControlPlugin::OnRosMsg, this, _1),
      //        ros::VoidPtr(), &this->rosQueue);

      // Dynamic reconfigure
        server_steering = new dynamic_reconfigure::Server<g35_gazebo::SteeringParamsConfig>(*this->rosNode_steering);
        // dynamic_reconfigure::Server<gazebo_plugins::PIDConfig> server;
        dynamic_reconfigure::Server<g35_gazebo::SteeringParamsConfig>::CallbackType f_steering;
        f_steering = boost::bind(&ControlPlugin::steering_callback, this, _1, _2);
        // server.setCallback(f);
        server_steering->setCallback(f_steering);

        server_drive = new dynamic_reconfigure::Server<g35_gazebo::DriveParamsConfig>(*this->rosNode_drive);
        dynamic_reconfigure::Server<g35_gazebo::DriveParamsConfig>::CallbackType f_drive;
        f_drive = boost::bind(&ControlPlugin::drive_callback, this, _1, _2);
        server_drive->setCallback(f_drive);

        server_drive_brake = new dynamic_reconfigure::Server<g35_gazebo::DriveBrakeParamsConfig>(*this->rosNode_drive_brake);
        dynamic_reconfigure::Server<g35_gazebo::DriveBrakeParamsConfig>::CallbackType f_drive_brake;
        f_drive_brake = boost::bind(&ControlPlugin::drive_brake_callback, this, _1, _2);
        server_drive_brake->setCallback(f_drive_brake);

        server_suspension = new dynamic_reconfigure::Server<g35_gazebo::SuspensionParamsConfig>(*this->rosNode_suspension);
        dynamic_reconfigure::Server<g35_gazebo::SuspensionParamsConfig>::CallbackType f_suspension;
        f_suspension = boost::bind(&ControlPlugin::suspension_callback, this, _1, _2);
        server_suspension->setCallback(f_suspension);

      // Ackermann message subscriber
      ros::SubscribeOptions so =
      ros::SubscribeOptions::create<ackermann_msgs::AckermannDrive>("ackermann_cmd",1,
        boost::bind(&ControlPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Steering angle publisher
      current_steering_angle_pub = this->rosNode->advertise<std_msgs::Float64>("current_steering_angle", 1);

      // Odometry publisher
      odometry_pub = this->rosNode->advertise<nav_msgs::Odometry>("odom", 1);
      odometry_frame = "odom";
      odom_source = ENCODER;
      robot_base_frame = "base_link";

      // Odometry truth publisher
      odometry_truth_pub = this->rosNode->advertise<nav_msgs::Odometry>("odom_truth", 1);
      odometry_truth_frame = "odom_truth";

      // TF broadcaster
      transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
      this->tf_prefix = tf::getPrefixParam(*this->rosNode);
      ROS_INFO("tfprefix = %s", this->tf_prefix.c_str());

       // Spin up the queue helper thread.
       this->rosQueueThread =
         std::thread(std::bind(&ControlPlugin::QueueThread, this));

       // listen to the update event (broadcast every simulation iteration)
       this->update_connection_ =
           event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ControlPlugin::UpdateChild, this ) );


      // Open file to write data
      // data_file = fopen("gazebo_data.txt", "w");


       // Start timer
       prevUpdateTime = this->model->GetWorld()->GetSimTime();
       print_timer = this->model->GetWorld()->GetSimTime();
       last_odom_update = this->model->GetWorld()->GetSimTime();
       sensor_update_period = 0.01;

    }

  public: void steering_callback(g35_gazebo::SteeringParamsConfig &config, uint32_t level) {
   ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f %f %f %f %f",
              config.SPos_P,
              config.SPos_I,
              config.SPos_D,
              config.SPos_Imax,
              config.SVel_max,
              config.SVel_P,
              config.SVel_I,
              config.SVel_D,
              config.SVel_Imax,
              config.STorque_max,
              config.SFriction,
              config.SAngle_max
        );

    // Set steering variables
      steering_P = config.SPos_P;
      steering_I = config.SPos_I;
      steering_D = config.SPos_D;
      steering_Imax = config.SPos_Imax;
      steering_Imin = -steering_Imax;

      steering_vel_max = config.SVel_max;
      steering_vel_min = -steering_vel_max;
      steering_vel_P = config.SVel_P;
      steering_vel_I = config.SVel_I;
      steering_vel_D = config.SVel_D;
      steering_vel_Imax = config.SVel_Imax;
      steering_vel_Imin = -steering_vel_Imax;

      steering_torque_max = config.STorque_max;
      steering_torque_min = -steering_torque_max;
      steering_friction = config.SFriction;
      steering_high = config.SAngle_max / 180.0 * M_PI; // radians but set in degrees
      steering_low = -steering_high;

      double midR_low = geom_wheel_base / tan(steering_low);
      double leftR_low = midR_low - geom_axle_width / 2.0;
      double rightR_low = midR_low + geom_axle_width / 2.0;

      double midR_high = geom_wheel_base / tan(steering_high);
      double leftR_high = midR_high - geom_axle_width / 2.0;
      double rightR_high = midR_high + geom_axle_width / 2.0;

      steering_lower_limit_left = atan(geom_wheel_base / leftR_low);
      double steering_lower_limit_right = atan(geom_wheel_base / rightR_low);
      steering_upper_limit_left = atan(geom_wheel_base / leftR_high);
      double steering_upper_limit_right = atan(geom_wheel_base / rightR_high);


    // Physics settings
      joint_steer_left->SetHighStop(0,math::Angle(steering_upper_limit_left));
      joint_steer_left->SetLowStop(0,math::Angle(steering_lower_limit_left));
      joint_steer_right->SetHighStop(0,math::Angle(steering_upper_limit_right));
      joint_steer_right->SetLowStop(0,math::Angle(steering_lower_limit_right));

      joint_steer_left->SetEffortLimit(0, steering_torque_max);
      joint_steer_right->SetEffortLimit(0, steering_torque_max);

      joint_steer_left->SetParam("friction", 0, 0.2);
      joint_steer_right->SetParam("friction", 0, 0.2);


  }

  public: void drive_callback(g35_gazebo::DriveParamsConfig &config, uint32_t level) {
    drive_P_accel = config.Daccel_P;
    drive_I_accel = config.Daccel_I;
    drive_D_accel = config.Daccel_D;

    drive_P_coast = config.Dcoast_P;
    drive_I_coast = config.Dcoast_I;
    drive_D_coast = config.Dcoast_D;

    drive_P_brake = config.Dbrake_P;
    drive_I_brake = config.Dbrake_I;
    drive_D_brake = config.Dbrake_D;

    drive_Imax = config.D_Imax;
    drive_Imin = -drive_Imax;

    drive_torque_max = config.DTorque_max;
    drive_torque_min = -drive_torque_max;

    wheel_velocity_limit = config.WVelocity_max;
    wheel_friction = config.WFriction;
    launch_control_on = config.W_LC;

    joint_drive_left->SetEffortLimit(0, drive_torque_max);
    joint_drive_right->SetEffortLimit(0, drive_torque_max);

    joint_drive_left->SetVelocityLimit(0, wheel_velocity_limit);
    joint_drive_right->SetVelocityLimit(0, wheel_velocity_limit);

    joint_free_left-> SetVelocityLimit(0, wheel_velocity_limit);
    joint_free_right->SetVelocityLimit(0, wheel_velocity_limit);

    joint_drive_left->SetParam("friction", 0, 0.2);
    joint_drive_right->SetParam("friction", 0, 0.2);

    joint_free_left->SetParam("friction", 0, 0.2);
    joint_free_right->SetParam("friction", 0, 0.2);
  }

  public: void drive_brake_callback(g35_gazebo::DriveBrakeParamsConfig &config, uint32_t level) {

  }


  public: void suspension_callback(g35_gazebo::SuspensionParamsConfig &config, uint32_t level) {
    shock_P = config.K_P;
    shock_I = config.K_I;
    shock_D = config.K_D;
    shock_Imax = config.K_Imax;
    shock_Imin = -shock_Imax;

    shock_effort_limit = config.KForce_max;
    shock_velocity_limit = config.KVelocity_max;
    shock_high_stop = config.KStop_max;

    for (int i = 0; i < 4; i++)
    {
      joint_shock[i]->SetEffortLimit(0, shock_effort_limit);
      joint_shock[i]->SetVelocityLimit(0, shock_velocity_limit);
      joint_shock[i]->SetLowStop(0,math::Angle(0));
      joint_shock[i]->SetHighStop(0,math::Angle(shock_high_stop));
    }
  }

  public: void InitParams(sdf::ElementPtr _sdf)
  {
    // Initialize namespace
    if (_sdf->HasElement("robotNamespace"))
    {
      robot_namespace = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
      ROS_INFO("Robot namespace = %s", robot_namespace.c_str());
    }
    else
    {
      robot_namespace.clear();
      ROS_INFO("No robot namespace defined");
    }

    // Initialize geometry
    if (_sdf->HasElement("wheelBase"))
    {
      this->geom_wheel_base = _sdf->Get<double>("wheelBase");
    }
    else
    {
      ROS_INFO("No wheelBase value specified");
      this->geom_wheel_base = 1;
    }
    printf("wheelBase = %.3f\n", this->geom_wheel_base);

    if (_sdf->HasElement("axleWidth"))
    {
      this->geom_axle_width = _sdf->Get<double>("axleWidth");
    }
    else
    {
      ROS_INFO("No axleWidth value specified");
      this->geom_axle_width = 1;
    }
    printf("axleWidth = %.3f\n", this->geom_axle_width);

    if (_sdf->HasElement("wheelRadius"))
    {
      this->geom_wheel_radius = _sdf->Get<double>("wheelRadius");
    }
    else
    {
      ROS_INFO("No wheelRadius value specified");
      this->geom_wheel_radius = 1;
    }
    printf("wheelRadius = %.3f\n", this->geom_wheel_radius);

    // Set joint effort and velocity limits
      // Note: SDF tag does not limit joint effort and velocity
      // Initialize PID variables
       // Drive
          drive_P_accel = 0.0;
          drive_I_accel = 0.0;
          drive_D_accel = 0.0;

          drive_P_coast = 0.0;
          drive_I_coast = 0.0;
          drive_D_coast = 0.0;

          drive_P_brake = 0.0;
          drive_I_brake = 0.0;
          drive_D_brake = 0.0;

          // Initialize drive wheel PID using Accel profile
          drive_P = drive_P_accel;
          drive_I = drive_I_accel;
          drive_D = drive_D_accel;

          drive_Imax = 0;
          drive_Imin = 0;
          drive_error = 0;
          drive_last_error = 0;

          drive_proportional = 0;
          drive_integral = 0;
          drive_derivative = 0;
          drive_output = 0;

          drive_torque_max = 0;
          drive_torque_min = -drive_torque_max;

          // Use launch control?
          launch_control_on = false;
          wheel_velocity_limit = 1;
          wheel_friction = 1;

          joint_drive_left->SetEffortLimit(0, drive_torque_max);
          joint_drive_right->SetEffortLimit(0, drive_torque_max);

          joint_drive_left->SetVelocityLimit(0, wheel_velocity_limit);
          joint_drive_right->SetVelocityLimit(0, wheel_velocity_limit);

          joint_free_left-> SetVelocityLimit(0, wheel_velocity_limit);
          joint_free_right->SetVelocityLimit(0, wheel_velocity_limit);

          joint_drive_left->SetParam("friction", 0, 0.2);
          joint_drive_right->SetParam("friction", 0, 0.2);

          joint_free_left->SetParam("friction", 0, 0.2);
          joint_free_right->SetParam("friction", 0, 0.2);


          // joint_drive_left->SetProvideFeedback(true); // enable feedback of force/torque

      // Steering: The steering joints have different limits to model Ackermann steering at maximum joint travel
      // Initialize steering PID
          steering_P = 0.0;
          steering_I = 0.0;
          steering_D = 0.0;
          steering_Imax = 0;
          steering_Imin = -steering_Imax;

          steering_vel_max = 0;
          steering_vel_min = -steering_vel_max;

          steering_left_last_error = 0;
          steering_right_last_error = 0;
          steering_left_integral = 0;
          steering_right_integral = 0;

          steering_left_output = 0;
          steering_right_output = 0;

          steering_vel_P = 0.0;
          steering_vel_I = 0.0;
          steering_vel_D = 0.0;
          steering_vel_Imax = 0;
          steering_vel_Imin = -steering_vel_Imax;
          steering_vel_left_integral = 0;
          steering_vel_right_integral = 0;
          steering_vel_left_last_error = 0;
          steering_vel_right_last_error = 0;
          steering_left_torque_output = 0;
          steering_right_torque_output = 0;

          steering_torque_max = 0;
          steering_torque_min = -steering_torque_max;

          // Set left and right angle limits, following Ackermann parallel linkage model
          steering_high = 0;  // steering angle upper limit, rad
          steering_low = -steering_high;  // steering angle lower limit, rad
          steering_friction = 0;

          double midR_low = geom_wheel_base / tan(steering_low);
          double leftR_low = midR_low - geom_axle_width / 2.0;
          double rightR_low = midR_low + geom_axle_width / 2.0;

          double midR_high = geom_wheel_base / tan(steering_high);
          double leftR_high = midR_high - geom_axle_width / 2.0;
          double rightR_high = midR_high + geom_axle_width / 2.0;

          double steering_lower_limit_left = atan(geom_wheel_base / leftR_low);
          double steering_lower_limit_right = atan(geom_wheel_base / rightR_low);
          double steering_upper_limit_left = atan(geom_wheel_base / leftR_high);
          double steering_upper_limit_right = atan(geom_wheel_base / rightR_high);

          joint_steer_left->SetHighStop(0,math::Angle(steering_upper_limit_left));
          joint_steer_left->SetLowStop(0,math::Angle(steering_lower_limit_left));
          joint_steer_right->SetHighStop(0,math::Angle(steering_upper_limit_right));
          joint_steer_right->SetLowStop(0,math::Angle(steering_lower_limit_right));

          joint_steer_left->SetParam("friction", 0, 0.2);
          joint_steer_right->SetParam("friction", 0, 0.2);


        // // Check limits
        // double low_stop = joint_steer_left->GetLowStop(0).Radian();
        // double high_stop = joint_steer_left->GetHighStop(0).Radian();
        // printf("low = %.3f high = %.3f\n", low_stop, high_stop);

      // Shocks
        shock_P = 0.0;
        shock_I = 0.0;
        shock_D = 0.0;

        shock_Imax = 0;
        shock_Imin = 0;
        desired_shock_position = 0.0;

        shock_effort_limit = 0;
        shock_velocity_limit = 0;
        shock_high_stop = 0;

        shock_last_error.resize(4);
        shock_integral.resize(4);
        shock_output.resize(4);

        for (int i = 0; i < 4; i++)
        {
          shock_last_error[i] = 0;
          shock_integral[i] = 0;
          shock_output[i] = 0;
        }

        for (int i = 0; i < 4; i++)
        {
          joint_shock[i]->SetEffortLimit(0, shock_effort_limit);
          joint_shock[i]->SetVelocityLimit(0, shock_velocity_limit);
          joint_shock[i]->SetLowStop(0,math::Angle(0));
          joint_shock[i]->SetHighStop(0,math::Angle(shock_high_stop));
        }
  }

  // public: void GetPIDParams(sdf::ElementPtr _sdf)
  // {
  //   // Set joint effort and velocity limits
  //     // Note: SDF tag does not limit joint effort and velocity
  //     // Initialize PID variables
  //      // Drive
  //        // Get SDF parameters
  //        math::Vector3 pid_vector;
  //
  //        if (_sdf->HasElement("accelPID"))
  //        {
  //          pid_vector = _sdf->Get<math::Vector3>("accelPID");
  //          drive_P_accel = pid_vector[0];
  //          drive_I_accel = pid_vector[1];
  //          drive_D_accel = pid_vector[2];
  //          printf("accelPID = %.3f %.3f %.3f\n", pid_vector[0], pid_vector[1], pid_vector[2]);
  //
  //        }
  //        else
  //        {
  //          ROS_INFO("No accel PID values specified");
  //          drive_P_accel = 0.0;
  //          drive_I_accel = 0.0;
  //          drive_D_accel = 0.0;
  //        }
  //
  //        if (_sdf->HasElement("coastPID"))
  //        {
  //          pid_vector = _sdf->Get<math::Vector3>("coastPID");
  //          drive_P_coast = pid_vector[0];
  //          drive_I_coast = pid_vector[1];
  //          drive_D_coast = pid_vector[2];
  //          printf("coastPID = %.3f %.3f %.3f\n", pid_vector[0], pid_vector[1], pid_vector[2]);
  //
  //        }
  //        else
  //        {
  //          ROS_INFO("No coast PID values specified");
  //          drive_P_coast = 0.0;
  //          drive_I_coast = 0.0;
  //          drive_D_coast = 0.0;
  //        }
  //
  //        if (_sdf->HasElement("brakePID"))
  //        {
  //          pid_vector = _sdf->Get<math::Vector3>("brakePID");
  //          drive_P_brake = pid_vector[0];
  //          drive_I_brake = pid_vector[1];
  //          drive_D_brake = pid_vector[2];
  //          printf("brakePID = %.3f %.3f %.3f\n", pid_vector[0], pid_vector[1], pid_vector[2]);
  //
  //        }
  //        else
  //        {
  //          ROS_INFO("No brake PID values specified");
  //          drive_P_brake = 0.0;
  //          drive_I_brake = 0.0;
  //          drive_D_brake = 0.0;
  //        }
  //
  //        // Initialize drive wheel PID using Accel profile
  //        drive_P = drive_P_accel;
  //        drive_I = drive_I_accel;
  //        drive_D = drive_D_accel;
  //
  //        drive_Imax = 0;
  //        drive_Imin = 0;
  //        drive_error = 0;
  //        drive_last_error = 0;
  //
  //        drive_proportional = 0;
  //        drive_integral = 0;
  //        drive_derivative = 0;
  //        drive_output = 0;
  //
  //        if (_sdf->HasElement("maxTorque"))
  //        {
  //          drive_torque_max = _sdf->Get<double>("maxTorque");
  //          printf("maxTorque = %.3f\n", drive_torque_max);
  //        }
  //        else
  //        {
  //          ROS_INFO("No max torque value specified");
  //          drive_torque_max = 0;
  //        }
  //
  //        drive_torque_min = -drive_torque_max;
  //
  //        // Use launch control?
  //        if (_sdf->HasElement("launchControl"))
  //        {
  //          launch_control_on = _sdf->Get<bool>("launchControl");
  //          printf("launchControl = %d\n", (int)launch_control_on);
  //        }
  //        else
  //        {
  //          ROS_INFO("No max suspension velocity value specified");
  //          launch_control_on = false;
  //        }
  //
  //        double wheel_velocity_limit;
  //
  //        if (_sdf->HasElement("maxVelocity"))
  //        {
  //          wheel_velocity_limit = _sdf->Get<double>("maxVelocity");
  //          printf("maxVelocity = %.3f\n", wheel_velocity_limit);
  //        }
  //        else
  //        {
  //          ROS_INFO("No max velocity value specified");
  //          wheel_velocity_limit = 10;
  //        }
  //
  //        double wheel_friction;
  //
  //        if (_sdf->HasElement("wheelFriction"))
  //        {
  //          wheel_friction = _sdf->Get<double>("wheelFriction");
  //          printf("wheelFriction = %.3f\n", wheel_friction);
  //        }
  //        else
  //        {
  //          ROS_INFO("No wheel friction value specified");
  //          wheel_friction = 0;
  //        }
  //
  //       joint_drive_left->SetEffortLimit(0, drive_torque_max);
  //       joint_drive_right->SetEffortLimit(0, drive_torque_max);
  //
  //       joint_drive_left->SetVelocityLimit(0, wheel_velocity_limit);
  //       joint_drive_right->SetVelocityLimit(0, wheel_velocity_limit);
  //       joint_free_left-> SetVelocityLimit(0, wheel_velocity_limit);
  //       joint_free_right->SetVelocityLimit(0, wheel_velocity_limit);
  //
  //       joint_drive_left->SetParam("friction", 0, 0.2);
  //       joint_drive_right->SetParam("friction", 0, 0.2);
  //       joint_free_left->SetParam("friction", 0, 0.2);
  //       joint_free_right->SetParam("friction", 0, 0.2);
  //
  //
  //       joint_drive_left->SetProvideFeedback(true); // enable feedback of force/torque
  //
  //     // Steering: The steering joints have different limits to model Ackermann steering at maximum joint travel
  //     // Initialize steering PID
  //       if (_sdf->HasElement("steeringPID"))
  //       {
  //         pid_vector = _sdf->Get<math::Vector3>("steeringPID");
  //         steering_P = pid_vector[0];
  //         steering_I = pid_vector[1];
  //         steering_D = pid_vector[2];
  //         printf("steeringPID = %.3f %.3f %.3f\n", pid_vector[0], pid_vector[1], pid_vector[2]);
  //
  //       }
  //       else
  //       {
  //         ROS_INFO("No steering PID values specified");
  //         steering_P = 0.0;
  //         steering_I = 0.0;
  //         steering_D = 0.0;
  //       }
  //
  //       if (_sdf->HasElement("steeringImax"))
  //       {
  //         steering_Imax = _sdf->Get<double>("steeringImax");
  //         printf("steeringImax = %.3f\n", wheel_friction);
  //       }
  //       else
  //       {
  //         ROS_INFO("No steering Imax value specified");
  //         steering_Imax = 0;
  //       }
  //
  //       if (_sdf->HasElement("steeringVelMax"))
  //       {
  //         steering_vel_max = _sdf->Get<double>("steeringVelMax");
  //         printf("steeringVelMax = %.3f\n", steering_vel_max);
  //       }
  //       else
  //       {
  //         ROS_INFO("No steering max velocity value specified");
  //         steering_vel_max = 0;
  //       }
  //
  //       // steering_Imax = 0;
  //       steering_Imin = -steering_Imax;
  //       // steering_vel_max = 10;
  //       steering_vel_min = -steering_vel_max;
  //
  //       steering_left_last_error = 0;
  //       steering_right_last_error = 0;
  //       steering_left_integral = 0;
  //       steering_right_integral = 0;
  //
  //       steering_left_output = 0;
  //       steering_right_output = 0;
  //
  //       if (_sdf->HasElement("steeringVelPID"))
  //       {
  //         pid_vector = _sdf->Get<math::Vector3>("steeringVelPID");
  //         steering_vel_P = pid_vector[0];
  //         steering_vel_I = pid_vector[1];
  //         steering_vel_D = pid_vector[2];
  //         printf("steeringVelPID = %.3f %.3f %.3f\n", pid_vector[0], pid_vector[1], pid_vector[2]);
  //
  //       }
  //       else
  //       {
  //         ROS_INFO("No steering velocity PID values specified");
  //         steering_vel_P = 0.0;
  //         steering_vel_I = 0.0;
  //         steering_vel_D = 0.0;
  //       }
  //
  //       if (_sdf->HasElement("steeringVelImax"))
  //       {
  //         steering_vel_Imax = _sdf->Get<double>("steeringImax");
  //         printf("steeringVelImax = %.3f\n", steering_vel_Imax);
  //       }
  //       else
  //       {
  //         ROS_INFO("No steering velocity Imax value specified");
  //         steering_vel_Imax = 0;
  //       }
  //
  //       // steering_vel_Imax = 0;
  //       steering_vel_Imin = -steering_vel_Imax;
  //
  //       steering_vel_left_integral = 0;
  //       steering_vel_right_integral = 0;
  //       steering_vel_left_last_error = 0;
  //       steering_vel_right_last_error = 0;
  //       steering_left_torque_output = 0;
  //       steering_right_torque_output = 0;
  //
  //
  //       if (_sdf->HasElement("steeringTorque"))
  //       {
  //         steering_torque_max = _sdf->Get<double>("steeringTorque");
  //         printf("steeringTorque = %.3f\n", steering_torque_max);
  //       }
  //       else
  //       {
  //         ROS_INFO("No max steering torque value specified");
  //         steering_torque_max = 0;
  //       }
  //
  //       steering_torque_min = -steering_torque_max;
  //
  //       // Set left and right angle limits, following Ackermann parallel linkage model
  //       double steer_high;  // steering angle upper limit, rad
  //
  //       if (_sdf->HasElement("steeringAngle"))
  //       {
  //         steer_high = _sdf->Get<double>("steeringAngle");
  //         printf("steeringAngle = %.3f\n", steer_high);
  //       }
  //       else
  //       {
  //         ROS_INFO("No max steering angle value specified");
  //         steer_high = 0;
  //       }
  //
  //       double steering_friction;
  //
  //       if (_sdf->HasElement("steeringFriction"))
  //       {
  //         steering_friction = _sdf->Get<double>("steeringFriction");
  //         printf("steeringFriction = %.3f\n", steering_friction);
  //       }
  //       else
  //       {
  //         ROS_INFO("No steering friction value specified");
  //         steering_friction = 0;
  //       }
  //
  //       double steer_low = -steer_high;  // steering angle lower limit, rad
  //       double midR_low = geom_wheel_base / tan(steer_low);
  //       double leftR_low = midR_low - geom_axle_width / 2.0;
  //       double rightR_low = midR_low + geom_axle_width / 2.0;
  //
  //       double midR_high = geom_wheel_base / tan(steer_high);
  //       double leftR_high = midR_high - geom_axle_width / 2.0;
  //       double rightR_high = midR_high + geom_axle_width / 2.0;
  //
  //       double steering_lower_limit_left = atan(geom_wheel_base / leftR_low);
  //       double steering_lower_limit_right = atan(geom_wheel_base / rightR_low);
  //       double steering_upper_limit_left = atan(geom_wheel_base / leftR_high);
  //       double steering_upper_limit_right = atan(geom_wheel_base / rightR_high);
  //
  //       joint_steer_left->SetHighStop(0,math::Angle(steering_upper_limit_left));
  //       joint_steer_left->SetLowStop(0,math::Angle(steering_lower_limit_left));
  //       joint_steer_right->SetHighStop(0,math::Angle(steering_upper_limit_right));
  //       joint_steer_right->SetLowStop(0,math::Angle(steering_lower_limit_right));
  //
  //       joint_steer_left->SetParam("friction", 0, 0.2);
  //       joint_steer_right->SetParam("friction", 0, 0.2);
  //
  //
  //       // // Check limits
  //       // double low_stop = joint_steer_left->GetLowStop(0).Radian();
  //       // double high_stop = joint_steer_left->GetHighStop(0).Radian();
  //       // printf("low = %.3f high = %.3f\n", low_stop, high_stop);
  //
  //     // Shocks
  //       // PID vectors
  //       shock_last_error.resize(4);
  //       shock_integral.resize(4);
  //       shock_output.resize(4);
  //
  //       for (int i = 0; i < 4; i++)
  //       {
  //         shock_last_error[i] = 0;
  //         shock_integral[i] = 0;
  //         shock_output[i] = 0;
  //       }
  //
  //       if (_sdf->HasElement("suspensionPID"))
  //       {
  //         pid_vector = _sdf->Get<math::Vector3>("suspensionPID");
  //         shock_P = pid_vector[0];
  //         shock_I = pid_vector[1];
  //         shock_D = pid_vector[2];
  //         printf("suspensionPID = %.3f %.3f %.3f\n", pid_vector[0], pid_vector[1], pid_vector[2]);
  //
  //       }
  //       else
  //       {
  //         ROS_INFO("No suspension PID values specified");
  //         shock_P = 0.0;
  //         shock_I = 0.0;
  //         shock_D = 0.0;
  //       }
  //
  //       // shock_P = 3200; //800
  //       // shock_I = 0;
  //       // shock_D = 40;
  //       shock_Imax = 0;
  //       shock_Imin = 0;
  //       desired_shock_position = 0.0;
  //
  //       double shock_effort_limit;
  //       double shock_velocity_limit;
  //
  //       if (_sdf->HasElement("suspensionMaxForce"))
  //       {
  //         shock_effort_limit = _sdf->Get<double>("suspensionMaxForce");
  //         printf("suspensionMaxForce = %.3f\n", shock_effort_limit);
  //       }
  //       else
  //       {
  //         ROS_INFO("No max suspension force value specified");
  //         shock_effort_limit = 0;
  //       }
  //
  //       if (_sdf->HasElement("suspensionMaxVelocity"))
  //       {
  //         shock_velocity_limit = _sdf->Get<double>("suspensionMaxVelocity");
  //         printf("suspensionMaxVelocity = %.3f\n", shock_velocity_limit);
  //       }
  //       else
  //       {
  //         ROS_INFO("No max suspension velocity value specified");
  //         shock_velocity_limit = 0;
  //       }
  //
  //       double shock_high_stop;
  //
  //       if (_sdf->HasElement("suspensionHighStop"))
  //       {
  //         shock_high_stop = _sdf->Get<double>("suspensionHighStop");
  //         printf("suspensionHighStop = %.3f\n", shock_high_stop);
  //       }
  //       else
  //       {
  //         ROS_INFO("No max suspension high stop value specified");
  //         shock_high_stop = 0;
  //       }
  //
  //     for (int i = 0; i < 4; i++)
  //     {
  //
  //       joint_shock[i]->SetEffortLimit(0, shock_effort_limit);
  //       joint_shock[i]->SetVelocityLimit(0, shock_velocity_limit);
  //       joint_shock[i]->SetLowStop(0,math::Angle(0));
  //       joint_shock[i]->SetHighStop(0,math::Angle(shock_high_stop));
  //
  //     }
  // }


    public: void UpdateSteeringPID()
    {
      // Left wheel
        // POSITION CONTROLLER
          // Get current angle
          double steering_pos = joint_steer_left->GetAngle(0).Radian();

          // Get angle error
          double steering_error = desired_steering_angle_left - steering_pos;

          // PID Proportional
          double steering_proportional = steering_P * steering_error;

          // PID Integral
          steering_left_integral += steering_I * steering_error * stepTime.Double();
          if (steering_left_integral > steering_Imax)
          {
            steering_left_integral = steering_Imax;
          }
          else if (steering_left_integral < steering_Imin)
          {
            steering_left_integral = steering_Imin;
          }

          // PID Derivative
          double steering_derivative = steering_D * (steering_error - steering_left_last_error) / stepTime.Double();
          steering_left_last_error = steering_error;

          // Constrain final PID output
          steering_left_output = steering_proportional + steering_left_integral + steering_derivative;
          if (steering_left_output > steering_vel_max)
          {
            steering_left_output = steering_vel_max;
          }
          else if (steering_left_output < steering_vel_min)
          {
            steering_left_output = steering_vel_min;
          }

        // VELOCITY CONTROLLER
          // Get current angle
          double steering_vel_left = joint_steer_left->GetVelocity(0);

          // Get angle error
          double steering_vel_left_error = steering_left_output - steering_vel_left;

          // PID Proportional
          double steering_vel_left_proportional = steering_vel_P * steering_vel_left_error;

          // PID Integral
          steering_vel_left_integral += steering_vel_I * steering_vel_left_error * stepTime.Double();
          if (steering_vel_left_integral > steering_vel_Imax)
          {
            steering_vel_left_integral = steering_vel_Imax;
          }
          else if (steering_vel_left_integral < steering_vel_Imin)
          {
            steering_vel_left_integral = steering_vel_Imin;
          }

          // PID Derivative
          double steering_vel_left_derivative = steering_vel_D * (steering_vel_left_error - steering_vel_left_last_error) / stepTime.Double();
          steering_vel_left_last_error = steering_vel_left_error;

          // Constrain final PID output
          steering_left_torque_output = steering_vel_left_proportional + steering_vel_left_integral + steering_vel_left_derivative;

          SP = steering_vel_left_proportional;
          SI = steering_vel_left_integral;
          SD = steering_vel_left_derivative;

      // Right wheel
        // POSITION CONTROLLER
          // Get current angle
          steering_pos = joint_steer_right->GetAngle(0).Radian();

          // Get angle error
          steering_error = desired_steering_angle_right - steering_pos;

          // PID Proportional
          steering_proportional = steering_P * steering_error;

          // PID Integral
          steering_right_integral += steering_I * steering_error * stepTime.Double();
          if (steering_right_integral > steering_Imax)
          {
            steering_right_integral = steering_Imax;
          }
          else if (steering_right_integral < steering_Imin)
          {
            steering_right_integral = steering_Imin;
          }

          // PID Derivative
          steering_derivative = steering_D * (steering_error - steering_right_last_error) / stepTime.Double();
          steering_right_last_error = steering_error;

          // Constrain final PID output
          steering_right_output = steering_proportional + steering_right_integral + steering_derivative;
          if (steering_right_output > steering_vel_max)
          {
            steering_right_output = steering_vel_max;
          }
          else if (steering_right_output < steering_vel_min)
          {
            steering_right_output = steering_vel_min;
          }

        // VELOCITY CONTROLLER
          // Get current angle
          double steering_vel_right = joint_steer_right->GetVelocity(0);

          // Get angle error
          double steering_vel_right_error = steering_right_output - steering_vel_right;

          // PID Proportional
          double steering_vel_right_proportional = steering_vel_P * steering_vel_right_error;

          // PID Integral
          steering_vel_right_integral += steering_vel_I * steering_vel_right_error * stepTime.Double();
          if (steering_vel_right_integral > steering_vel_Imax)
          {
            steering_vel_right_integral = steering_vel_Imax;
          }
          else if (steering_vel_right_integral < steering_vel_Imin)
          {
            steering_vel_right_integral = steering_vel_Imin;
          }

          // PID Derivative
          double steering_vel_right_derivative = steering_vel_D * (steering_vel_right_error - steering_vel_right_last_error) / stepTime.Double();
          steering_vel_left_last_error = steering_vel_left_error;

          // Constrain final PID output
          steering_right_torque_output = steering_vel_right_proportional + steering_vel_right_integral + steering_vel_right_derivative;


    }


    public: void UpdateShockPID()
    {
      for (int i = 0; i < 4; i++)
      {
        // Get current position: use GetAngle(0).Radian(), travel limits are set in SDF
        double shock_position = joint_shock[i]->GetAngle(0).Radian();

        // Get position error
        double shock_error = desired_shock_position - shock_position;

        // PID proportional
        double shock_proportional = shock_P * shock_error;

        // PID integral
        shock_integral[i] += shock_I * shock_error * stepTime.Double();
        if (shock_integral[i] > shock_Imax)
        {
          shock_integral[i] = shock_Imax;
        }
        else if (shock_integral[i] < shock_Imin)
        {
          shock_integral[i] = shock_Imin;
        }

        // PID derivative
        double shock_derivative = shock_D * (shock_error - shock_last_error[i]) / stepTime.Double();
        shock_last_error[i] = shock_error;

        // Final PID output should be trunated automatically by the above joint settings
        shock_output[i] = shock_proportional + shock_integral[i] + shock_derivative;

        // printf("joint_pos = %.3f\n", shock_position);
      }
    }


    public: void UpdateDrivePID()
    {
      // boost::mutex::scoped_lock scoped_lock ( lock );

      // Notes:
      // Torque is set for only a single iteration
      // Damping and friction are active on wheels from the SDF tags


      // Get the rotational velocities in rad/s of both drive wheels
      double drive_wheel_left_vel = joint_drive_left->GetVelocity(0);
      double drive_wheel_right_vel = joint_drive_right->GetVelocity(0);
      double free_wheel_left_vel = joint_free_left->GetVelocity(0);
      double free_wheel_right_vel = joint_free_right->GetVelocity(0);

      // drive_wheel_speedometer = 0.5* (drive_wheel_left_vel + drive_wheel_right_vel) * geom_wheel_radius * 3.6 * 1.6;
      drive_wheel_speedometer = 0.5* (free_wheel_left_vel + free_wheel_right_vel) * geom_wheel_radius * 3.6 / 1.61;

      double steering_pos_left = joint_steer_left->GetAngle(0).Radian();
      double steering_pos_right = joint_steer_right->GetAngle(0).Radian();
      double steering_pos_avg = (steering_pos_left + steering_pos_right) / 2.0;

      // Calculate average velocity of both axles
      double free_wheel_avg_vel = (free_wheel_left_vel + free_wheel_right_vel) / 2.0;
      double rear_axle_radius = geom_wheel_base / sqrt(pow(tan(steering_pos_avg),2));  // can be infinity at zero angle
      // printf("radius = %.3f\n", rear_axle_radius);
      double front_axle_radius = sqrt(pow(rear_axle_radius, 2) + pow(geom_wheel_base,2));
      double drive_wheel_expected_vel = rear_axle_radius / front_axle_radius * free_wheel_avg_vel; // The non-slip speed of rear wheels given speed of front wheels

      double drive_wheel_avg_vel = (drive_wheel_left_vel + drive_wheel_right_vel) / 2;  // Actual speed of rear wheels
      front_axle_avg_vel = free_wheel_avg_vel;
      rear_axle_desired_vel = drive_wheel_expected_vel;
      rear_axle_vel = drive_wheel_avg_vel;

      if (launch_control_on)
      {
        // Increase drive wheel torque until velocity exceeds front-axle-based synchronized rear wheel speed, then back off
        // Need to vary the P gain here
        // Check for slip condition and modify P gain if necessary
          // Check slip
          if (drive_wheel_avg_vel > drive_wheel_expected_vel + 1 || drive_wheel_avg_vel < drive_wheel_expected_vel - 1)
          {
            // lower P gain: make sure vehicle is not clipping when spawned (creates high initial front wheel velocity, and P gain goes negative)
            drive_P = drive_P - 0.05 * drive_P_accel;
            if (drive_P <= 0)
            {
              drive_P = 0.0001; // Make sure P gain does not become negative
            }
            // printf("slip\n");
          }
          else
          {
            drive_P = drive_P_accel;  // full power if traction is available
          }
          //PID
            // // Check for coasting or break and modify gains
            // if (desired_vel < drive_wheel_avg_vel)
            // {
            //   if (desired_vel >= 0)
            //   {
            //     drive_P = drive_P_coast;
            //   }
            //   else
            //   {
            //     desired_vel = 0;
            //     drive_P = drive_P_brake;
            //   }
            // }
            // else
            // {
            //   drive_P = drive_P_accel;
            // }

            // Proportional
            drive_error = desired_vel - drive_wheel_avg_vel;
            drive_proportional = drive_P * drive_error;

            // Integral
            drive_integral += drive_I * drive_error * stepTime.Double();
            if (drive_integral > drive_Imax)
            {
              drive_integral = drive_Imax;
            }
            else if (drive_integral < drive_Imin)
            {
              drive_integral = drive_Imin;
            }

            // Derivative
            drive_derivative = drive_D * (drive_error - drive_last_error) / stepTime.Double();
            drive_last_error = drive_error;

            // Constrain final PID output
            drive_output = drive_proportional + drive_integral + drive_derivative;
            if (drive_output > drive_torque_max)
            {
              drive_output = drive_torque_max;
            }
            else if (drive_output < drive_torque_min)
            {
              drive_output = drive_torque_min;
            }


      }
      else
      {
        // printf("wheel_vel = %.6f\n", drive_wheel_avg_vel);
        //PID
          // Check for coasting or break and modify gains
          if (desired_vel < drive_wheel_avg_vel)
          {
            if (desired_vel >= 0)
            {
              brake_flag = false;
              drive_P = drive_P_coast;
            }
            else
            {
              desired_vel = 0;
              brake_flag = true;
              drive_P = drive_P_brake;
            }
          }
          else
          {
            brake_flag = false;
            drive_P = drive_P_accel;
          }

          // Proportional
          drive_error = desired_vel - drive_wheel_avg_vel;
          drive_proportional = drive_P * drive_error;

          // Integral
          drive_integral += drive_I * drive_error * stepTime.Double();
          if (drive_integral > drive_Imax)
          {
            drive_integral = drive_Imax;
          }
          else if (drive_integral < drive_Imin)
          {
            drive_integral = drive_Imin;
          }

          // Derivative
          drive_derivative = drive_D * (drive_error - drive_last_error) / stepTime.Double();
          drive_last_error = drive_error;

          // Constrain final PID output
          drive_output = drive_proportional + drive_integral + drive_derivative;
          if (drive_output > drive_torque_max)
          {
            drive_output = drive_torque_max;
          }
          else if (drive_output < drive_torque_min)
          {
            drive_output = drive_torque_min;
          }

      }


        // printf("stepTime = %.6f avg_vel = %.6f drive_error = %.6f drive_proportional = %.6f drive_integral = %.6f drive_derivative = %.6f\n", stepTime.Double(), drive_wheel_avg_vel, drive_error, drive_proportional, drive_integral, drive_derivative);

        // printf("desired_vel = %.6f avg_vel = %.6f drive_proportional = %.6f drive_output = %.6f\n", desired_vel, drive_wheel_avg_vel, drive_proportional, drive_output);

      // printf("Time Sim = %.6f  Time ROS = %.6f \n", stepTime.Double(), stepTime_ros);
      // std::cout <<"P = " << drive_proportional << "  I = " << drive_integral << "  D = " << drive_derivative << std::endl;
      // std::cout << "desired_vel = " << desired_vel << "  avg_vel = " << drive_wheel_avg_vel << std::endl;
      // std::cout << "wheel speeds = " << drive_wheel_left_vel << "  " << drive_wheel_right_vel << "  time step = " << stepTime.Double() << std::endl;
      // std::cout << "output torque = " << drive_output << std::endl;

    }


    public: void UpdateABSDrivePID()
    {
      // boost::mutex::scoped_lock scoped_lock ( lock );

      // Notes:
      // Torque is set for only a single iteration
      // Damping and friction are active on wheels from the SDF tags

      // Get the rotational velocities in rad/s of both drive wheels
      double drive_wheel_left_vel = joint_drive_left->GetVelocity(0);
      double drive_wheel_right_vel = joint_drive_right->GetVelocity(0);
      double free_wheel_left_vel = joint_free_left->GetVelocity(0);
      double free_wheel_right_vel = joint_free_right->GetVelocity(0);

      // drive_wheel_speedometer = 0.5* (drive_wheel_left_vel + drive_wheel_right_vel) * geom_wheel_radius * 3.6 * 1.6;
      drive_wheel_speedometer = 0.5* (free_wheel_left_vel + free_wheel_right_vel) * geom_wheel_radius * 3.6 / 1.61;

      double steering_pos_left = joint_steer_left->GetAngle(0).Radian();
      double steering_pos_right = joint_steer_right->GetAngle(0).Radian();
      double steering_pos_avg = (steering_pos_left + steering_pos_right) / 2.0;

      // Calculate average velocity of both axles
      double free_wheel_avg_vel = (free_wheel_left_vel + free_wheel_right_vel) / 2.0;
      double rear_axle_radius = geom_wheel_base / sqrt(pow(tan(steering_pos_avg),2));  // can be infinity at zero angle
      // printf("radius = %.3f\n", rear_axle_radius);
      double front_axle_radius = sqrt(pow(rear_axle_radius, 2) + pow(geom_wheel_base,2));
      double drive_wheel_expected_vel = rear_axle_radius / front_axle_radius * free_wheel_avg_vel; // The non-slip speed of rear wheels given speed of front wheels

      double drive_wheel_avg_vel = (drive_wheel_left_vel + drive_wheel_right_vel) / 2;  // Actual speed of rear wheels
      front_axle_avg_vel = free_wheel_avg_vel;
      rear_axle_desired_vel = drive_wheel_expected_vel;
      rear_axle_vel = drive_wheel_avg_vel;

      // Average speed of all wheels
      double wheels_avg_speed = 0.25 * (free_wheel_left_vel + free_wheel_right_vel + drive_wheel_left_vel + drive_wheel_right_vel);

      // Check each wheel for difference threshold. Reduce brake torque for locked wheels
      double max_brake_torque = 600;
      double min_brake_torque = 200;
      double abs_vel_thresh = 1;

      if (free_wheel_left_vel < wheels_avg_speed - abs_vel_thresh)
      {

      }

      //PID


        // Check for coasting or break and modify gains
        if (desired_vel < drive_wheel_avg_vel)
        {
          if (desired_vel >= 0)
          {
            brake_flag = false;
            drive_P = drive_P_coast;
          }
          else
          {
            desired_vel = 0;
            brake_flag = true;
            drive_P = drive_P_brake;
          }
        }
        else
        {
          brake_flag = false;
          drive_P = drive_P_accel;
        }

        // Proportional
        drive_error = desired_vel - drive_wheel_avg_vel;
        drive_proportional = drive_P * drive_error;

        // Integral
        drive_integral += drive_I * drive_error * stepTime.Double();
        if (drive_integral > drive_Imax)
        {
          drive_integral = drive_Imax;
        }
        else if (drive_integral < drive_Imin)
        {
          drive_integral = drive_Imin;
        }

        // Derivative
        drive_derivative = drive_D * (drive_error - drive_last_error) / stepTime.Double();
        drive_last_error = drive_error;

        // Constrain final PID output
        drive_output = drive_proportional + drive_integral + drive_derivative;
        if (drive_output > drive_torque_max)
        {
          drive_output = drive_torque_max;
        }
        else if (drive_output < drive_torque_min)
        {
          drive_output = drive_torque_min;
        }

    }

    public: void UpdateDriveSimplePID()
    {
      // boost::mutex::scoped_lock scoped_lock ( lock );

      // Notes:
      // Torque is set for only a single iteration
      // Damping and friction are active on wheels from the SDF tags

      // Get the rotational velocities in rad/s of both drive wheels
      double drive_wheel_left_vel = joint_drive_left->GetVelocity(0);
      double drive_wheel_right_vel = joint_drive_right->GetVelocity(0);
      double free_wheel_left_vel = joint_free_left->GetVelocity(0);
      double free_wheel_right_vel = joint_free_right->GetVelocity(0);

      // drive_wheel_speedometer = 0.5* (drive_wheel_left_vel + drive_wheel_right_vel) * geom_wheel_radius * 3.6 * 1.6;
      // drive_wheel_speedometer = 0.5* (free_wheel_left_vel + free_wheel_right_vel) * geom_wheel_radius * 3.6 / 1.61;
      drive_wheel_speedometer = 0.5* (free_wheel_left_vel + free_wheel_right_vel) * geom_wheel_radius;


      double steering_pos_left = joint_steer_left->GetAngle(0).Radian();
      double steering_pos_right = joint_steer_right->GetAngle(0).Radian();
      double steering_pos_avg = (steering_pos_left + steering_pos_right) / 2.0;

      // Calculate average velocity of both axles
      double free_wheel_avg_vel = (free_wheel_left_vel + free_wheel_right_vel) / 2.0;
      double rear_axle_radius = geom_wheel_base / sqrt(pow(tan(steering_pos_avg),2));  // can be infinity at zero angle
      // printf("radius = %.3f\n", rear_axle_radius);
      double front_axle_radius = sqrt(pow(rear_axle_radius, 2) + pow(geom_wheel_base,2));
      double drive_wheel_expected_vel = rear_axle_radius / front_axle_radius * free_wheel_avg_vel; // The non-slip speed of rear wheels given speed of front wheels

      double drive_wheel_avg_vel = (drive_wheel_left_vel + drive_wheel_right_vel) / 2;  // Actual speed of rear wheels
      front_axle_avg_vel = free_wheel_avg_vel;
      rear_axle_desired_vel = drive_wheel_expected_vel;
      rear_axle_vel = drive_wheel_avg_vel;


      //PID
        drive_P = drive_P_accel;
        drive_I = drive_I_accel;
        drive_D = drive_D_accel;


        // Proportional
        drive_error = desired_vel - drive_wheel_avg_vel;
        drive_proportional = drive_P * drive_error;

        // Integral
        drive_integral += drive_I * drive_error * stepTime.Double();
        if (drive_integral > drive_Imax)
        {
          drive_integral = drive_Imax;
        }
        else if (drive_integral < drive_Imin)
        {
          drive_integral = drive_Imin;
        }

        // Derivative
        drive_derivative = drive_D * (drive_error - drive_last_error) / stepTime.Double();
        drive_last_error = drive_error;

        // Constrain final PID output
        drive_output = drive_proportional + drive_integral + drive_derivative;
        if (drive_output > drive_torque_max)
        {
          drive_output = drive_torque_max;
        }
        else if (drive_output < drive_torque_min)
        {
          drive_output = drive_torque_min;
        }

    }

    public: void UpdateThrottleBrakePID()
    {
      // Get the rotational velocities in rad/s of both drive wheels
      double drive_wheel_left_vel = joint_drive_left->GetVelocity(0);
      double drive_wheel_right_vel = joint_drive_right->GetVelocity(0);
      double free_wheel_left_vel = joint_free_left->GetVelocity(0);
      double free_wheel_right_vel = joint_free_right->GetVelocity(0);

      // drive_wheel_speedometer = 0.5* (drive_wheel_left_vel + drive_wheel_right_vel) * geom_wheel_radius * 3.6 * 1.6;
      drive_wheel_speedometer = 0.5* (free_wheel_left_vel + free_wheel_right_vel) * geom_wheel_radius * 3.6 / 1.61;

      double drive_wheel_avg_vel = (drive_wheel_left_vel + drive_wheel_right_vel) / 2;  // Actual speed of rear wheels


      //PID
        drive_P = drive_P_coast;
        drive_I = drive_I_coast;
        drive_D = drive_D_coast;


        // Proportional
        drive_error = desired_vel - drive_wheel_avg_vel;
        drive_proportional = drive_P * drive_error;
        drive_integral += drive_I * drive_error * stepTime.Double();
        if (drive_integral > drive_Imax)
        {
          drive_integral = drive_Imax;
        }
        else if (drive_integral < drive_Imin)
        {
          drive_integral = drive_Imin;
        }

        // Derivative
        drive_derivative = drive_D * (drive_error - drive_last_error) / stepTime.Double();
        drive_last_error = drive_error;

        // Constrain final PID output
        drive_output = drive_proportional + drive_integral + drive_derivative;
        if (drive_output > drive_torque_max)
        {
          drive_output = drive_torque_max;
        }
        else if (drive_output < drive_torque_min)
        {
          drive_output = drive_torque_min;
        }

    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const ackermann_msgs::AckermannDriveConstPtr &_msg)
    {
      boost::mutex::scoped_lock scoped_lock ( lock );

      this->desired_vel_msg = _msg->speed;
      this->desired_steering_angle_msg = _msg->steering_angle;
      if (_msg->acceleration < 0)
      {
        this->ebrake_flag = true;
      }
      else
      {
        this->ebrake_flag = false;
      }

    }



    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }


    private: void GetControls()
    {
      boost::mutex::scoped_lock scoped_lock ( lock );

      this->desired_vel = this->desired_vel_msg / geom_wheel_radius;
      this->desired_steering_angle = this->desired_steering_angle_msg / 180.0 * M_PI; // Convert from degrees to radians

    }



    public: void UpdateChild()
    {
      // Visual
      // visPub->Publish(visualMsg);

      // Get setpoints from ROS callback update
        GetControls();

      // Update time step
      common::Time currTime = this->model->GetWorld()->GetSimTime();
      this->stepTime = currTime - this->prevUpdateTime;
      this->prevUpdateTime = currTime;

      // Update drive PID from updated wheel velocity setpoint-
        // UpdateABSDrivePID();
        UpdateDriveSimplePID();

      // Update steering PID
        // Get steering setpoint for left and right wheels (inf value at 0 degrees)
        if (desired_steering_angle == 0)
        {
          desired_steering_angle_left = 0.0;
          desired_steering_angle_right = 0.0;
        }
        else
        {
          double midR = geom_wheel_base / tan(desired_steering_angle);
          double leftR = midR - geom_axle_width / 2.0;
          double rightR = midR + geom_axle_width / 2.0;

          desired_steering_angle_left = atan(geom_wheel_base / leftR);
          desired_steering_angle_right = atan(geom_wheel_base / rightR);
        }

        UpdateSteeringPID();

      // Update shock PID
        UpdateShockPID();

        // Odometry
        UpdateOdometryEncoder();

      if (this->model->GetWorld()->GetSimTime() - print_timer > sensor_update_period)
      {
        print_timer = this->model->GetWorld()->GetSimTime();

        // Update TF
        publishTF();

        // Publish Odometry message        joint_drive_left->SetParam("friction", 0, 1);
        publishOdometry();

        // Publish speedometer
        // printf("Speedometer = %10.2f brake flag = %d\n", drive_wheel_speedometer, (int)brake_flag);
        // printf("Speedometer = %10.2f Torque = %10.2f\n", drive_wheel_speedometer, drive_output);

        // Publish PID outputs
        // printf("desired angle = %10.2f, current angle = %10.2f\n", desired_steering_angle_left, joint_steer_left->GetAngle(0).Radian());
        // printf("s_up_limit = %10.2f, s_low_limit = %10.2f\n", steering_upper_limit_left, steering_lower_limit_left);
        // printf("SP = %10.2f, SI = %10.2f, SD = %10.2f, Total_torque = %10.2f\n", SP, SI, SD, steering_left_torque_output);
        // Write CSV data to .txt file

        // Steering debug
        // printf("P_gain = %.3f, desired_vel = %.3f, front_vel = %.3f, rear_des_vel = %.3f, rear_vel = %.3f\n", drive_P, desired_vel, front_axle_avg_vel, rear_axle_desired_vel, rear_axle_vel);



        // printf("drive_output_update = %.6f\n", this->drive_output);
        // math::Vector3 torque_vector = drive_wheel_left->GetRelativeTorque();
        //
        // physics::JointWrench jw = joint_drive_left->GetForceTorque(0);
        // math::Vector3 jw_torque = jw.body1Torque;
        // math::Vector3 jt = joint_drive_left->GetLinkTorque(0);
        //
        // printf("wheel_torque = %.6f wheel_torque2 = %.6f drive_output = %.6f\n", jw_torque[1], jt[1], drive_output);
        // printf("drive_left = %.6f drive_right = %.6f steer_left = %.6f steer_right = %.6f\n", drive_output, drive_output, steering_left_output, steering_right_output);
        // printf("midR = %.3f leftR = %.3f rightR = %.3f\n", midR, leftR, rightR);
        // printf("angle_mid = %.3f angle_left = %.3f angle_right = %.3f\n", desired_steering_angle, desired_steering_angle_left, desired_steering_angle_right);
        // printf("shocks: %.3f %.3f %.3f %.3f\n", shock_output[0], shock_output[1], shock_output[2], shock_output[3]);
      }

      // Apply drive torque
      // drive_wheel_left->SetTorque(math::Vector3(0, this->drive_output, 0));
      // drive_wheel_right->SetTorque(math::Vector3(0, this->drive_output, 0));


      // Apply front brakes if brake_flag is triggered
      // if (brake_flag)
      // {
      //   // joint_free_left->SetForce(0, drive_output);
      //   // joint_free_right->SetForce(0, drive_output);
      //
      //   double brake_torque = 400;
      //   joint_drive_left->SetParam("friction", 0, brake_torque);
      //   joint_drive_right->SetParam("friction", 0, brake_torque);
      //
      //   joint_free_left->SetParam("friction", 0, brake_torque);
      //   joint_free_right->SetParam("friction", 0, brake_torque);
      // }
      // else
      // {
      //   double brake_torque = 1;
      //   joint_drive_left->SetParam("friction", 0, brake_torque);
      //   joint_drive_right->SetParam("friction", 0, brake_torque);
      //
      //   joint_free_left->SetParam("friction", 0, brake_torque);
      //   joint_free_right->SetParam("friction", 0, brake_torque);
      //
      //   joint_drive_left->SetForce(0, drive_output);
      //   joint_drive_right->SetForce(0, drive_output);
      // }

      // Check ebrake
      if (ebrake_flag)
      {
          double brake_torque = 800;
          joint_drive_left->SetParam("friction", 0, brake_torque);
          joint_drive_right->SetParam("friction", 0, brake_torque);

          joint_free_left->SetParam("friction", 0, brake_torque);
          joint_free_right->SetParam("friction", 0, brake_torque);
      }
      else
      {
          double brake_torque = 1;
          joint_drive_left->SetParam("friction", 0, brake_torque);
          joint_drive_right->SetParam("friction", 0, brake_torque);

          joint_free_left->SetParam("friction", 0, brake_torque);
          joint_free_right->SetParam("friction", 0, brake_torque);

          joint_drive_left->SetForce(0, drive_output);
          joint_drive_right->SetForce(0, drive_output);
      }


      // Apply steering torque
      // link_steer_left->SetTorque(math::Vector3(0, 0, this->steering_left_output));
      // link_steer_right->SetTorque(math::Vector3(0, 0, this->steering_right_output));

      joint_steer_left->SetForce(0, steering_left_torque_output);
      joint_steer_right->SetForce(0, steering_right_torque_output);

      // joint_steer_left->SetPosition(0, 0.0);
      // joint_steer_right->SetPosition(0, 0.0);

      // Apply shock forces
      for (int i = 0; i < 4; i++)
      {
        joint_shock[i]->SetForce(0, shock_output[i]);
      }

      // Debug
      // printf("shock outputs = %.3f %.3f %.3f %.3f\n", shock_output[0], shock_output[1], shock_output[2], shock_output[3]);
      // printf("shock_travel = %.3f %.3f %.3f %.3f\n", shock_last_error[0], shock_last_error[1], shock_last_error[2], shock_last_error[3]);
    }

    private: void publishSteeringAngle()
    {
      current_steering_angle_pub.publish(current_steering_angle_msg);
    }

    private: void publishTF()
    {
        ros::Time currentTF_time = ros::Time::now();
        for ( int i = 0; i < 10; i++ ) {

            std::string link_child_frame = tf::resolve(tf_prefix, joints_all[i]->GetChild()->GetName ());
            std::string link_parent_frame = tf::resolve(tf_prefix, joints_all[i]->GetParent()->GetName ());


            // std::string wheel_frame = gazebo_ros_->resolveTF(joints_all[i]->GetChild()->GetName ());
            // std::string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName ());

            math::Pose pose_link = joints_all[i]->GetChild()->GetRelativePose();

            tf::Quaternion qt ( pose_link.rot.x, pose_link.rot.y, pose_link.rot.z, pose_link.rot.w );
            tf::Vector3 vt ( pose_link.pos.x, pose_link.pos.y, pose_link.pos.z );

            tf::Transform tf_link ( qt, vt );
            // transform_broadcaster_->sendTransform (
            //     tf::StampedTransform ( tf_link, currentTF_time, link_parent_frame, link_child_frame ) );
        }
    }


    private: void UpdateOdometryEncoder()
    {
      common::Time current_time = this->model->GetWorld()->GetSimTime();
      double seconds_since_last_update = ( current_time - last_odom_update ).Double();
      last_odom_update = current_time;

      // Get axle parameters
        double drive_left_vel = joint_drive_left->GetVelocity(0);
        double drive_right_vel = joint_drive_right->GetVelocity(0);
        double drive_avg_vel = (drive_left_vel + drive_right_vel) / 2.0;

        double free_left_vel = joint_free_left->GetVelocity(0);
        double free_right_vel = joint_free_right->GetVelocity(0);
        double free_avg_vel = (free_left_vel + free_right_vel) / 2.0;

        double steer_left_angle = joint_steer_left->GetAngle(0).Radian();
        double steer_right_angle = joint_steer_right->GetAngle(0).Radian();
        double steer_avg_angle_est = (steer_left_angle + steer_right_angle) / 2.0;
        double steer_angle_from_left = atan(geom_wheel_base / (geom_wheel_base / tan(steer_left_angle) + geom_axle_width / 2.0));
        double steer_angle_from_right = atan(geom_wheel_base / (geom_wheel_base / tan(steer_right_angle) - geom_axle_width / 2.0));
        double steer_avg_angle = (steer_angle_from_left + steer_angle_from_right) / 2.0;

        // Steer angle message
        current_steering_angle_msg.data = steer_avg_angle;
        publishSteeringAngle();

        // Get rear axle radial velocity around turning circle
        double w_rear = drive_avg_vel * geom_wheel_radius * tan(steer_avg_angle) / geom_wheel_base;

        // Get front axle radial velocity around turning circle
        // double w_front = free_avg_vel * geom_wheel_radius / (sqrt(pow(geom_wheel_base, 2) + pow(geom_wheel_base / tan(steer_avg_angle), 2)));
        double w_front = free_avg_vel * geom_wheel_radius * sin(steer_avg_angle) / geom_wheel_base;

        // Get average radial velocity
        double w_avg = (w_rear + w_front) / 2.0;

        // Get longitudinal and lateral velocity at midpoint of vehicle (location of base_link)
          // Steer angle at midpoint

          // Velocities
          // double vel_total = w_avg * sqrt(pow(geom_wheel_base / 2.0, 2) + pow(geom_wheel_base / tan(steer_avg_angle), 2));
          // double vel_total = w_avg * geom_wheel_base / 2.0 / sin(theta_mid);
          // double vel_x = vel_total * cos(theta_mid);
          // double vel_y = vel_total * sin(theta_mid);
          double vel_y = free_avg_vel * geom_wheel_radius * sin(steer_avg_angle) / 2.0;
          // double vel_x = (free_avg_vel * geom_wheel_radius * cos(steer_avg_angle) + drive_avg_vel * geom_wheel_radius) / 2.0;
          double vel_x = free_avg_vel * geom_wheel_radius * cos(steer_avg_angle);


        // Get position on map
        double dx = vel_x * seconds_since_last_update;
        double dy = vel_y * seconds_since_last_update;
        double dtheta = w_avg * seconds_since_last_update;

        pose_encoder.x += (dx * cos(pose_encoder.theta) - dy * sin(pose_encoder.theta));
        pose_encoder.y += (dy * cos(pose_encoder.theta) + dx * sin(pose_encoder.theta));
        pose_encoder.theta += dtheta;


        tf::Quaternion qt;
        tf::Vector3 vt;
        qt.setRPY ( 0,0,pose_encoder.theta );
        vt = tf::Vector3 ( pose_encoder.x, pose_encoder.y, 0 );

        odom_msg.pose.pose.position.x = vt.x();
        odom_msg.pose.pose.position.y = vt.y();
        odom_msg.pose.pose.position.z = vt.z();

        odom_msg.pose.pose.orientation.x = qt.x();
        odom_msg.pose.pose.orientation.y = qt.y();
        odom_msg.pose.pose.orientation.z = qt.z();
        odom_msg.pose.pose.orientation.w = qt.w();

        odom_msg.twist.twist.angular.z = w_avg;
        odom_msg.twist.twist.linear.x = vel_x;
        odom_msg.twist.twist.linear.y = vel_y;

        // Debug
        // printf("des_angle = %.3f, avg_act_angle = %.3f, est_avg_angle = %.3f\n", desired_steering_angle, steer_avg_angle, steer_avg_angle_est);
        // printf("des_left = %.3f, act_left = %.3f, des_right = %.3f, act_right = %.3f\n",
        //       desired_steering_angle_left, steer_left_angle, desired_steering_angle_right, steer_right_angle);
        // printf("w_rear = %.3f w_front = %.3f vel_x = %.3f vel_y = %.3f \n", w_rear, w_front, vel_x, vel_y);
        // printf("dx = %.3f dy = %.3f dtheta = %.3f sslu = %.3f\n", dx, dy, dtheta, seconds_since_last_update);
    }

    private: void publishOdometry ()
    {

        ros::Time current_time = ros::Time::now();
        // std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
        // std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

        std::string odom_frame = tf::resolve(tf_prefix, odometry_frame);
        std::string odom_truth_frame = tf::resolve(tf_prefix, odometry_truth_frame);

        std::string base_footprint_frame = tf::resolve(tf_prefix, robot_base_frame);


        tf::Quaternion qt;
        tf::Vector3 vt;

        // Publish odometry from encoder
            // getting data form encoder integration
            qt = tf::Quaternion ( odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w );
            vt = tf::Vector3 ( odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z );

            // tf::Transform base_footprint_to_odom ( qt, vt );
            // transform_broadcaster_->sendTransform (
            //     tf::StampedTransform ( base_footprint_to_odom, current_time,
            //                            odom_frame, base_footprint_frame ) );


            // set covariance
            odom_msg.pose.covariance[0] = 0.00001;
            odom_msg.pose.covariance[7] = 0.00001;
            odom_msg.pose.covariance[14] = 1000000000000.0;
            odom_msg.pose.covariance[21] = 1000000000000.0;
            odom_msg.pose.covariance[28] = 1000000000000.0;
            odom_msg.pose.covariance[35] = 0.001;


            // set header
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = odom_frame;
            odom_msg.child_frame_id = base_footprint_frame;

            odometry_pub.publish ( odom_msg );

        // Publish odometry from world
            // getting data form gazebo world
            math::Pose pose = model->GetWorldPose();
            qt = tf::Quaternion ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
            vt = tf::Vector3 ( pose.pos.x, pose.pos.y, pose.pos.z );

            tf::Transform base_footprint_to_odom_truth ( qt, vt );
            transform_broadcaster_->sendTransform (
                tf::StampedTransform ( base_footprint_to_odom_truth, current_time,
                                       odom_truth_frame, base_footprint_frame ) );

            odom_truth_msg.pose.pose.position.x = vt.x();
            odom_truth_msg.pose.pose.position.y = vt.y();
            odom_truth_msg.pose.pose.position.z = vt.z();

            odom_truth_msg.pose.pose.orientation.x = qt.x();
            odom_truth_msg.pose.pose.orientation.y = qt.y();
            odom_truth_msg.pose.pose.orientation.z = qt.z();
            odom_truth_msg.pose.pose.orientation.w = qt.w();

            // get velocity in /odom frame
            math::Vector3 linear;
            linear = model->GetWorldLinearVel();
            odom_truth_msg.twist.twist.angular.z = model->GetWorldAngularVel().z;

            // convert velocity to child_frame_id (aka base_footprint)
            float yaw = pose.rot.GetYaw();
            odom_truth_msg.twist.twist.linear.x = cosf ( yaw ) * linear.x + sinf ( yaw ) * linear.y;
            odom_truth_msg.twist.twist.linear.y = cosf ( yaw ) * linear.y - sinf ( yaw ) * linear.x;


            // tf::Transform base_footprint_to_odom_truth ( qt, vt );
            // transform_broadcaster_->sendTransform (
            //     tf::StampedTransform ( base_footprint_to_odom_truth, current_time,
            //                            odom_truth_frame, base_footprint_frame ) );


            // set covariance
            odom_truth_msg.pose.covariance[0] = 0.00001;
            odom_truth_msg.pose.covariance[7] = 0.00001;
            odom_truth_msg.pose.covariance[14] = 1000000000000.0;
            odom_truth_msg.pose.covariance[21] = 1000000000000.0;
            odom_truth_msg.pose.covariance[28] = 1000000000000.0;
            odom_truth_msg.pose.covariance[35] = 0.001;


            // set header
            odom_truth_msg.header.stamp = current_time;
            odom_truth_msg.header.frame_id = odom_truth_frame;
            odom_truth_msg.child_frame_id = base_footprint_frame;

            odometry_truth_pub.publish ( odom_truth_msg );
            pose.pos.z += 2;
            if (ebrake_flag)
            {
              msgs::Set(visualMsg.mutable_material()->mutable_diffuse(), common::Color(1.0, 0, 0, 1.0));
              msgs::Set(visualMsg.mutable_material()->mutable_emissive(), common::Color(0.0, 0.0, 0.0, 0.0));
              msgs::Set(visualMsg.mutable_material()->mutable_specular(), common::Color(0.1, 0.1, 0.1, 1.0));
              msgs::Set(visualMsg.mutable_material()->mutable_ambient(), common::Color(1.0, 0.0, 0.0, 1.0));


            }
            else
            {
              msgs::Set(visualMsg.mutable_material()->mutable_diffuse(), common::Color(0.0, 0, 0, 1.0));
              msgs::Set(visualMsg.mutable_material()->mutable_emissive(), common::Color(0.0, 0.0, 0.0, 0.0));
              msgs::Set(visualMsg.mutable_material()->mutable_specular(), common::Color(0.1, 0.1, 0.1, 1.0));
              msgs::Set(visualMsg.mutable_material()->mutable_ambient(), common::Color(1.0, 0.0, 0.0, 1.0));


            }
            msgs::Set(visualMsg.mutable_pose(), pose.Ign());
            // visPub->Publish(visualMsg);


    }


    // Gazebo
      /// \brief Pointer to the model.
      private: physics::ModelPtr model;

      /// \brief Pointer to the joint.
      private: physics::LinkPtr drive_wheel_left, drive_wheel_right;
      private: physics::LinkPtr free_wheel_left, free_wheel_right;

      private: physics::JointPtr joint_drive_left, joint_drive_right;
      private: physics::JointPtr joint_free_left, joint_free_right;

      private: physics::JointPtr joint_steer_left, joint_steer_right;
      private: physics::LinkPtr link_steer_left, link_steer_right;

      private: physics::JointPtr joint_shock_front_left;
      private: physics::JointPtr joint_shock_front_right;
      private: physics::JointPtr joint_shock_rear_left;
      private: physics::JointPtr joint_shock_rear_right;
      private: std::vector<physics::JointPtr> joint_shock;
      private: std::vector<physics::JointPtr> joints_all;

      /// \brief A PID controller for the joint.
      private: common::PID pid_drive_left;
      private: common::PID pid_drive_right;

      // World iteration update
      private: event::ConnectionPtr update_connection_;

      // Lock
      private: boost::mutex lock;

      // Debugging
      private: bool first_iter = false;


    // ROS
      /// \brief A node use for ROS transport
      private: std::string robot_namespace;
      private: std::unique_ptr<ros::NodeHandle> rosNode;
      private: std::unique_ptr<ros::NodeHandle> rosNode_steering;
      private: std::unique_ptr<ros::NodeHandle> rosNode_drive;
      private: std::unique_ptr<ros::NodeHandle> rosNode_drive_brake;
      private: std::unique_ptr<ros::NodeHandle> rosNode_suspension;


      /// \brief A ROS subscriber
      private: ros::Subscriber rosSub;

      /// \brief A ROS callbackqueue that helps process messages
      private: ros::CallbackQueue rosQueue;

      /// \brief A thread the keeps running the rosQueue
      private: std::thread rosQueueThread;

      // Dynamic reconfigure
      private: dynamic_reconfigure::Server<g35_gazebo::SteeringParamsConfig> *server_steering;
      private: dynamic_reconfigure::Server<g35_gazebo::DriveParamsConfig> *server_drive;
      private: dynamic_reconfigure::Server<g35_gazebo::SuspensionParamsConfig> *server_suspension;
      private: dynamic_reconfigure::Server<g35_gazebo::DriveBrakeParamsConfig> *server_drive_brake;


      // Transform broadcaster
      private: boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      private: std::string tf_prefix;

      // Odometry publisher ENCODER
      private: ros::Publisher odometry_pub;
      private: geometry_msgs::Pose2D pose_encoder;
      private: common::Time last_odom_update;
      nav_msgs::Odometry odom_msg;
      private: std::string odometry_frame;
      OdomSource odom_source;
      private: std::string robot_base_frame;

      // Odometry publisher TRUTH
      private: ros::Publisher odometry_truth_pub;
      private: geometry_msgs::Pose2D pose_encoder_truth;
      nav_msgs::Odometry odom_truth_msg;
      private: std::string odometry_truth_frame;

      // Steer angle publisher
      private: std_msgs::Float64 current_steering_angle_msg;
      private: ros::Publisher current_steering_angle_pub;

      // Control variables
      private: common::Time print_timer;
      private: double desired_vel_msg, desired_steering_angle_msg;
      private: double desired_vel, desired_steering_angle, desired_steering_angle_left, desired_steering_angle_right;
      private: double desired_shock_position;
      private: bool launch_control_on;

      // Vehicle geometry variables
      private: double geom_wheel_base;
      private: double geom_axle_width;
      private: double geom_wheel_radius;

      // Timing variables
      private: double sensor_update_period;

      // PID
        // Drive wheels
        private: double drive_P, drive_I, drive_D, drive_Imax, drive_Imin, drive_error, drive_last_error;
        private: double drive_proportional, drive_integral, drive_derivative;
        private: double drive_output, drive_torque_max, drive_torque_min;
        private: double drive_P_accel, drive_P_coast, drive_P_brake, drive_I_accel, drive_I_coast, drive_I_brake, drive_D_accel, drive_D_coast, drive_D_brake;
        private: double front_axle_avg_vel, rear_axle_desired_vel, rear_axle_vel;
        private: double wheel_velocity_limit, wheel_friction;

        private: bool brake_flag, ebrake_flag;
        private: double drive_wheel_speedometer;
        private: double brakeFL, brakeFR, brakeRL, brakeRR;
        private: double brakePos, throttlePos;

        // Steering
        private: double steering_P, steering_I, steering_D, steering_Imax, steering_Imin;
        private: double steering_left_integral, steering_left_last_error, steering_right_integral, steering_right_last_error;
        private: double steering_left_output, steering_right_output;
        private: double steering_torque_max, steering_torque_min;

        private: double steering_vel_P, steering_vel_I, steering_vel_D, steering_vel_Imax, steering_vel_Imin;
        private: double steering_vel_left_integral, steering_vel_right_integral, steering_vel_left_last_error, steering_vel_right_last_error;
        private: double steering_left_torque_output, steering_right_torque_output;
        private: double steering_vel_max, steering_vel_min;

        private: double steering_high, steering_low, steering_friction;

        private: double steering_upper_limit_left, steering_lower_limit_left;
        private: double SP, SI, SD;



        // Shocks
        private: std::vector<double> shock_last_error, shock_integral, shock_output;
        private: double shock_P, shock_I, shock_D, shock_Imax, shock_Imin;

        private: double shock_effort_limit, shock_velocity_limit, shock_high_stop;
        // private: double drive_proportional, drive_integral, drive_derivative;
        // private: double drive_output, drive_torque_max, drive_torque_min;


      private: common::Time prevUpdateTime;
      private: common::Time stepTime;

      // Writing to file
      private: FILE *data_file;

      // Gazebo Visuals
      private: transport::NodePtr node;
      private: transport::PublisherPtr visPub;
      private: msgs::Visual visualMsg;

      private: physics::LinkPtr visual_link;


  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)
}
#endif
