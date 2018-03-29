/*
  \file     obst_avoid_nmpc_v2.h
  \brief    Header file for Obstacle Avoidance NMPC Algorithm.

  \author   Stephen Geiger <sag0020@tigermail.auburn.edu>
  \date     May 9, 2017
*/

#ifndef OBST_AVOID_NMPC_H
#define OBST_AVOID_NMPC_H

// ROS Includes
#include <ros/ros.h>
#include <ros/time.h>

// C++ Includes
#include <math.h>
#include <tgmath.h>
#include <cmath>
#include <vector>
#include <random>

// Eigen Package include
#include <eigen3/Eigen/Dense>

// Define PI
#ifndef PI
#define PI 3.141592653589793
#endif

class ObstAvoidNMPC
{
public:
	ObstAvoidNMPC();
	~ObstAvoidNMPC(){};

	std::vector <double> init_lla;
	std::vector <double> target_POS;
	double target_n[3];
	Eigen::VectorXd target;
	struct Obstacle
	{
		bool ARE_OBSTACLES;
		int NUM_OBSTACLES;
		Eigen::MatrixXd pos;
		Eigen::MatrixXd vel;
	};
	Obstacle obstacle;
	Eigen::VectorXd dist_obst;
	bool initTarget;
	bool initObstacle;
	bool initState;
	double r_avoid;
	int VEHICLE_ID;

	double t_h;
	// double t_c;
	double vel_max;
	int ARE_OBSTACLES_COST_FUNCTION;
	int NO_OBSTACLES_COST_FUNCTION;
	double gamma;
	double zeta;
	double chi;
	double kappa;
	double xi;
	double epsilon;

	Eigen::VectorXd optimization(Eigen::VectorXd X, Eigen::VectorXd u, Eigen::VectorXd u_last, double t_h, double dt, double r_avoid, Eigen::Vector3d target, Obstacle obstacle);

private:
	// ----- Pattern Search Variables -----
	Eigen::MatrixXd zero_vec;
	Eigen::MatrixXd eye4;
	Eigen::MatrixXd D;
	Eigen::Vector4d u_pattern;
	double tau;
	double w0;
	double theta;
	double lambda;
	double del_u;
	double del_u_threshold;
	bool success;
	double J_pattern;
	int constr_viol_pat;

	// ----- Particle Swarm Variables -----
	double mu;
	double nu;
	double vel_threshold;
	int numIts;
	double particle;
	double J_min;
	bool SUCCESS_POLL;
	bool ATTEMPT_1;
	bool ATTEMPT_2;
	Eigen::MatrixXd u_particles;
	Eigen::MatrixXd u_hat;
	Eigen::MatrixXd best_u_particles;
	Eigen::MatrixXd vel_particles;
	Eigen::VectorXd u_optimal;
	Eigen::VectorXi constraints;
	Eigen::VectorXd J_particles;
	Eigen::VectorXd J_best_particles;
	std::default_random_engine generator;
	double J_hat;
	int constr_viol_hat;


	// ----- Variables for cost evaluation -----
	Eigen::VectorXd X_prev;
	double J_total;
	Eigen::MatrixXd pred_obst_pos;
	Eigen::MatrixXd pred_obst_vel;
	double t;
	double delta_f;
	double delta_f_prev;
	double rel_x_obst;
	double rel_y_obst;
	double min_dist_ob;
	double err_yaw_thresh;
	Eigen::VectorXd err_yaw_ob;
	double min_err_yaw_ob;
	double rel_x_tar;
	double rel_y_tar;
	double dist_tar;
	double err_yaw;
	double err_norm;
	double J;
	double cost;
	int constr_viol;

	// ----- Variables for Runge-Kutta 4th Order -----
	double t_rk4;
	Eigen::VectorXd k1;
	Eigen::VectorXd k2;
	Eigen::VectorXd k3;
	Eigen::VectorXd k4;
	Eigen::VectorXd X_k2;
	Eigen::VectorXd X_k3;
	Eigen::VectorXd X_k4;

	// ----- Variables for Vehicle Model -----
	double mass;
	double Izz;
	double a;
	double b;
	double Calpha_f;
	double Calpha_r;
	double X_pos;
	double Y_pos;
	double psi;
	double psi_dot;
	double v_y;
	double B_tire, C_tire, D_tire, E_tire;
	double alpha_f, alpha_r;
	double Fy_f, Fy_r;
	double psi_ddot;
	double v_y_dot;
	double X_dot;
	double Y_dot;


	Eigen::VectorXd vehicle_model(Eigen::VectorXd X, Eigen::VectorXd u, double t, double t_h, double dt);
	void runge_kutta_4(Eigen::VectorXd& X, Eigen::VectorXd u, double t, double t_h, double dt);
	void cost_evaluation(double& cost, int& constr_eval, Eigen::VectorXd X, Eigen::VectorXd u, Eigen::VectorXd u_last, double t_h, double dt, double r_avoid, Eigen::Vector3d target, Obstacle obstacle);

};


#endif
