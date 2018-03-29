/*
  \file     obst_avoid_nmpc_v2.cpp
  \brief    C++ file for Obstacle Avoidance NMPC Algorithm.

  \author  Stephen Geiger <sag0020@tigermail.auburn.edu>
  \date     May 9, 2017
*/

#include "obst_avoid_nmpc.h"

ObstAvoidNMPC::ObstAvoidNMPC(): zero_vec(4,1), eye4(4,4), D(4,9), target(3), u_optimal(4)
{
	eye4.setIdentity();
	zero_vec.setZero();
}

Eigen::VectorXd ObstAvoidNMPC::vehicle_model(Eigen::VectorXd X, Eigen::VectorXd u, double t, double t_h, double dt)
{
	Eigen::VectorXd dyn_eqs(5);

	delta_f = u(0) + u(1)*t/t_h + u(2)*pow(t/t_h,2);
	// if (t <= t_c)
	// {
	// 	delta_f = u(0) + u(1)*t/t_c + u(2)*pow(t/t_c,2);
	// } else {
	// 	delta_f = u(0) + u(1) + u(2);
	// }

	// ----- Vehicle parameters -----
	if (VEHICLE_ID == 0)
	{
		// Prowler Parameters
		mass = 544; 	 //  Mass               [kg]
		Izz = 280; 		 //  Yaw intertia       [kg-m^2]
		a = 1.4478*0.48; //  front weight split [m]
		b = 1.4478*0.52; //  rear weight split  [m]
	} else if (VEHICLE_ID == 1) {
		// G-35 Parameters
		mass = 1728.15;  //  Mass               [kg]
		Izz = 2400;		 //  Yaw intertia       [kg-m^2]
		a = 2.849*0.48;  //  front weight split [m]
		b = 2.849*0.52;  //  rear weight split  [m]
	}

	// ----- Tire Parameters -----
	B_tire = 9.55;
	C_tire = 1.3;
	D_tire = 6920;
	// D_tire = 3000;
	E_tire = 0;
	Calpha_f = 40000;
	Calpha_r = 40000;
	// Calpha_f = 91674;
	// Calpha_r = 152788;

	// ----- Assign Vehicle States -----
	X_pos = X(0);
	Y_pos = X(1);
	psi = X(2);
	psi_dot = X(3);
	v_y = X(4);

	// ----- Calculate Slip Angles -----
	alpha_f = delta_f - atan2(v_y + a*psi_dot, u(3));
	alpha_r = atan2(-v_y + b*psi_dot, u(3));

	// ----- Calculate Lateral Forces -----
	// Fy_f = D_tire * sin(C_tire * atan2(B_tire*alpha_f - E_tire*(B_tire*alpha_f - atan2(B_tire*alpha_f,1)),1));
	// Fy_r = D_tire * sin(C_tire * atan2(B_tire*alpha_r - E_tire*(B_tire*alpha_r - atan2(B_tire*alpha_r,1)),1));
	Fy_f = Calpha_f*alpha_f;
	Fy_r = Calpha_r*alpha_r;

	// ----- Dynamic Equations -----
	psi_ddot = 1/Izz * (a*Fy_f*cos(delta_f) - b*Fy_r);
	v_y_dot = 1/mass * (Fy_f*cos(delta_f) + Fy_r) - u(3)*psi_dot;
	X_dot = u(3) * cos(psi) - v_y * sin(psi);
	Y_dot = u(3) * sin(psi) + v_y * cos(psi);

	// ----- Assign Output Variable -----
	dyn_eqs << X_dot, Y_dot, psi_dot, psi_ddot, v_y_dot;
	return dyn_eqs;
}

void ObstAvoidNMPC::runge_kutta_4(Eigen::VectorXd& X, Eigen::VectorXd u, double t, double t_h, double dt)
{
	// std::cout << "Numerical Integration: rk4" << std::endl;
	t_rk4 = t;
	k1 = vehicle_model(X, u, t_rk4, t_h, dt);

	t_rk4 = t + 0.5*dt;
	X_k2 = X + 0.5*dt*k1;
	k2 = vehicle_model(X_k2, u, t_rk4, t_h, dt);

	t_rk4 = t + 0.5*dt;
	X_k3 = X + 0.5*dt*k2;
	k3 = vehicle_model(X_k3, u, t_rk4, t_h, dt);

	t_rk4 = t + dt;
	X_k4 = X + dt*k3;
	k4 = vehicle_model(X_k4, u, t_rk4, t_h, dt);

	X = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);
}

void ObstAvoidNMPC::cost_evaluation(double& cost, int& constr_viol, Eigen::VectorXd X, Eigen::VectorXd u, Eigen::VectorXd u_last, double t_h, double dt, double r_avoid, Eigen::Vector3d target, Obstacle obstacle)
{
	cost = 0;
	J = 0;
	J_total = 0;
	pred_obst_pos = obstacle.pos;
	pred_obst_vel = obstacle.vel;
	xi = 15; // Weight on yaw rate
	t = 0;
	min_dist_ob = pow(10,10);
	min_err_yaw_ob = pow(10,20);
	err_yaw_thresh = 3*PI/180;

if (obstacle.ARE_OBSTACLES)
{
	for (int i=0; i<obstacle.NUM_OBSTACLES; i++)
	{
		pred_obst_pos(i,0) = obstacle.pos(i,0)*cos(X(2)) - obstacle.pos(i,1)*sin(X(2)) + X(0);
		pred_obst_pos(i,1) = obstacle.pos(i,0)*sin(X(2)) + obstacle.pos(i,1)*cos(X(2)) + X(1);
		// u_last(1) and u_last(2) are used to pass through the vehicle velocity in the navigation frame.
		pred_obst_vel(i,0) = u_last(1) + obstacle.vel(i,0)*cos(X(2)) - obstacle.vel(i,1)*sin(X(2)) + X(3) * (obstacle.pos(i,0)*cos(X(2)) - obstacle.pos(i,1)*sin(X(2)));
		pred_obst_vel(i,1) = u_last(2) + obstacle.vel(i,0)*sin(X(2)) + obstacle.vel(i,1)*cos(X(2)) - X(3) * (obstacle.pos(i,0)*sin(X(2)) + obstacle.pos(i,1)*cos(X(2)));
	}
}
	while (t <= t_h)
	{
		if (t>0)
		{
			delta_f_prev = u(0) + u(1)*(t-dt)/t_h + u(2)*pow((t-dt)/t_h,2);
		}

		// ----- Front Steer Angle -----
			delta_f = u(0) + u(1)*t/t_h + u(2)*pow(t/t_h,2);

		// ----- Project One Time Step Using RK4 -----
		X_prev = X;
		runge_kutta_4(X,u,t,t_h,dt);
		// ----- Perform if there are Obstacles -----
		if (obstacle.ARE_OBSTACLES)
		{
			// ----- Project the Obstacles Forward in Time -----
			dist_obst.resize(obstacle.NUM_OBSTACLES);
			dist_obst.setZero();
			err_yaw_ob.resize(obstacle.NUM_OBSTACLES);
			err_yaw_ob.setZero();
			for (int i=0; i < obstacle.NUM_OBSTACLES; i++)
			{
				// Using a constant velocity, constant heading model here
				pred_obst_pos(i,0) = pred_obst_pos(i,0) + dt*(pred_obst_vel(i,0));
				pred_obst_pos(i,1) = pred_obst_pos(i,1) + dt*(pred_obst_vel(i,1));

				// Distance Magnitude
				dist_obst(i) = pow(pow(pred_obst_pos(i,0)-X(0),2) + pow(pred_obst_pos(i,1)-X(1),2) ,0.5);
				// Evaluate Minimum Distance to Obstacle
				if (i>0 && dist_obst(i) < dist_obst(i-1))
				{
					min_dist_ob = dist_obst(i);
				} else if (i == 0) {
					min_dist_ob = dist_obst(i);
				}

				// Obstacle Yaw Error Measured in Body Frame
				err_yaw_ob(i) = atan2((obstacle.pos(i,1)-X(1))*cos(X(2))-(obstacle.pos(i,0)-X(0))*sin(X(2)),(obstacle.pos(i,1)-X(1))*sin(X(2))+(obstacle.pos(i,0)-X(0))*cos(X(2)));
				// Evaluate Minimum Yaw Angle that is smaller than the threshold
				if (i>0 && err_yaw_ob(i) < err_yaw_ob(i-1) && fabs(err_yaw_ob(i)) < err_yaw_thresh)
				{
					min_err_yaw_ob = err_yaw_ob(i);
				} else if (i == 0 && fabs(err_yaw_ob(i)) < err_yaw_thresh) {
					min_err_yaw_ob = err_yaw_ob(i);
				}
			}
		} else {
			// If there aren't any obstacles, the minimum distance is set to a large value to prevent an accidental application of the hard constraint on obstacle avoidance
			min_dist_ob = pow(10,10);
			min_err_yaw_ob = pow(10,20);
		}

		// ----- Relative Distance to Target -----
		rel_x_tar = target(0) - X(0);
		rel_y_tar = target(1) - X(1);
		dist_tar = pow(pow(rel_x_tar,2) + pow(rel_y_tar,2) ,0.5); // Distance Magnitude
		// Resolve target into body frame and calculate yaw error
		err_yaw = atan2(rel_y_tar*cos(X(2))-rel_x_tar*sin(X(2)),rel_y_tar*sin(X(2))+rel_x_tar*cos(X(2)));
		// Determine offset from path using Law of Cosines
		err_norm = dist_tar*sin(acos(((pow(X(0),2)+pow(X(1),2))-pow(dist_tar,2)-(pow(target(0),2)+pow(target(1),2)))/(-2*dist_tar*pow(pow(target(0),2)+pow(target(1),2),0.5))));

		// ----- Evaluate Cost -----
		// J = 0.5*(gamma*pow(dist_tar,2) + epsilon*pow(delta_f,2) + xi*pow(X(3),2) + chi*pow(delta_f - delta_f_prev,2));
		// J = 0.5*(gamma*pow(dist_tar,2) + kappa*pow(err_yaw,2));
		// J = 0.5*(gamma*pow(dist_tar,2) + zeta*pow(1/min_dist_ob,2));
		// J = 0.5*(gamma*pow(dist_tar,2) + zeta*pow(1/min_dist_ob,2) + chi*pow(1/min_err_yaw_ob,2));
		// J = 0.5*(gamma*pow(dist_tar,2));
		if (obstacle.ARE_OBSTACLES)
		{
			if (ARE_OBSTACLES_COST_FUNCTION == 0) {
				J = 0.5*(gamma*pow(dist_tar,2) + zeta*pow(1/min_dist_ob,2) + epsilon*pow(err_norm,2) + kappa*pow(err_yaw,2));
			} else if (ARE_OBSTACLES_COST_FUNCTION == 1) {
			  J = 0.5*(gamma*pow(dist_tar,2) + zeta*pow(1/min_dist_ob,2) + kappa*pow(err_yaw,2));
			} else if (ARE_OBSTACLES_COST_FUNCTION == 2) {
			  J = 0.5*(gamma*pow(dist_tar,2) + zeta*pow(1/min_dist_ob,2) + epsilon*pow(err_norm,2));
			} else if (ARE_OBSTACLES_COST_FUNCTION == 3) {
				J = 0.5*(gamma*pow(dist_tar,2) + zeta*pow(1/min_dist_ob,2));
			}
		} else {
			if (NO_OBSTACLES_COST_FUNCTION == 0)
			{
				J = 0.5*(gamma*pow(dist_tar,2) + kappa*pow(err_yaw,2) + epsilon*pow(err_norm,2));
			} else if (NO_OBSTACLES_COST_FUNCTION == 1) {
				J = 0.5*(gamma*pow(dist_tar,2) + kappa*pow(err_yaw,2));
			} else if (NO_OBSTACLES_COST_FUNCTION == 2) {
				J = 0.5*(gamma*pow(dist_tar,2) + epsilon*pow(err_norm,2));
			} else if (NO_OBSTACLES_COST_FUNCTION == 3) {
				J = 0.5*(gamma*pow(dist_tar,2));
			}
		}

		J_total = J_total + J;

		// ----- Apply Hard Constraints -----
		// Minimum distance to obstacle
		// printf("Maximum Velocity: %10.2f\n",vel_max);
		// if ((min_dist_ob < r_avoid) && (obstacle.ARE_OBSTACLES))
		// {
		// 	// std::cout << "Breaking the r_avoid constraint" << std::endl;
		// 	constr_viol = 1;
		// 	J_total = pow(10,20);
		// 	t = t_h;
		// Maximum steer angle
		// } else
		if (fabs(delta_f) > 35*PI/180) {
			constr_viol = 1;
			J_total = pow(10,20);
			t = t_h + 10;
		// // Maximum steer rate
	} else if (fabs(u(0) - u_last(0)) > 5*PI/180) {
			constr_viol = 1;
			J_total = pow(10,20);
			t = t_h + 10;
		// } else if (fabs(delta_f-delta_f_prev) > 0.5*PI/180) {
		// 	constr_viol = 1;
		// 	J_total = pow(10,20);
		// 	t = t_h;
		// Maximum velocity
		} else if (u(3) > vel_max) {
			constr_viol = 1;
			J_total = pow(10,20);
			t = t_h + 10;
		// Minimum velocity
		} else if (u(3) < 0) {
			constr_viol = 1;
			J_total = pow(10,20);
			t = t_h + 10;
		// Maximum Longitudinal Acceleration
	// } else if (fabs(u_last(3) - u(3)) > 2.5) {
	// 		constr_viol = 1;
	// 		J_total = pow(10,20);
	// 		t = t_h;
		// No broken constraints
		} else {
			constr_viol = 0;
		}

		t = t + dt;
	}
	cost = J_total;
}

Eigen::VectorXd ObstAvoidNMPC::optimization(Eigen::VectorXd X, Eigen::VectorXd u, Eigen::VectorXd u_last, double t_h, double dt, double r_avoid, Eigen::Vector3d target, Obstacle obstacle)
{
	// ----- Pattern Search Initializations -----
	D << 1, 0, 0, 0, -1,  0,  0,  0, 0,
		 0, 1, 0, 0,  0, -1,  0,  0, 0,
		 0, 0, 1, 0,  0,  0, -1,  0, 0,
		 0, 0, 0, 1,  0,  0,  0, -1, 0;

	tau = 2;
	w0 = -1;
	theta = pow(tau,w0);
	lambda = 1;
	del_u = 1;
	// del_u_threshold = 0.005;
	del_u_threshold = 0.01;

	// ----- Particle Swarm Initializations -----
	vel_threshold = 0.5;
	ATTEMPT_1 = false;
	ATTEMPT_2 = false;
	SUCCESS_POLL = false;
	mu = 1; // Cognitition parameter: mu > 0
	nu = 1; // Social parameter: nu <= 2
	// Number of particles
	const int s = 10;
	// const int xmax = 0.05;
	const int xmax = PI/2;
	const int xmin = -xmax;
	const double vel_distr_max = u_last(3) + 2.5;
	// const double vel_distr_min = u_last(3) - 2.5;
	// const double vel_distr_max = vel_max;
	J_min = pow(10,30);
	J_hat = pow(10,20);
	u_optimal << 0, 0, 0, 0;
	numIts = 1;
	cost = pow(10,10);
	constr_viol = 1;
	constr_viol_hat = 1;
	// constraints = Eigen::MatrixXd::Zero(s,1);
	constraints.setZero(s);
	u_particles = Eigen::MatrixXd::Zero(4,s);
	u_hat = Eigen::MatrixXd::Zero(4,s);
	best_u_particles = Eigen::MatrixXd::Zero(4,s);
	vel_particles = Eigen::MatrixXd::Zero(4,s);
	Eigen::VectorXd vel_part_norm(s);
	// J_particles = Eigen::MatrixXd::Zero(s,1);
	J_particles.setZero(s);
	std::uniform_real_distribution<double> distribution1(-3*PI/180,3*PI/180);
	std::normal_distribution<double> distribution_norm(0,57*PI/180);
	std::uniform_real_distribution<double> distribution2(xmin,xmax);
	// std::uniform_real_distribution<double> distribution3(vel_distr_min,vel_distr_max);
	std::uniform_real_distribution<double> distribution3(0,vel_max);
	std::uniform_real_distribution<double> distribution4(0,1);


	for (int i=0; i<s; i++)
	{
		constraints(i) = 1.0;
	}

	// std::cout << "Initializing Particles" << std::endl;
	// while (u_optimal.rows() < 3)
	// {
		for (int i=0; i < s; i++)
		{
			// std::cout << "Initializing Particle " << i+1 << std::endl;
			while (constraints(i) == 1) //&& numIts < 500)
			{
				// if (numIts > 5000)
				// {
				// 	std::cout << "Stuck In Initializaiton loop" << std::endl;
				// }
				// if (i == 0 && ATTEMPT_1 == false)
				// {
				// 	u_particles.col(i) = Eigen::MatrixXd::Zero(4,1);
			  // } else
				 if (i == 1 && ATTEMPT_2 == false) {
					u_particles.col(i) << 0, 0, 0, vel_max;
				} else {
					// u_particles(0,i) = u_last(0) + distribution1(generator);
					u_particles(0,i) = u_last(0) * distribution_norm(generator);
					u_particles(1,i) = distribution2(generator);
					u_particles(2,i) = distribution2(generator);
					u_particles(3,i) = distribution3(generator);
				}

			    // printf("Run Cost Evaluation\n");
				cost_evaluation(cost, constr_viol, X, u_particles.col(i), u_last, t_h, dt, r_avoid, target, obstacle);
				J_particles(i) = cost;
				constraints(i) = constr_viol;
				if (i == 0 && constraints(i) == 1)
				{
					ATTEMPT_1 = true;
				} else if (i == 1 && constraints(i) == 1) {
					ATTEMPT_2 = true;
				}
				numIts++;
			}
				if (J_particles(i) < J_min && constraints(i) == 0)
				{
					J_min = J_particles(i);
					u_optimal = u_particles.col(i);
				}
			numIts = 1;
		}

	J_best_particles = J_particles;
	best_u_particles = u_particles;

	vel_particles = mu*distribution4(generator)*u_particles + nu*distribution4(generator)*u_particles;

	numIts = 1;

	while (del_u > del_u_threshold) //&& numIts < 100)
	{
		// ----- Search Step -----
		for (int i=0; i < s; i++)
		{
			int k = 0;

			while (k <= 8)
			{
				u_pattern = u_particles.col(i) + del_u * D.col(k);
				cost_evaluation(J_hat, constr_viol_hat, X, u_pattern, u_last, t_h, dt, r_avoid, target, obstacle);
				if ((J_hat < J_particles(i)) && (constr_viol_hat == 0))
				{
					J_particles(i) = J_hat;
					u_particles.col(i) = u_pattern;
					if (k <= 6)
					{
						k = 7;
					}
					SUCCESS_POLL = true;
				}
				k++;
			}

			// Check if new input is better than best input for particle
			if ((J_particles(i) < J_best_particles(i)) && (constr_viol_hat == 0))
			{
				best_u_particles.col(i) = u_particles.col(i);
				J_best_particles(i) = J_particles(i);

				// Check if new input is the optimal input
				if (J_best_particles(i) < J_min)
				{
					// std::cout << "Update Optimal Input" << std::endl;
					u_optimal = best_u_particles.col(i);
					J_min = J_best_particles(i);
					del_u = lambda*del_u;
				}
			}

			// Update Particles

			// if (i == 0)
			// {
			// 	u_particles.col(i) << 0, 0, 0, 0;
			// } else
			if(i == 1) {
				u_particles.col(i) << 0, 0, 0, vel_max;
			} else {
				vel_particles.col(i) = vel_particles.col(i) + mu*distribution4(generator)*(best_u_particles.col(i) - u_particles.col(i)) +
							    nu*distribution4(generator)*(u_optimal - u_particles.col(i));
				u_particles.col(i) = u_particles.col(i) + vel_particles.col(i);
			}

			// for (int i=0;i<vel_particles.cols(); i++)
			// {
			// 	vel_part_norm(i) = pow(pow(vel_particles(0,i),2)+pow(vel_particles(1,i),2)+pow(vel_particles(2,i),2)+pow(vel_particles(3,i),2),0.5);
			// }

		}

		// Evaluate Successful Iteration
		if (SUCCESS_POLL)
		{
			del_u = lambda*del_u;
		} else {
			del_u = theta*del_u;
		}

		SUCCESS_POLL = false;
		numIts++;
	}

 // 	 printf("Finish Optimization\n");
	return u_optimal;
}
