/**
* Kalman filter implementation using Eigen. Based on the following
* introductory paper:
*
*     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#pragma once

class Tracker {

public:

  /**
  * Create a tracker with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  Tracker(
      int association_limit,
      int dissociation_limit,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& Bw,
      const Eigen::MatrixXd& H,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  );

  // Destructor
  ~Tracker();

  /**
  * Initialize the tracker with a guess for initial states.
  */
  void init(double t0, const Eigen::VectorXd& x0);

  /**
  * Reset the tracker (due to lost track)
  */
  void reset();

  /**
  * Time update of tracker
  */
  void timeUpdate(double dt);

  /**
  * Get the Mahalanobis distance to given measurement
  */
  double getMahalanobis(Eigen::VectorXd& y);

  /**
  * Check if measurement falls within validation gate
  */
  bool isValid(double chi_squared, double chi_thresh);

  /**
  * Update tracker with previously given measurement and return estimate
  */
  Eigen::VectorXd measurementUpdate(Eigen::VectorXd& y);

  /**
  * Increment association variable
  */
  void incAssociation();

  /**
  * Increment dissociation variable
  */
  void incDissociation();

  /**
  * Get ACTIVE flag so program can see of tracker has dissociated itself from object
  */

  bool isActive();

  /**
  * Set ID for this tracker
  */
  void setID(int tracker_id);

  /**
  * Get ID of this tracker
  */
  int getID();

  /**
  * Calculate vector of covariance ellipse parameters
  */
  Eigen::VectorXd getEllipse(double chi_thresh);

  /**
  * Return state vector
  */
  Eigen::VectorXd getState();

private:

  // Matrices for computation
  Eigen::MatrixXd A, Bw, H, Q, R, P, K, P0;
  Eigen::MatrixXd S;  // innovation covariance

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the active?
  bool active;

  // Tracker ID
  int tracker_id;

  // Association and dissociation limits
  int association;
  int dissociation;
  int association_limit;
  int dissociation_limit;

  // n-size identity
  Eigen::MatrixXd I;

  // Measurement vector
  Eigen::VectorXd y_hat;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;



};
