/**
* Implementation of KalmanFilter class.
*
* @author: Velislav Stamenov
* @date: 2017.05.27
*/

#include <iostream>
#include <stdexcept>

#include "tracker.h"

// Constructor
Tracker::Tracker(
    int association_limit,
    int dissociation_limit,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& Bw,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
{
  // I.setIdentity();
  this->association_limit = association_limit;
  this->dissociation_limit = dissociation_limit;
  this->A = A;
  this->Bw = Bw;
  this->H = H;
  this->Q = Q;
  this->R = R;
  this->P0 = P;
  this->P = P;
  this->S = P;
  this->m = H.rows();
  this->n = A.rows();
  this->active = false;
  this->I.resize(this->n,this->n);
  this->I.setIdentity();
  this->x_hat.resize(this->n);
  this->x_hat_new.resize(this->n);
  this->y_hat.resize(this->m);

  // : association_limit(association_limit), dissociation_limit(dissociation_limit),
  //   A(A), Bw(Bw), H(H), Q(Q), R(R), P0(P), P(P), S(P),
  //   m(H.rows()), n(A.rows()), active(false),
  //   I(n, n), x_hat(n), x_hat_new(n), y_hat(m)
}

// Destructor
Tracker::~Tracker() {}

// Initialize tracker
void Tracker::init(double t0, const Eigen::VectorXd& x0){
  // Set tracking start time
    this->t0 = t0;
  // Set association counter
    association = 1;
  // Set dissociation counter
    dissociation = 0;
  // Set tracker ACTIVE flag
    active = true;
  // Set initial state estimate
    x_hat = x0;
  // Set initial P covariance
    P = P0;
  // Set initial innovation covariance
    S = H*P*H.transpose() + R;


}

// Reset the tracker
void Tracker::reset() {
  // do nothing with this function for now
}

// Time update
void Tracker::timeUpdate(double dt){
  this->dt = dt;
  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  // Get innovation covariance S (for Mahalanobis check later)
  S = H*P*H.transpose() + R;

  //debug
  // std::cout << "x_hat_new = " << x_hat_new << std::endl;
  // std::cout << "P = " << P << std::endl;
  // std::cout << "S = " << S << std::endl;
}

// Get Mahanalobis distance
double Tracker::getMahalanobis(Eigen::VectorXd& y){
  // Get innovation
  y_hat = y - H*x_hat_new;

  // Check Mahalanobis distance for measurement
  Eigen::VectorXd chi_squared_vec = y_hat.transpose()*S.inverse()*y_hat;
  // std::cout << "tracker ID = " << tracker_id << "  mahalanobis = " << chi_squared_vec << std::endl;

    //debugging
    // std::cout << " R = " << R << std::endl;
    // std::cout << " P = " << P << std::endl;
    // std::cout << " S = " << S << std::endl;
    // std::cout << " S.inverse = " << S.inverse() << std::endl;
    // std::cout << "x_hat_new = " << x_hat_new << std::endl;
    // std::cout << " y = " << y << std::endl;
    // std::cout << " innovation y_hat = " << y_hat << std::endl;


  double chi_squared = chi_squared_vec[0];

  return chi_squared;
}

// Check if selected measurement falls within validation gate
bool Tracker::isValid(double chi_squared, double chi_thresh){
  if (chi_squared <= chi_thresh)
  {
    incAssociation();
    return 1; // Valid
  }
  else
  {
    incDissociation();
    return 0; // Not valid
  }
}

// Measurement update (returns latest estimate)
Eigen::VectorXd Tracker::measurementUpdate(Eigen::VectorXd& y){
  K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
  x_hat_new += K * (y - H*x_hat_new);
  P = (I - K*H)*P;
  x_hat = x_hat_new;

  // debug
  // std::cout << "I = " << I << std::endl;
  // std::cout << "K = " << K << std::endl;
  // std::cout << "x_hat = " << x_hat << std::endl;
  // std::cout << "P = " << P << std::endl;
  return x_hat;
}

// Get tracker state
bool Tracker::isActive(){
  return active;
}

// Increment association variable
void Tracker::incAssociation(){
  association++;
  dissociation = 0;
  // active = true;
}

// Increment dissociation variable
void Tracker::incDissociation(){
  dissociation++;
  if (dissociation > dissociation_limit || association < association_limit)
  {
    // printf("tracker ID = %d  association = %d  dissociation = %d\n", tracker_id, association, dissociation);

    active = false; // Turn off tracker
    dissociation = 0; // Reset dissociation
  }
  association = 0;
}

// Set tracker ID
void Tracker::setID(int tracker_id){
  this->tracker_id = tracker_id;
}

// Get tracker ID
int Tracker::getID(){
  return this->tracker_id;
}

// Get covariance ellipse
Eigen::VectorXd Tracker::getEllipse(double chi_thresh){
  // Use Cholesky decomposition to check if positive definite
  Eigen::LLT<Eigen::MatrixXd> lltOfA(P);
  if (lltOfA.info() == Eigen::NumericalIssue)
  {
    throw std::runtime_error("Possibly non positive definite P!");
    // Deactivate tracker if P becomes non positive definite
    active = false;
    Eigen::VectorXd J(3);
  }
  else
  {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
    // es.compute(P.block<2,2>(0,0), true);
    es.compute(S, true);
    double eigenvalue0 = es.eigenvalues()[0];
    double eigenvalue1 = es.eigenvalues()[1];
    Eigen::VectorXd eigenvector0 = es.eigenvectors().col(0);
    Eigen::VectorXd eigenvector1 = es.eigenvectors().col(1);

    // Eigen::EigenSolver<Eigen::MatrixXd> es;
    // // es.compute(P.block<2,2>(0,0), true);
    // es.compute(S, true);
    // std::complex<double> eigenvalue0 = es.eigenvalues()[0];
    // std::complex<double> eigenvalue1 = es.eigenvalues()[1];
    // Eigen::VectorXcd eigenvector0 = es.eigenvectors().col(0);
    // Eigen::VectorXcd eigenvector1 = es.eigenvectors().col(1);

    // std::cout << "EV0 = " << eigenvalue0 << std::endl;
    // std::cout << "EV1 = " << eigenvalue1 << std::endl;
    // std::cout << "EVEC0 = " << eigenvector0 << std::endl;
    // std::cout << "EVEC1 = " << eigenvector1 << std::endl;
    // std::cout << "P = " << P << std::endl;

    double major_axis = eigenvalue0 > eigenvalue1 ? 0 : 1;
    double minor_axis = eigenvalue0 > eigenvalue1 ? 1 : 0;
    double major_axis_length = 2 * sqrt(chi_thresh * es.eigenvalues()[major_axis]);
    double minor_axis_length = 2 * sqrt(chi_thresh * es.eigenvalues()[minor_axis]);
    double angle = atan2(es.eigenvectors().col(major_axis)[1], es.eigenvectors().col(major_axis)[0]);

    // printf("MAJ = %5.2f  MIN = %5.2f  ANGLE = %5.2f\n", major_axis_length, minor_axis_length, angle);

    Eigen::VectorXd J(3);
    J(0) = major_axis_length;
    J(1) = minor_axis_length;
    J(2) = angle;
    return J;
    // Eigen::EigenSolver<Eigen::MatrixXd> es(P.block<2,2>(0,0));
    //  std::complex<double> eigenvalue1 = es.eigenvalues()[0];
    //  std::complex<double> eigenvalue2 = es.eigenvalues()[1];
    //  Eigen::MatrixXd eigenvectors = es.eigenvectors();
    //  Eigen::VectorXd eigenvector1 = es.eigenvectors().col(0);
    //  Eigen::VectorXd eigenvector2 = es.eigenvectors().col(1);

  }
}

// Get estimated state
Eigen::VectorXd Tracker::getState(){
  return x_hat_new;
}
