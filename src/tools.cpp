#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

  // Copied from lesson Evaluating KF Performance 2

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
     std::cout << "Invalid estimations vector size" << std::endl;
     return rmse;
  }

  // TODO: accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd residual = estimations[i] -  ground_truth[i];

    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // TODO: calculate the mean
  rmse = rmse / estimations.size();

  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   double px = x_state[0];
   double py = x_state[1];
   double vx = x_state[2];
   double vy = x_state[3];

   MatrixXd Hj = MatrixXd(3,4);
   Hj << 0,0,0,0,
         0,0,0,0,
         0,0,0,0;

   // calculate this outside of if statement because it 
   // is used to check for division by zero
   double px2_py2 = px*px + py*py;

   if (fabs(px2_py2) > 0.0001) {
      // compute the Jacobian matrix
      
      // these only need to be calculated inside the if statement
      double sqrt_px2_py2 = sqrt(px2_py2);
      double rt32_px2_py2 = pow(sqrt_px2_py2,3);

      Hj(0,0) = px / sqrt_px2_py2;
      Hj(1,0) = -py / px2_py2;
      Hj(2,0) = py * (vx*py - vy*px) / rt32_px2_py2;

      Hj(0,1) = py / sqrt_px2_py2;
      Hj(1,1) = px / px2_py2;
      Hj(2,1) = px * (vy*px - vx*py) / rt32_px2_py2;

      Hj(2,2) = px / sqrt_px2_py2;

      Hj(2,3) = py / sqrt_px2_py2;
      
   } else {
      std::cout << "Jacobian division by zero" << std::endl;
   }

   return Hj;
}
