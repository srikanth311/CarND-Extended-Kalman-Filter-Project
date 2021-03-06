#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd rmse(4);
   rmse << 0, 0, 0, 0;

   if(estimations.size() != ground_truth.size() || estimations.size() == 0) {
       std::cout << "Invalid estimation data or invalid ground truth data - check the input." << std::endl;
       return rmse;
   }

   for(unsigned int i = 0; i < estimations.size(); ++i) {
       VectorXd residual = estimations [i] - ground_truth [i];
       //VectorXd residual = diff.array() * diff.array();
       //rmse = rmse + residual;
       residual = residual.array() * residual.array();
       rmse += residual;
   }

   rmse = rmse / estimations.size();
   rmse = rmse.array().sqrt();
   std::cout << "RMSE == " << rmse << std::endl;

   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Hj(3, 4);

   // recovery state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);


   float c1 = px*px+py*py;
   float c2 = sqrt(c1);
   float c3 = (c1 * c2);

   // Checking division by zero...
   if(fabs(c1) < 1e-4) {
       std::cout << "Error in calculating Jacobian matrix - Division by zero" << std::endl;
       return Hj;
   }

   Hj << (px/c2),   (py/c2),    0,  0,
         -(py/c1),  (px/c1),    0,  0,
         py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3,  px/c2,  py/c2;

   return Hj;
}
