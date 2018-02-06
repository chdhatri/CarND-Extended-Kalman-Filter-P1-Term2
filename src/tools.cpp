#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0){
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }
    
    //accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i){
        
        VectorXd residual = estimations[i] - ground_truth[i];
        
        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }
    
    //calculate the mean
    rmse = rmse / estimations.size();
    
    //calculate the squared root
    rmse = rmse.array().sqrt();
    
    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    float px_2 = px * px;
    float py_2 = py * py;
    float den  = sqrt(px_2+py_2);
    float den_2 = sqrt(px_2+py_2) * sqrt(px_2+py_2);
    float den_3 = sqrt(px_2+py_2) * sqrt(px_2+py_2) * sqrt(px_2+py_2);
    float prod = vx * py - vy * px;
    
    //TODO: YOUR CODE HERE
    
    //check division by zero
    Hj << px/den, py/den, 0,0,
    -py/den_2, px/den_2,0,0,
    (py * prod)/den_3,  (px * prod)/den_3, px/den, py/den;
    
    
    //compute the Jacobian matrix
    
    return Hj;
}
