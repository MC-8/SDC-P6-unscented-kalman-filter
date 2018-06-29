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
	rmse << 0,0,0,0;
  
    if ((estimations.size() != ground_truth.size()) && (!(ground_truth.size()>0)))
    {
        cout << "Invalid estimation size or ground truth size!" << endl;
        return rmse;
    }
	//accumulate squared residuals
	VectorXd rmse_sum_squared(4);
	VectorXd val_diff(4);
	rmse_sum_squared << 0,0,0,0;
	for(int i=0; i < estimations.size(); ++i)
  {
	    val_diff = estimations[i] - ground_truth[i];
      rmse_sum_squared = rmse_sum_squared.array() + (val_diff.array()*val_diff.array());
	}

	//calculate the mean
	VectorXd rmse_sum_squared_mean(4);
  rmse_sum_squared_mean = rmse_sum_squared.array()/estimations.size();

	//calculate the squared root
  rmse = rmse_sum_squared_mean.array().sqrt();
	//rmse << 1.1,2.2,3.3,4.4;
  //return the result
	return rmse;
}