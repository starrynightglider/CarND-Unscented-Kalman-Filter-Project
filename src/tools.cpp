#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4); 
  rmse.fill(0.0);

  if (estimations.size()!=ground_truth.size() || estimations.empty()) {
    cout<<"Invalid RMSE input"<<endl;
    return rmse;
  }
  for (int i=0; i<estimations.size(); ++i){
    VectorXd diff = estimations[i]-ground_truth[i];
    VectorXd sq = diff.array() * diff.array();
	  rmse +=sq;
  }
  return (rmse/estimations.size()).array().sqrt();
}
