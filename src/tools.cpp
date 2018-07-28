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
   rmse << 0,0,0,0;

   /* check the validity of the inputs, Sanity Checks */
   if ( ( estimations.size() == 0 ) || ( estimations.size() != ground_truth.size() ) )
   {
      return rmse;
   }

   /* Accumulate squared residuals */
   for(unsigned int idx=0; idx < estimations.size(); ++idx){
      VectorXd temp = estimations[idx] - ground_truth[idx];
      temp = temp.array() * temp.array();
      rmse += temp;
   }

   /* Calculate the mean */
   rmse /= estimations.size();

   /* Calculate the squared root */
   rmse = rmse.array().sqrt();

   return rmse;
}