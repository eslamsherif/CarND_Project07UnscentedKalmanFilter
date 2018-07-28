#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
   VectorXd rmse(4);
   rmse << 0,0,0,0;

    unsigned int est_size = estimations.size();

   /* check the validity of the inputs, Sanity Checks */
    if ( ( est_size  == 0 )           ||
         ( ground_truth.size() == 0 ) ||
         ( est_size  != ground_truth.size() ) 
       )
   {
        cout << "ERROR:(003) RMSE Preconditions violated." << endl;
        return rmse;
   }

   /* Accumulate squared residuals */
    for(unsigned int i=0; i < est_size; ++i)
    {
        VectorXd temp = estimations[i] - ground_truth[i];
        temp = temp.array() * temp.array();
        rmse += temp;
   }

   /* Calculate the mean */
    rmse /= est_size;

   /* Calculate the squared root */
   rmse = rmse.array().sqrt();

   return rmse;
}