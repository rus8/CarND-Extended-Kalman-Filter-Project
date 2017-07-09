#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

#define pi 3.14159265358979323846

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
    /**
    * Constructor.
    */
    Tools();

    /**
    * Destructor.
    */
    virtual ~Tools();

    /**
    * A helper method to calculate RMSE.
    */
    static VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

    /**
    * A helper method to calculate Jacobians.
    */
    static MatrixXd CalculateJacobian(const VectorXd& x_state);


    static double NormalizeAngle(double angle);

};

#endif /* TOOLS_H_ */
