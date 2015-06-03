
#ifndef _CLASS_PDCONSTRAINT
#define _CLASS_PDCONSTRAINT

#include "prmpath/Jacobian.h"
#include "prmpath/Robot.h"
#include <Eigen/Dense>

namespace ik
{

class PartialDerivativeConstraint {

public:
	 PartialDerivativeConstraint();
     virtual ~PartialDerivativeConstraint();

public:
    virtual double Evaluate(planner::Node* /*limb*/, Eigen::VectorXd /*minDofs*/, Eigen::VectorXd /*maxDofs*/, const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const Eigen::Vector3d& /*direction*/) = 0;
};
} // namespace ik
#endif //_CLASS_PDCONSTRAINT
