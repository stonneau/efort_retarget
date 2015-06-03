
#ifndef _CLASS_FMCONSTRAINT
#define _CLASS_FMCONSTRAINT

#include "PartialDerivativeConstraint.h"

namespace ik{

class ForceManipulabilityConstraint : public PartialDerivativeConstraint
{

public:
	 ForceManipulabilityConstraint();
	~ForceManipulabilityConstraint();

public:
    virtual double Evaluate(planner::Node* /*limb*/, Eigen::VectorXd /*minDofs*/, Eigen::VectorXd /*maxDofs*/,  const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const Eigen::Vector3d& /*direction*/);

private:
    double ForceManipulability(Jacobian& /*jacobian*/, const Eigen::Vector3d& /*direction*/);
};
}
// namespace ik
#endif //_CLASS_FMCONSTRAINT
