
#ifndef _CLASS_MATCHTARGETCONSTRAINT
#define _CLASS_MATCHTARGETCONSTRAINT

#include "PartialDerivativeConstraint.h"
#include "prmpath/sampling/Sample.h"

namespace ik{

class MatchTargetConstraint : public PartialDerivativeConstraint
{

public:
     MatchTargetConstraint(const planner::Node* target);
    ~MatchTargetConstraint();

public:
    virtual double Evaluate(planner::Node* /*limb*/, Eigen::VectorXd /*minDofs*/, Eigen::VectorXd /*maxDofs*/,  const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const Eigen::Vector3d& /*direction*/);

public:
    const planner::sampling::Sample target_;

};
}
// namespace ik
#endif //_CLASS_MATCHTARGETCONSTRAINT
