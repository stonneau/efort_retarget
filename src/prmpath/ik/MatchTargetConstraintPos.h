
#ifndef _CLASS_MATCHTARGETCONSTRAINTPOS
#define _CLASS_MATCHTARGETCONSTRAINTPOS

#include "PartialDerivativeConstraint.h"
#include "prmpath/sampling/Sample.h"

namespace ik{

class MatchTargetConstraintPos : public PartialDerivativeConstraint
{

public:
     MatchTargetConstraintPos(planner::Node* target);
    ~MatchTargetConstraintPos();

public:
    virtual double Evaluate(planner::Node* /*limb*/, Eigen::VectorXd /*minDofs*/, Eigen::VectorXd /*maxDofs*/,  const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const Eigen::Vector3d& /*direction*/);

public:
    planner::Node* target_;

};
}
// namespace ik
#endif //_CLASS_MATCHTARGETCONSTRAINT
