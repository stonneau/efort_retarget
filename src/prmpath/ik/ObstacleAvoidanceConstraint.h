
#ifndef _CLASS_OBSTACLECONSTRAINT
#define _CLASS_OBSTACLECONSTRAINT

#include "PartialDerivativeConstraint.h"
#include "collision/Collider.h"

namespace ik{

class ObstacleAvoidanceConstraint : public PartialDerivativeConstraint
{

public:
     ObstacleAvoidanceConstraint(planner::Collider& collider);
    ~ObstacleAvoidanceConstraint();

public:
    virtual double Evaluate(planner::Node* /*limb*/, Eigen::VectorXd /*minDofs*/, Eigen::VectorXd /*maxDofs*/,  const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const Eigen::Vector3d& /*direction*/);

private:
    planner::Collider& collider_;

};
}
// namespace ik
#endif //_CLASS_OBSTACLECONSTRAINT
