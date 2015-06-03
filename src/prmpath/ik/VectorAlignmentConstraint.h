
#ifndef _CLASS_VECALIGNCONSTRAINT
#define _CLASS_VECALIGNCONSTRAINT

#include "PartialDerivativeConstraint.h"

namespace ik{

class VectorAlignmentConstraint : public PartialDerivativeConstraint
{

public:
     VectorAlignmentConstraint(const Eigen::Vector3d& alignmentAxis);
	~VectorAlignmentConstraint();

public:
    virtual double Evaluate(planner::Node* /*limb*/, Eigen::VectorXd /*minDofs*/, Eigen::VectorXd /*maxDofs*/,  const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const Eigen::Vector3d& /*direction*/);

public:
    const Eigen::Vector3d alignmentAxis_;

};
}
// namespace ik
#endif //_CLASS_FMCONSTRAINT
