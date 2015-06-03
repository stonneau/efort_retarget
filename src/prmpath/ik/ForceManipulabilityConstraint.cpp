
#include "ForceManipulabilityConstraint.h"

using namespace Eigen;
using namespace ik;

ForceManipulabilityConstraint::ForceManipulabilityConstraint()
{
	// NOTHING
}

ForceManipulabilityConstraint::~ForceManipulabilityConstraint()
{
	// NOTHING
}

double ForceManipulabilityConstraint::Evaluate(planner::Node* limb, Eigen::VectorXd minDofs, Eigen::VectorXd maxDofs,  const int joint, Jacobian& jacobianMinus, Jacobian& jacobianPlus, float epsilon, const Vector3d& direction)
{
    double res = double ((ForceManipulability(jacobianPlus, direction) - ForceManipulability(jacobianMinus, direction)) / (epsilon * 2)) ;
    res = res < 0 ? -res : res;
    return res;
}
double ForceManipulabilityConstraint::ForceManipulability(Jacobian& jacobian, const Vector3d& direction)
{ 
    double r = ((direction).transpose()*jacobian.GetJacobianProduct()*(direction));
    return r; // not 1/sqrt(r) because manipulability wille be minimized;
}



