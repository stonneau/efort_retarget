
#ifndef _CLASS_IKSOLVER
#define _CLASS_IKSOLVER

#include "prmpath/Robot.h"

#include <Eigen/Dense>


namespace ik
{
class PartialDerivativeConstraint;
enum Constraint{FTR, AxisAlign, ForceManip, Unknown};

class IKSolver
{

public:
     IKSolver(const float espilon = 0.001f, const float treshold = 0.001f, const float stepSize = 0.01f, const float stepsizeOptim=0.01f);
	~IKSolver();

public:
    //true if target reached
    bool StepClamping(planner::Node* /*limb*/, const Eigen::Vector3d& /*target*/, const Eigen::Vector3d& /*direction*/, bool optimize = false) const;
    bool StepClamping(planner::Node* /*limb*/, const Eigen::VectorXd& /*positionConstraints*/) const;
    bool StepClamping(planner::Node* /*limb*/, const Eigen::Vector3d& /*target*/, const Eigen::Vector3d& /*direction*/, const std::vector<PartialDerivativeConstraint*>& /*constraints*/, bool optimize = false) const;
    void AddConstraint(Constraint constraint);

private:
    //void PartialDerivative (planner::Node* /*limb*/, const Eigen::Vector3d& /*direction*/, Eigen::VectorXd & /*velocities*/, const int /*joint*/) const;

	const float epsilon_;
    const float treshold_;
    const float stepsize_;
    const float stepsizeOptim_;
    std::vector<PartialDerivativeConstraint*> constraints_;
};
} // namespace ik
#endif //_CLASS_IKSOLVER
