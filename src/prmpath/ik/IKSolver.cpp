
#include "IKSolver.h"
#include "ik/PartialDerivativeConstraint.h"
#include "ik/VectorAlignmentConstraint.h"
#include "ik/ForceManipulabilityConstraint.h"
#include "prmpath/Jacobian.h"
#include "sampling/Sample.h"

#include <vector>
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace planner;
using namespace ik;


namespace
{

void GetPoseRec(planner::Node* limb,  VectorXd& res, int& id)
{
    if(limb->children.empty()) return;
    res[id++] = limb->value;
    /*if(limb->children.empty()) return;
    GetPoseRec(limb->children.front(), res, id);*/
    for(std::vector<Node*>::iterator it = limb->children.begin(); it!= limb->children.end(); ++it)
    {
        GetPoseRec(*it, res, id);
    }
}

Eigen::VectorXd GetPose(planner::Node* limb)
{
    int id = 0;
    VectorXd res(planner::GetNumChildren(limb));
    GetPoseRec(limb, res, id);
    limb->Update();
    return res;
}

void SetPoseRec(planner::Node* limb,  const VectorXd& res, int& id)
{
    if(limb->children.empty() || id == res.rows()) return;
    limb->value = res[id++];
    SetPoseRec(limb->children.front(), res, id);
    for(std::vector<Node*>::iterator it = limb->children.begin(); it!= limb->children.end(); ++it)
    {
        SetPoseRec(*it, res, id);
    }
}

void SetPose(planner::Node* limb, const VectorXd& res)
{
    int id = 0;
    SetPoseRec(limb, res, id);
    limb->Update();
}

void PartialDerivative(planner::Node* limb, const Vector3d& direction, VectorXd& velocities, const int joint, const float epsilon
                       , const std::vector<PartialDerivativeConstraint*>& constraints)
{
    Eigen::VectorXd dofs = GetPose(limb);// saving previous configuration
    Eigen::VectorXd minDofs, plusDofs;
    // performing derivative using finite differences method
    minDofs = dofs;
    minDofs(joint) = dofs[joint] - epsilon;
    SetPose(limb, minDofs);
    Jacobian jacobMinus(limb);

    plusDofs = dofs;
    plusDofs(joint) = dofs[joint] + epsilon;
    SetPose(limb, plusDofs);
    Jacobian jacobPlus(limb);

    SetPose(limb, dofs); // restoring config

    int i =0;
    for(std::vector<PartialDerivativeConstraint*>::const_iterator it = constraints.begin(); it!= constraints.end(); ++it)
    {
        ++ i;
        velocities(joint) += (*it)->Evaluate(limb, minDofs, plusDofs, i, jacobMinus, jacobPlus, epsilon, direction);
    }
    if (i!= 0)
    {
        velocities(joint) = velocities(joint) / i;
    }
    else
    {
        velocities(joint) = 0;
    }
}

void PartialDerivatives(planner::Node* limb, const Vector3d& direction, VectorXd& velocities, const float epsilon, const std::vector<PartialDerivativeConstraint*>& constraints)
{
    for(int i =0; i< velocities.rows() ;++i)
    {
        PartialDerivative(limb, direction, velocities, i, epsilon, constraints);
    }
}
}


IKSolver::IKSolver(const float espilon, const float treshold, const float stepsize, const float stepsizeOptim)
: epsilon_(espilon)
, treshold_(treshold)
, stepsize_(stepsize)
, stepsizeOptim_(stepsizeOptim)
{
	// NOTHING
}

IKSolver::~IKSolver()
{
    for(std::vector<PartialDerivativeConstraint*>::iterator it = constraints_.begin();
        it!= constraints_.end();++it)
    {
        delete (*it);
    }
}


//REF: Boulic : An inverse kinematics architecture enforcing an arbitrary number of strict priority levels
bool IKSolver::StepClamping(planner::Node* limb, const Eigen::Vector3d& target, const Vector3d& direction, const std::vector<PartialDerivativeConstraint*>& constraints, bool optimize) const
{
    bool ret = false;

    Jacobian jacobian(limb);
    Eigen::VectorXd postureVariation(Eigen::VectorXd::Zero(jacobian.GetJacobian().cols()));
    PartialDerivatives(limb, direction, postureVariation, epsilon_, constraints);

    Vector3d force = target - planner::GetEffectorCenter(limb);
	
    if(force.norm () < treshold_) // reached treshold
    {
        ret = true;
    }
    VectorXd velocities;
    MatrixXd J = jacobian.GetJacobian(); int colsJ = J.cols(); int rowsJ = J.rows();
    Vector3d dX = force;
    dX.normalize();
    dX*=stepsize_;

    MatrixXd p0 = MatrixXd::Identity(colsJ, colsJ);
    MatrixXd nullSpace = p0;

	// init all joints to free.
    std::vector<bool> freeJoint;
	for(int i = 0; i < colsJ; ++i)
	{
       freeJoint.push_back(true);
	}

    bool clamp = false;
    //entering clamping loop
    do
    {
        jacobian.GetNullspace(p0, nullSpace); // Pn(j) = P0(j) - Jtr * J
        velocities = jacobian.GetJacobianInverse() * dX;
        if(optimize)
        {
            velocities += nullSpace * postureVariation * stepsizeOptim_;
        }

        // now to the "fun" part
        clamp = false;
        for(int i =0; i < colsJ; ++ i)
        {
            if(freeJoint[i])
            {
                Node * dof = planner::GetChild(limb, i + limb->id);
                double nval = dof->value + velocities(i);
                double overload = 0;// nval > dof->maxAngleValue ? (nval - dof->maxAngleValue) : (nval < dof->minAngleValue ? (nval - dof->minAngleValue) : 0);
                if(false)//overload != 0.) // clamping happened
                {
                    freeJoint[i] = false;
                    clamp = true;
                    dX -= J.col(i) * overload;
                    J.col(i) = VectorXd::Zero(rowsJ);
                    p0(i,i) = 0;
                }
                dof->value = (nval - overload);
            }
        }
        if(clamp)
            jacobian.SetJacobian(J);
    } while(clamp);
    limb->Update();
    return ret;
}

bool IKSolver::StepClamping(planner::Node* limb, const Eigen::Vector3d& target, const Vector3d& direction, bool optimize ) const
{
    return StepClamping(limb,target,direction, constraints_, optimize);
}

bool IKSolver::StepClamping(planner::Node* limb, const Eigen::VectorXd& positionConstraints) const
{
    bool ret = false;

    Jacobian jacobian(limb, 0., true);

    /*Node* node = limb;
    for(int i =0; i< 3; ++i)
    {
        node = node->children[0];
    }*/


    Eigen::VectorXd force = positionConstraints - planner::AsPosition(limb);

    if(force.norm () < treshold_) // reached treshold
    {
        return true;
    }
    MatrixXd J = jacobian.GetJacobian(); int colsJ = J.cols(); int rowsJ = J.rows();
    VectorXd dX = force;
    dX.normalize();
    dX*=stepsize_;


    VectorXd velocities = jacobian.GetJacobianInverse() * dX;

    for(int i =0; i < colsJ; ++ i)
    {
        Node * dof = planner::GetChild(limb, i + limb->id);
        double nval = dof->value + velocities(i);
        dof->value = (nval);
    }

    limb->Update();
    return ret;
}

void IKSolver::AddConstraint(Constraint constraint)
{
    switch(constraint)
    {
        case(FTR):
        {
            //constraints_.push_back(new ForceManipulabilityConstraint());
        }
        case(ForceManip):
        {
            constraints_.push_back(new ForceManipulabilityConstraint());
            break;
        }
        case(AxisAlign):
        {
            //constraints_.push_back(new VectorAlignmentConstraint());
            break;
        }
        default:
        {
            // NOTHING
        }
    }
}
