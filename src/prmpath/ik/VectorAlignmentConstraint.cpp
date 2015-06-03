
#include "VectorAlignmentConstraint.h"
#include "prmpath/Robot.h"

#include <Eigen/Dense>

using namespace Eigen;
using namespace ik;
using namespace planner;

VectorAlignmentConstraint::VectorAlignmentConstraint(const Eigen::Vector3d& alignmentAxis)
    : alignmentAxis_(alignmentAxis)
{
	// NOTHING
}

VectorAlignmentConstraint::~VectorAlignmentConstraint()
{
	// NOTHING
}

namespace
{
    planner::Object* GetEffector(planner::Node* limb)
    {
        if(limb->children.size() != 0)
        {
            planner::Object* res = GetEffector(limb->children[0]);
            if(res) return res;
        }
        return limb->current;
    }

    Eigen::Vector3d GetArmVector(planner::Node* limb)
    {
        planner::Object* source = GetEffector(limb);
        Eigen::Matrix3d ore = source->GetOrientation();
        ore.inverse();
        Eigen::Vector3d to = ore * limb->effectorNormal;
        return to;
    }


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
}

double VectorAlignmentConstraint::Evaluate(planner::Node* limb, Eigen::VectorXd minDofs, Eigen::VectorXd maxDofs,  const int joint, Jacobian& jacobianMinus, Jacobian& jacobianPlus, float epsilon, const Vector3d& direction)
{
    Vector3d dirNorm = direction;
    dirNorm.normalize();
    // check dot product of angle with direction
    float valMin, valPlus;
    Eigen::VectorXd save = GetPose(limb);
    SetPose(limb, minDofs);
    Eigen::Vector3d armAlign = GetArmVector(limb);
    valMin = -armAlign.dot(alignmentAxis_);
    SetPose(limb, maxDofs);
    armAlign = GetArmVector(limb);
    valPlus = -armAlign.dot(alignmentAxis_);
     double res = double (valPlus - valMin) / (epsilon * 2) ;
    SetPose(limb, save);
    return res;
}


