
#include "ObstacleAvoidanceConstraint.h"
#include "prmpath/Robot.h"

#include <Eigen/Dense>

using namespace Eigen;
using namespace ik;
using namespace planner;


namespace
{
// find minimal distance between 2 sets limb and obstacles except for effector

}

ObstacleAvoidanceConstraint::ObstacleAvoidanceConstraint(Collider &collider)
    : collider_(collider)
{
	// NOTHING
}

ObstacleAvoidanceConstraint::~ObstacleAvoidanceConstraint()
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

    float MinDistance(planner::Node* limb, Collider& collider)
    {
        double minDistance = std::numeric_limits<double>::max();
        double currentDistance;
        if(limb->current)
            minDistance = planner::DistanceToClosestObject(limb->current, &collider);
        for(std::vector<planner::Node*>::iterator it = limb->children.begin(); it != limb->children.end(); ++it)
        {
            currentDistance = MinDistance(*it, collider);
            if(currentDistance < minDistance)
                minDistance = currentDistance;
        }
        return minDistance;
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

double ObstacleAvoidanceConstraint::Evaluate(planner::Node* limb, Eigen::VectorXd minDofs, Eigen::VectorXd maxDofs,  const int joint, Jacobian& jacobianMinus, Jacobian& jacobianPlus, float epsilon, const Vector3d& direction)
{
   // check dot product of angle with direction
    float valMin, valPlus;
    Eigen::VectorXd save = GetPose(limb);
    SetPose(limb, minDofs);
    valMin = MinDistance(limb, collider_);
    SetPose(limb, maxDofs);
    valPlus = MinDistance(limb, collider_);
    double res = double (valPlus - valMin) / (epsilon * 2) ;
    SetPose(limb, save);
    return res;
}


