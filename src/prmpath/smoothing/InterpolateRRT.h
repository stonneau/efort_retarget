/**
* \file smooth.h
* \brief Helper struct to perform path computation and smoothing
* for an manipulation task
* \author Steve T.
* \version 0.1
* \date 09/05/2014
*
*/
#ifndef _INTERPOLATE_RRT
#define _INTERPOLATE_RRT

#include "collision/Collider.h"
#include "Robot.h"
#include "prmpath/sampling/Sample.h"

#include <Eigen/Dense>

namespace planner
{
    std::vector<Object*> CollectObjects(const planner::Node* limb)
    {
        std::vector<Object*> res;
        const planner::Node* node = limb;
        while(node)
        {
            if(node->current) res.push_back(node->current);
        }
        return res;
    }

    struct InterpolateRRT
    {
         InterpolateRRT(const planner::Robot* robot, const planner::Node* limb, const sampling::T_Samples& samples, Collider& collider)
             : robot_(new planner::Robot(*robot)), limb_(new planner::Node(*limb)), samples_(samples), collider_(collider), limbObjects(CollectObjects(limb)) {}
        ~InterpolateRRT() {}
         planner::Robot* robot_;
         planner::Node* limb_;
         const sampling::T_Samples& samples_;
         Collider& collider_;
         std::vector<Object*> limbObjects;
    };


    sampling::T_Samples computeKeyFrames(InterpolateRRT& rrt, const planner::Robot* robotFrom, const planner::Robot* robotTo,
                                         const sampling::Sample& from, const sampling::Sample& to);

} //namespace planner
#endif //_INTERPOLATE_RRT
