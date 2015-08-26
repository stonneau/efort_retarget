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
    struct InterpolateRRT
    {
         InterpolateRRT(const planner::Node* limb, const sampling::T_Samples& samples, const Collider& collider)
             : limb_(limb), samples_(samples), collider_(collider) {}
        ~InterpolateRRT() {}
         const planner::Node* limb_;
         const sampling::T_Samples& samples_;
         const Collider& collider_;
    };

    sampling::T_Samples computeKeyFrames(const planner::Node* from, const planner::Node* to);

} //namespace planner
#endif //_INTERPOLATE_RRT
