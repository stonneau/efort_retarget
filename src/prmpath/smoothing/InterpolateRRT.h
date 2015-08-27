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
#include "prmpath/Robot.h"
#include "prmpath/sampling/Sample.h"
#include "smooth.h"

#include <Eigen/Dense>

namespace planner
{
    class LimbNode
    {
    public:
        LimbNode(planner::Robot* robot, planner::Node* limb, std::vector<planner::Object*>& objects, const std::vector<double>& weight, const double t, const Configuration& configuration)
            : robot_(robot), limb_(limb), sample_(0),configuration_(configuration), objects_(objects),weights(weight), t_(t)
        {}
        LimbNode(const LimbNode& clone)
            : robot_(clone.robot_), limb_(clone.limb_), sample_(clone.sample_),configuration_(clone.configuration_),
              objects_(clone.objects_),weights(clone.weights), t_(clone.t_)
        {}
        ~LimbNode(){}
        planner::Robot* robot_;
        planner::Node* limb_;
        sampling::Sample* sample_;
        const Configuration configuration_;
        std::vector<planner::Object*>& objects_;
        const std::vector<double>& weights;
        const double t_;
    };


    std::vector<Object*> CollectObjects(const planner::Node* limb);

    struct InterpolateRRT
    {
         InterpolateRRT(const planner::Robot* robot, const planner::Node* limb, const sampling::T_Samples& samples, Collider& collider)
             : robot_(new planner::Robot(*robot)), limb_(new planner::Node(*limb)), samples_(samples), collider_(collider), limbObjects(CollectObjects(limb_)) {}
        ~InterpolateRRT() {}
         planner::Robot* robot_;
         planner::Node* limb_;
         const sampling::T_Samples& samples_;
         Collider& collider_;
         std::vector<Object*> limbObjects;
    };


    std::vector<LimbNode*> computeKeyFrames(InterpolateRRT& rrt, const planner::Robot* robotFrom, const planner::Robot* robotTo,
                                         const sampling::Sample& from, const sampling::Sample& to);

} //namespace planner
#endif //_INTERPOLATE_RRT
