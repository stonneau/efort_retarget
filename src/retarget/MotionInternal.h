/**
* \file MotionI.h
* \brief Structure describing the complete MotionI of a robot
* \author Steve T.
* \version 0.1
* \date 28/04/2015
*
*/
#ifndef _STRUCT_RETARGETER
#define _STRUCT_RETARGETER


#include "collision/Object.h" // this needs to move out
#include "prmpath/CompleteScenario.h"
#include "prmpath/Robot.h"

#include <string>

#include <Eigen/Dense>
#include <memory>

namespace efort
{
struct PImpl;

struct Contact
{
    int limbIndex_;
    int startFrame_;
    int endFrame_;
    Eigen::Vector3d worldPosition_;
    Eigen::Vector3d surfaceNormal_;
    std::size_t objectId_;
    std::size_t triangleId_;

    bool equals(const Contact& other) const
    {
        return limbIndex_ == other.limbIndex_ && startFrame_ == other.startFrame_
                && endFrame_ == other.endFrame_;
    }
};

struct Frame
{
    Eigen::VectorXd configuration_;
    std::vector<Contact> contacts_;
};

struct MotionI
{
    Frame Retarget(const std::size_t /*frameid*/) const; //tmp: waht for objs ?
    planner::State* Retarget(planner::Robot* current, const std::size_t /*frameid*/, const std::vector<Eigen::Vector3d>& /*target*/, planner::Object::T_Object& objects) const; //tmp: waht for objs ?
    std::vector<planner::State*> Retarget(planner::Robot* current, const std::vector<Eigen::VectorXd>& frameConfigurations, planner::Object::T_Object& objects) const;

    std::vector<Frame> frames_;
private:
    std::auto_ptr<PImpl> pImpl_;
    friend MotionI* LoadMotionI(planner::CompleteScenario* scenario);
};

MotionI* LoadMotionI(planner::CompleteScenario* scenario);
void DumpMotion(const MotionI* motion);
} //namespace efort
#endif //Retargeter
