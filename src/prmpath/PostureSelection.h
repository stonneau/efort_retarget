/**
* \file PostureSelection.h
* \brief Helper functions to retrieve contact configurations
* articulation.
* \author Steve T.
* \version 0.1
* \date 09/06/2014
*
*/
#ifndef _POSTURESELECTION
#define _POSTURESELECTION

#include "prmpath/Robot.h"
#include "prmpath/sampling/Sample.h"
#include "prmpath/CompleteScenario.h"

namespace planner
{

sampling::Sample* GetPosturesInContact(Robot& robot, Node* limb, const sampling::T_Samples &samples
                                         , Object::T_Object& obstacles, const Eigen::Vector3d& direction
                                       , CompleteScenario &scenario);

sampling::Sample* GetPosturesInContact(Robot& robot, Node* limb, const sampling::T_Samples &samples
                                         , Object::T_Object& obstacles, const Eigen::Vector3d& direction, Eigen::Vector3d& position, Eigen::Vector3d &normalVector
                                       , CompleteScenario &scenario, const std::vector<planner::Sphere*> next_roms, const Sphere* current_rom = 0);

sampling::Sample* GetCollisionFreePosture(Robot& robot, Node* limb, const sampling::T_Samples &samples
                                         , Object::T_Object& obstacles);


sampling::T_Samples GetContactCandidates(Robot& robot, Node* limb, const sampling::T_Samples &samples
                                         , Object::T_Object& obstacles, const Eigen::Vector3d& direction);

sampling::T_Samples GetPosturesOnTarget(Robot& robot, Node* limb, const sampling::T_Samples &samples
                                         , Object::T_Object& obstacles, const Eigen::Vector3d &worldposition);

planner::T_State PostureSequence(planner::CompleteScenario& scenario, int dpethcontact = 2);
} // namespace planner
#endif //_POSTURESELECTION
