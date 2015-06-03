/**
* \file JointConstraint.h
* \brief Helper struct that contains angle values for given joint
* articulation.
* \author Steve T.
* \version 0.1
* \date 09/04/2014
*
*/
#ifndef _CLASS_JOINTCONSTRAINT
#define _CLASS_JOINTCONSTRAINT

#include "prmpath/Robot.h"

namespace planner
{
/**
File description:
constraint joint_name="JOINT_NAME" min="" max="" default=""
...
rom joint1="JOINT_NAME" joint2="JOINT_NAME" joint3="JOINT_NAME"
*/
bool LoadJointConstraints(Robot& robot, const std::string& filename);
} // namespace planner
#endif //_CLASS_JOINTCONSTRAINT
