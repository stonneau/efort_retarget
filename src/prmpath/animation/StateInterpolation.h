/**
* \file JointConstraint.h
* \brief Helper struct that contains angle values for given joint
* articulation.
* \author Steve T.
* \version 0.1
* \date 09/04/2014
*
*/
#ifndef _CLASS_STATE_INTERPOLATION
#define _CLASS_STATE_INTERPOLATION

#include "prmpath/CompleteScenario.h"

namespace planner
{
planner::T_State Animate(const planner::CompleteScenario& scenario, const planner::State& from, const planner::State& to, int framerate);
planner::T_State Animate(const planner::CompleteScenario& scenario, const planner::T_State& fullpath, int framerate);
} // namespace planner
#endif //_CLASS_STATE_INTERPOLATION
