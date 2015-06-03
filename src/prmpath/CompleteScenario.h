/**
* \file CompleteScenario.h
* \brief Helper struct that contains a complete scenario
* for an manipulation task
* \author Steve T.
* \version 0.1
* \date 09/05/2014
*
*/
#ifndef _STRUCT_COMPLETESCENARIO
#define _STRUCT_COMPLETESCENARIO

#include <vector>
#include <string>

#include "prm/Scenario.h"
#include "prmpath/Robot.h"
#include "prmpath/JointConstraint.h"
#include "prmpath/sampling/Sample.h"
#include "smoothing/smooth.h"
#include "collision/Sphere.h"

namespace planner
{

struct State
{
    State():value(0), stable(false){}
    State(const State* parent);
    ~State(){if(value)delete value;}
    std::vector<int> contactLimbs;
    std::vector<Eigen::Vector3d> contactLimbPositions;
    std::vector<Eigen::Vector3d> contactLimbPositionsNormals;
    Robot* value;
    bool stable;

    bool InContact(int limb, Eigen::Vector3d& target, Eigen::Vector3d& normal) const
    {
        int id = 0;
        for(; id < contactLimbs.size(); ++id)
        {
            if(contactLimbs[id] == limb)
            {
                target = contactLimbPositions[id];
                normal = contactLimbPositionsNormals[id];
                return true;
            }
        }
        return false;
    }
};

typedef std::vector<State*> T_State;

struct CompleteScenario
{
     CompleteScenario();
    ~CompleteScenario();

    bool SavePath(const std::string& outfilename);

    Scenario* scenario; //prm + objs
    Robot* robot;
    std::vector<planner::Node*> limbs;
    std::vector<double> limbspeed;
    std::vector<planner::Sphere> limbRoms;
    std::vector<sampling::T_Samples> limbSamples;
    Model* from;
    Model* to;
    CT_Model path;
    std::vector<planner::Robot*> completePath;
    State initstate;
    SplinePath* spline;
    bool relocateEnglobing;
    T_State states; // optional
};

/*
File description:
PRMSCENARIO file="path/to/scenario"
ROBOT_SKELETON file="path/to/urdf"
ROBOT_CONSTRAINTS file="path/to/joint_constraints"
CONSTANT_ROTATION matrix="a00 a01 a02 a03 ... a30 a31 a32 a33"
PATH_FROM matrix="a00 a01 a02 a03 ... a30 a31 a32 a33"
PATH_TO matrix="a00 a01 a02 a03 ... a30 a31 a32 a33"
LIMB joint_name="joint_name"
...
LIMB joint_name="joint_name"
NBSAMPLES N=""
INITCONTACTS 2 3
*/
CompleteScenario* CompleteScenarioFromFile(const std::string& file, double scaleEnglobing = 1.);
bool SaveStates(const T_State& states, const std::string& outfilename);
bool ExportContactStates(const T_State& states, std::vector<planner::Node*>& limbs, const std::string& outfilename);
T_State LoadStates(const std::string& infilename, const planner::Robot* model);

} //namespace planner
#endif //_STRUCT_COMPLETESCENARIO
