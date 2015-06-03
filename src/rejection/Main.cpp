#include "collision/ParserObj.h"
#include "collision/Object.h"
#include "collision/Collider.h"
#include "tools/MatrixDefs.h"
#include "prm/SimplePRM.h"
#include "prm/Generator.h"
#include "prm/Scenario.h"
#include "prmpath/Robot.h"
#include "prmpath/sampling/Sample.h"
#include "prmpath/JointConstraint.h"
#include "prmpath/CompleteScenario.h"
#include "prm/Scenario.h"
#include "prmpath/PostureSelection.h"
#include "prmpath/Export/BVHExporter.h"
#include "prmpath/Export/ITOMPExporter.h"
#include "prmpath/ik/IKSolver.h"
#include "prmpath/ik/VectorAlignmentConstraint.h"
#include "prmpath/ik/ForceManipulabilityConstraint.h"
#include "prmpath/ik/ObstacleAvoidanceConstraint.h"
#include "prmpath/animation/StateInterpolation.h"
#include "Timer.h"

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Geometry>
#ifdef WIN32
#include <windows.h>
#endif


using namespace std;
using namespace Eigen;

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

bool LimbColliding(planner::Node* limb, planner::Object::T_Object& obstacles, bool effector = true)
{
    if( limb->current && ((effector || limb->current != GetEffector(limb)) && limb->current->IsColliding(obstacles)))
    {
            return true;
    }
    if(limb->children.size() == 0)
        return false;
    return LimbColliding(limb->children[0], obstacles);
}

bool HasContact(planner::Robot* robot, planner::CompleteScenario* cscenario, planner::Node* limb, const planner::sampling::T_Samples& samples)
{
    Eigen::Vector3d normal, projection;
    planner::Object* effector = GetEffector(limb);
    for(planner::sampling::T_Samples::const_iterator sit = samples.begin(); sit != samples.end(); ++sit)
    {
        planner::sampling::LoadSample(*(*sit),limb);
        if(!(planner::IsSelfColliding(robot, limb) || LimbColliding(limb, cscenario->scenario->objects_)))
        {
            for(planner::Object::T_Object::iterator oit = cscenario->scenario->objects_.begin(); oit != cscenario->scenario->objects_.end(); ++oit)
            {
                if(effector->InContact(*oit,0.01, normal, projection))
                {
                    return true;
                }
            }
        }
    }
    return false;
}


double faisLeTafUnefois(const std::string& scenarioFile, std::vector<double>& rejection, double scaleEnglobing = 1.)
{
    double good = 0; double bad = 0;
    planner::CompleteScenario* cscenario = planner::CompleteScenarioFromFile(scenarioFile, scaleEnglobing);
    planner::Scenario& scenario = *(cscenario->scenario);
    planner::Robot* robot = cscenario->robot;
    // get generator, from scenario
    planner::Generator generator(scenario.objects_, scenario.objects_, scenario.model_); // TODO MEME
    //#pragma omp parallel for
    int nbtries (0);
    for(std::size_t i = 0; i< 10000; ++i)
    {
         std::vector<size_t> contactlimbs;
         planner::Model*  model = generator(contactlimbs);
         nbtries += contactlimbs.size();
         //planner::Robot* rob = new planner::Robot(*(cscenario->robot));
         robot->SetPosition(model->GetPosition(), false);
         robot->SetRotation(model->GetOrientation(), true);
         std::vector<planner::Node*> limbs;
         for(std::vector<planner::Node*>::iterator it = cscenario->limbs.begin()
             ; it!=cscenario->limbs.end(); ++it)
         {
             limbs.push_back(planner::GetChild(robot,(*it)->id));
         }

         int lIndex = 0;
         bool contact(false);
         for(std::vector<std::size_t>::const_iterator itl = contactlimbs.begin(); itl != contactlimbs.end(); ++itl)
         {
            for(std::vector<planner::Node*>::iterator lit = limbs.begin(); lit != limbs.end() && !contact; ++lit, ++lIndex)
             {
                if ( contactlimbs.front() == lIndex && HasContact(robot, cscenario, *lit, cscenario->limbSamples[lIndex]))
                {
                    contact = true;
                }
             }
            if(contact) {++good;} else {++bad;}
         }
         delete model;
    }
    std::cout << "scenario " << scenarioFile <<  std::endl;
    std::cout << "good " << good << std::endl;
    std::cout << "bad " << bad << std::endl;
    std::cout << "rejection rate " << bad /(good + bad) << std::endl;
    return  (bad /(good + bad));
    delete cscenario;
}

}

int main(int argc, char *argv[])
{
    std::string scenarioPathHuman("../benchmarks/");
    // human scenarios
    std::vector<std::string> targets;
    targets.push_back(scenarioPathHuman + "truck_front10000.scen");
    //targets.push_back(scenarioPathHuman + "between10000.scen");
    //targets.push_back(scenarioPathHuman + "chair10000.scen");
    //targets.push_back(scenarioPathHuman + "truck_spider.scen");
    //targets.push_back(scenarioPathHuman + "climbingspider.scen");

    std::stringstream outstream;

    std::vector<double> rejection;
    double scale = 1.;
    for(double scale = 2.8; scale <= 2.8; scale = scale + 0.2)
    {
        for(std::vector<std::string>::const_iterator sit = targets.begin(); sit != targets.end(); ++sit)
        {
            double res = faisLeTafUnefois(*sit, rejection, scale);
            outstream << *sit << "\t" <<  scale << "\t" <<  res << std::endl;
        }
    }

    ofstream outfile;
    std::string outfilename("rejection.txt");
    outfile.open(outfilename.c_str());
    if (outfile.is_open())
    {
        outfile << outstream.rdbuf();
        outfile.close();
    }
    else
    {
        std::cout << "Can not open outfile " << outfilename << std::endl;
        return false;
    }
}
