
#include "generate_contacts.h"

#include "prmpath/CompleteScenario.h"
#include "collision/Object.h"

using std::string;

using planner::Node;
using planner::Object;

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

    Eigen::Vector3d GetEffectorPos(planner::Node* limb)
    {
        if(limb->children.size() != 0)
        {
            return GetEffectorPos(limb->children.front());
        }
        else
        {
            return limb->position;
        }
    }

    bool isInContact(planner::Node* limb, planner::Object::T_Object& obstacles
                     , Eigen::Vector3d& normal, Eigen::Vector3d& projection)
    {
        planner::Object* effector = GetEffector(limb);
        for(planner::Object::T_Object::iterator oit = obstacles.begin(); oit != obstacles.end(); ++oit)
        {
            if(effector->InContact(*oit,0.1, normal, projection))
                return true;
        }
    }

    std::vector<Node*> Limbs(planner::CompleteScenario& cs,
                             planner::State* state)
    {
        std::vector<Node*> limbs;
        for(std::vector<Node*>::iterator it = cs.limbs.begin()
            ; it!=cs.limbs.end(); ++it)
        {
            limbs.push_back(planner::GetChild(state->value,(*it)->id));
        }
        return limbs;
    }

    void ContactForState(planner::CompleteScenario& cs,
                         planner::State* previous, planner::State* state, planner::State* next, const double treshold)
    {
        std::vector<Node*> limbs = Limbs(cs, state);
        std::vector<Node*> limbsp = Limbs(cs, previous);
        std::vector<Node*> limbsn = Limbs(cs, next);

        int lIndex = 0;
        std::vector<Node*>::iterator plit = limbsp.begin();
        std::vector<Node*>::iterator nlit = limbsn.begin();
        for(std::vector<Node*>::iterator lit = limbs.begin(); lit != limbs.end();
            ++lit, ++plit, ++nlit, ++lIndex)
        {
            Eigen::Vector3d projection, normal;
            if(isInContact(*lit, cs.scenario->objects_,normal,projection))
            {
                // compute speed
                double speed = ((GetEffectorPos(*nlit) - GetEffectorPos(*plit)) / 2).norm();
                if (speed * (GetEffectorPos(*lit) - projection).norm() < treshold)
                {
                    state->contactLimbs.push_back(lIndex);
                    state->contactLimbPositions.push_back(projection);
                    state->contactLimbPositionsNormals.push_back(normal);
                }
            }
        }
    }
}

void gen_contacts::GenerateContacts(const std::string& scenarioFile, const std::string& statefile, const double treshold)
{
    planner::CompleteScenario* cs = planner::CompleteScenarioFromFile(scenarioFile);
    ContactForState(*cs, cs->states.front(),cs->states.front(),cs->states.front(), treshold);
    assert(cs->states.size()>3);
    for(planner::T_State::iterator sit = cs->states.begin()+1;
        sit != cs->states.end()-1; ++sit)
    {
        ContactForState(*cs,*(sit-1), *sit, *(sit+1), treshold);
    }
    ContactForState(*cs, *(cs->states.end()-2),cs->states.back(),cs->states.back(), treshold);
    planner::SaveStates(cs->states,statefile);
}
