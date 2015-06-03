
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

    void ContactForState(planner::CompleteScenario& cs, planner::State* state)
    {
        std::vector<Node*> limbs;
        for(std::vector<Node*>::iterator it = cs.limbs.begin()
            ; it!=cs.limbs.end(); ++it)
        {
            limbs.push_back(planner::GetChild(state->value,(*it)->id));
        }

        int lIndex = 0;
        for(std::vector<Node*>::iterator lit = limbs.begin(); lit != limbs.end(); ++lit, ++lIndex)
        {
            Eigen::Vector3d projection, normal;
            if(isInContact(*lit, cs.scenario->objects_,normal,projection))
            {
                state->contactLimbs.push_back(lIndex);
                state->contactLimbPositions.push_back(projection);
                state->contactLimbPositionsNormals.push_back(normal);
            }
        }
    }
}

void gen_contacts::GenerateContacts(const std::string& scenarioFile, const std::string& statefile)
{
    planner::CompleteScenario* cs = planner::CompleteScenarioFromFile(scenarioFile);
    for(planner::T_State::iterator sit = cs->states.begin();
        sit != cs->states.end(); ++sit)
    {
        ContactForState(*cs,*sit);
    }
    planner::SaveStates(cs->states,statefile);
}
