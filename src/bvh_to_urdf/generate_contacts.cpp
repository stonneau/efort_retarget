
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

    // remove any contact that lasts less than 10 frames.
    void CleanStates(planner::CompleteScenario& cs)
    {
        //maintain current nb vectors for each limb
        std::vector<int> framesPerLimb;
        for(int i=0; i< cs.limbs.size(); ++i)
        {
            framesPerLimb.push_back(0);
        }
        planner::T_State& states = cs.states;
        std::size_t frameid(0);
        for(planner::T_State::iterator it = states.begin();
            it != states.end(); ++it, ++frameid)
        {
            planner::State* state = *it;
            for(int i=0; i< framesPerLimb.size(); ++i)
            {
                // contact found
                if(std::find(state->contactLimbs.begin()
                             ,state->contactLimbs.end()
                             ,i) != state->contactLimbs.end())
                {
                    // contact not already existing
                    framesPerLimb[i] += 1;
                }
                else // no contact
                {
                    int nbFrames = framesPerLimb[i];
                    if(nbFrames > 0 && nbFrames < 10)
                    {
                        // remove previous contacts which are
                        // obviously not long enough
                        for(int s = frameid - nbFrames; s < frameid; ++s)
                        {
                            // find contact index.
                            planner::State* current = states[s];
                            for(int cid1 =0; cid1 < current->contactLimbs.size(); ++cid1)
                            {
                                if(current->contactLimbs[cid1] == i)
                                {
                                    //current->manipulabilities.erase(current->manipulabilities.begin()+i);
                                    current->contactLimbs.erase(current->contactLimbs.begin()+cid1);
                                    current->contactLimbPositions.erase(current->contactLimbPositions.begin()+cid1);
                                    current->contactLimbPositionsNormals.erase(current->contactLimbPositionsNormals.begin()+cid1);
                                }
                            }
                        }
                    }
                    framesPerLimb[i] = 0;
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
    CleanStates(*cs);
    planner::SaveStates(cs->states,statefile);
}
