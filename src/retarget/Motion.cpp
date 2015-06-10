#include "retarget/Motion.h"
#include "prmpath/CompleteScenario.h"
#include "prmpath/PostureSelection.h"

#include "collision/Sphere.h"
#include "prmpath/CompleteScenario.h"
#include "prmpath/ik/IKSolver.h"
#include "prmpath/ik/VectorAlignmentConstraint.h"


struct efort::PImpl
{
    PImpl(const std::string& scenario)
        : cScenario_(planner::CompleteScenarioFromFile(scenario))
        , states_(cScenario_->states)
        , fullBodyIkSolver_( 0.001f,0.01f, 0.05f)
    {
        // init contact by limbs
        for(std::size_t i=0; i< cScenario_->limbs.size(); ++i)
        {
            std::vector<Contact> limbContacts;
            contacts_.push_back(limbContacts);
        }
    }

    PImpl(planner::CompleteScenario* cScenario)
        : cScenario_(cScenario)
        , states_(cScenario_->states)
        , fullBodyIkSolver_( 0.001f,0.01f, 0.05f)
    {
        // init contact by limbs
        for(std::size_t i=0; i< cScenario_->limbs.size(); ++i)
        {
            std::vector<Contact> limbContacts;
            contacts_.push_back(limbContacts);
        }
    }

    ~PImpl()
    {
        delete cScenario_;
    }
    planner::CompleteScenario* cScenario_;
    planner::T_State states_;
    std::vector<std::vector<Contact> > contacts_;
    ik::IKSolver fullBodyIkSolver_;
};

using namespace efort;
using namespace planner;

namespace
{
    Object* GetEffector(Node* limb)
    {
        if(limb->children.size() != 0)
        {
            Object* res = GetEffector(limb->children[0]);
            if(res) return res;
        }
        return limb->current;
    }

    bool LimbColliding(Node* limb, planner::Object::T_Object& obstacles, bool effector = true)
    {
        if( limb->current && ((effector || limb->current != GetEffector(limb)) && limb->current->IsColliding(obstacles)))
        {
                return true;
        }
        if(limb->children.size() == 0)
            return false;
        return LimbColliding(limb->children[0], obstacles);
    }

    void SolveIk(Node* limb, const Eigen::Vector3d& target, const Eigen::Vector3d& normal)
    {
        ik::IKSolver solver;//(0.001f, 0.001f,0.1f);
        ik::VectorAlignmentConstraint constraint(normal);
        std::vector<ik::PartialDerivativeConstraint*> constraints;
        int limit = 1000;
        int limit2 = 200;
        while(limit > 0 && !solver.StepClamping(limb, target, normal, constraints, true))
        {
            limit--;
        }
        constraints.push_back(&constraint);
        while(limit2 > 0 )
        {
            limit2--;
            solver.StepClamping(limb, target, normal, constraints, true);
        }
    }

    //std::vector<planner::State*> propagate()

    // false if joint limits or obstacle not respected
    bool ComputeState(const Eigen::VectorXd& configuration, planner::State* state)
    {
        return false;
    }

    void PerformFullIk(planner::Robot& robot, const Eigen::VectorXd& framePositions, const ik::IKSolver& solver)
    {
        // we are only interested in internal joint positions...
        Eigen::VectorXd internalJoints = framePositions.tail(framePositions.rows()-3);
        // ... therefore removing robot translation joint
        Node* root = robot.node->children[0];
        int limit = 10;
        while(limit > 0 &&! solver.StepClamping(root,internalJoints))
        {
            limit--;
        }
    }

    void EffectorTargetRec(Node* current, const Eigen::VectorXd& framePositions, std::vector<Eigen::Vector3d>& targets,
                           std::size_t& id)
    {
        if(current->tag.find("endsite") != std::string::npos)
        {
            targets.push_back(framePositions.segment<3>(id*3));
        }
        else
        {
            if(current->offset != Eigen::Vector3d::Zero()) ++id;
            for(std::vector<Node*>::iterator it = current->children.begin();
                it != current->children.end(); ++it)
            {
                EffectorTargetRec(*it, framePositions, targets, id);
            }
        }
    }

    std::vector<Eigen::Vector3d> RetrieveEffectorTargets(const Eigen::VectorXd& framePositions, const planner::Robot& robot)
    {
        std::vector<Eigen::Vector3d> targets;
        std::size_t id(0);
        EffectorTargetRec(robot.node->children[0],framePositions,targets,id);
        return targets;
    }
}

Eigen::VectorXd Motion::Retarget(const std::size_t frameid, const Eigen::VectorXd& framePositions, const T_PointReplacement& objectModifications) const
{
    // retrieving frame
    const Frame& cframe = frames_[frameid];
    // cloning reference robot from frame.
    planner::Robot robot(*pImpl_->states_[frameid]->value);
    // moving robot to new position
    robot.SetPosition(framePositions.head<3>(), true);
    // performing ik to reconstruct joint variation:
    PerformFullIk(robot, framePositions, pImpl_->fullBodyIkSolver_);
    // retrieve effector targets
    std::vector<Eigen::Vector3d> targets = RetrieveEffectorTargets(framePositions, robot);

    //retrieving updated objects
    const ObjectDictionary& dictionnary = pImpl_->cScenario_->scenario->objDictionnary;
    std::vector<std::size_t> newObjectIds;
    planner::Object::T_Object objects =  dictionnary.recreate(objectModifications, pImpl_->cScenario_->scenario->objects_, newObjectIds);

    // now we can start the retargetting
    std::size_t id(0);
    for(std::vector<Contact>::const_iterator cit = cframe.contacts_.begin();
        cit !=cframe.contacts_.end(); ++cit, ++id)
    {
        // get corresponding limb with contact
        Node* limb = planner::GetChild(&robot,pImpl_->cScenario_->limbs[cit->limbIndex_]->id);
        // Approximating ROM with a sphere to check whether point belongs
        Sphere sphereCurrent(robot.currentRotation * robot.constantRotation.transpose() * pImpl_->cScenario_->limbRoms[cit->limbIndex_].center_ + robot.currentPosition,
                              pImpl_->cScenario_->limbRoms[cit->limbIndex_].radius_ * 1.5);
        bool contactMaintained(false);
        if(Contains(sphereCurrent, targets[id])) // point is contained: v0 accept configuration
        {
            //if(!LimbColliding(limb,objects,false))
            {
                contactMaintained = true;
std::cout << "configuration maintained for limb" << limb->tag << std::endl;
            }
        }
std::cout << "configuration invalid for limb" << limb->tag << std::endl;
        // otherwise, perform retarget
        if(!contactMaintained)
        {
            Eigen::Vector3d position, normal;
            std::vector<planner::Sphere*> dm;
            planner::sampling::Sample* nc =
                    planner::GetPosturesInContact(robot, limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],
                                                  objects,cit->surfaceNormal_,position, normal, *(pImpl_->cScenario_), dm);
            if(nc) // new contact found
            {
                planner::sampling::LoadSample(*nc, limb);
                SolveIk(limb, position, normal);
            }
            else // no contact found, just return a collision free posture
            {
                nc = planner::GetCollisionFreePosture(robot,limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],objects);
                if(nc) planner::sampling::LoadSample(*nc, limb);
            }
        }
    }

    // delete newly created objects
    for(std::vector<std::size_t>::const_iterator cit = newObjectIds.begin();
        cit != newObjectIds.end(); ++cit)
    {
        delete objects[*cit];
    }
    Eigen::VectorXd res = framePositions;
    res.tail(framePositions.rows()-3) =planner::AsPosition(robot.node->children[0]);
    return res;
}

#if INTERNAL
planner::Robot* Motion::RetargetInternal(const std::size_t frameid, const Eigen::VectorXd& framePositions, const T_PointReplacement& objectModifications) const
{
    // retrieving frame
    const Frame& cframe = frames_[frameid];
    // cloning reference robot from frame.
    planner::Robot* robot = new planner::Robot(*pImpl_->states_[frameid]->value);
    // moving robot to new position
    robot->SetPosition(framePositions.head<3>(), true);
    // performing ik to reconstruct joint variation:
    PerformFullIk(*robot, framePositions, pImpl_->fullBodyIkSolver_);
    // retrieve effector targets
    std::vector<Eigen::Vector3d> targets = RetrieveEffectorTargets(framePositions, *robot);

    //retrieving updated objects
    const ObjectDictionary& dictionnary = pImpl_->cScenario_->scenario->objDictionnary;
    std::vector<std::size_t> newObjectIds;
    planner::Object::T_Object objects =  dictionnary.recreate(objectModifications, pImpl_->cScenario_->scenario->objects_, newObjectIds);

    // now we can start the retargetting
    std::size_t id(0);
    for(std::vector<Contact>::const_iterator cit = cframe.contacts_.begin();
        cit !=cframe.contacts_.end(); ++cit, ++id)
    {
        // get corresponding limb with contact
        Node* limb = planner::GetChild(robot,pImpl_->cScenario_->limbs[cit->limbIndex_]->id);
        // Approximating ROM with a sphere to check whether point belongs
        Sphere sphereCurrent(robot->currentRotation * robot->constantRotation.transpose() * pImpl_->cScenario_->limbRoms[cit->limbIndex_].center_ + robot->currentPosition,
                              pImpl_->cScenario_->limbRoms[cit->limbIndex_].radius_ * 1.5);
        bool contactMaintained(false);
        if(Contains(sphereCurrent, targets[id])) // point is contained: v0 accept configuration
        {
            //if(!LimbColliding(limb,objects,false))
            {
                contactMaintained = true;
std::cout << "configuration maintained for limb" << limb->tag << std::endl;
            }
        }
        // otherwise, perform retarget
        if(!contactMaintained)
        {
std::cout << "configuration invalid for limb" << limb->tag << std::endl;
            Eigen::Vector3d position, normal;
            std::vector<planner::Sphere*> dm;
            planner::sampling::Sample* nc =
                    planner::GetPosturesInContact(*robot, limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],
                                                  objects,cit->surfaceNormal_,position, normal, *(pImpl_->cScenario_), dm);
            if(nc) // new contact found
            {
                planner::sampling::LoadSample(*nc, limb);
                SolveIk(limb, position, normal);
std::cout << "found contact " << limb->tag << std::endl;
            }
            else // no contact found, just return a collision free posture
            {
                nc = planner::GetCollisionFreePosture(*robot,limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],objects);
                if(nc) planner::sampling::LoadSample(*nc, limb);
std::cout << "no contact found contact" << limb->tag << std::endl;
            }
        }
    }

    // delete newly created objects
    for(std::vector<std::size_t>::const_iterator cit = newObjectIds.begin();
        cit != newObjectIds.end(); ++cit)
    {
        delete objects[*cit];
    }
    //Eigen::VectorXd res = framePositions;
    //res.tail(framePositions.rows()-3) =planner::AsPosition(robot->node->children[0]);
    return robot;
}
#endif


namespace
{
    std::vector<Frame> FramesFromStates(efort::PImpl* pImpl)
    {
        std::vector<Frame> res;
        // pour le moment on charge le chemin
        int numFrame = 0;
        std::vector< std::vector<std::size_t> > contactids; // storing references to contacts created at each frame
        for(planner::T_State::const_iterator sit_1 = pImpl->states_.begin();
            sit_1 != pImpl->states_.end(); ++sit_1, ++numFrame)
        {
            std::vector<std::size_t> frameContactIds;
            for(int i=0; i< pImpl->contacts_.size(); ++i)
            {
                frameContactIds.push_back(-1);
            }
            // create vectors
            State& cState = **sit_1;
            int cid = 0;
            for(std::vector<int>::const_iterator cit = cState.contactLimbs.begin();
                cit != cState.contactLimbs.end(); ++cit, ++cid)
            {
                //find position
                bool newContact(true);
                if(!pImpl->contacts_[*cit].empty())
                {
                    Contact& previous = pImpl->contacts_[*cit].back();
                    if(previous.endFrame_ == numFrame-1 && (previous.worldPosition_ - cState.contactLimbPositions[cid]).norm() < 0.01)
                    {
                        previous.endFrame_ = numFrame;
                        newContact = false;
                    }
                }
                if(newContact)
                {
                    Contact contact;
                    contact.startFrame_ = numFrame;
                    contact.endFrame_ = numFrame;
                    contact.limbIndex_ = *cit;
                    contact.surfaceNormal_ =  cState.contactLimbPositionsNormals[cid];
                    /*contact.triangleId_ = -1;
                    contact.objectId_ = -1;*/
                    contact.worldPosition_ =cState.contactLimbPositions[cid];
                    pImpl->contacts_[*cit].push_back(contact);
                }
                frameContactIds[*cit] = pImpl->contacts_[*cit].size()-1;
            }
            contactids.push_back(frameContactIds);
        }
        numFrame = 0;
        for(planner::T_State::const_iterator sit_1 = pImpl->states_.begin();
            sit_1 != pImpl->states_.end(); ++sit_1, ++numFrame)
        {
            Frame frame;
            //frame.configuration_ = planner::AsConfiguration((*sit_1)->value);
            for(int i=0; i< pImpl->contacts_.size(); ++i)
            {
                std::size_t id = contactids[numFrame][i];
                if(id != -1)
                {
                    frame.contacts_.push_back(pImpl->contacts_[i][id]);
                }
            }
            res.push_back(frame);

        }
        return res;
    }
}

Motion* efort::LoadMotion(const std::string& scenario)
{
    Motion* motion = new Motion;
    motion->pImpl_.reset(new PImpl(scenario));
    motion->frames_ = FramesFromStates(motion->pImpl_.get());
    return motion;
}


#if INTERNAL
Motion* efort::LoadMotion(CompleteScenario *scenario)
{
    Motion* motion = new Motion;
    motion->pImpl_.reset(new PImpl(scenario));
    motion->frames_ = FramesFromStates(motion->pImpl_.get());
    return motion;
}
#endif


