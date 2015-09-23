#include "retarget/Motion.h"
#include "prmpath/CompleteScenario.h"
#include "prmpath/PostureSelection.h"

#include "collision/Sphere.h"
#include "prmpath/CompleteScenario.h"
#include "prmpath/ik/IKSolver.h"
#include "prmpath/ik/VectorAlignmentConstraint.h"


void KeptIndices(planner::Node* node, int& last, int& current, std::vector<std::pair<int, int> >& res)
{
    if(node->offset != Eigen::Vector3d::Zero())
    {
        current += 3;
    }
    if(node->children.size() == 0 && node->tag.find("head") == std::string::npos)
    {
        res.push_back(std::make_pair(last, current));
        current +=3;
        last = current;
    }
    for(std::vector<planner::Node*>::const_iterator cit = node->children.begin();
        cit != node->children.end(); ++cit)
    {
        KeptIndices(*cit, last, current, res);
    }
}


std::vector<std::pair<int, int> > KeptIndices(planner::Robot* robot)
{
    std::vector<std::pair<int, int> > res;
    int current = 2;
    int last = 0;
    KeptIndices(robot->node,last,current,res);
    return res;
}

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


        if(planner::GetChild(cScenario_->initstate.value,"upper_left_leg_z_joint") != 0)
        {
            useFantomJoints = true;
            keptIndexes_ = KeptIndices(cScenario_->initstate.value);
        }
        else
        {
            useFantomJoints = false;
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
        if(planner::GetChild(cScenario_->initstate.value,"upper_left_leg_z_joint") != 0)
        {
            useFantomJoints = true;
            keptIndexes_ = KeptIndices(cScenario_->initstate.value);
        }
        else
        {
            useFantomJoints = false;
        }
    }

    Eigen::VectorXd adaptVector(const Eigen::VectorXd& input)
    {
        std::size_t size = input.rows() - 18 * 3;
        Eigen::VectorXd res(size);
        int current(0);
        for(std::vector<std::pair<int, int> >::const_iterator cit = keptIndexes_.begin();
        cit != keptIndexes_.end(); ++cit)
        {
            int margin = cit->second - cit->first;
            res.segment(current, margin) = input.segment(cit->first, margin);
            current+= margin;
        }
        return res;
    }

    std::vector<Eigen::VectorXd> adaptVector(const std::vector<Eigen::VectorXd>& framePositions)
    {
        std::vector<Eigen::VectorXd>  res;
        for( std::vector<Eigen::VectorXd>::const_iterator cit = framePositions.begin();
             cit != framePositions.end(); ++cit)
        {
            res.push_back(adaptVector(*cit));
        }
        return res;
    }


    planner::Model* ModelFromRobot(planner::Robot* robot)
    {
        planner::Model* model = new planner::Model(cScenario_->scenario->model_);
        model->SetPosition(robot->currentPosition);
        model->SetOrientation(robot->currentRotation);
        return model;
    }

    ~PImpl()
    {
        delete cScenario_;
    }
    planner::CompleteScenario* cScenario_;
    planner::T_State states_;
    std::vector<std::vector<Contact> > contacts_;
    std::vector<std::pair<int, int> > keptIndexes_;
    ik::IKSolver fullBodyIkSolver_;
    bool useFantomJoints;
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
        Object* eff = GetEffector(limb);
        if( limb->current && ((effector || limb->current != GetEffector(limb)) && limb->current->IsColliding(obstacles)))
        {
                return true;
        }
        if(limb->children.size() == 0)
            return false;
        return LimbColliding(limb->children[0], obstacles, effector);
    }

    void SolveIk(Node* limb, const Eigen::Vector3d& target, const Eigen::Vector3d& normal, int limit=1000
            , int limit2 = 200, bool normalconstraint = true)
    {
        ik::IKSolver solver;//(0.001f, 0.001f,0.1f);
        ik::VectorAlignmentConstraint constraint(normal);
        std::vector<ik::PartialDerivativeConstraint*> constraints;
        /*int limit = 100;
        int limit2 = 2;*/
        while(limit > 0 && !solver.StepClamping(limb, target, normal, constraints, true))
        {
            limit--;
        }
        constraints.push_back(&constraint);
        while(limit2 > 0 &&  normalconstraint)
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
        if(current->tag.find("endsite") != std::string::npos
                || current->tag.find("effector") != std::string::npos)
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


    struct EqualityCheck
    {
      EqualityCheck(const Contact & st) : st_(st) { }
      const Contact & st_;
      bool operator()(const Contact& lhs) const { return st_.equals(lhs); }
    };

    typedef std::vector<Contact> T_FailedContact;

    //index of next frame to checks
    std::size_t FrameInvalid(const efort::PImpl* pImpl_, Robot& robot, T_FailedContact& failedContacts, const Motion& motion, const std::size_t frameid, const Eigen::VectorXd& framePositions,
                      std::vector<Eigen::Vector3d>& targets, planner::Object::T_Object& obstacles, const int retargetType, bool force=false)
    {
        std::size_t nextFrameToCheck(frameid +1); // todo
        // retrieving frame
        const Frame& cframe = motion.frames_[frameid];
        // cloning reference robot from frame.
        // moving robot to new position
        robot.SetPosition(framePositions.head<3>(), true);
        // performing ik to reconstruct joint variation:
        PerformFullIk(robot, framePositions, pImpl_->fullBodyIkSolver_);
        // retrieve effector targets
        std::size_t id(0);
        EffectorTargetRec(robot.node->children[0],framePositions,targets,id);
        id = 0;
        for(std::vector<Contact>::const_iterator cit = cframe.contacts_.begin();
            cit !=cframe.contacts_.end(); ++cit, ++id)
        {
            bool retarget(force);
            // Approximating ROM with a sphere to check whether point belongs
            if(!retarget && (efort::reachability & (retargetType  & 0xFFFF)) != 0)
            {
                Sphere sphereCurrent(robot.currentRotation * robot.constantRotation.transpose() * pImpl_->cScenario_->limbRoms[cit->limbIndex_].center_ + robot.currentPosition,
                                      pImpl_->cScenario_->limbRoms[cit->limbIndex_].radius_ * 1.5);
                if((!Contains(sphereCurrent, targets[id]) || force)
                        && failedContacts.end() ==
                           std::find_if(failedContacts.begin(), failedContacts.end(), EqualityCheck(*cit))) // point is contained: v0 accept configuration
                {
                    retarget = true;
                }
            }
            if(!retarget && (efort::collision & (retargetType  & 0xFFFF)) != 0)
            {
                // retrieve kimb;
                Node* limb = planner::GetChild(&robot, pImpl_->cScenario_->limbs[cit->limbIndex_]->id);
                if(failedContacts.end() ==
                        std::find_if(failedContacts.begin(), failedContacts.end(), EqualityCheck(*cit))
                        && LimbColliding(limb, obstacles,false))
                {
                    retarget = true;
                }
            }
            if(retarget)
            {
                failedContacts.push_back(*cit);
                //nextFrameToCheck = std::max((int)nextFrameToCheck, (*cit).endFrame_);
            }
        }
        return nextFrameToCheck;
    }

    //index of next frame to checks
    std::size_t FrameInvalid(const efort::PImpl* pImpl_, Robot& robot, T_FailedContact& failedContacts, const Motion& motion, const std::size_t frameid, const Eigen::VectorXd& framePositions,
                      std::vector<Eigen::Vector3d>& targets, planner::Object::T_Object& obstacles, const int retargetType, const std::vector<bool> forcemask)
    {
        std::size_t nextFrameToCheck(frameid +1); // todo
        // retrieving frame
        const Frame& cframe = motion.frames_[frameid];
        // cloning reference robot from frame.
        // moving robot to new position
        robot.SetPosition(framePositions.head<3>(), true);
        // performing ik to reconstruct joint variation:
        PerformFullIk(robot, framePositions, pImpl_->fullBodyIkSolver_);
        // retrieve effector targets
        std::size_t id(0);
        EffectorTargetRec(robot.node->children[0],framePositions,targets,id);
        id = 0;
        for(std::vector<Contact>::const_iterator cit = cframe.contacts_.begin();
            cit !=cframe.contacts_.end(); ++cit, ++id)
        {
            bool retarget(forcemask[cit->limbIndex_]);
            // Approximating ROM with a sphere to check whether point belongs
            if(!retarget && (efort::reachability & (retargetType  & 0xFFFF)) != 0)
            {
                Sphere sphereCurrent(robot.currentRotation * robot.constantRotation.transpose() * pImpl_->cScenario_->limbRoms[cit->limbIndex_].center_ + robot.currentPosition,
                                      pImpl_->cScenario_->limbRoms[cit->limbIndex_].radius_ * 1.5);
                if((!Contains(sphereCurrent, targets[id]) || forcemask[cit->limbIndex_])
                        && failedContacts.end() ==
                           std::find_if(failedContacts.begin(), failedContacts.end(), EqualityCheck(*cit))) // point is contained: v0 accept configuration
                {
                    retarget = true;
                }
            }
            if(!retarget && (efort::collision & (retargetType  & 0xFFFF)) != 0)
            {
                // retrieve kimb;
                Node* limb = planner::GetChild(&robot, pImpl_->cScenario_->limbs[cit->limbIndex_]->id);
                if(failedContacts.end() ==
                        std::find_if(failedContacts.begin(), failedContacts.end(), EqualityCheck(*cit))
                        && LimbColliding(limb, obstacles,false))
                {
                    retarget = true;
                }
            }
            if(retarget)
            {
                failedContacts.push_back(*cit);
                //nextFrameToCheck = std::max((int)nextFrameToCheck, (*cit).endFrame_);
            }
        }
        return nextFrameToCheck;
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
        // otherwise, perform retarget
        if(!contactMaintained)
        {
std::cout << "configuration invalid for limb" << limb->tag << std::endl;
            Eigen::Vector3d position, normal;
            std::vector<planner::Sphere*> dm;
            double manipulability = pImpl_->states_[frameid]->manipulabilities[id];
            planner::sampling::Sample* nc =
                    planner::GetPosturesInContact(robot, limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],
                                                  objects,cit->surfaceNormal_,position, normal, *(pImpl_->cScenario_), manipulability,  dm, &sphereCurrent);
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
    res.tail(framePositions.rows()-3) =planner::AsPosition(robot.node->children[0], pImpl_->useFantomJoints);
    return res;
}

std::vector<Eigen::VectorXd> Motion::RetargetContact(const std::size_t frameid, const Eigen::VectorXd& framePositions,
                                                     const T_PointReplacement& objectModifications, bool force) const
{
    std::vector<Eigen::VectorXd> positions;
    // check how we are doing this frame
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


    std::vector<std::size_t> invalidIds;
    std::vector<Eigen::Vector3d> nextTargets;
    std::vector<Eigen::Vector3d> nextNormals;
    std::size_t furtherframe_(0);

    //v0: just find a contact satisfied for all contact frames (first and last)
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
                contactMaintained = true && !force;
std::cout << "configuration maintained for limb" << limb->tag << std::endl;
            }
        }
        // otherwise, perform retarget
        if(!contactMaintained)
        {
std::cout << "configuration invalid for limb" << limb->tag << std::endl;
            Eigen::Vector3d position, normal;
            std::vector<planner::Sphere*> dm;
            //push last frame sphere
            const Contact& contact = *cit;
            furtherframe_ = (furtherframe_ < contact.endFrame_) ? contact.endFrame_ : furtherframe_;
            planner::Robot * rLastFrame = pImpl_->states_[contact.endFrame_]->value;
            Sphere sphereLastFram(robot->currentRotation * rLastFrame->constantRotation.transpose() * pImpl_->cScenario_->limbRoms[cit->limbIndex_].center_ + rLastFrame->currentPosition,
                                  pImpl_->cScenario_->limbRoms[cit->limbIndex_].radius_ * 1.5);
            dm.push_back(&sphereLastFram);
            double manipulability = pImpl_->states_[frameid]->manipulabilities[id];
            //make sure position valid for all frames
            planner::sampling::Sample* nc =
                    planner::GetPosturesInContact(*robot, limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],
                                                  objects,cit->surfaceNormal_,position, normal, *(pImpl_->cScenario_), manipulability,  dm, &sphereCurrent);
            if(nc) // new contact found
            {
                planner::sampling::LoadSample(*nc, limb);
                nextTargets.push_back(position);
                nextNormals.push_back(normal);
                invalidIds.push_back(limb->id);
                SolveIk(limb, position, normal);
                pImpl_->cScenario_->states[frameid]->contactLimbPositions[id] = position;
                pImpl_->cScenario_->states[frameid]->contactLimbPositionsNormals[id] = normal;
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
    //res.push_back(robot);
    planner::Robot* r [300];
    r[0] = robot;
    #pragma omp parallel for
    for(int i = frameid; i <=furtherframe_; ++i)
    {
        planner::Robot* current = new planner::Robot(*pImpl_->states_[i]->value);
        std::size_t currentId(0);
        for(std::vector<std::size_t>::const_iterator ids = invalidIds.begin(); ids != invalidIds.end(); ++ids, ++currentId)
        {
            if(frames_[frameid].contacts_[currentId].endFrame_ >= i)
            {
                planner::Node* limb = planner::GetChild(current,*ids);
                planner::Node* refLimb = planner::GetChild(robot,*ids);
                sampling::Sample s(refLimb);
                planner::sampling::LoadSample(s,limb);
                SolveIk(limb, nextTargets[currentId], nextNormals[currentId],200,20, false);
            }
        }
        r[i-frameid] = current;
    }
    for(std::size_t i=0; i<=furtherframe_-frameid;++i)
    {
        Eigen::VectorXd res = framePositions;
        res.head(3) = robot->currentPosition;
        res.tail(framePositions.rows()-3) =planner::AsPosition(r[i]->node->children[0], pImpl_->useFantomJoints);
        delete r[i];
        positions.push_back(res);
    }
    std::cout << "nbframes " << positions.size() << std::endl;
    return positions; // TODO
}

std::vector<Eigen::VectorXd> Motion::RetargetContact(const std::size_t frameid, const Eigen::VectorXd& framePositions,
                                                     const T_PointReplacement& objectModifications, const std::vector<bool>& forcemask) const
{
    Eigen::VectorXd position;
    if(pImpl_->useFantomJoints)
    {
        position = pImpl_->adaptVector(framePositions);
    }
    else
    {
        position = framePositions;
    }
    std::vector<Eigen::VectorXd> positions;
    // check how we are doing this frame
    // retrieving frame
    const Frame& cframe = frames_[frameid];
    // cloning reference robot from frame.
    planner::Robot* robot = new planner::Robot(*pImpl_->states_[frameid]->value);
    // moving robot to new position
    robot->SetPosition(position.head<3>(), true);
    // performing ik to reconstruct joint variation:
    PerformFullIk(*robot, position, pImpl_->fullBodyIkSolver_);
    // retrieve effector targets
    std::vector<Eigen::Vector3d> targets = RetrieveEffectorTargets(position, *robot);

    //retrieving updated objects
    const ObjectDictionary& dictionnary = pImpl_->cScenario_->scenario->objDictionnary;
    std::vector<std::size_t> newObjectIds;
    planner::Object::T_Object objects =  dictionnary.recreate(objectModifications, pImpl_->cScenario_->scenario->objects_, newObjectIds);


    std::vector<std::size_t> invalidIds;
    std::vector<Eigen::Vector3d> nextTargets;
    std::vector<Eigen::Vector3d> nextNormals;
    std::size_t furtherframe_(0);

    //v0: just find a contact satisfied for all contact frames (first and last)
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
                contactMaintained = true && !forcemask[cit->limbIndex_];
if(contactMaintained) std::cout << "configuration maintained for limb" << limb->tag << std::endl;
            }
        }
        // otherwise, perform retarget
        if(!contactMaintained)
        {
std::cout << "configuration invalid for limb" << limb->tag << std::endl;
            Eigen::Vector3d position, normal;
            std::vector<planner::Sphere*> dm;
            //push last frame sphere
            const Contact& contact = *cit;
            furtherframe_ = (furtherframe_ < contact.endFrame_) ? contact.endFrame_ : furtherframe_;
            planner::Robot * rLastFrame = pImpl_->states_[contact.endFrame_]->value;
            Sphere sphereLastFram(robot->currentRotation * rLastFrame->constantRotation.transpose() * pImpl_->cScenario_->limbRoms[cit->limbIndex_].center_ + rLastFrame->currentPosition,
                                  pImpl_->cScenario_->limbRoms[cit->limbIndex_].radius_ * 1.5);
            dm.push_back(&sphereLastFram);
            double manipulability = pImpl_->states_[frameid]->manipulabilities[id];
            //make sure position valid for all frames
            planner::sampling::Sample* nc =
                    planner::GetPosturesInContact(*robot, limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],
                                                  objects,cit->surfaceNormal_,position, normal, *(pImpl_->cScenario_), manipulability,  dm, &sphereCurrent);
            if(nc) // new contact found
            {
                planner::sampling::LoadSample(*nc, limb);
                nextTargets.push_back(position);
                nextNormals.push_back(normal);
                invalidIds.push_back(limb->id);
                SolveIk(limb, position, normal);
                pImpl_->cScenario_->states[frameid]->contactLimbPositions[id] = position;
                pImpl_->cScenario_->states[frameid]->contactLimbPositionsNormals[id] = normal;
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
    //res.push_back(robot);
    planner::Robot* r [300];
    r[0] = robot;
    #pragma omp parallel for
    for(int i = frameid; i <=furtherframe_; ++i)
    {
        planner::Robot* current = new planner::Robot(*pImpl_->states_[i]->value);
        std::size_t currentId(0);
        for(std::vector<std::size_t>::const_iterator ids = invalidIds.begin(); ids != invalidIds.end(); ++ids, ++currentId)
        {
            if(frames_[frameid].contacts_[currentId].endFrame_ >= i)
            {
                planner::Node* limb = planner::GetChild(current,*ids);
                planner::Node* refLimb = planner::GetChild(robot,*ids);
                sampling::Sample s(refLimb);
                planner::sampling::LoadSample(s,limb);
                SolveIk(limb, nextTargets[currentId], nextNormals[currentId],200,20, false);
            }
        }
        r[i-frameid] = current;
    }
    for(std::size_t i=0; i<=furtherframe_-frameid;++i)
    {
        Eigen::VectorXd res = framePositions;
        res.head(3) = robot->currentPosition;
        res.tail(framePositions.rows()-3) =planner::AsPosition(r[i]->node->children[0], pImpl_->useFantomJoints);
        delete r[i];
        positions.push_back(res);
    }
    std::cout << "nbframes " << positions.size() << std::endl;
    return positions; // TODO
}

#include "prmpath/animation/StateInterpolation.h"

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
                    if(previous.endFrame_ == numFrame-1) // && (previous.worldPosition_ - cState.contactLimbPositions[cid]).norm() < 0.15)
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
            Eigen::VectorXd conf = planner::AsPosition((*sit_1)->value->node->children[0], pImpl->useFantomJoints);
            frame.configuration_ = Eigen::VectorXd(conf.rows()+3);
            frame.configuration_.head(3) = (*sit_1)->value->currentPosition;
            frame.configuration_.tail(conf.rows()) =conf;
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

void Motion::ReloadMotion()
{
    pImpl_->states_ = pImpl_->cScenario_->states;
    frames_ = FramesFromStates(pImpl_.get());
}

#include "prm/SimpleRRT.h"

void Motion::DoRRT(const std::size_t frameidFrom, const Eigen::VectorXd& frameFrom, const Eigen::VectorXd& frameTo,
                                             const T_PointReplacement &objectModifications, bool useSplines)
{
    Eigen::VectorXd positionFrom, positionTo;
    if(pImpl_->useFantomJoints)
    {
        positionFrom = pImpl_->adaptVector(frameFrom);
        positionTo = pImpl_->adaptVector(frameTo);
    }
    else
    {
        positionFrom = frameFrom;
        positionTo = frameTo;
    }
    planner::State* sFrom = new planner::State(pImpl_->states_[frameidFrom]);
    planner::State* sTo = new planner::State(pImpl_->states_[frameidFrom+1]);
    planner::Robot* robotFrom = sFrom->value;
    planner::Robot* robotTo = sTo->value;
    robotFrom->SetPosition(positionFrom.head<3>(), true);
    robotTo->SetPosition(positionTo.head<3>(), true);
    PerformFullIk(*robotFrom, positionFrom, pImpl_->fullBodyIkSolver_);
    PerformFullIk(*robotTo, positionTo, pImpl_->fullBodyIkSolver_);

    Model* mFrom = pImpl_->ModelFromRobot(robotFrom);
    Model* mTo = pImpl_->ModelFromRobot(robotTo);

    const ObjectDictionary& dictionnary = pImpl_->cScenario_->scenario->objDictionnary;
    std::vector<std::size_t> newObjectIds;
    planner::Object::T_Object objects =  dictionnary.recreate(objectModifications, pImpl_->cScenario_->scenario->objects_, newObjectIds);

    SimpleRRT rrt(mFrom,mTo,objects,pImpl_->cScenario_->scenario->neighbourDistance_,
                   pImpl_->cScenario_->scenario->size_, pImpl_->cScenario_->scenario->neighbours_);

    CT_Model path(rrt.GetPath());
    planner::T_State res = planner::PostureSequence(*pImpl_->cScenario_, path, sFrom);
    planner::T_State newStates;
    if(useSplines && !res.empty())
    {
        planner::T_State statesinterpolate[1000];
        //planner::T_State::const_iterator cit = res.begin();
        //planner::T_State::const_iterator cit2 = res.begin(); ++cit2;
        #pragma omp parallel for
        for(int i =1; i< res.size(); ++i)
        {
            statesinterpolate[i] = planner::Animate(*pImpl_->cScenario_, res[i-1], res[i], 24, useSplines, false, true);
        }
        for(int i =0; i< res.size()-1; ++i)
        {
            newStates.insert(newStates.end(),statesinterpolate[i].begin(), statesinterpolate[i].end());
        }
    }
    else
    {
        newStates = res;
    }
    //planner::T_State newStates = planner::Animate(*pImpl_->cScenario_, *sFrom, *sTo, 24, useSplines, useRRT);
    pImpl_->cScenario_->states.insert(pImpl_->cScenario_->states.begin()+frameidFrom+1,newStates.begin(),newStates.end());
    ReloadMotion();
}

void Motion::Interpolate(const std::size_t frameid, const Eigen::VectorXd& frameFrom, const Eigen::VectorXd& frameTo, bool useSplines, bool useRRT)
{
    Eigen::VectorXd positionFrom, positionTo;
    if(pImpl_->useFantomJoints)
    {
        positionFrom = pImpl_->adaptVector(frameFrom);
        positionTo = pImpl_->adaptVector(frameTo);
    }
    else
    {
        positionFrom = frameFrom;
        positionTo = frameTo;
    }
    planner::State* sFrom = new planner::State(pImpl_->states_[frameid]);
    planner::State* sTo = new planner::State(pImpl_->states_[frameid+1]);
    planner::Robot* robotFrom = sFrom->value;
    planner::Robot* robotTo = sTo->value;
    robotFrom->SetPosition(positionFrom.head<3>(), true);
    robotTo->SetPosition(positionTo.head<3>(), true);
    PerformFullIk(*robotFrom, positionFrom, pImpl_->fullBodyIkSolver_);
    PerformFullIk(*robotTo, positionTo, pImpl_->fullBodyIkSolver_);
    planner::T_State newStates = planner::Animate(*pImpl_->cScenario_, *sFrom, *sTo, 24, useSplines, useRRT);
    pImpl_->cScenario_->states.insert(pImpl_->cScenario_->states.begin()+frameid+1,newStates.begin(),newStates.end());
    ReloadMotion();
}


void Motion::Interpolate(const std::size_t frameid, const Eigen::VectorXd& frameFrom, const Eigen::VectorXd& frameTo, const int nbFrames, bool useSplines, bool useRRT)
{
    Eigen::VectorXd positionFrom, positionTo;
    if(pImpl_->useFantomJoints)
    {
        positionFrom = pImpl_->adaptVector(frameFrom);
        positionTo = pImpl_->adaptVector(frameTo);
    }
    else
    {
        positionFrom = frameFrom;
        positionTo = frameTo;
    }
    planner::State* sFrom = new planner::State(pImpl_->states_[frameid]);
    planner::State* sTo = new planner::State(pImpl_->states_[frameid+1]);
    planner::Robot* robotFrom = sFrom->value;
    planner::Robot* robotTo = sTo->value;
    robotFrom->SetPosition(positionFrom.head<3>(), true);
    robotTo->SetPosition(positionTo.head<3>(), true);
    PerformFullIk(*robotFrom, positionFrom, pImpl_->fullBodyIkSolver_);
    PerformFullIk(*robotTo, positionTo, pImpl_->fullBodyIkSolver_);
    planner::T_State newStates = planner::Animate(*pImpl_->cScenario_, *sFrom, *sTo, nbFrames, useSplines, useRRT, true);
    pImpl_->cScenario_->states.insert(pImpl_->cScenario_->states.begin()+frameid+1,newStates.begin(),newStates.end());
    ReloadMotion();
}


bool RetargetLimbContact(const efort::PImpl* pImpl_, planner::Robot* r[], const Contact& contact,  planner::Object::T_Object& objects, std::vector<FrameReport>& res
                         , const std::size_t frameStart, const std::size_t frameEnd)
{
    std::vector<std::size_t> invalidIds;
    std::vector<Eigen::Vector3d> nextTargets;
    std::vector<Eigen::Vector3d> nextNormals;
    bool contactfound(false);

    //initial robot:
    std::size_t frameid(std::max(contact.startFrame_,(int)frameStart));
    planner::Robot* robot = r[frameid];
    //v0: just find a contact satisfied for all contact frames (first and last)
    std::size_t id(0);
    // get corresponding limb with contact
    Node* limb = planner::GetChild(robot,pImpl_->cScenario_->limbs[contact.limbIndex_]->id);
    // Approximating ROM with a sphere to check whether point belongs
    Sphere sphereCurrent(robot->currentRotation * robot->constantRotation.transpose() * pImpl_->cScenario_->limbRoms[contact.limbIndex_].center_ + robot->currentPosition,
                          pImpl_->cScenario_->limbRoms[contact.limbIndex_].radius_ * 1.5);

    // perform retarget
std::cout << "configuration invalid for limb" << limb->tag << std::endl;
    Eigen::Vector3d position, normal;
    std::vector<planner::Sphere*> dm;
    //push last frame sphere
    planner::Robot * rLastFrame = pImpl_->states_[contact.endFrame_]->value;
    Sphere sphereLastFram(robot->currentRotation * rLastFrame->constantRotation.transpose() * pImpl_->cScenario_->limbRoms[contact.limbIndex_].center_ + rLastFrame->currentPosition,
                          pImpl_->cScenario_->limbRoms[contact.limbIndex_].radius_ * 1.5);
    dm.push_back(&sphereLastFram);
    double manipulability = pImpl_->states_[frameid]->manipulabilities[id];
    //make sure position valid for all frames
    planner::sampling::Sample* nc =
            planner::GetPosturesInContact(*robot, limb, pImpl_->cScenario_->limbSamples[contact.limbIndex_],
                                          objects,contact.surfaceNormal_,position, normal, *(pImpl_->cScenario_), manipulability,  dm, &sphereCurrent);
    if(nc) // new contact found
    {
        //planner::sampling::LoadSample(*nc, limb);
        nextTargets.push_back(position);
        nextNormals.push_back(normal);
        invalidIds.push_back(limb->id);
        SolveIk(limb, position, normal);
        pImpl_->cScenario_->states[frameid]->contactLimbPositions[id] = position;
        pImpl_->cScenario_->states[frameid]->contactLimbPositionsNormals[id] = normal;
std::cout << "found contact " << limb->tag << std::endl;
        contactfound = true;
    }
    else // no contact found, just return a collision free posture
    {
        nc = planner::GetCollisionFreePosture(*robot,limb, pImpl_->cScenario_->limbSamples[contact.limbIndex_],objects);
        if(nc) planner::sampling::LoadSample(*nc, limb);
std::cout << "no contact found for" << limb->tag << std::endl;
    }
    //Eigen::VectorXd res = framePositions;
    //res.tail(framePositions.rows()-3) =planner::AsPosition(robot->node->children[0]);
    //res.push_back(robot);
    #pragma omp parallel for
    for(int i = frameid; i <= std::min(contact.endFrame_, (int)frameEnd); ++i)
    {
        planner::Robot* current = r[i];
        std::size_t currentId(0);
        for(std::vector<std::size_t>::const_iterator ids = invalidIds.begin(); ids != invalidIds.end(); ++ids, ++currentId)
        {
            FrameReport& fr = res[i-frameid];
            fr.contactStates[contact.limbIndex_] = 1;
            fr.retargeted_ = true;
            planner::Node* limb = planner::GetChild(current,*ids);
            planner::Node* refLimb = planner::GetChild(robot,*ids);
            sampling::Sample s(refLimb);
            planner::sampling::LoadSample(s,limb);
            SolveIk(limb, nextTargets[currentId], nextNormals[currentId],200,20, false);
        }
    }
    return contactfound;
}

std::vector<FrameReport> Motion::RetargetMotion(const std::vector<Eigen::VectorXd>& framePositions, const T_PointReplacement& objectModifications,
                                                const int retargetType, const std::size_t frameStart, bool force) const
{
    //collect object updates
    //retrieving updated objects
    const ObjectDictionary& dictionnary = pImpl_->cScenario_->scenario->objDictionnary;
    std::vector<std::size_t> newObjectIds;
    planner::Object::T_Object objects =  dictionnary.recreate(objectModifications, pImpl_->cScenario_->scenario->objects_, newObjectIds);

    std::vector<FrameReport> res;
    assert(framePositions.size() + frameStart  < this->frames_.size());
    // create robot for each frame;
    int nbLimbs = pImpl_->cScenario_->limbs.size();
    planner::Robot* r [400];
    for(std::size_t i = frameStart; i < frameStart + framePositions.size(); ++i)
    {
        r[i] = new planner::Robot(*pImpl_->states_[i]->value);
        res.push_back(FrameReport(i, nbLimbs));
    }
    // now find invalid states
    // i incremented by frameInvalid
    T_FailedContact failedContacts;
    std::vector<Eigen::Vector3d> targets;
    for(std::size_t i = frameStart; i < frameStart + framePositions.size();)
    {
        i = FrameInvalid(pImpl_.get(), *r[i],failedContacts, *this, i, framePositions[i-frameStart],targets, objects, retargetType, force);
    }
std::cout << "size contacts" << failedContacts.size() << std::endl;
    // all wrong contacts are pushed in failedContacts, can now run IK and find new contacts
    for(T_FailedContact::const_iterator cit = failedContacts.begin();
        cit != failedContacts.end(); ++cit)
    {
        RetargetLimbContact(this->pImpl_.get(), r, *cit, objects, res, frameStart, frameStart + framePositions.size()-1);
    }

    std::vector<FrameReport>::iterator cit = res.begin();
    for(; cit !=res.end(); ++cit)
    {
        if(cit->retargeted_)
        {
            //cit->pose_ = planner::AsPosition(r[cit->frameId_]->node->children[0]);
            cit->pose_ = framePositions[0];
            cit->pose_.head(3) = r[cit->frameId_]->currentPosition;
            cit->pose_.tail(cit->pose_.rows()-3) = planner::AsPosition(r[cit->frameId_]->node->children[0], pImpl_->useFantomJoints);
        }
        delete r[cit->frameId_];
    }

    // delete newly created objects
    for(std::vector<std::size_t>::const_iterator cit = newObjectIds.begin();
        cit != newObjectIds.end(); ++cit)
    {
        delete objects[*cit];
    }
    return res;
}

std::vector<FrameReport> Motion::RetargetMotion(const std::vector<Eigen::VectorXd>& framePositions, const T_PointReplacement& objectModifications,
                                                const int retargetType, const std::size_t frameStart, const std::vector<bool>& forcemask) const
{
    //collect object updates
    //retrieving updated objects
    const ObjectDictionary& dictionnary = pImpl_->cScenario_->scenario->objDictionnary;
    std::vector<std::size_t> newObjectIds;
    planner::Object::T_Object objects =  dictionnary.recreate(objectModifications, pImpl_->cScenario_->scenario->objects_, newObjectIds);

    std::vector<FrameReport> res;
    assert(framePositions.size() + frameStart  < this->frames_.size());
    // create robot for each frame;
    int nbLimbs = pImpl_->cScenario_->limbs.size();
    planner::Robot* r [400];
    for(std::size_t i = frameStart; i < frameStart + framePositions.size(); ++i)
    {
        r[i] = new planner::Robot(*pImpl_->states_[i]->value);
        res.push_back(FrameReport(i, nbLimbs));
    }
    // now find invalid states
    // i incremented by frameInvalid
    T_FailedContact failedContacts;
    std::vector<Eigen::Vector3d> targets;
    for(std::size_t i = frameStart; i < frameStart + framePositions.size();)
    {
        i = FrameInvalid(pImpl_.get(), *r[i],failedContacts, *this, i, framePositions[i-frameStart],targets, objects, retargetType, forcemask);
    }
std::cout << "size contacts" << failedContacts.size() << std::endl;
    // all wrong contacts are pushed in failedContacts, can now run IK and find new contacts
    for(T_FailedContact::const_iterator cit = failedContacts.begin();
        cit != failedContacts.end(); ++cit)
    {
        RetargetLimbContact(this->pImpl_.get(), r, *cit, objects, res, frameStart, frameStart + framePositions.size()-1);
    }

    std::vector<FrameReport>::iterator cit = res.begin();
    for(; cit !=res.end(); ++cit)
    {
        if(cit->retargeted_)
        {
            //cit->pose_ = planner::AsPosition(r[cit->frameId_]->node->children[0]);
            cit->pose_ = framePositions[0];
            cit->pose_.head(3) = r[cit->frameId_]->currentPosition;
            cit->pose_.tail(cit->pose_.rows()-3) = planner::AsPosition(r[cit->frameId_]->node->children[0], pImpl_->useFantomJoints);
        }
        delete r[cit->frameId_];
    }

    // delete newly created objects
    for(std::vector<std::size_t>::const_iterator cit = newObjectIds.begin();
        cit != newObjectIds.end(); ++cit)
    {
        delete objects[*cit];
    }
    return res;
}

#if INTERNAL
std::vector<planner::Robot*> Motion::RetargetMotionInternal(const std::vector<Eigen::VectorXd>& framePositions, const T_PointReplacement& objectModifications,
                                                            const std::size_t frameStart, const int retargetType, bool force) const
{
    //collect object updates
    //retrieving updated objects
    std::vector<Eigen::VectorXd> position;
    if(pImpl_->useFantomJoints)
    {
        position = pImpl_->adaptVector(framePositions);
    }
    else
    {
        position = framePositions;
    }
    const ObjectDictionary& dictionnary = pImpl_->cScenario_->scenario->objDictionnary;
    std::vector<std::size_t> newObjectIds;
    planner::Object::T_Object objects =  dictionnary.recreate(objectModifications, pImpl_->cScenario_->scenario->objects_, newObjectIds);


    std::vector<FrameReport> res;
    assert(position.size() + frameStart  < this->frames_.size());
    // create robot for each frame;
    int nbLimbs = pImpl_->cScenario_->limbs.size();
    planner::Robot* r [400];
    for(std::size_t i = frameStart; i < frameStart + position.size(); ++i)
    {
        r[i] = new planner::Robot(*pImpl_->states_[i]->value);
        res.push_back(FrameReport(i, nbLimbs));
    }
    // now find invalid states
    // i incremented by frameInvalid
    T_FailedContact failedContacts;
    std::vector<Eigen::Vector3d> targets;
    for(std::size_t i = frameStart; i < frameStart + position.size();)
    {
        i = FrameInvalid(pImpl_.get(), *r[i],failedContacts, *this, i, position[i-frameStart],targets, objects, retargetType, force);
    }


std::cout << "size contacts" << failedContacts.size() << std::endl;
    // all wrong contacts are pushed in failedContacts, can now run IK and find new contacts
    for(T_FailedContact::const_iterator cit = failedContacts.begin();
        cit != failedContacts.end(); ++cit)
    {
        RetargetLimbContact(this->pImpl_.get(), r, *cit, objects, res, frameStart, frameStart + position.size()-1);
    }

    std::vector<FrameReport>::iterator cit = res.begin();
    for(; cit !=res.end(); ++cit)
    {
        if(cit->retargeted_)
        {
            //cit->pose_ = planner::AsPosition(r[cit->frameId_]->node->children[0]);
            cit->pose_ = framePositions[0];
            cit->pose_.head(3) = r[cit->frameId_]->currentPosition;
            cit->pose_.tail(cit->pose_.rows()-3) = planner::AsPosition(r[cit->frameId_]->node->children[0], pImpl_->useFantomJoints);
        }
// load robot TMP
        //delete r[cit->frameId_];
    }


    for(std::vector<FrameReport>::const_iterator cit = res.begin(); cit != res.end(); ++cit)
    {
        const FrameReport& fr = *cit;
        if(fr.retargeted_)
        {
            std::cout << " frame retargeted " << fr.frameId_ << std::endl;
            //std::cout << "configuration \n " << fr.pose_ << std::endl;
        }
    }

    // delete newly created objects
    for(std::vector<std::size_t>::const_iterator cit = newObjectIds.begin();
        cit != newObjectIds.end(); ++cit)
    {
        delete objects[*cit];
    }

    std::vector<planner::Robot*> robots;
    for(std::size_t i=frameStart; i<frameStart + framePositions.size();++i)
    {
        robots.push_back(r[i]);
    }
    return robots;
}
#endif


#if INTERNAL
planner::Robot* Motion::RetargetInternal(const std::size_t frameid, const Eigen::VectorXd& framePositions, const T_PointReplacement& objectModifications) const
{
    Eigen::VectorXd position;
    if(pImpl_->useFantomJoints)
    {
        position = pImpl_->adaptVector(framePositions);
    }
    else
    {
        position = framePositions;
    }
    // retrieving frame
    const Frame& cframe = frames_[frameid];
    // cloning reference robot from frame.
    planner::Robot* robot = new planner::Robot(*pImpl_->states_[frameid]->value);
    // moving robot to new position
    robot->SetPosition(position.head<3>(), true);
    // performing ik to reconstruct joint variation:
    PerformFullIk(*robot, position, pImpl_->fullBodyIkSolver_);
    // retrieve effector targets
    std::vector<Eigen::Vector3d> targets = RetrieveEffectorTargets(position, *robot);

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
            double manipulability = pImpl_->states_[frameid]->manipulabilities[id];
            planner::sampling::Sample* nc =
                    planner::GetPosturesInContact(*robot, limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],
                                                  objects,cit->surfaceNormal_,position, normal, *(pImpl_->cScenario_), manipulability,  dm, &sphereCurrent);

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

std::vector<planner::Robot*> Motion::RetargetContactInternal(const std::size_t frameid, const Eigen::VectorXd& framePositions, const T_PointReplacement& objectModifications, bool force) const
{
    Eigen::VectorXd position;
    if(pImpl_->useFantomJoints)
    {
        position = pImpl_->adaptVector(framePositions);
    }
    else
    {
        position = framePositions;
    }
    std::vector<planner::Robot*> res;
    // check how we are doing this frame
    // retrieving frame
    const Frame& cframe = frames_[frameid];
    // cloning reference robot from frame.
    planner::Robot* robot = new planner::Robot(*pImpl_->states_[frameid]->value);
    // moving robot to new position
    robot->SetPosition(position.head<3>(), true);
    // performing ik to reconstruct joint variation:
    PerformFullIk(*robot, position, pImpl_->fullBodyIkSolver_);
    // retrieve effector targets
    std::vector<Eigen::Vector3d> targets = RetrieveEffectorTargets(position, *robot);

    //retrieving updated objects
    const ObjectDictionary& dictionnary = pImpl_->cScenario_->scenario->objDictionnary;
    std::vector<std::size_t> newObjectIds;
    planner::Object::T_Object objects =  dictionnary.recreate(objectModifications, pImpl_->cScenario_->scenario->objects_, newObjectIds);


    std::vector<std::size_t> invalidIds;
    std::vector<Eigen::Vector3d> nextTargets;
    std::vector<Eigen::Vector3d> nextNormals;
    std::size_t furtherframe_(0);

    //v0: just find a contact satisfied for all contact frames (first and last)
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
                contactMaintained = true && !force;
                if(contactMaintained) std::cout << "configuration maintained for limb" << limb->tag << std::endl;
            }
        }
        // otherwise, perform retarget
        if(!contactMaintained)
        {
std::cout << "configuration invalid for limb" << limb->tag << std::endl;
            Eigen::Vector3d position, normal;
            std::vector<planner::Sphere*> dm;
            //push last frame sphere
            const Contact& contact = *cit;
            furtherframe_ = (furtherframe_ < contact.endFrame_) ? contact.endFrame_ : furtherframe_;
            planner::Robot * rLastFrame = pImpl_->states_[contact.endFrame_]->value;
            Sphere sphereLastFram(robot->currentRotation * rLastFrame->constantRotation.transpose() * pImpl_->cScenario_->limbRoms[cit->limbIndex_].center_ + rLastFrame->currentPosition,
                                  pImpl_->cScenario_->limbRoms[cit->limbIndex_].radius_ * 1.5);
            dm.push_back(&sphereLastFram);
            double manipulability = pImpl_->states_[frameid]->manipulabilities[id];
            //make sure position valid for all frames
            planner::sampling::Sample* nc =
                    planner::GetPosturesInContact(*robot, limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],
                                                  objects,cit->surfaceNormal_,position, normal, *(pImpl_->cScenario_), manipulability,  dm, &sphereCurrent);
            if(nc) // new contact found
            {
                planner::sampling::LoadSample(*nc, limb);
                nextTargets.push_back(position);
                nextNormals.push_back(normal);
                invalidIds.push_back(limb->id);
                SolveIk(limb, position, normal);
                pImpl_->cScenario_->states[frameid]->contactLimbPositions[id] = position;
                pImpl_->cScenario_->states[frameid]->contactLimbPositionsNormals[id] = normal;
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
    //res.push_back(robot);
    planner::Robot* r [300];
    r[0] = robot;
    #pragma omp parallel for
    for(int i = frameid; i <=furtherframe_; ++i)
    {
        planner::Robot* current = new planner::Robot(*pImpl_->states_[i]->value);
        std::size_t currentId(0);
        for(std::vector<std::size_t>::const_iterator ids = invalidIds.begin(); ids != invalidIds.end(); ++ids, ++currentId)
        {
            if(frames_[frameid].contacts_[currentId].endFrame_ >= i)
            {
                planner::Node* limb = planner::GetChild(current,*ids);
                planner::Node* refLimb = planner::GetChild(robot,*ids);
                sampling::Sample s(refLimb);
                planner::sampling::LoadSample(s,limb);
                SolveIk(limb, nextTargets[currentId], nextNormals[currentId],200,20, false);
            }
        }
        r[i-frameid] = current;
    }
    for(std::size_t i=0; i<=furtherframe_-frameid;++i)
    {
        res.push_back(r[i]);
    }
    std::cout << "nbframes " << res.size() << std::endl;
    return res;
}

#endif


Motion* efort::LoadMotion(const std::string& scenario)
{
    Motion* motion = new Motion;
    motion->pImpl_.reset(new PImpl(scenario));
    motion->frames_ = FramesFromStates(motion->pImpl_.get());
    return motion;
}


#if INTERNAL
void DumpFrames(const std::vector< std::vector<Contact> >& ccontacts)
{
    for(int i =0; i < ccontacts.size(); ++i)
    {
        std::cout << " member " << i << std::endl;
        const std::vector<Contact>& contacts = ccontacts[i];
        for(std::vector<Contact>::const_iterator cit = contacts.begin();
            cit != contacts.end(); ++cit)
        {
            const Contact& c = *cit;
            std::cout << "\t contact " << c.startFrame_ << "to " << c.endFrame_ << std::endl;
        }
        std::cout << "end member " << i << std::endl;
    }
}

Motion* efort::LoadMotion(CompleteScenario *scenario)
{
    Motion* motion = new Motion;
    motion->pImpl_.reset(new PImpl(scenario));
motion->pImpl_->useFantomJoints = false;
    motion->frames_ = FramesFromStates(motion->pImpl_.get());
    DumpFrames(motion->pImpl_->contacts_);
    return motion;
}
#endif


