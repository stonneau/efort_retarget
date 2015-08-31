#include "prmpath/animation/StateInterpolation.h"
#include "prmpath/ik/IKSolver.h"
#include "prmpath/ik/ObstacleAvoidanceConstraint.h"
#include "prmpath/ik/PartialDerivativeConstraint.h"
#include "prmpath/ik/VectorAlignmentConstraint.h"
#include "prmpath/ik/MatchTargetConstraint.h"

#include "tools/ExpMap.h"

#include "spline/bezier_curve.h"
#include "spline/exact_cubic.h"

#include <vector>

using namespace std;
using namespace planner;

namespace
{
    typedef Eigen::Vector3d point_t;
    //typedef spline::exact_cubic <double, double, 3, true, point_t> exact_cubic_t;
    typedef spline::bezier_curve  <double, double, 3, true, point_t> exact_cubic_t;
    //typedef std::pair<double, point_t> Waypoint;
    typedef point_t Waypoint;
    typedef std::vector<Waypoint> T_Waypoint;

    planner::C2_Point MakeConfiguration(const planner::State& state)
    {
        matrices::ExpMap emap(state.value->currentRotation);
        return std::make_pair(state.value->currentPosition, emap.log());
    }

    planner::C2_Point MakeConfiguration(const planner::Object* state)
    {
        matrices::ExpMap emap(state->GetOrientation());
        return std::make_pair(state->GetPosition(), emap.log());
    }

    planner::Object* GetEffector(planner::Node* limb, int& id)
    {
        if(limb->current)
            id = limb->id;
        if(limb->children.size() != 0)
        {
            planner::Object* res = GetEffector(limb->children[0], id);
            if(res)
            {
                return res;
            }
        }
        return limb->current;
    }


    struct InterpolateLine
    {
        InterpolateLine(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
            : from_(from), to_(to){}
        ~InterpolateLine(){}

        Eigen::Vector3d operator () (double t) const
        {
            return from_ + t * (to_ - from_);
        }

        double distance() const
        {
            return (to_ - from_).norm();
        }

        Eigen::Vector3d from_;
        Eigen::Vector3d to_;
    };

    exact_cubic_t* createSpline(const InterpolatePath* path)
    {
        T_Waypoint waypoints;
        for(T_MilePoint::const_iterator cit = path->milePoints_.begin();
            cit != path->milePoints_.end(); ++cit)
        {
            //waypoints.push_back(std::make_pair(cit->first,cit->second.first));
            waypoints.push_back(cit->second.first);
        }
        //waypoints.push_back(std::make_pair(1.,path->max().first));
        return new exact_cubic_t(waypoints.begin(),waypoints.end());
    }

    struct InterpolateSpline
    {
        InterpolateSpline(const InterpolatePath* path, Collider& collider, const Model& model)
            : cspline_(createSpline(path))
            , path_(path)
        {

        }
        ~InterpolateSpline(){delete cspline_;delete path_;}

        Eigen::Vector3d operator () (double t) const
        {
            return (*cspline_)(t);
        }

        double distance() const
        {
            return ((*cspline_)(1) - (*cspline_)(0)).norm();
        }
        exact_cubic_t* cspline_;
        //planner::SplinePath spline_;
        const ParamFunction* path_;
    };

    bool CheckPathRec(Collider& collider, planner::Object& object, const planner::InterpolatePath& path, const double step, double& start)
    {
        start += step; if(start >= 1) return true;
        const Configuration c=  path.Evaluate(start);
        object.SetOrientation(c.second);
        object.SetPosition(c.first);
        if(collider.IsColliding(&object))
        {
            return false;
        }
        return CheckPathRec(collider, object, path, step, start);
    }

    InterpolatePath* createInitPath(const State& from, const State& to, Collider& collider, Model& model, const int effectorid)
    {
        const planner::Node* limbFrom = planner::GetChild(from.value, effectorid);
        const planner::Node* limbTo = planner::GetChild(to.value, effectorid);

        const planner::Node* parent = limbTo;
        while(parent->position == limbTo->position)
        {
            parent = parent->parent;
        }
        std::vector<C2_Point> res; res.push_back(MakeConfiguration(limbFrom->current));
        Eigen::Vector3d offset = parent->position - limbTo->position;
        offset =  Eigen::Vector3d (0,1,0);
        offset.normalize();
        C2_Point cFrom = MakeConfiguration(limbFrom->current);
        C2_Point cTo = MakeConfiguration(limbTo->current);
        planner::InterpolatePath* initpath = new  planner::InterpolatePath(cFrom,cTo,0,1);
        double start(0);
        bool collisionFree= (cFrom.first - cTo.first).norm() < 0.001;
        int nbsteps = (int)((limbFrom->position - limbTo->position).norm() * 20.);
        int step = 0;
        double stepsize = 0.05;
        while(step < nbsteps && !collisionFree)
        {
            if(CheckPathRec(collider,*model.englobed,*initpath,stepsize,start))
            {
                collisionFree = true;
            }
            else // collision
            {
                bool ok(false);
                for (int i = 0; i<1 && !ok; ++i)
                {
                    model.SetPosition(model.GetPosition() + 0.3* offset);
                    ok = collider.IsColliding(model.englobed);
                }
                if(true)
                {
                    res.push_back(MakeConfiguration(model.englobed));
                    delete initpath;
                    initpath = new planner::InterpolatePath(MakeConfiguration(model.englobed),MakeConfiguration(limbTo->current),0,1);
                    start = 0;
                    stepsize = 0.05 * nbsteps / (nbsteps - step); // adjusting stepsize to reduced path
                    collisionFree = true;
                }
            }
        }
        delete initpath;
        res.push_back(MakeConfiguration(limbTo->current));
        return new planner::InterpolatePath(res, true);
    }

    struct DoIk
    {
        DoIk(const planner::CompleteScenario& scenario, const planner::State* state)
            : scenario(scenario)
            , solver(0.001, 0.01)
        {
            for(std::vector<int>::const_iterator cit = state->contactLimbs.begin();
                cit != state->contactLimbs.end(); ++cit)
            {
                planner::Node* limb =  planner::GetChild(state->value,scenario.limbs[*cit]->id);
                std::vector<ik::PartialDerivativeConstraint*> constraints;
                ik::MatchTargetConstraint* constraint = new ik::MatchTargetConstraint(limb);
                constraints.push_back(constraint);
                allconstraints.push_back(constraints);
            }
        }

        ~DoIk()
        {
            for(std::vector< std::vector<ik::PartialDerivativeConstraint*> >::iterator cit = allconstraints.begin();
                cit != allconstraints.end(); ++cit)
            {
                for(std::vector<ik::PartialDerivativeConstraint*>::iterator it2 = cit->begin();
                    it2 != cit->end(); ++it2)
                {
                    delete(*it2);
                }
            }
        }

        void operator ()(planner::State* state, int limit = 10)
        {
            ik::IKSolver solver;
            std::vector<Eigen::Vector3d>::iterator posit = state->contactLimbPositions.begin();
            std::vector<Eigen::Vector3d>::iterator normit = state->contactLimbPositionsNormals.begin();
            int limbId = 0;
            for(std::vector<int>::const_iterator cit = state->contactLimbs.begin();
                cit != state->contactLimbs.end(); ++cit, ++posit, ++normit, ++limbId)
            {
                planner::Node* limb =  planner::GetChild(state->value,scenario.limbs[*cit]->id);
                int limite = limit;
                while(limite > 0 && !solver.StepClamping(limb, *posit, *posit, allconstraints[limbId], true))
                {
                    limite--;
                }
            }
        }

        ik::IKSolver solver;
        std::vector< std::vector<ik::PartialDerivativeConstraint*> > allconstraints;
        const planner::CompleteScenario& scenario;
    };

    // index in "to" state vectors
    std::vector<int> GetModifiedContacts(const planner::State& from, const planner::State& to)
    {
        std::vector<int> res;
        int lIndex = 0;
        for(std::vector<int>::const_iterator cit = to.contactLimbs.begin();
            cit != to.contactLimbs.end(); ++cit, ++lIndex)
        {
            const Eigen::Vector3d& positionTo = from.contactLimbPositions[lIndex];

            int lIndex2 = 0;
            bool notfound(true);
            for(std::vector<int>::const_iterator cit2 = from.contactLimbs.begin();
                cit2 != from.contactLimbs.end() && notfound; ++cit2, ++lIndex2)
            {
                if(*cit2 == *cit)
                {
                    const Eigen::Vector3d& positionFrom = to.contactLimbPositions[lIndex2];
                    notfound = false;
                    if((positionFrom - positionTo).norm() > 0.00001)
                    {
                        res.push_back(lIndex);
                    }
                    break;
                }
            }
            if(notfound) // new contact created
            {
                res.push_back(lIndex);
            }
        }
        return res;
    }

    std::vector<InterpolateLine> ContactInterpolation(const std::vector<int>& contacts, const planner::CompleteScenario& scenario, const planner::State& from, const planner::State& to)
    {
        std::vector<InterpolateLine> res;
        for(std::vector<int>::const_iterator cit = contacts.begin();
            cit != contacts.end(); ++cit)
        {
            int limbindex = to.contactLimbs[*cit];
            // Find position in initial Configuration
            res.push_back(InterpolateLine(
                planner::GetEffectorCenter(planner::GetChild(from.value, scenario.limbs[limbindex]->id)),
                to.contactLimbPositions[*cit]));
        }
        return res;
    }

    std::vector<InterpolateSpline*> ContactSplineInterpolation(const std::vector<int>& contacts, const planner::CompleteScenario& scenario, const planner::State& from, const planner::State& to)
    {
        std::vector<InterpolateSpline*> res;
        for(std::vector<int>::const_iterator cit = contacts.begin();
            cit != contacts.end(); ++cit)
        {
            int limbindex = to.contactLimbs[*cit];
            // Find position in initial Configuration
            //create Model
            int effId(-1);
            planner::Node * limb = planner::GetChild(from.value, scenario.limbs[limbindex]->id);
            planner::Object* obj = GetEffector(limb, effId);
            Collider collider(scenario.scenario->objects_);
            Model model; model.englobed = new planner::Object(*obj);
            res.push_back(new InterpolateSpline( createInitPath(from, to, collider, model, effId), collider, model));
        }
        return res;
    }

    double GetMinTime(const planner::CompleteScenario& scenario, const std::vector<InterpolateSpline*>& lines)
    {
        double min = 0;
        for(std::vector<InterpolateSpline*>::const_iterator cit = lines.begin()
            ; cit != lines.end(); ++cit)
        {
            double time = std::min ((*cit)->distance() / (scenario.limbspeed.front() + 0.000000000001), 4.); // TODO
            if(time > min)
            {
                min = time;
            }
        }
        return min;
    }

    struct InterpolateContacts
    {
        InterpolateContacts(const planner::CompleteScenario& scenario, const planner::State& from, const planner::State& to)
            :involvedContacts_(GetModifiedContacts(from, to))
            ,contactInterpolation_(ContactSplineInterpolation(involvedContacts_, scenario, from, to))
            , minTime_(GetMinTime(scenario, contactInterpolation_))
        {
            // NOTHING
        }
        ~InterpolateContacts()
        {
            for(std::vector<InterpolateSpline*>::const_iterator cit = contactInterpolation_.begin()
                ; cit != contactInterpolation_.end(); ++cit)
            {
                delete *cit;
            }
        }
        void operator ()(planner::State& current, double time) const
        {
            std::vector<InterpolateSpline*>::const_iterator intit = contactInterpolation_.begin();
            for(std::vector<int>::const_iterator cit = involvedContacts_.begin();
                cit!= involvedContacts_.end(); ++cit, ++intit)
            {
                current.contactLimbPositions[*cit] = (**intit)(time);
            }
        }
        const std::vector<int> involvedContacts_;
        const std::vector<InterpolateSpline*> contactInterpolation_;
        const double minTime_;
    };

    planner::T_State AnimateInternal(const planner::CompleteScenario& scenario, const planner::State& from, const planner::State& to, planner::T_State& res, int framerate)
    {
        InterpolateContacts interpolate(scenario,from,to);
        planner::InterpolatePath path(MakeConfiguration(from),MakeConfiguration(to),0,1);
        DoIk doIk(scenario,&to);
        res.push_back(new State(&from));
        State* current = new State(&from);
        current->contactLimbPositions = to.contactLimbPositions;
        current->contactLimbPositionsNormals = to.contactLimbPositionsNormals;
        current->contactLimbs = to.contactLimbs;
        double nbFrames = std::max((double)(framerate)*interpolate.minTime_,2.);
        double stepsize = double(1) / double(nbFrames-1); double step = stepsize;
        for(int i = 1; i< nbFrames-1; ++i)
        {
            current = new State(current);
            planner::Configuration conf = path.Evaluate(step);
            current->value->SetFullRotation(conf.second, false);
            current->value->SetPosition(conf.first, true);
            interpolate(*current, step);
            doIk(current);
            res.push_back(current);
            step += stepsize;
        }
        res.push_back(new State(&to));
        return res;
    }
}


#include "prmpath/smoothing/InterpolateRRT.h"

planner::T_State insertRRTForOneLimb(const planner::CompleteScenario& scenario,
                           planner::InterpolateRRT& rrt, planner::Node* limb, const int limbIndex, const planner::State& from, const planner::State& to)
{
    planner::T_State res;
    planner::Node* limbfrom =  planner::GetChild(from.value,limb->id);
    planner::Node* limbTo =  planner::GetChild(to.value,limb->id);
    planner::sampling::Sample sFrom(limbfrom);
    planner::sampling::Sample sTo(limbTo);
    std::vector<planner::LimbNode*> nodes = planner::computeKeyFrames(rrt,from.value, to.value,sFrom,sTo);
    if(nodes.empty())
    {
        std::cout << "rrt failed " <<  limb->tag <<  std::endl;
        //res.push_back(new planner::State(&from));
        res.push_back(new planner::State(&to));
        return res;
    }
    DoIk doIk(scenario,&to);
    for(std::vector<planner::LimbNode*>::const_iterator cit = nodes.begin()+1;cit != nodes.end(); ++cit)
    {
        planner::LimbNode* ln = *cit;
        planner::State* ns (0);
        if(cit == nodes.begin())
        {
            ns = new planner::State(&from);
        }
        else if(cit +1 == nodes.end())
        {
            ns = new planner::State(&to);
        }
        else
        {
            ns = new planner::State(&from, limbIndex);
            ns->value->SetRotation(ln->configuration_.second, false);
            ns->value->SetPosition(ln->configuration_.first, true);
            planner::Node* limb = planner::GetChild(ns->value, ln->limb_->id);
            planner::sampling::LoadSample(*ln->sample_,limb);
            doIk(ns, 10);
        }
        res.push_back(ns);
    }
    return res;
}


planner::T_State insertRRT(const planner::CompleteScenario& scenario, const planner::T_State& fullpath);

void insertRRTBetweenStates(const planner::CompleteScenario& scenario,
                           std::vector<planner::InterpolateRRT*>& rrts,
                           const planner::State& from, const planner::State& to,
                           planner::T_State& res, int limbindex)
{
    std::vector<int> modifiedContacts = GetModifiedContacts(from,to);
    planner::T_State resPerLimb;
    bool limbChanged (false);
    for(std::vector<int>::const_iterator cit = modifiedContacts.begin();
        cit != modifiedContacts.end(); ++cit)
    {
        if (limbindex == to.contactLimbs[*cit])
        {
            planner::Node * limb = planner::GetChild(from.value, scenario.limbs[limbindex]->id);
            resPerLimb = insertRRTForOneLimb(scenario,*rrts[limbindex],limb,limbindex,from,to);
            limbChanged = true;
            for(planner::T_State::const_iterator cit = resPerLimb.begin(); cit != resPerLimb.end();
                ++cit)
            {
                res.push_back(*cit);
            }
            break;
        }
    }
    if(!limbChanged)
    {
        res.push_back(new planner::State(&to));
    }
}

planner::T_State insertRRT(const planner::CompleteScenario& scenario, const planner::T_State& fullpath)
{
    planner::T_State limbres = fullpath;
    planner::T_State res;
    // create RRTInterpolate for each limb,
    Collider collider(scenario.scenario->objects_);
    std::vector<planner::InterpolateRRT*> rrts;
    for(int i = 0; i< scenario.limbs.size(); ++i)
    {
        rrts.push_back(new InterpolateRRT(
                           scenario.robot,scenario.limbs[i],scenario.limbSamples[i],collider));
    }
    for(int lid = 0; lid < scenario.limbs.size(); ++lid)
    //for(int lid = 2; lid < 3; ++lid)
    {
        res.push_back(new planner::State (fullpath.front()));
        planner::T_State::const_iterator cit1 = limbres.begin();
        planner::T_State::const_iterator cit2 = limbres.begin(); ++cit2;
        //int i = 0;
        do
        {
            insertRRTBetweenStates(scenario,rrts,**cit1,**cit2,res, lid);
            ++cit1; ++cit2;
             //++i;
        } while(cit2 != limbres.end());
        limbres = res;
        res.clear();
    }
    // todo delete states
    for(std::vector<planner::InterpolateRRT*>::const_iterator cit = rrts.begin();
        cit != rrts.end(); ++cit)
    {
        delete * cit;
    }
    return limbres;
}

planner::T_State planner::Animate(const planner::CompleteScenario& scenario, const planner::State& from, const planner::State& to, int framerate)
{    
    planner::T_State fullpath;
    fullpath.push_back(new State(&from));fullpath.push_back(new State(&to));
    planner::T_State rrtres = insertRRT(scenario, fullpath);
    /*planner::T_State res;
    AnimateInternal(scenario,from,to,res,framerate);*/
    return rrtres;
}

planner::T_State planner::Animate(const planner::CompleteScenario& scenario, const planner::T_State& fullpath, int framerate)
{
    planner::T_State rrtres = insertRRT(scenario, fullpath);
    planner::T_State res;
    planner::T_State::const_iterator cit1 = rrtres.begin(); ++cit1;
    planner::T_State::const_iterator cit2 = rrtres.begin(); ++cit2;++cit2;
    int i = 0;
    do
    {
        AnimateInternal(scenario,**cit1,**cit2,res,framerate);
        ++cit1; ++cit2;
         ++i;
    } while(cit2 != rrtres.end());
    return res;
    return rrtres;
}
