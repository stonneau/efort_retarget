
#include "InterpolateRRT.h"
#include "planner/RRT.h"
#include "smooth.h"

namespace planner
{
class LimbNode
{
public:
    LimbNode(planner::Robot* robot, planner::Node* limb, std::vector<planner::Object*>& objects, const std::vector<double>& weight, const double t, const Configuration& configuration)
        : robot_(robot), limb_(limb), sample_(0),configuration_(configuration), objects_(objects),weights(weight), t_(t)
    {}
    ~LimbNode(){}
    planner::Robot* robot_;
    planner::Node* limb_;
    sampling::Sample* sample_;
    const Configuration& configuration_;
    std::vector<planner::Object*>& objects_;
    const std::vector<double>& weights;
    const double t_;
};

double Distance(const LimbNode* obj1, const LimbNode* obj2)
{
    // quaternion distance http://math.stackexchange.com/questions/90081/quaternion-distance
    double res = 0;
    std::vector<double>::const_iterator wit = obj1->weights.begin();
    std::vector<double>::const_iterator cit2 = obj1->sample_->values.begin();
    for(std::vector<double>::const_iterator cit = obj1->sample_->values.begin();
        cit != obj1->sample_->values.end(); ++cit, ++cit2,++wit)
    {
        res += ((*cit2) - (*cit)) * (*wit);
    }
    //weight based on number of objects;
    // TODO include angles
    res += (obj1->sample_->effectorPosition - obj2->sample_->effectorPosition).norm() * (*wit);
    return res;
}

class LimbGenerator
{
public:
     LimbGenerator(planner::Robot* robot, planner::Node* limb, std::vector<planner::Object*>& objects, const sampling::T_Samples& samples, Collider& collider, const ParamFunction* interpolate)
         : samples_(samples)
         , collider_(collider)
         , robot_(robot)
         , limb_(limb)
         , objects_(objects)
         , interpolate_(interpolate)
     {
        // init weights
         float w = 9;
         float totalWeight = 0;
         planner::Node* node = limb_;
         int nbIt = samples.front()->values.size();
         while(nbIt>0)
         {
             if(node->id != limb_->id && node->offset.norm() != 0)
             {
                 w *= 2 / 3;
             }
             weights.push_back(w);
             totalWeight += w;
             node = node->children[0];
             --nbIt;
         }
         // normalize
         totalWeight*=2;
         for(int i = 0; i< weights.size(); ++i)
         {
             weights[i] /= totalWeight;
         }
         weights.push_back(0.5); // position of effector counts half
     }

    ~LimbGenerator(){}
     const sampling::T_Samples& samples_;
     Collider& collider_;
     planner::Robot* robot_;
     planner::Node* limb_;
     std::vector<planner::Object*> objects_;
     std::vector<double> weights;
     const ParamFunction* interpolate_;


     LimbNode* operator()()
     {
         double min = 0;
         double max = interpolate_->tmax();
         double t = ((double) rand() / (RAND_MAX)) * (max - min) + min;
         Configuration c = interpolate_->Evaluate(t);
         robot_->SetPosition(c.first, false);
         robot_->SetRotation(c.second, true);
         LimbNode* res = new LimbNode(robot_,limb_,objects_,weights,t,c);
         int limit = samples_.size();
         while(limit > 0)
         {
            int nb = (rand() % (int)(samples_.size()));
            res->sample_ = samples_[nb];
            planner::sampling::LoadSample(*res->sample_,limb_);
            if(!collider_.IsColliding(objects_))
            {
                return res;
            }
            limit--;
         }
         delete res;
         return 0;
     }
};

bool StraightLine(Collider& collider, Object* obj, const Configuration& a, const Configuration& b, double inc=0.05)
{
    const Eigen::Vector3d& va = a.first; const Eigen::Vector3d& vb = b.first;
    Eigen::Quaterniond qa(a.second);
    Eigen::Quaterniond qb(b.second);
    Eigen::Vector3d offset;
    Eigen::Quaterniond qOff;
    Eigen::Matrix3d qrot;
    for(double t = 0; t < 1; t = t + inc)
    {
        offset = va + t * (vb - va);
        qOff = qa.slerp(t, qb);
        //offrot = ea + t * (eb - ea);
        qrot = qOff.matrix();
        obj->SetPosition(offset);
        obj->SetOrientation(qrot);
        if(collider.IsColliding(obj)) return false;
    }
    return true;
}


class LimbPlanner
{
public:
    LimbPlanner(Object::T_Object& objects, Collider& collider, double distanceStep, const ParamFunction* interpolate)
        : collider_(collider), objects_(objects), dStep_(distanceStep), interpolate_(interpolate)
    {
        // NOTHING
    }

   ~LimbPlanner();

    bool operator ()(const LimbNode *ma, const LimbNode *mb)
    {
        if(ma->t_ > mb->t_) return false;
        double distance = Distance(ma,mb);
        if(distance / (mb->t_ - ma->t_) > 1) return false; // too fast
        double inc = distance / dStep_;
        std::vector<planner::Configuration> cas;
        std::vector<planner::Configuration> cbs;
        ma->robot_->SetPosition(ma->configuration_.first,false);
        ma->robot_->SetFullRotation(ma->configuration_.second,true);
        planner::sampling::LoadSample(*ma->sample_,ma->limb_);
        for(std::vector<planner::Object*>::const_iterator cit = objects_.begin();
        cit != objects_.end(); ++cit)
        {
            cas.push_back(std::make_pair((*cit)->GetPosition(), (*cit)->GetOrientation()));
        }

        mb->robot_->SetPosition(mb->configuration_.first,false);
        mb->robot_->SetFullRotation(mb->configuration_.second,true);
        planner::sampling::LoadSample(*mb->sample_,mb->limb_);
        for(std::vector<planner::Object*>::const_iterator cit = objects_.begin();
        cit != objects_.end(); ++cit)
        {
            cbs.push_back(std::make_pair((*cit)->GetPosition(), (*cit)->GetOrientation()));
        }

        std::vector<planner::Configuration>::const_iterator casit = cas.begin();
        std::vector<planner::Configuration>::const_iterator cbsit = cbs.begin();
        for(std::vector<planner::Object*>::iterator cit = objects_.begin();
        cit != objects_.end(); ++cit, ++casit, ++cbsit)
        {
            if(!StraightLine(collider_,*cit,*casit,*cbsit,inc)) return false;
        }
        return true;
    }

    Collider& collider_;
    planner::Robot* robot_;
    planner::Node* limb_;
    std::vector<planner::Object*> objects_;
    const double dStep_;
    const ParamFunction* interpolate_;

};

typedef RRT<LimbNode, LimbGenerator, LimbPlanner, double, true, 1000> rrt_t;

#include "tools/ExpMap.h"
sampling::T_Samples computeKeyFrames(InterpolateRRT& rrt, const planner::Robot* robotFrom, const planner::Robot* robotTo, const sampling::Sample& froms, const sampling::Sample& tos)
{
    matrices::ExpMap emap(robotFrom->currentRotation);
    C2_Point a = std::make_pair(robotFrom->currentPosition, emap.log());

    matrices::ExpMap emap2(robotTo->currentRotation);
    C2_Point b = std::make_pair(robotTo->currentPosition, emap2.log());
    ParamFunction* path = new InterpolatePath(a,b,0,1);

    LimbGenerator generator(rrt.robot_,rrt.limb_,rrt.limbObjects,rrt.samples_,rrt.collider_,path);
    LimbPlanner localPlanner(rrt.limbObjects,rrt.collider_,0.01,path);
    LimbNode* from = new LimbNode(rrt.robot_,rrt.limb_,rrt.limbObjects,generator.weights,0, std::make_pair(robotFrom->currentPosition, robotFrom->constantRotation));
    from->sample_ = new planner::sampling::Sample(froms);
    LimbNode* to = new LimbNode(rrt.robot_,rrt.limb_,rrt.limbObjects,generator.weights,0, std::make_pair(robotTo->currentPosition, robotTo->constantRotation));
    to->sample_ = new planner::sampling::Sample(tos);
    rrt_t plan(&generator, &localPlanner, from, to, Distance, 10, 1000,10,true);
    sampling::T_Samples res;
    for(rrt_t::T_NodeContentPath::const_iterator cit = plan.path_.begin();
        cit != plan.path_.end(); ++cit)
    {
        res.push_back((*cit)->sample_);
    }
    delete path;
    return res;
}
}
