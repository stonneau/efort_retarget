/**
* \file SimplePRM.cpp
* \brief A PRM concrete implementation
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/

#include "SimpleRRT.h"
#include "collision/ParserObj.h"
#include "planner/RRT.h"

#include "LocalPlanner.h"
#include "Generator.h"

#include <fstream>
#include <iostream>
#include <string>

#if PROFILE
#include "Timer.h"
#endif

namespace planner
{
    float DistanceRRT(const Model* obj1, const Model* obj2)
    {
        // TODO include angles
        const Eigen::Vector3d& a = obj1->GetPosition();
        const Eigen::Vector3d& b = obj2->GetPosition();
        float p = (float)(sqrt((b.x() - a.x()) * (b.x() - a.x()) + (b.y() - a.y()) * (b.y() - a.y()) + (b.z() - a.z()) * (b.z() - a.z())));

		const Eigen::Vector3d& ea = obj1->GetOrientation().eulerAngles(0, 1, 2);
        const Eigen::Vector3d& eb = obj2->GetOrientation().eulerAngles(0, 1, 2);
        float q = (float)(sqrt((eb.x() - ea.x()) * (eb.x() - ea.x()) * 2 + (eb.y() - ea.y()) * (eb.y() - ea.y()) + (eb.z() - ea.z()) * (eb.z() - ea.z())));
        return 0.6f * p + 0.4f * q;
        //return p + q;
    }
    typedef RRT<Model, planner::Generator, LocalPlanner, float, false, 10000> rrt_t;

	struct PImpl
	{
        PImpl(Model* from, Model* to, Object::T_Object& objects, Object::T_Object& collisionObjects, float neighbourDistance, int size, int k)
            : planner_(objects, collisionObjects, *from)
        {
            Eigen::Matrix3d ori = from->GetOrientation();
            std::cout << "from orientation" << ori.eulerAngles(0, 1, 2) << std::endl;
            std::cout << "from position" << from->GetPosition() << std::endl;
            std::cout << "to position" << to->GetPosition() << std::endl;
             std::vector<double> bounds; // min max rotations
            bounds.push_back(-1.72524);bounds.push_back(-1.);
            bounds.push_back(-0.1);bounds.push_back(0.);
            bounds.push_back(-1);bounds.push_back(1);
            const Eigen::Vector3d& pf = from->GetPosition();
            const Eigen::Vector3d& pt = to->GetPosition();
            bounds.push_back(std::min(pt.x(),pf.x()) -20);bounds.push_back(std::max(pt.x(),pf.x()) +20);
            bounds.push_back(std::min(pt.y(),pf.y()) -10);bounds.push_back(std::max(pt.y(),pf.y())+10 );
            bounds.push_back(std::min(pt.z(),pf.z()) );bounds.push_back(std::max(pt.z(),pf.z()));
            Generator* gen = new Generator(bounds, objects, collisionObjects, *from); // TODO MEME
            //Generator* gen = new Generator(objects, collisionObjects, *from); // TODO MEME
			
#if PROFILE
Timer timer;
timer.Start();
#endif
            rrt_ = new rrt_t(gen, &planner_, from, to, DistanceRRT, neighbourDistance, size, k, true);
#if PROFILE
timer.Stop();
std::cout << " rrt Generation log " << std::endl;
std::cout << " Number of nodes :" << prm_->nodeContents_.size() << std::endl;
std::cout << " Graph generation time :" << timer.GetTime() << std::endl;
#endif
            // feel prmNodes
        }

        PImpl(Model* from, Model* to, Object::T_Object& objects, Object::T_Object& collisionObjects, int size)
            : planner_(objects, collisionObjects, *from)
        {
           rrt_ = new rrt_t(size);
        }

		~PImpl()
		{
            delete rrt_;
		}

        Object* operator()()
		{
            return 0;
		}

        rrt_t* rrt_;
        LocalPlanner planner_;
        T_Model prmNodes_;

	};
}

using namespace planner;

SimpleRRT::SimpleRRT(Model* from, Model* to, Object::T_Object &objects, float neighbourDistance, int size, int k)
    : objects_(objects)
    , CollisionObjects_(objects)
{
    pImpl_.reset(new PImpl(from, to, objects_, CollisionObjects_, neighbourDistance, size, k));
}

SimpleRRT::SimpleRRT(Model *from, Model* to, Object::T_Object &objects, Object::T_Object &collisionObjects, float neighbourDistance, int size,
                     int k)
    : objects_(objects)
    , CollisionObjects_(collisionObjects)
{
    pImpl_.reset(new PImpl(from, to, objects_, CollisionObjects_, neighbourDistance, size, k));
}


SimpleRRT::~SimpleRRT()
{
	// NOTHING
}

CT_Model SimpleRRT::GetPath()
{
    std::cout <<"found path ? " << !pImpl_->rrt_->path_.empty() << std::endl;
    return pImpl_->rrt_->path_;
}

std::vector<Eigen::Matrix4d> SimpleRRT::Interpolate(const CT_Model &path, int steps)
{
    std::vector<Eigen::Matrix4d> res, tmp;
	int each = steps/(int)path.size();
    CT_Model::const_iterator it2= path.begin(); ++it2;
    for(CT_Model::const_iterator it= path.begin(); it2!= path.end(); ++it, ++it2)
	{
		tmp = pImpl_->planner_.Interpolate(*it, *it2, each);
		
		for(std::vector<Eigen::Matrix4d>::const_iterator cit = tmp.begin(); cit!=tmp.end(); ++cit)
		{
			res.push_back(*cit);
		}
	}
	return res;
}
