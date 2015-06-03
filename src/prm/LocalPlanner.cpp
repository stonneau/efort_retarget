/**
* \file WorldABC.cpp
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/

#include <iostream>
#include "LocalPlanner.h"
#include <Eigen/Dense>


namespace planner
{
    bool StraightLine(const Object* a, const Object* b, LocalPlanner& planner, std::vector<Eigen::Matrix4d>& path, double tInc=0.05)
    {
		std::vector<Eigen::Matrix4d> res;
        // v0 : draw line between a and b and check every 1 / 5 of the path if there is a collision
		Model tmp(planner.model_);
		tmp.SetOrientation(a->GetOrientation());
		tmp.SetPosition(a->GetPosition());
        // compute line
        Eigen::Vector3d line = b->GetPosition() - a->GetPosition();
        // norm
        double step = 0.1;
        if(line.norm() == 0) return false;
        Eigen::Vector3d norm = line;
        norm.normalize(); line = line * step;
        Eigen::Vector3d offset;
        Eigen::Vector3d offrot;
        Eigen::Matrix3d offrotmat;
        // TODO rotate
        //  Vr = Va + t .(Vb - Va )

        // euler angle decomposition

        Eigen::Vector3d ea = a->GetOrientation().eulerAngles(2, 0, 2);
        Eigen::Vector3d eb = b->GetOrientation().eulerAngles(2, 0, 2);
        Eigen::Vector3d va = a->GetPosition();
        Eigen::Vector3d vb = b->GetPosition();

        /*Vector3f ea = mat.eulerAngles(2, 0, 2);

        "2" represents the z axis and "0" the x axis, etc. The returned angles are such that we have the following equality:
        * mat == AngleAxisf(ea[0], Vector3f::UnitZ())
        * * AngleAxisf(ea[1], Vector3f::UnitX())
        * * AngleAxisf(ea[2], Vector3f::UnitZ());*/

        /*Perform linear interpolation*/
		float linenorm = (float)line.norm();
		float nbSteps = (float)(linenorm / tInc);
		float inc = 1 / nbSteps;
		for(double t = 0; t <= 1; t = t + inc)
        {
            offset = va + t * (vb - va);
            offrot = ea + t * (eb - ea);
            offrotmat = Eigen::AngleAxisd(offrot[0],  Eigen::Vector3d::UnitZ())
                     *  Eigen::AngleAxisd(offrot[1],  Eigen::Vector3d::UnitX())
                     *  Eigen::AngleAxisd(offrot[2],  Eigen::Vector3d::UnitZ());
            tmp.SetPosition(offset);
            tmp.SetOrientation(offrotmat);
            if(planner.IsColliding(tmp.englobed) || planner.IsInContact(tmp.englobing).empty())
            {
                return false;
            }
			else
			{
				Eigen::Matrix4d config = Eigen::Matrix4d::Identity();
				config.block<3,3>(0,0) = offrotmat;
				config.block<3,1>(0,3) = offset;
				res.push_back(config);
			}
        }
		for(std::vector<Eigen::Matrix4d>::const_iterator cit = res.begin(); cit!=res.end(); ++cit)
		{
			path.push_back(*cit);
		}
        return true;
    }

    bool RotateAt(const Object* a, const Object* b, float when, LocalPlanner& planner, std::vector<Eigen::Matrix4d>& path, int steps=10)
    {
        // v0 : draw line between a and b and check every 1 / 5 of the path if there is a collision
        Object tmp(*a);
        // compute line
		double tInc1, tInc2;
		tInc1 = 1. / (steps* when);
		tInc2 = 1. / (steps* (1-when));
        Eigen::Vector3d position = a->GetPosition() +  when *(b->GetPosition() - a->GetPosition());
        tmp.SetPosition(position);
        if(StraightLine(a, &tmp, planner, path))
        {
            tmp.SetOrientation(b->GetOrientation());
			return StraightLine(&tmp, b, planner, path);
        }
        return false;
    }

    bool AstarLike(const Object* a, const Object* b, LocalPlanner& planner, std::vector<Eigen::Matrix4d>& path, int nbrs, int steps)
    {
        if(steps == 0) return false;
        // step distance given by distance to cover.
        double stepdistance = (b->GetPosition() - a->GetPosition()).norm() / steps;
        double bestDistance = std::numeric_limits<double>::max();
        Object best(*a);
        // random directionss
        for(int i=0; i < nbrs; ++i)
        {
            Eigen::Vector3d dir;
            for(int k=0; k<3; ++k)
            {
                dir(k) = (double) rand() / (RAND_MAX);
            }
            dir.normalize();
            dir = dir*stepdistance;
            Object tmp(*a);
            tmp.SetPosition(a->GetPosition() + dir);
            if(StraightLine(a,&tmp, planner, path))
            {
                if(StraightLine(&tmp, b, planner, path))
                {
                    return true;
                }
                else
                {
                    double currentDistance = (b->GetPosition() - tmp.GetPosition()).norm();
                    if(currentDistance < bestDistance)
                    {
                        bestDistance = currentDistance;
                        best.SetOrientation(tmp.GetOrientation());
                        best.SetPosition(tmp.GetPosition());
                    }
                }
            }
        }
        if(bestDistance < std::numeric_limits<double>::max())
        {
            return AstarLike(&best, b, planner, path, nbrs, steps-1);
        }
        else
        {
            return false;
        }
    }
}

using namespace planner;

LocalPlanner::LocalPlanner(Object::T_Object& objects, Object::T_Object &collisionObjects, const Model & model)
    : Collider(collisionObjects)
	, model_(model)
    , contactObjects_(objects)
{
	// NOTHING
}


LocalPlanner::~LocalPlanner()
{
	// NOTHING
}

bool LocalPlanner::operator ()(const Model *ma, const Model *mb, int stage)
{

    const Object * a = ma->englobed;
    const Object * b = mb->englobed;
	std::vector<Eigen::Matrix4d> path;
    bool found = StraightLine(a, b, *this, path);
    if(!found && stage >0)
    {
        found = RotateAt(a, b, 0.5, *this, path) || RotateAt(a, b, 0, *this, path) || RotateAt(a, b, 1, *this, path);
        if(!found && stage >1)
		{
			found = AstarLike(a, b, *this,path, 15, 9);
		}
    }
    return found;
}


std::vector<size_t> LocalPlanner::IsInContact(Object::T_Object& objects)
{
    size_t id = 0;
    std::vector<size_t> res;
    for(Object::T_Object::iterator it = objects.begin();
        it != objects.end(); ++it, ++id)
    {
        if((*it)->IsColliding(contactObjects_))
            res.push_back(id);
    }
    return res;
}

std::vector<Eigen::Matrix4d> LocalPlanner::Interpolate(const Model *ma, const Model *mb, int nbSteps)
{
    // tmp before actual planning
    const Object * a = ma->englobed;
    const Object * b = mb->englobed;
	std::vector<Eigen::Matrix4d> path;
    bool found = StraightLine(a, b, *this, path, 1./nbSteps);
    if(!found)
    {
        found = RotateAt(a, b, 0.5, *this, path, nbSteps) || RotateAt(a, b, 0, *this, path, nbSteps) || RotateAt(a, b, 1, *this, path, nbSteps)
        || AstarLike(a, b, *this,path, 15, nbSteps);
    }
    return path;
}
