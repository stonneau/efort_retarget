/**
* \file Model.h
* \brief Helper struct for englobed object and englobing manipulation
* \author Steve T.
* \version 0.1
* \date 08/25/2014
*
*/
#ifndef _STRUCT_MODEL
#define _STRUCT_MODEL

#include "collision/Object.h"
#include "collision/Collider.h"

#include <iostream>

namespace planner
{

struct Model;
typedef std::vector<Model*> T_Model;
typedef T_Model::iterator IT_Model;
typedef std::vector<const Model*> CT_Model;

struct Model
{
    Model(const Model& model)
	{
        englobed = new Object(*model.englobed);
        for(Object::T_Object::const_iterator it = model.englobing.begin();
            it != model.englobing.end(); ++it)
        {
            englobing.push_back(new Object(*(*it)));
        }
	}
    Model(){}
    ~Model()
    {
		if(englobed) delete englobed;
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it)
        {
            delete(*it);
        }
    }

    void SetOrientation(const Eigen::Matrix3d& orientation)
    {
        englobed->SetOrientation(orientation);
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it)
        {
            (*it)->SetOrientation(orientation);
        }
    }

    void SetPosition(const Eigen::Vector3d& position)
    {
        englobed->SetPosition(position);
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it)
        {
            (*it)->SetPosition(position);
        }
    }

    std::vector<size_t> EnglobingCollision(Object* obj) // updates collision indexes
    {
        std::vector<size_t> res;
        collisionIndexes.clear();
        size_t id = 0;
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it, ++id)
        {
            if((*it)->IsColliding(obj))
            {
                res.push_back(id);
            }
        }
        return res;
    }

    std::vector<size_t> EnglobingCollision(Object::T_Object& objs) // updates collision indexes
    {
        std::vector<size_t> res;
        collisionIndexes.clear();
        size_t id = 0;
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it, ++id)
        {
            if((*it)->IsColliding(objs))
            {
                res.push_back(id);
            }
        }
        return res;
    }

    std::vector<size_t> EnglobingCollisionGround(Object* obj, double tolerance = 0.7) // updates collision indexes
    {
        Eigen::Vector3d normal(0,0.8,0);
        std::vector<size_t> res;
        collisionIndexes.clear();
        size_t id = 0;
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it, ++id)
        {
            if((*it)->name_.find("leg") != std::string::npos && (*it)->IsColliding(obj,normal, tolerance))
            {
                res.push_back(id);
            }
        }
        return res;
    }

    std::vector<size_t> EnglobingCollisionClimb(Object* obj, double tolerance = 0.7) // updates collision indexes
    {
        Eigen::Vector3d normal(0,0.8,0);
        std::vector<size_t> res;
        collisionIndexes.clear();
        size_t id = 0;
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it, ++id)
        {
            if((*it)->IsColliding(obj,normal, tolerance))
            {
                res.push_back(id);
            }
        }
        return res;
    }

    std::vector<size_t> EnglobingCollisionClimb(Object::T_Object& objs, double tolerance = 0.7) // updates collision indexes
    {
        Eigen::Vector3d normal(0,0.8,0);
        std::vector<size_t> res;
        collisionIndexes.clear();
        size_t id = 0;
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it, ++id)
        {
            if((*it)->IsColliding(objs,normal, tolerance))
            {
                res.push_back(id);
            }
        }
        return res;
    }

    bool ReachabilityCondition(Collider& collider) // updates collision indexes
    {
        collisionIndexes.clear();
        size_t id = 0;
        bool englobingCollision = false;
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end() && !englobingCollision; ++it, ++id)
        {
            if(collider.IsColliding(*it))
            {
                englobingCollision = true;
            }
        }
        return englobingCollision && !collider.IsColliding(englobed);
    }

    bool ReachabilityConditionGround(Collider& collider, double tolerance) // constrains collision with vertival
    {
        collisionIndexes.clear();
        size_t id = 0;
        bool englobingCollision = false;
        bool oneleg = false;
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end() && !englobingCollision; ++it, ++id)
        {
            if((*it)->name_.find("leg") != std::string::npos && collider.IsCollidingGround(*it, tolerance))
            {
                englobingCollision = true;
            }
        }
        return englobingCollision && !collider.IsColliding(englobed);
    }

    const Eigen::Matrix3d& GetOrientation() const
    {
        return englobed->GetOrientation();
    }
    const Eigen::Vector3d& GetPosition() const
    {
        return englobed->GetPosition();
    }

    void SortEnglobingByName(const std::vector<std::string>& names)
    {
        Object::T_Object res;
        for(std::vector<std::string>::const_iterator cit = names.begin(); cit != names.end(); ++cit)
        {
            if((*cit) == "") break;
            for(Object::T_Object::const_iterator oit = englobing.begin();
                oit != englobing.end(); ++oit)
            {
                std::size_t found = (*oit)->name_.find(*cit);
                if (found!=std::string::npos)
                {
                    Object* obj = *oit;
                    res.push_back(obj);
                }
            }
        }
        if(res.size() == englobing.size())
        {
            englobing = res;
        }
        else
        {
            std::cout << "names of limbs and ROM NOT matching. Can not sort correctly";
        }
    }

    Object* englobed;
    Object::T_Object englobing;
// TMP SERIOUSLY THIS NEEDS TO GET OUT OF HERE FOR PRM TYPE
    std::vector<size_t> collisionIndexes;
};

} //namespace planner
#endif //_STRUCT_MODEL
