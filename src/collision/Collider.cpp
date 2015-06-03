#include "Collider.h"
#include "Object.h"

using namespace planner;

Collider::Collider(const Object::T_Object& objects)
    : objects_(objects)
{
    // NOTHING
}

Collider::~Collider()
{
    // NOTHING
}

bool Collider::IsColliding(Object *model)
{
    return model->IsColliding(objects_);
}

bool Collider::IsCollidingGround(Object *model, double tolerance)
{
    Eigen::Vector3d normal(0,1,0);
    return model->IsColliding(objects_, normal, tolerance);
}

bool Collider::IsColliding()
{
    for(Object::T_Object::iterator it = objects_.begin();
        it != objects_.end();
        ++it)
    {
        Object::T_Object::iterator it2 = it; ++it2;
        for(; it2 != objects_.end(); ++it2)
        {
            if((*it)->IsColliding(it2, objects_.end()))
                return true;
        }
    }
    return false;
}

double planner::DistanceToClosestObject(Object* object, Collider* collider)
{
    double minDistance = std::numeric_limits<double>::max();
    double currentDistance;
    for(Object::T_Object::iterator it = collider->objects_.begin();
        it != collider->objects_.end();
        ++it)
    {
        currentDistance = object->Distance(*it);
        if(currentDistance < minDistance)
        {
            minDistance = currentDistance;
        }
    }
    return minDistance;
}
