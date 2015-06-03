/**
* \file Collider.h
* \brief Helper class for collision detection
* \author Steve T.
* \version 0.1
* \date 16/07/2014
*
*/
#ifndef _CLASS_COLLIDER
#define _CLASS_COLLIDER

#include "Object.h"

#include <vector>

namespace planner
{
class Collider
{
public:
    ///  \brief Constructor
    ///  \param objects List of objects checked by the colliders
     Collider(const Object::T_Object& objects);
    ~Collider();

     /// brief collision testing between an Object and the Objects of the Collider
     bool IsColliding(Object* object);
     bool IsCollidingGround(Object *model, double tolerance);

     /// brief collision testing between all the Objects of the Collider
     bool IsColliding();


private:
    Object::T_Object objects_;

public:
    friend double DistanceToClosestObject(Object* object, Collider* collider);
};
double DistanceToClosestObject(Object* object, Collider* collider);
}//namespace planner;
#endif //_CLASS_COLLIDER
