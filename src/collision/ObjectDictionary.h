/**
* \file ObjectDictionary.h
* \brief Maintain information about objects so as
to recreate them dynamically upon vertices modification
* \author Steve T.
* \version 0.1
* \date 01/06/2015
*
*/
#ifndef _CLASS_ObjectDictionary
#define _CLASS_ObjectDictionary

#include <string>
#include <vector>

#include "Object.h"

namespace planner
{

struct ObjectTri
{
    std::size_t points [3];
};

struct ObjectData
{
    std::vector<ObjectTri> triangles_;
    planner::T_Vector3 normals_;
    std::string name;
};

typedef std::vector<ObjectData> Dictionary;
typedef std::vector<std::pair<std::size_t, Eigen::Vector3d> > T_PointReplacement;


struct ObjectDictionary
{
    planner::Object::T_Object recreate(const T_PointReplacement& replacement, const Object::T_Object &objects
                                       , std::vector<std::size_t>& newObjectIds) const;
    std::vector<Eigen::Vector3d> points;
    std::vector<std::vector<std::size_t> > objectsLinkedToVertex;
    Dictionary dic;
};

}//namespace planner;
#endif //_CLASS_ObjectDictionary
