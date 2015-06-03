#include "ObjectDictionary.h"
#include "MatrixDefsInternal.h"
#include "PQP/PQP.h"


using namespace std;
using namespace matrices;
using namespace planner;


void Vector3toArray(const Vector3& vect, double* arr)
{
    for(int i=0; i<3; ++i)
    {
        arr[i] = vect[i];
    }
}

Eigen::Vector3d FindPointIndex(const std::size_t index, const std::vector<Eigen::Vector3d>& points, const T_PointReplacement &replacement)
{
    for(T_PointReplacement::const_iterator cit = replacement.begin();
        cit!=replacement.end(); ++cit)
    {
        if(cit->first == index)
            return cit->second;
    }
    return points[index];
}

planner::Object* CreateObject(const ObjectData& data, const std::vector<Eigen::Vector3d>& dictionaryPoints, const T_PointReplacement &replacement)
{
    //create PQP MODEL
    PQP_Model* m = new PQP_Model;
    std::size_t currentIndex_(0);
    m->BeginModel();
    for(std::vector<ObjectTri>::const_iterator tit = data.triangles_.begin();
        tit != data.triangles_.end(); ++tit)
    {
        PQP_REAL points [3][3];
        for(int i=0; i< 3; ++i)
        {
            Vector3toArray(FindPointIndex(tit->points[i],dictionaryPoints, replacement),points[i]);
        }
        m->AddTri(points[0], points[1], points[2], currentIndex_++);
    }
    m->EndModel();
    return new planner::Object(m,data.normals_, data.name);
}

planner::Object::T_Object planner::ObjectDictionary::recreate(const T_PointReplacement &replacement, const planner::Object::T_Object& objects, std::vector<std::size_t>& newObjectIds) const
{
    Object::T_Object res;
    // collect modified objects
    for(T_PointReplacement::const_iterator cit = replacement.begin();
        cit!=replacement.end(); ++cit)
    {
        const std::vector<std::size_t>& linkedObjects = objectsLinkedToVertex[cit->first];
        for(std::vector<std::size_t>::const_iterator lit = linkedObjects.begin();
            lit != linkedObjects.end(); ++lit)
        {
            if(std::find(newObjectIds.begin(), newObjectIds.end(), *lit) == newObjectIds.end())
            {
                newObjectIds.push_back(*lit);
            }
        }
    }

    for(std::size_t lit = 0; lit < objects.size(); ++lit)
    {
        if(std::find(newObjectIds.begin(), newObjectIds.end(), lit) != newObjectIds.end())
        {
            res.push_back(CreateObject(dic[lit],points,replacement));
        }
        else
        {
            res.push_back(objects[lit]);
        }
    }
    return res;
}

