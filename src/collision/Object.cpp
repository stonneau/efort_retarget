#include "Object.h"

#include "MatrixDefsInternal.h"


namespace
{
    void EigenToDoubleVector(const Eigen::Vector3d& from, PQP_REAL* to)
    {
        for(int i=0; i<3; ++i)
        {
            to[i] = from(i);
        }
    }

    void DoubleVectorToEigen(Eigen::Vector3d& to, const PQP_REAL* from)
    {
        for(int i=0; i<3; ++i)
        {
            to[i] = from[i];
        }
    }

    void EigenToDoubleMatrix(const Eigen::Matrix3d& from, PQP_REAL to [3][3])
    {
        for(int i=0; i<3; ++i)
        {
            for(int j=0; j<3; ++j)
            {
                to[i][j] = from(i,j);
            }
        }
    }

    Eigen::Vector3d ProjPoint2Triangle(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                       const Eigen::Vector3d &p2, const Eigen::Vector3d &source_position)
    {
         Eigen::Vector3d edge0 = p1 - p0;
         Eigen::Vector3d edge1 = p2 - p0;
         Eigen::Vector3d v0 = p0 - source_position;

         double a = edge0.dot(edge0);
         double b = edge0.dot(edge1);
         double c = edge1.dot(edge1);
         double d = edge0.dot(v0);
         double e = edge1.dot(v0);

         double det = a*c - b*b;
         double s = b*e - c*d;
         double t = b*d - a*e;

         double lower_bound = 0.0, upper_bound = 1.0;


         if ( s + t < det )
         {
            if ( s < 0.0 )
            {
                if ( t < 0.0 )
                {
                    if ( d < 0.0 )
                    {
                        s = std::min(std::max(-d/a, lower_bound), upper_bound);
                        t = 0.0;
                    }
                    else
                    {
                         s = 0.0;
                         t = std::min(std::max(-e/c, lower_bound), upper_bound);
                    }
                 }
                 else {
                     s = 0.0;
                     t = std::min(std::max(-e/c, lower_bound), upper_bound);
                 }
             }
             else if ( t < 0.0 ) {
                 s = std::min(std::max(-d/a, lower_bound), upper_bound);
                 t = 0.0;
             }
             else {
                 float invDet = 1.0 / det;
                 s *= invDet;
                 t *= invDet;
             }
         }
         else {
             if ( s < 0.0 ) {
                 double tmp0 = b+d;
                 double tmp1 = c+e;
                 if ( tmp1 > tmp0 ) {
                     double numer = tmp1 - tmp0;
                     double denom = a-2*b+c;
                     s = std::min(std::max(numer/denom, lower_bound), upper_bound);
                     t = 1-s;
                 }
                 else {
                     t = std::min(std::max(-e/c, lower_bound), upper_bound);
                     s = 0.f;
                 }
             }
             else if ( t < 0.f ) {
                 if ( a+d > b+e ) {
                     double numer = c+e-b-d;
                     double denom = a-2*b+c;
                     s = std::min(std::max(numer/denom, lower_bound),upper_bound);
                     t = 1-s;
                 }
                 else {
                     s = std::min(std::max(-e/c, lower_bound), upper_bound);
                     t = 0.f;
                 }
             }
             else {
                 double numer = c+e-b-d;
                 double denom = a-2*b+c;
                 s = std::min(std::max(numer/denom, lower_bound), upper_bound);
                 t = 1.0 - s;
             }
         }
         return p0 + s * edge0 + t * edge1;
    }

}

using namespace planner;

Object::Object(PQP_Model* model, std::string name)
    : model_(model)
    , position_(Eigen::Vector3d::Zero())
    , orientation_(Eigen::Matrix3d::Identity())
    , name_(name)
{
    EigenToDoubleMatrix(orientation_, pqpOrientation_);
    EigenToDoubleVector(position_, pqpPosition_);
}

Object::Object(PQP_Model* model, const T_Vector3& normals, std::string name)
    : normals_(normals)
    , model_(model)
    , position_(Eigen::Vector3d::Zero())
    , orientation_(Eigen::Matrix3d::Identity())
    , name_(name)
{
    assert(normals_.empty() || normals_.size() == model->num_tris);
    EigenToDoubleMatrix(orientation_, pqpOrientation_);
    EigenToDoubleVector(position_, pqpPosition_);
}

Object::Object(const Object& parent)
    : position_(Eigen::Vector3d::Zero())
    , orientation_(Eigen::Matrix3d::Identity())
    , normals_(parent.normals_)
    , name_(parent.name_)
{
    model_ = new PQP_Model;
    model_->BeginModel();
    for (int i =0; i< parent.model_->num_tris; ++i)
    {
         // TODO CHECK MEME ?
        model_->AddTri(parent.model_->tris[i].p1, parent.model_->tris[i].p2, parent.model_->tris[i].p3, parent.model_->tris[i].id);
    }
    model_->EndModel();
    SetPosition(parent.GetPosition());
    SetOrientation(parent.GetOrientation());
    EigenToDoubleMatrix(orientation_, pqpOrientation_);
    EigenToDoubleVector(position_, pqpPosition_);
}

Object::~Object()
{
    delete model_;
    //delete pqpOrientation_;
    //delete pqpPosition_;
}

double Object::Distance(Object* object)
{
    PQP_DistanceResult result;
    PQP_Distance(&result, pqpOrientation_, pqpPosition_, model_,
                 object->pqpOrientation_, object->pqpPosition_, object->model_,
                 0.01, 0.01, 2);
    return result.distance;
}

bool Object::InContact(Object* object, double epsilon)
{
    PQP_ToleranceResult result;
    PQP_Tolerance(&result, pqpOrientation_, pqpPosition_, model_,
                 object->pqpOrientation_, object->pqpPosition_, object->model_,
                 epsilon, 2);
    return result.CloserThanTolerance();
}


#include <iostream>

bool Object::InContact(Object* object, double epsilon, Eigen::Vector3d& normal, Eigen::Vector3d& proj)
{
    PQP_ToleranceResult result;
    PQP_Tolerance(&result, pqpOrientation_, pqpPosition_, model_,
                 object->pqpOrientation_, object->pqpPosition_, object->model_,
                 epsilon, 2);
    if(!result.CloserThanTolerance()) return false;
    if(object->normals_.empty())
    {
        std::cout << "no normals";
        return true;
    }
    // finding closest triangle.
    // make sure as many normals as tris;
    assert(object->model_->num_tris == object->normals_.size());
    double minDistance = std::numeric_limits<double>::max();
    double currentDistance;
    Eigen::Vector3d currentProjection;
    Eigen::Vector3d p1, p2, p3;
    // triangles in model not in the order they were inserted ! check ID
    for(int i=0; i < object->model_->num_tris; ++i)
    {
        const Tri& t = object->model_->tris[i];
        DoubleVectorToEigen(p1, t.p1);
        DoubleVectorToEigen(p2, t.p2);
        DoubleVectorToEigen(p3, t.p3);
        currentProjection = ProjPoint2Triangle(p1,p2,p3, this->position_);
        currentDistance = (currentProjection - this->position_).norm();
        if(currentDistance < minDistance)
        {
            normal = object->normals_[t.id];
            proj = currentProjection;
            minDistance = currentDistance;
        }
    }
    return true;
}

/*bool Object::InContact(Object* object, double epsilon, Eigen::Vector3d& normal, Eigen::Vector3d& proj)
{
    Eigen::Vector3d projection;
    //PQP_ToleranceResult result;
    //PQP_Tolerance(&result, pqpOrientation_, pqpPosition_, model_,
    //             object->pqpOrientation_, object->pqpPosition_, object->model_,
    //             epsilon, 2);
    //bool res = result.CloserThanTolerance();
    PQP_DistanceResult result;
    PQP_Distance(&result, pqpOrientation_, pqpPosition_, model_,
                 object->pqpOrientation_, object->pqpPosition_, object->model_,
                 epsilon, epsilon, 2);
    bool res = result.Distance() <= epsilon;
    if(res)
    {
        if (object->normals_.empty())
        {
            std::cout << "no normals";
            return res;
        }
        int i= 0;
        double distance = std::numeric_limits<double>::max();
        double tmp = distance;
        Eigen::Vector3d p1, p2, p3, source;
        DoubleVectorToEigen(source, result.P2());
        for(T_Vector3::const_iterator cit = object->normals_.begin(); cit != object->normals_.end(); ++cit, ++i)
        {
            DoubleVectorToEigen(p1, this->model_->tris[i].p1);
            DoubleVectorToEigen(p2, this->model_->tris[i].p2);
            DoubleVectorToEigen(p3, this->model_->tris[i].p3);
            projection = ProjPoint2Triangle(p1,p2,p3, source);
            tmp = (projection - source).norm();
            if(tmp < distance)
            {
                normal = *cit;
                proj = projection;
            }
        }
    }
    return res;
}*/

bool Object::InContact(Object* object, double epsilon, Eigen::Vector3d& normal, const std::vector<Eigen::Vector3d>& positions)
{
    PQP_ToleranceResult result;
    PQP_Tolerance(&result, pqpOrientation_, pqpPosition_, model_,
                 object->pqpOrientation_, object->pqpPosition_, object->model_,
                 epsilon, 2);
    bool res = result.CloserThanTolerance();
    if(res)
    {
        if (object->normals_.empty())
        {
            std::cout << "no normals";
            return res;
        }
        int i= 0;
        double distance = std::numeric_limits<double>::max();
        double tmp = distance;
        Eigen::Vector3d p1, p2, p3, source;
        DoubleVectorToEigen(source, result.P1());
        for(T_Vector3::const_iterator cit = object->normals_.begin(); cit != object->normals_.end(); ++cit, ++i)
        {
            DoubleVectorToEigen(p1, object->model_->tris[i].p1);
            DoubleVectorToEigen(p2, object->model_->tris[i].p2);
            DoubleVectorToEigen(p3, object->model_->tris[i].p3);
            tmp = (ProjPoint2Triangle(p1,p2,p3, source) - source).norm();
            if(tmp < distance)
            {
                normal = *cit;
            }
        }
        int nbfalse = 0;
        Eigen::Vector3d barycenter(0,0,0);
        for(std::vector<Eigen::Vector3d>::const_iterator cit = positions.begin()
            ; cit != positions.end(); ++cit)
        {
            barycenter += *cit;
        }
        barycenter = barycenter / positions.size();
        if((ProjPoint2Triangle(p1,p2,p3, barycenter) -(barycenter)).norm() > 2 * epsilon)
            return false;
    }
    return res;
}

bool Object::IsColliding(Object* object)
{
    PQP_CollideResult cres;
    PQP_Collide(&cres, pqpOrientation_, pqpPosition_, model_,
                object->pqpOrientation_, object->pqpPosition_, object->model_, PQP_FIRST_CONTACT);
    return cres.Colliding();
}

bool Object::IsColliding(T_Object& objects)
{
    PQP_CollideResult cres;
    for(T_Object::iterator it = objects.begin();
        it != objects.end(); ++it)
    {
        PQP_Collide(&cres, pqpOrientation_, pqpPosition_, model_,
                    (*it)->pqpOrientation_, (*it)->pqpPosition_, (*it)->model_, PQP_FIRST_CONTACT);
        if (cres.Colliding())
        {
            return true;
        }
    }
    return false;
}

bool Object::IsColliding(T_Object& objects, const Eigen::Vector3d &normal, double tolerance)
{
    for(T_Object::iterator it = objects.begin();
        it != objects.end(); ++it)
    {
        if(IsColliding(*it,normal,tolerance)) return true;
    }
    return false;
}

bool Object::IsColliding(Object* objet, const Eigen::Vector3d &normal, double tolerance)
{
    PQP_CollideResult cres;
        PQP_Collide(&cres, pqpOrientation_, pqpPosition_, model_,
                    objet->pqpOrientation_, objet->pqpPosition_, objet->model_, PQP_ALL_CONTACTS);
    if (cres.Colliding())
    {
        for(int i =0; i < cres.NumPairs(); ++i)
        {
            //find triangle
            const Eigen::Vector3d& obstaclenorm = this->normals_[cres.Id2(i)];
            if(normal.dot(obstaclenorm) >= tolerance)
            {
                return true;
            }
        }
    }
    return false;
}


bool Object::IsColliding(const T_Object::iterator& from,  const T_Object::iterator& to)
{
    T_Object::iterator it = from;
    PQP_CollideResult cres;
    for(;it != to; ++it)
    {
        PQP_Collide(&cres, pqpOrientation_, pqpPosition_, model_,
                    (*it)->pqpOrientation_, (*it)->pqpPosition_, (*it)->model_, PQP_FIRST_CONTACT);
        if (cres.Colliding())
        {
            return true;
        }
    }
    return false;
}

void Object::SetOrientation(const Eigen::Matrix3d& orientation)
{
    orientation_ = orientation;
    EigenToDoubleMatrix(orientation_, pqpOrientation_);
}

void Object::SetPosition(const Eigen::Vector3d& position)
{
    position_ = position;
    EigenToDoubleVector(position_, pqpPosition_);
}

const Eigen::Matrix3d& Object::GetOrientation() const
{
    return orientation_;
}

const Eigen::Vector3d& Object::GetPosition() const
{
    return position_;
}


const PQP_Model* Object::GetModel() const
{
    return model_;
}

double planner::MinDistance(const Eigen::Vector3d& point, Object* object, Eigen::Vector3d& projection, Eigen::Vector3d& normal)
{
    assert(object->model_->num_tris == object->normals_.size());
    double minDistance = std::numeric_limits<double>::max();
    double currentDistance;
    Eigen::Vector3d currentProjection;
    Eigen::Vector3d p1, p2, p3;
    for(int i =0; i<object->model_->num_tris; ++i)
    {
        const Tri& t = object->model_->tris[i];
        DoubleVectorToEigen(p1, t.p1);
        DoubleVectorToEigen(p2, t.p2);
        DoubleVectorToEigen(p3, t.p3);
        currentProjection = ProjPoint2Triangle(p1,p2,p3, point);
        currentDistance = (currentProjection - point).norm();
        if(currentDistance < minDistance)
        {
            normal = object->normals_[t.id];
            projection = currentProjection;
            minDistance = currentDistance;
        }
    }
    return minDistance;
}
