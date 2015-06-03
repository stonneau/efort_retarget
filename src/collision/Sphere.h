/**
* \file Sphere.h
* \brief Representation of a 3D Sphere. Can only be used for sphere sphere collision
* \author Steve T.
* \version 0.1
* \date 16/07/2014
*
*/
#ifndef _CLASS_SPHERE
#define _CLASS_SPHERE

#include <Eigen/Dense>

#include <vector>

namespace planner
{
enum SphereCollisionType
{
	None = 0,
	sphereContained = 1,
	tangentialExterior = 2,
	cercle = 3
};

struct SphereCollisionRes
{
	SphereCollisionType collisionType;
	bool infoComputed; // true if center and radius defined
	// radius and center only defined
	// center and radius only defined  if collisionType >= 2
	Eigen::Vector3d center; // tangential point or  intersection circle center
	double radius; // radius of intersection center, 0 if tangential
	
};


/// \class Sphere
/// \brief Description of a collidable 3d Sphere.
class Sphere
{
public:
    ///  \brief Constructor
    ///  \param center 3d position of the sphere center
    ///  \param radius radius of the sphere
    Sphere(const Eigen::Vector3d& center, const double radius) : center_(center), radius_(radius){}

    ///  \brief Constructor
    ///  \param x x position of the sphere center
    ///  \param y y position of the sphere center
    ///  \param z z position of the sphere center
    ///  \param radius radius of the sphere
    Sphere(const double x, const double y, const double z, const double radius) : center_(Eigen::Vector3d(x,y,z)), radius_(radius){}

     ///  \brief Destructor
    ~Sphere(){}

public:
    Eigen::Vector3d center_;
    double radius_;
};

const  SphereCollisionRes Intersect(const Sphere& a, const Sphere& b, bool computePoint = false);
const  std::vector<int> Intersect(const Sphere& a, const std::vector<Sphere>spheres);
const  bool Contains(const Sphere& a, const Eigen::Vector3d& point, double margin = 1);
}//namespace planner;
#endif //_CLASS_COLLIDER
