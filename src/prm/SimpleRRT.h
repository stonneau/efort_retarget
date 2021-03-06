/**
* \file SimplePRM.h
* \brief A PRM concrete implementation, and serialization methods associated
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/
#ifndef _CLASS_SIMPLERRT
#define _CLASS_SIMPLERRT

#include <vector>
#include <memory>

#include "collision/Object.h"
#include "Model.h"

#include <string>
#include <array>

namespace planner
{
struct PImpl;

/// \class SimpleRRT
/// \brief Concrete implementation of a probabilistic roadmap (PRM)
/// Candidate Nodes are generated in a purely random manner
/// Scaled euclidian distance is used as a metric
/// Connectivity is checked using a simple straight line planner 
class SimpleRRT
{
public:
	///\brief Constructor
    ///\param model Model used for the sample generation
    ///\param objects Description of the world
	///\param neighbourDistance maximum distance for which a node can be a neighbour of another
	///\param size number of nodes to be generated
	///\param k maximum number of neighbours for a given Node. Default value is 10
    SimpleRRT(Model* from, Model* to, Object::T_Object &objects, float neighbourDistance, int size = 1000, int k = 10);

    ///\brief Constructor
    ///\param model Model used for the sample generation
    ///\param objects Description of the world
    ///\param collisionObjects Description of collision world
    ///\param neighbourDistance maximum distance for which a node can be a neighbour of another
    ///\param size number of nodes to be generated
    ///\param k maximum number of neighbours for a given Node. Default value is 10
    SimpleRRT(Model* from, Model* to, Object::T_Object &objects, Object::T_Object &collisionObjects, float neighbourDistance,
              int size = 1000, int k = 10);

	///\brief Destructor
     ~SimpleRRT();

    ///\brief Computes a path between two nodes. If the nodes are not on the graph, the closest nodes
    ///are chosen and connected to the path.
    ///\param from Starting configuration
    ///\param to Goal configuration
    ///\param neighbourDistance maximum distance for which a node can be a neighbour of another
    ///\return a list of Object corresponding to the path. If no path was found the list is empty
     CT_Model GetPath();

	///\brief Interpolated configurations along path
    ///\param path path
    ///\param steps number of computed configurations
    ///\return a list of Object corresponding to the path. If no path was found the list is empty
     std::vector<Eigen::Matrix4d> Interpolate(const CT_Model &path, int /*steps*/);

private:
    Object::T_Object objects_;
    Object::T_Object CollisionObjects_;
    std::auto_ptr<PImpl> pImpl_;
};

} //namespace planner
#endif //_CLASS_SIMPLERRT
