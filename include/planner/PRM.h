/**
* \file Graph.h
* \brief A Generic PRM planner implementation
* \author Steve T.
* \version 0.1
* \date 07/06/2014
*
* 
*/
#ifndef _CLASS_PRM
#define _CLASS_PRM

#include "astar/Graph.h"
#include "astar/AStar.h"
#include "planner/Component.h"

#include <limits>       // std::numeric_limits

namespace planner
{
/// \enum PlannerStage
/// \brief description of increasing complexity of planner
enum Stage
{
    simpleConnect,
    componentConnect,
    componentGrow
};

/// \class PRM
/// \brief Generic implementation of a probabilistic roadmap (PRM) as a astar::Graph.
/// Generator and LocalPlanner are template class
/// Distance is a method given as a parameter.
/// A modified A* algorithm is used to solve request for NodeContent not included in the graph
template<class NodeContent, class Generator, class LocalPlanner, typename Numeric=float, int Dim=10000>
class PRM : public  astar::Graph<NodeContent, Numeric, Dim, int, true>
{

public:
    typedef astar::Graph<NodeContent, Numeric, Dim, int, true> graph_t;
	typedef Numeric (*Distance)		  (const NodeContent*, const NodeContent*);
    typedef std::vector<const NodeContent*> T_NodeContentPath;

public:
    /// \brief Constructor
    /// \param size number of nodes to be generated
    PRM(int size = Dim)
        : graph_t()
        , size_(size)
    {
        // NOTHING
    }

    /// \brief Constructor
    /// \param generator instance of Generator template class that will generate collision-free configurations using operator()
    /// \param localPlanner instance of LocalPlanner class; operator() takes two NodeContent and a PlannerLevel and
	/// returns true whether two nodes can be connected (a collision free-path exists)
    /// \param distance Function used to measure the distance between two nodes. It
	///  has the signature Numeric (*Distance) (const NodeContent*, const NodeContent* )
    /// \param neighbourDistance maximum distance for which a node can be a neighbour of another
    /// \param size number of nodes to be generated
    /// \param k maximum number of neighbours for a given Node. Default value is 10
    /// \param Use a visiblity PRM or the classical PRM construction method
    PRM(Generator* generator, LocalPlanner* localPlanner, Distance distance, Numeric neighbourDistance, int size = Dim, int k=10, bool visibility = false)
        : graph_t()
        , size_(size)
    {
        if(visibility)
        {
            GenerateVisibilityPRM(generator, localPlanner, distance, neighbourDistance, size, k);
        }
        else
        {
            GeneratePRM(generator, localPlanner, distance, neighbourDistance, size, k);
        }
    }

	///\brief Destructor
	 ~PRM(){}

	///  \brief Computes a path between arbitrary NodeContent, not necessarily in the graph.
	///  \param from the entry point in the search
	///  \param to the destination that is to be reached
	///  \param dist Function used to measure the distance between two nodes. It 
	///  has the signature Numeric (*Distance) (const NodeContent*, const NodeContent* )
	///  \param localPlanner boolean method that returns true whether two nodes can be connected (a collision free-path exists)
	///	 \param neighbourDistance maximum distance for which a node can be a neighbour of another
	///  \param return : vector of NodeContent traversed to reach the goal. Empty if no pathfinding failed
     T_NodeContentPath ComputePath(const NodeContent* from, const NodeContent* to, Distance dist, LocalPlanner* localPlanner, Numeric neighbourDistance, bool ingraph = false) const
     {
        int start_id = GetClosestPointInGraph(from, dist, localPlanner, neighbourDistance, ingraph);
        int goal_id = GetClosestPointInGraph(to, dist, localPlanner, neighbourDistance, ingraph);
        typename astar_t::Path path;
		T_NodeContentPath res;
		if(start_id != -1 && goal_id !=-1)
		{
			astar_t astar(*this);
			if(astar.ComputePath(start_id, goal_id, path, dist))
			{
                //if(!ingraph)
                    res.push_back(from);
				for(std::list<int>::const_iterator it = path.begin(); it != path.end(); ++it)
				{
                    res.push_back(graph_t::nodeContents_[*it]);
				}
                //if(!ingraph)
                    res.push_back(to);
			}
		 }
		 return res;
     }

private:
    int GetClosestPointInGraph(const NodeContent* node, Distance dist, LocalPlanner* localPlanner, Numeric neighbourDistance, bool ingraph) const //todo this is really expensive at the moment
	{
		Numeric min_distance = std::numeric_limits<Numeric>::max();
		int current_index = 0; 
		int closest_index = -1; 
        for(typename PRM::T_NodeContentPtr::const_iterator it = graph_t::nodeContents_.begin();
                it != graph_t::nodeContents_.end() && current_index < size_;
				++it, ++current_index)
		{
			Numeric current_distance = dist(node,*it);
            if(current_distance < min_distance && (ingraph || (current_distance < neighbourDistance && (*localPlanner)(node,*it, simpleConnect))))
			{
				closest_index = current_index;
				min_distance = current_distance;
			}
		}
		return closest_index;
	}

    void ConnectComponents(Components& components, LocalPlanner* localPlanner, Distance distance, Numeric neighbourDistance)
    {
        components.Sort();
        for(std::vector<Component>::iterator it = components.components.begin();
            it!= components.components.end(); ++it)
        {
            std::vector<Component>::iterator it2 = it; ++it2;
            for(;it2 != components.components.end(); ++it2)
            {
                if(it->size() < 15) // MAX2 // ATTEMPT ALL
                {
                    for(Component::iterator it3 = it->begin();
                        it3!= it->end(); ++it3)
                    {
                        NodeContent* node = this->nodeContents_[*it3];
                        std::vector<int> closest = GetClosestPoints( node, *it2, 10, distance); //K2.1
                        for (std::vector<int>::const_iterator nodeit = closest.begin();
                             nodeit != closest.end(); ++nodeit)
                        {
                            if(distance(node,this->nodeContents_[*nodeit]) <= neighbourDistance && (*localPlanner)(node,this->nodeContents_[*nodeit], componentConnect))
                            {
                                graph_t::AddEdge(*it3, *nodeit);
                                components.AddConnection(*it3, *nodeit);
                                return;
                            }
                        }
                    }
                }
            }
        }
    }

    std::vector<int> GetClosestPoints(const NodeContent* node, const planner::Component& component, int nbNeighbours, Distance distance)
    {
        typedef std::pair<int, Numeric> DistanceNode;
        std::list< DistanceNode > distances;
        for(Component::iterator it = component.begin();
            it!= component.end(); ++it)
        {
            Numeric dist = distance(node, this->nodeContents_[*it]);
            typename std::list< DistanceNode >::iterator it2 = distances.begin();
            for(;it2 != distances.end(); ++it2)
            {
                if(dist < it2->second)
                {
                    break;
                }
            }
            distances.insert(it2, std::make_pair(*it, dist));
        }
        std::vector<int> res; int i =0;
        for(typename std::list< DistanceNode >::iterator it = distances.begin();
            it != distances.end() && i < nbNeighbours; ++it, ++i)
        {
            res.push_back(it->first);
        }
        return res;
    }

    void GeneratePRM(Generator* generator, LocalPlanner* localPlanner, Distance distance, Numeric neighbourDistance, int size = Dim, int k=10)
    {
        Components components;
        for(int i=0; i< size; ++i)
        {
            NodeContent* node = (*generator)();
            if(node == 0) return;
            int id = AddNode(node);
            int current_index = 0;
            int connected = 0;
            for(typename PRM::T_NodeContentPtr::iterator it = graph_t::nodeContents_.begin();
                 current_index < id && it != graph_t::nodeContents_.end() && connected <k ;
                ++it, ++current_index)
            {
                if(current_index != id && distance(node,*it) <= neighbourDistance && (*localPlanner)(node,*it, simpleConnect))
                {
                    if(graph_t::edges_[current_index].size() < ((unsigned int) k))
                    {
                        graph_t::AddEdge(id, current_index);
                        components.AddConnection(current_index, id);
                        ++connected;
                    }
                }
            }
            if(connected == 0) // do not add
            {
                components.AddConnection(id);
            }
        }
        size_t prevsize, newsize;
        prevsize = components.components.size();
        ConnectComponents(components, localPlanner, distance, neighbourDistance);
        newsize = components.components.size();
        while(prevsize != newsize)
        {
            prevsize = newsize;
            ConnectComponents(components, localPlanner, distance, neighbourDistance);
            newsize = components.components.size();
        }
    }

    void GenerateVisibilityPRM(Generator* generator, LocalPlanner* localPlanner, Distance distance, Numeric neighbourDistance, int M = Dim, int k=10)
    {
        int ntry = 0;
        Components guards;
		int id = -1;
        while(ntry < M)
        {
            bool foundGvis = false;
            bool connect = false;
            int gvis = -1;
            NodeContent* node = (*generator)();
            for(std::vector<Component>::iterator git = guards.components.begin();
                git != guards.components.end() && !connect; ++ git)
            {
                bool found = false;
                for(Component::const_iterator cit = (*git).begin();
                    cit != (*git).end() && !found; ++cit)
                {
                    int g = *cit;
                    if(distance(node,this->nodeContents_[g]) <= neighbourDistance && (*localPlanner)(node,this->nodeContents_[g], componentConnect))
                    {
                        found = true;
                        if(!foundGvis)
                        {
                            foundGvis = true;
                            gvis = g;
                        }
                        else
                        {
                            id = AddNode(node);
                            graph_t::AddEdge(id, gvis);
                            graph_t::AddEdge(id, g);
                            guards.AddConnection(id, gvis);
                            guards.AddConnection(id, g); // auto merge en thoerie
                            connect = true;
							git = guards.components.begin();
							break;
                        }
                    }
                }
            }
            if(!foundGvis) // guard
            {
                int id = AddNode(node);
                guards.AddConnection(id);
                ntry = 0;
            }
            else
            {
                ++ntry;
            }
			size_ = id;
        }
    }

private:
	typedef astar::AStar<NodeContent, Numeric, Dim, int, true> astar_t;
	int size_;

private:
	PRM(const PRM&);
	PRM& operator=(const PRM&);
};
} //namespace planner
#endif //_CLASS_PRM
