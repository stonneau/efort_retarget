/**
* \file Graph.h
* \brief A Generic PRM planner implementation
* \author Steve T.
* \version 0.1
* \date 07/06/2014
*
* 
*/
#ifndef _CLASS_RRT
#define _CLASS_RRT

#include "astar/Graph.h"
#include "astar/AStar.h"
#include "planner/Component.h"

#include <limits>       // std::numeric_limits

namespace planner
{
/// \class RRT
/// \brief Generic implementation of a (bi-RRT) as a astar::Graph.
/// Generator and LocalPlanner are template class
/// Distance is a method given as a parameter.
template<class NodeContent, class Generator, class LocalPlanner, typename Numeric=float, bool Ordered=false, int Dim=10000>
class RRT
{

public:
    typedef astar::Graph<NodeContent, Numeric, Dim, int, true> graph_t;
	typedef Numeric (*Distance)		  (const NodeContent*, const NodeContent*);
    typedef std::vector<const NodeContent*> T_NodeContentPath;

public:
    /// \brief Constructor
    /// \param size max number of nodes to be generated
    RRT(int size = Dim)
        : size_(size)
    {
        // NOTHING
    }

    /// \brief Constructor
    /// \param generator instance of Generator template class that will generate collision-free configurations using operator()
    /// \param localPlanner instance of LocalPlanner class; operator() takes two NodeContent and a PlannerLevel and
	/// returns true whether two nodes can be connected (a collision free-path exists)
    /// \param from the entry point in the search
    /// \param to the destination that is to be reached
    /// \param distance Function used to measure the distance between two nodes. It
	///  has the signature Numeric (*Distance) (const NodeContent*, const NodeContent* )
    /// \param neighbourDistance maximum distance for which a node can be a neighbour of another
    /// \param size max number of nodes to be generated
    /// \param k maximum number of neighbours for a given Node. Default value is 10
    /// \param biRRT use single RRT or bi-RRT to solve problem
    RRT(Generator* generator, LocalPlanner* localPlanner, NodeContent* from, NodeContent* to, Distance distance, const Numeric neighbourDistance, int size = Dim, int k=10, bool biRRT=false)
        : from_(from)
        , to_(to)
        , size_(size)
        , path_(biRRT ? GenerateBiRRT(generator, localPlanner, distance, neighbourDistance) :
                        GenerateRRT(generator, localPlanner, distance, neighbourDistance))
    {
        // NOTHING
    }

	///\brief Destructor
     ~RRT(){}


private:
    typedef astar::AStar<NodeContent, Numeric, Dim, int, true> astar_t;
    NodeContent* from_;
    NodeContent* to_;
    int size_;
    graph_t g1_;
    graph_t g2_;

public:
    const T_NodeContentPath path_;

private:
    int GetClosestPointInGraph(const graph_t& g, const NodeContent* node, Distance dist, LocalPlanner* localPlanner, Numeric neighbourDistance, bool ingraph) const //todo this is really expensive at the moment
    {
        Numeric min_distance = std::numeric_limits<Numeric>::max();
        int current_index = 0;
        int closest_index = -1;
        for(typename graph_t::T_NodeContentPtr::const_iterator it = g.nodeContents_.begin();
                it <= g.nodeContents_.end() && current_index <= g.currentIndex_;
                ++it, ++current_index)
        {
            Numeric current_distance = dist(node,*it);
            if(current_distance < min_distance && (ingraph || (current_distance < neighbourDistance && (*localPlanner)(node,*it))))
            {
                closest_index = current_index;
                min_distance = current_distance;
            }
        }
        return closest_index;
    }

    int GetRandomPointInGraph(const graph_t& g) const //todo this is really expensive at the moment
    {
        return rand() % (int)(g.currentIndex_+1);
    }

     T_NodeContentPath ComputePath(const graph_t& g, const int from, const int to, Distance dist) const
     {
        typename astar_t::Path path;
        T_NodeContentPath res;
        if(from != -1 && to !=-1)
        {
            astar_t astar(g);
            if(astar.ComputePath(from, to, path, dist))
            {
                //res.push_back(g.nodeContents_[from]);
                for(std::list<int>::const_iterator it = path.begin(); it != path.end(); ++it)
                {
                    res.push_back(g.nodeContents_[*it]);
                }
                //res.push_back(g.nodeContents_[to]);
            }
         }
         return res;
     }

    enum ExtendRet
    {
       trapped,reached,advanced
    };

    ExtendRet Extend(NodeContent* node, graph_t& g, LocalPlanner* localPlanner, Distance distance, const Numeric neighbourDistance, bool reverse, bool cloneIfAdding=false)
    {
        const int& nearestId = Ordered ? GetRandomPointInGraph(g) : GetClosestPointInGraph(g, node, distance, localPlanner, neighbourDistance, true);
        NodeContent* nearest = g.nodeContents_[nearestId];
        if(!reverse && (*localPlanner) (nearest,node))
        {
            int id = g.AddNode(cloneIfAdding ? (new NodeContent(*node)) : node);
            g.AddEdge(nearestId,id,Ordered);
            return reached; // TODO ADVANCED
        }
        else if(reverse && (*localPlanner) (node,nearest))
        {
            int id = g.AddNode(cloneIfAdding ? (new NodeContent(*node)) : node);
            g.AddEdge(id,nearestId,Ordered);
            return reached; // TODO ADVANCED
        }
        else
            return trapped;
    }

    T_NodeContentPath GenerateRRT(Generator* generator, LocalPlanner* localPlanner, Distance distance, const Numeric neighbourDistance)
    {
        if((*localPlanner)(from_, to_))
        {
            T_NodeContentPath res;
            res.push_back(from_);
            res.push_back(to_);
            return res;
        }
        int startId = g1_.AddNode(from_);
        for(int k = 0; k < size_; ++k)
        {
            NodeContent* node = (*generator)();
            if(node == 0) break;
                if(Extend(node,g1_,localPlanner, distance, neighbourDistance, false, false) != trapped)
                {
                    int nodeId = g1_.currentIndex_;
                    if ((*localPlanner)(node, to_))
                    {
                        int endId = g1_.AddNode(to_);
                        g1_.AddEdge(nodeId,endId,Ordered);
                        T_NodeContentPath res = ComputePath(g1_,startId,endId,distance);
                        if(!res.empty())
                        {
                            shortcut(res, localPlanner);
                            return res;
                        }
                    }
                }
        }
        T_NodeContentPath res;
        return res;
    }

    void shortcutrec(T_NodeContentPath& res, LocalPlanner* localPlanner, const int currentIndex)
    {
        if(currentIndex >= res.size()) return;
        bool erased = false;
        for(int i = currentIndex+2; i< res.size() && ! erased; ++i)
        {
            if((*localPlanner)(res[currentIndex], res[i]))
            {
                erased = true;
                res.erase(res.begin() + currentIndex + 1, res.begin() + i);
            }
        }
        if(erased)
        {
            shortcutrec(res, localPlanner, currentIndex);
        }
        else
        {
            shortcutrec(res, localPlanner, currentIndex+1);
        }
    }

    void shortcut(T_NodeContentPath& path, LocalPlanner* localPlanner)
    {
        shortcutrec(path, localPlanner,0);
    }

    T_NodeContentPath GenerateBiRRT(Generator* generator, LocalPlanner* localPlanner, Distance distance, Numeric neighbourDistance)
    {
        if((*localPlanner)(from_, to_))
        {
            T_NodeContentPath res;
            res.push_back(from_);
            res.push_back(to_);
            return res;
        }
        int startId = g1_.AddNode(from_);
        int endId = g2_.AddNode(to_);
        graph_t* gtmp;
        graph_t* ga = &g1_;
        graph_t* gb = &g2_;
        bool inverse = false;
        for(int k = 0; k < size_; ++k)
        {
            NodeContent* node = (*generator)();
            if(node == 0) break;
            if(Extend(node,*ga,localPlanner, distance, neighbourDistance, inverse) != trapped)
            {
                if(Extend(node,*gb,localPlanner, distance, neighbourDistance, !inverse, true) == reached)
                {
                    int nodeId = g1_.currentIndex_;
                    T_NodeContentPath res1 = ComputePath(g1_, startId,nodeId,distance);
                    nodeId = g2_.currentIndex_;
                    T_NodeContentPath res2 = ComputePath(g2_, nodeId,endId,distance);
                    if(!(res1.empty() ||res2.empty() ))
                    {
                        res1.pop_back();
                        res1.insert(res1.end(), res2.begin(), res2.end());
                        shortcut(res1,localPlanner);
                        return res1;
                    }
                }
            }
            //swap
            gtmp = ga;
            ga = gb;
            gb = gtmp;
            inverse = !inverse;
        }
        T_NodeContentPath res;
        return res;
    }

private:
    RRT(const RRT&);
    RRT& operator=(const RRT&);
};
} //namespace planner
#endif //_CLASS_RRT
