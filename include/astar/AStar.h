/**
* \file Graph.h
* \brief AStar implementation for generic undirected Graphs
* \author Steve T.
* \version 0.1
* \date 07/02/2014
*
* This file contains the Classes that allow to compute a path
* in an undirected Graph using the AStar (A*) Algorithm
* 
*/
#ifndef _CLASS_ASTAR
#define _CLASS_ASTAR

#include "Graph.h"

#include <algorithm>
#include <vector>
#include <list>
#include <map>

namespace astar
{
template<typename Numeric=float, typename Index = int>
struct CompareOpenSetValues
{
	CompareOpenSetValues(const std::map<Index,Numeric>& costs)
		: costs_(costs) {}

	~CompareOpenSetValues() {}

	bool operator()(Index a, Index b) const
	{
		return (costs_.at(a) < costs_.at(b));
	}

	const std::map<Index,Numeric>& costs_;
};

/// \class AStar
/// \brief Implementation of the AStar Algorithm. Template matches
/// with the target undirected Graph.
/// Parameter AllowCycles indicates whether cycles are allowed or not in the graph.
template<class NodeContent, typename Numeric=float, int Dim=10000, typename Index=int, bool AllowCycles=false>
class AStar
{
public:
	typedef Graph<NodeContent, Numeric, Dim, Index, AllowCycles> graph_t;
	typedef typename std::list<Index> Path;
	typedef Numeric (*Distance) (const NodeContent*, const NodeContent* );

private:
	typedef typename std::list<Index> OpenSet;
	typedef typename std::map<Index,bool> T_Node;
	typedef typename std::map<Index,Numeric> T_Cost;
	typedef typename std::map<Index,Index> T_Parent;

public:
    ///  \brief Constructor
	///  \param graph the graph on which the Pathdinfing is to be made
	 AStar(const graph_t& graph)
		 : graph_(graph){}

	///\brief Destructor
	 ~AStar(){}

public:
	///  \brief Computes a path between two nodes in a graph. TODO: Existence
	///	  of the nodes is not checked at this time
	///  \param from the entry point in the search
	///  \param to the destination that is to be reached
	///  \param path list of nodes traversed to reach the goal. Left empty if no path was found
	///  \param dist Function used to measure the distance between two nodes. It 
	///  has the signature Numeric (*Distance) (const NodeContent*, const NodeContent* )
	///  \param return : true if a path was found, false otherwise
    bool ComputePath(const Index from, const Index to, Path& path, Distance dist, Numeric costPerNode = 0) const
	{
		OpenSet openSet;
		T_Node closedSet;
		T_Parent cameFrom;
		T_Cost costFromStart;
		T_Cost estimatedTotalCost;
		NodeContent* goal = graph_.nodeContents_[to];
		CompareOpenSetValues<Numeric, Index> compare(estimatedTotalCost);

		openSet.push_back(from);
		costFromStart[from] = 0;
		estimatedTotalCost[from] = dist(graph_.nodeContents_[from], goal);

		while(openSet.size())
		{
			Index currentNode = openSet.front();
			openSet.pop_front();
			if(currentNode == to)
			{
				ReconstructPath(path, cameFrom, to);
				return true;
			}
			closedSet[currentNode] = true;
            for(typename graph_t::T_Connexion::const_iterator it = graph_.edges_[currentNode].begin(); it != graph_.edges_[currentNode].end(); ++it)
			{
				if(closedSet.find(*it) != closedSet.end())
				{
					continue;
				}
				Numeric currentGScore = costFromStart[currentNode] + dist(graph_.nodeContents_[currentNode], graph_.nodeContents_[*it]) + costPerNode;
                typename OpenSet::iterator openit = std::find(openSet.begin(), openSet.end(), currentNode);
                typename T_Cost::iterator costit = costFromStart.find(*it);
				if(openit == openSet.end() || (costit != costFromStart.end() && costit->second > currentGScore))
				{
					cameFrom[*it] = currentNode;
					costFromStart[*it] = currentGScore;
					Numeric currentTotalScrore = currentGScore + dist(graph_.nodeContents_[*it], goal);
					estimatedTotalCost[*it] = currentTotalScrore;
					if(openit == openSet.end())
					{
						openSet.push_front(*it);
						openSet.sort(compare);
					}
				}
			}
		}
		return false;
	}

public:
	const graph_t& graph_;

private:
	void ReconstructPath(Path& path, const T_Parent& cameFrom, const Index to) const
	{
        typename T_Parent::const_iterator it = cameFrom.find(to);
		path.push_front(to);
		if(it != cameFrom.end())
		{
			ReconstructPath(path, cameFrom, it->second);
		}
	}

private:
	AStar(const AStar&);
	AStar& operator=(const AStar&);
};
} //namespace astar
#endif //_CLASS_ASTAR
