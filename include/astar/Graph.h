/**
* \file Graph.h
* \brief Description of an undirected generic Graph
* \author Steve T.
* \version 0.1
* \date 07/02/2014
*
* A generic template class for undirected Graphs.
* 
*/
#ifndef _CLASS_GRAPH
#define _CLASS_GRAPH

#include <vector>
#include <array>
#include <algorithm>    // std::find

namespace astar
{
	
/// \class Graph
/// \brief A generic template class for undirected Graphs. AllowCycles template parameter
/// indicates whether cycles are allowed or not in the graph.
template<class NodeContent, typename Numeric=float, int Dim=10000, typename Index=int, bool AllowCycles=false>
class Graph
{
    //friend class AStar<NodeContent, Numeric, Dim, Index, AllowCycles> ;

public:
	typedef std::vector<Index> T_Connexion;
	typedef std::array<T_Connexion, Dim> T_Edges;
	typedef std::array<NodeContent*, Dim> T_NodeContentPtr;

public:
	///\brief Constructor
	 Graph()
		 : currentIndex_(-1) {}
		 
	///\brief Destructor
	~Graph()
	{
		for(int i=0; i <= currentIndex_; ++i)
		{
			delete nodeContents_[i]; // Is this the right place to do this ?
		}
	}

public:
	///  \brief Adds a NodeContent to the graph, if the graph does not alreay contain
	///  Dim nodes.
	///  \param node the NodeContent to be added to the graph
	///  \param return : the index of the added NodeContent, which is equal to the total number of nodes -1
	Index AddNode(NodeContent* node)
	{
        if(currentIndex_ < Dim)
		{
            nodeContents_[++currentIndex_] = node;
		}
        return currentIndex_;
    }

	///  \brief Adds an Edge between indicated nodes, if it does not already exist.
	///  If AllowCycles is set to false, then the edge is not added if it indtroduces a cycle 
	///  \param a index of one connected node
	///  \param b index of the other connected node
	///  \param return : true if the insertion was successful, false otherwise
	bool AddEdge(Index a, Index b)
	{
		if(std::find(edges_[a].begin(), edges_[a].end(), b) == edges_[a].end())
		{
			edges_[a].push_back(b);
			edges_[b].push_back(a);
			if(AllowCycles || !HasCycle())
				return true;
			else
				RemoveEdge(a, b);
		}
		return false;
	}

	///  \brief Removes an Edge between indicated nodes, if it exists.
	///  If AllowCycles is set to false, then the edge is not added if it indtroduces a cycle 
	///  \param a index of one connected node
	///  \param b index of the other connected node
	///  \param return : true if the removal was successful, false otherwise
	bool RemoveEdge(Index a, Index b)
	{
        typename T_Connexion::iterator it = std::find(edges_[a].begin(), edges_[a].end(), b);
		if(it != edges_[a].end())
		{
			edges_[a].erase(it);
		}
		it = std::find(edges_[b].begin(), edges_[b].end(), a);
		if(it != edges_[b].end())
		{
			edges_[b].erase(it);
			return true;
		}
		return false;
	}

	
	///  \brief Costful operation to check whether the graph has cycles
	///  Used internally for edge creation when AllowCycles is set to false.
	///  \param return : true if the graph has a cycle
	bool HasCycle() const
	{
		bool *visited = new bool[currentIndex_+1];
		for (int i = 0; i < currentIndex_; i++)
		{
			visited[i] = false;
		}
		bool res = false;
		for (int u = 0; u < currentIndex_; u++)
		{
			if (!visited[u] && HasCycleInternal(u, -1, visited))
			{
				res = true;
				break;
			}
		}
		delete visited;
		return res;
	}

private: 
	bool HasCycleInternal(Index currentNode, Index parent, bool* visited) const
	{
		visited[currentNode] = true;
        typename T_Connexion::const_iterator it = edges_[currentNode].begin();
		for(; it != edges_[currentNode].end(); ++it)
		{
			if(!visited[*it])
			{
				if(HasCycleInternal(*it, currentNode, visited)) return true;
			}
			else
			{
				if(*it != parent) return true;
			}
		}
		return false;
    }
	
private:
	Graph(const Graph&);
	Graph& operator=(const Graph&);

public:
	T_Edges edges_;
	T_NodeContentPtr nodeContents_;
	int currentIndex_;
};
} //namespace astar
#endif //_CLASS_GRAPH
