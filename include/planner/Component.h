/**
* \file Component.h
* \brief Helper structure to handle connected node in Graph
* \author Steve T.
* \version 0.1
* \date 07/06/2014
*
* 
*/
#ifndef _STRUCT_COMPONENT
#define _STRUCT_COMPONENT

#include <set>          // set
#include <vector>          // set
#include <algorithm>

namespace planner
{
    typedef std::set<int> Component;

    struct CompareComponent {
      bool operator() (Component i,Component j) { return (i.size()<j.size());}
    } ;

    struct Components
    {
         Components(){}
        ~Components(){}

         void AddConnection(int oldNode, int newNode)
         {
             for(std::vector<Component>::iterator it = components.begin();
                 it != components.end(); ++it)
             {
                 if(it->find(oldNode) != it->end())
                 {
                     //see if newNode alreay elsewhere; if so, merge
                     std::vector<Component>::iterator it2 =it; ++it2;
                     for(; it2 != components.end(); ++it2)
                     {
                        if(it2->find(newNode) != it2->end())
                        {
                            it2->insert(it->begin(), it->end());
                            components.erase(it);
                            return;
                        }
                     }
                     it->insert(newNode);
                     it->insert(oldNode);
                     return;
                 }
                 else if(it->find(newNode) != it->end())
                 {
                     //see if newNode alreay elsewhere; if so, merge
                     std::vector<Component>::iterator it2 =it; ++it2;
                     for(; it2 != components.end(); ++it2)
                     {
                        if(it2->find(oldNode) != it2->end())
                        {
                            it2->insert(it->begin(), it->end());
                            components.erase(it);
                            return;
                        }
                     }
                     it->insert(newNode);
                     it->insert(oldNode);
                     return;
                 }
             }
             Component newComponent;
             newComponent.insert(newNode);
             newComponent.insert(oldNode);
             components.push_back(newComponent);
         }

         void AddConnection(int newNode)
         {
             Component newComponent;
             newComponent.insert(newNode);
             components.push_back(newComponent);
         }

         void Sort()
         {
             std::sort(components.begin(), components.end(), comp);
         }

         CompareComponent comp;
         std::vector<Component> components;
    };

} //namespace planner
#endif //_STRUCT_COMPONENT
