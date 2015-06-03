/**
* \file SimplePRM.h
* \brief Helper struct that contains all the data required for using a prm.
* Scenario is loaded from a description file. Depending on the look of the file
* the SimplePRM instance can be loaded from a file or simply generated.
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/
#ifndef _STRUCT_SCENARIO
#define _STRUCT_SCENARIO

#include "prm/SimplePRM.h"
#include "collision/Object.h"
#include "collision/ObjectDictionary.h"

#include <exception>
#include <string>

namespace planner
{
class ScenarioException: public std::exception
{
public:
    ScenarioException(const std::string& message) throw()
		:mess(message){}
 
     virtual const char* what() const throw()
     {
         return mess.c_str();
     }

	 virtual ~ScenarioException() throw(){}
 
private:
    std::string mess;
};


struct Scenario
{
	/// \brief Constructor
    /// \param filepath path to a text file describing the a PRM Scenario
	/// 2 types of file can exist, one for generating a prm, the other one for loading it from a file:
	/// MODEL = path/to/objfile (contains exactly two objects)
	/// OBJECT = path/to/objfile (contains any number of objects)
	/// OBJECT = path/to/objfile (contains any number of objects) // this line can appear several times
	/// SIZE = N D K (N number of nodes in the prm, D neighbour distance, K number of neighbours)
	/// 
	///
	/// MODEL = path/to/objfile (contains exactly two objects)
	/// OBJECT = path/to/objfile (contains any number of objects)
	/// OBJECT = path/to/objfile (contains any number of objects) // this line can appear several times
	/// PRMFILE = path/to/prmFile
     Scenario(const std::string& filepath, double scaleEnglobing = 1.);
    ~Scenario();

	//bool SaveScenario(const std::string& filepath);

	Object::T_Object objects_;
    Object::T_Object contactObjects_;
    Object* point_;
	Model model_;
	SimplePRM* prm;
    ObjectDictionary objDictionnary;
};

} //namespace planner
#endif //_STRUCT_SCENARIO
