/**
* \file ParserObj.h
* \brief Utility functions for loading 3D objects
from an obj file.
* \author Steve T.
* \version 0.1
* \date 16/07/2014
*
*/
#ifndef _CLASS_ParserObj
#define _CLASS_ParserObj

#include <string>
#include <vector>

#include "Object.h"
#include "ObjectDictionary.h"

namespace planner
{

typedef std::vector<std::pair<std::size_t, std::size_t> > T_ObjId_Vert;
typedef std::vector<T_ObjId_Vert> T_verticeObject;

///  \brief Loads a set of Object from an obj files.
///  TODO: Only accepts triangles and quads objects.
///  \param filename path to the obj file to be loaded.
///  \param asOneObject if true obj file is considered one single object
///  \param return : a list of Object from the obj file.
Object::T_Object ParseObj(const std::string&  /*filename*/, const bool asOneObject = false, double scaleEnglobing = 1.);


///  \brief Loads a set of Object from an obj files and concatenates
///  them to a list given in parameters.
///  TODO: Only accepts triangles and quads objects.
///  \param filename path to the obj file to be loaded.
///  \param objects a T_Object at the end of which the new Object will be concatanated.
///  \param asOneObject if true obj file is considered one single object
///  \param return : a list of Object from the obj file.
void ParseObj(const std::string& /*filename*/, Object::T_Object& /*objects*/, const bool asOneObject = false, double scaleEnglobing = 1.);

///  \brief Loads a set of Object from an obj files and concatenates
///  them to a list given in parameters.
///  TODO: Only accepts triangles and quads objects.
///  \param filename path to the obj file to be loaded.
///  \param objects a T_Object at the end of which the new Object will be concatanated.
///  \param dic reference to an ObjectDictionary containing the ids and location of each point in the obj file.
///  \param asOneObject if true obj file is considered one single object
///  \param return : a list of Object from the obj file.
void ParseObj(const std::string& /*filename*/, Object::T_Object& /*objects*/, ObjectDictionary& /*dic*/, const bool asOneObject = false, double scaleEnglobing = 1.);
}//namespace planner;
#endif //_CLASS_ParserObj
