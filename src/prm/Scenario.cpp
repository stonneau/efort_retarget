/**
* \file Scenario.cpp
* \brief A PRM concrete implementation
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/

#include "Scenario.h"

#include "collision/ParserObj.h"

#include <fstream>
#include <iostream>
#include <string>

#include "tools/TimerPerf.h"
#include "tools/Timer.h"

using namespace std;
using namespace planner;



Scenario::Scenario(const std::string& filepath, double scaleEnglobing)
{
    ifstream myfile (filepath);
    string line;
    if (myfile.is_open())
    {
        Eigen::Matrix4d tmp;
        bool inConnexions = false;
		bool generating = false;
		bool loading = false;
		bool modelFound = false;
		bool visibility = false;
        bool point = false;
		std::string prmfile("");
		int size = -1; int neighbours = -1; float neighbourDistance =-1.;
        while (myfile.good())
        {
            getline(myfile, line);
			if(line.find("MODEL = ") == 0 && !modelFound)
			{
				modelFound = true;
				line = line.substr(8);
				std::string englobed = line.substr(0, line.find(" "));
				std::string englobing = line.substr(line.find(" ") +1);
                Object::T_Object modelobjs = ParseObj(englobed,false, scaleEnglobing);
                planner::ParseObj(englobing, model_.englobing);
                if(modelobjs.size() != 1)
				{
					std::string error("ERROR: MODEL FILE CONTAINS MORE THAN TWO OBJECTS ");
					error += filepath;
					throw ScenarioException(error);
				}
				model_.englobed  = modelobjs[0];
                //model_.englobing = modelobjs[1];
			}
			else if(line.find("OBJECT = ") == 0)
			{
                ParseObj(line.substr(9), objects_, objDictionnary);
			}
            else if(line.find("OBJECTCONTACT = ") == 0)
            {
                ParseObj(line.substr(16), contactObjects_);
            }
			else if(line.find("PRMFILE = ") == 0 && !loading)
			{
				loading = true;
				prmfile = line.substr(10);
			}
			else if(line.find("SIZE = ") == 0 && !loading)
			{
				generating = true;
				std::string::size_type sz, sz2;     // alias of size_t
				line = line.substr(7);
                size = std::stoi (line,&sz);
				sz2 = sz;
                neighbourDistance = std::stof (line.substr(sz), &sz);
                neighbours = std::stoi (line.substr(sz2 + sz));
			}
			else if(line.find("VISIBILITY") == 0)
			{
				visibility = true;
                std::cout << "VISIBILITY" << std::endl;
			}
		}
		if((generating && loading) || !(generating ||loading))
		{
			std::string error("ERROR: bad scenario file; Found instructions to generate and load prm conflicting ");
			error += filepath;
			throw ScenarioException(error);
		}
		else if(generating)
		{
            if(contactObjects_.empty())
            {
                TimerPerf tp;
                Timer timer; timer.Start();
std::cout << " begining prm construction timer" << std::endl;
                prm = new SimplePRM(model_, objects_, neighbourDistance, size, neighbours, visibility);
                timer.Stop();
std::cout << " prm construction timer, time :" << tp.elapsedTime() << "\n mon timer" << timer.GetTime()  <<  std::endl;
            }
            else
            {
                prm = new SimplePRM(model_, contactObjects_, objects_, neighbourDistance, size, neighbours, visibility);
            }
		}
		else
		{
            if(contactObjects_.empty())
            {
                prm = planner::LoadPRM(prmfile, objects_, model_);
            }
            else
            {
                prm = planner::LoadPRM(prmfile, contactObjects_, objects_, model_);
            }
		}
        if(!point)
        {
            point_ = ParseObj("../rami/misc/point.obj").front();
        }
	}
	else
	{
		std::string error("ERROR: can not find scenario file ");
		error += filepath;
		throw ScenarioException(error);
    }
}

Scenario::~Scenario()
{
    for(Object::T_Object::iterator it = objects_.begin();
        it != objects_.end(); ++it)
    {
        delete *it;
    }
    delete point_;
	delete prm;
}
