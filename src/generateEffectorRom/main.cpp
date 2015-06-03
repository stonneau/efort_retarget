#include "collision/ParserObj.h"
#include "collision/Object.h"
#include "collision/Collider.h"
#include "tools/MatrixDefs.h"
#include "prm/SimplePRM.h"
#include "prm/Scenario.h"
#include "prmpath/Robot.h"
#include "prmpath/sampling/Sample.h"
#include "prmpath/CompleteScenario.h"

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Geometry>
#ifdef WIN32
#include <windows.h>
#endif


using namespace std;
using namespace Eigen;
using namespace planner;

namespace
{
	Eigen::Vector3d Effectorcentroid(planner::Node* limb)
	{
		planner::Node* tmp = limb;
		while(!tmp->children.empty())
		{
			tmp = tmp->children.front();
		}
		Eigen::Vector3d res(Eigen::Vector3d::Zero());
        tmp = tmp->parent;
        return tmp->position;
	}

	void WriteSamplePosition(planner::sampling::Sample* sample, planner::Node* limb, std::stringstream& outstream)
	{
		planner::sampling::LoadSample(*sample, limb);
        Eigen::Vector3d pos = Effectorcentroid(limb); // TODO - limb->position;
		outstream << pos(0) << "," << pos(1) << "," << pos(2) <<'\n';
	}

	
	planner::Node* RootObj(planner::Node* limb)
	{
		planner::Node* node = limb;
		planner::Node* res(0);
		while(node)
		{			
			if(node->current) res = node;
			node = node->parent;
		}
		return res;
	}
}


int main(int argc, char *argv[])
{
    if(argc < 2 )
	{
		std::cout << "no input file" << std::endl;
		return -1;
	}
	std::string robotfile = argv[1];
	planner::CompleteScenario* scenario = planner::CompleteScenarioFromFile(robotfile);
	
    //planner::Node* root = RootObj((*scenario->limbs.begin()));
	std::vector<planner::sampling::T_Samples>::iterator sit = scenario->limbSamples.begin();
	for(std::vector<Node*>::iterator it = scenario->limbs.begin();
		it != scenario->limbs.end(); ++it, ++sit)
	{
		std::stringstream outstream;
		for(planner::sampling::T_Samples::iterator sample = (*sit).begin();
			sample != (*sit).end(); ++ sample)
		{
            WriteSamplePosition(*sample, (*it), outstream);
		}
		ofstream outfile;
		std::string outfilename = (*it)->tag + ".erom";
		outfile.open(outfilename.c_str());
		if (outfile.is_open())
		{
			outfile << outstream.rdbuf();
			outfile.close();
		}
		else
		{
			std::cout << "Can not open outfile " << outfilename << std::endl;
			return false;
		}
	}
	delete scenario;
    return 0;
}
