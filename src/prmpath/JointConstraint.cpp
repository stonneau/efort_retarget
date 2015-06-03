#include "JointConstraint.h"
#include "prmpath/ROM.h"

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace planner;

namespace
{
    std::string ExtractQuotes(const std::string& line, int& index)
    {
        int quoteStart = line.find("\"", index);
        int quoteEnd = line.find("\"", quoteStart+1);
        index = quoteEnd +1;
        return line.substr(quoteStart+1, quoteEnd - quoteStart -1);
    }

    std::vector<std::string> ExtractQuotesRec(const std::string& line, int n)
    {
        int index = 0;
        std::vector<string> res;
        for(int i=0; i<n; ++i)
        {
            res.push_back(ExtractQuotes(line, index));
        }
        return res;
    }

    Eigen::Vector3d VectorFromString(const std::string& line)
    {
        char x[255],y[255],z[255];
        sscanf(line.c_str(),"%s %s %s",x,y,z);
        return Eigen::Vector3d(strtod (x, NULL), strtod(y, NULL), strtod(z, NULL));
    }

    bool AssignJointLimits(const std::string line, Robot& robot)
    {
        vector<string> data = ExtractQuotesRec(line,4);
        if(data.size()!=4) return false;
        Node* node = planner::GetChild(robot.node, data[0]);
        if(node)
        {
            char min[10], max[10], defaut[10];
            sscanf(data[1].c_str(),"%s",min);
            sscanf(data[2].c_str(),"%s",max);
            sscanf(data[3].c_str(),"%s",defaut);
            node->maxAngleValue = strtod (max, NULL) * M_PI / 180;
            node->minAngleValue = strtod (min, NULL) * M_PI / 180;
            node->defaultAngleValue = strtod (defaut, NULL) * M_PI / 180;
            node->value = node->defaultAngleValue;
            return true;
        }
        else
        {
            std::cout << "wrong joint limit description, can't find name " << data[0] << std::endl;
            return false;
        }
    }

    bool CreateGroup(const std::string line, Robot& robot)
    {
        vector<string> data = ExtractQuotesRec(line,4);
        if(data.size()!=4) return false;
        try
        {
            rom::ROM rom = planner::rom::ROMFromFile(data[3]);
            planner::NodeRom* group = new planner::NodeRom(rom);
            group->groupId = robot.roms.size();
            for(int i =0; i<3; ++i)
            {
                Node* node = planner::GetChild(robot.node, data[i]);
                if(node)
                {
                    node->romId = group->groupId;
                    group->group[i] = node;
                }
                else
                {
                    std::cout << "wrong joint limit description, can't find name " << data[i] << std::endl;
                    return false;
                }
            }
            robot.roms.push_back(group);
            return true;
        }
        catch (rom::ROMException& e)
        {
            cout << e.what() << std::endl;
            return false;
        }
    }

    bool CreateNormal(const std::string line, Robot& robot)
    {
        vector<string> data = ExtractQuotesRec(line,2);
        if(data.size()!=2) return false;
        Node* node = planner::GetChild(robot.node, data[0]);
        if(node)
        {
            node->effectorNormal = VectorFromString(data[1]);
            return true;
        }
        return false;
    }
}

bool planner::LoadJointConstraints(Robot& robot, const std::string& filename)
{
    /**
    File description:
    constraint joint_name="JOINT_NAME" min="" max="" default=""
    ...
    rom joint1="JOINT_NAME" joint2="JOINT_NAME" joint3="JOINT_NAME" romfile =""
    */
    bool error = false;
    std::ifstream myfile (filename);
    if (myfile.is_open())
    {
        int line_n = 0;
        std::string line;
        while (myfile.good())
        {
            getline(myfile, line);
            ++ line_n;
            if(line.find("constraint") != std::string::npos)
            {
                if(!AssignJointLimits(line, robot))
                {
                    error = true;
                    std::cout << "There were error in joint limit description file " << filename << " at line " << line_n << ": " << line <<  std::endl;
                }
            }
            else if(line.find("rom") != std::string::npos)
            {
                if(!CreateGroup(line, robot))
                {
                    error = true;
                    std::cout << "There were error in joint limit description file " << filename << " at line " << line_n << ": " << line <<  std::endl;
                }
            }
            else if(line.find("normal") != std::string::npos)
            {
                if(!CreateNormal(line, robot))
                {
                    error = true;
                    std::cout << "There were error in joint limit description file " << filename << " at line " << line_n << ": " << line <<  std::endl;
                }
            }
        }
        myfile.close();
    }
    else
    {
        error = true;
        std::cout << "can not found joint limit description file " << filename << std::endl;
    }
    return !error;
}
