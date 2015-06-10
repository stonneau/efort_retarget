#include "CompleteScenario.h"
#include "prmpath/PostureSelection.h"
#include "prmpath/sampling/Sample.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>


#include "tools/Timer.h"

using namespace planner;

using namespace std;

State::State(const State *parent)
    :value(new planner::Robot(*parent->value))
    , stable(parent->stable)
    , contactLimbs(parent->contactLimbs)
    , contactLimbPositions(parent->contactLimbPositions)
    , contactLimbPositionsNormals(parent->contactLimbPositionsNormals)
{
    // NOTHING
}

CompleteScenario::CompleteScenario()
    : scenario(0)
    , robot(0)
    , from(0)
    , to(0)
    , spline(0)
{
    // NOTHING
}

CompleteScenario::~CompleteScenario()
{
    delete scenario;
    delete robot;
    delete from;
    delete to;
    delete spline;
    for(std::vector<planner::Robot*>::iterator it = completePath.begin();
        it != completePath.end(); ++it)
    {
        delete *it;
    }
}

namespace
{
void WriteNodeLine(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& position, std::stringstream& outstream)
{
    for(int i=0; i<3; ++i)
    {
        for(int j=0; j<3; ++j)
        {
            outstream << rotation(i,j) << " ";
        }
        outstream << position(i) << " ";
    }
    outstream << std::endl;
}
}

bool CompleteScenario::SavePath(const std::string& outfilename)
{
    //std::string outfilename ("../tests/entrance.path");
    std::stringstream outstream;
    outstream << "size " << (int)(path.size()) << std::endl;
    for(CT_Model::const_iterator it = path.begin();
        it!= path.end(); ++it)
    {
        WriteNodeLine((*it)->GetOrientation(),(*it)->GetPosition(), outstream);
    }
    ofstream outfile;
    outfile.open(outfilename.c_str());
    if (outfile.is_open())
    {
        outfile << outstream.rdbuf();
        outfile.close();
        return true;
    }
    else
    {
        std::cout << "Can not open outfile " << outfilename << std::endl;
        return false;
    }
}

namespace
{
 /*   File description:
    PRMSCENARIO file="path/to/scenario"
    ROBOT_SKELETON file="path/to/urdf"
    ROBOT_CONSTRAINTS file="path/to/joint_constraints"
    PATH_FROM matrix="a00 a01 a02 a03 ... a30 a31 a32 a33"
    PATH_TO matrix="a00 a01 a02 a03 ... a30 a31 a32 a33"
    LIMB joint_name="joint_name"
    ...
    LIMB joint_name="joint_name"
    NBSAMPLES N=""*/

    void PrintError(const std::string& item, const std::string& filename)
    {
        std::cout << "Missing description for item " << item << "in file" << filename << std::endl;
    }

    std::string ExtractQuotes(const std::string& line)
    {
        int quoteStart = line.find("\"");
        int quoteEnd = line.find("\"", quoteStart+1);
        return line.substr(quoteStart+1, quoteEnd - quoteStart -1);
    }

    std::vector<std::string> splitSpace(const std::string& s)
    {
        //Eclate une chane au niveau de ses espaces.
        std::vector<std::string> ret;
        std::string s1="";
        for(unsigned int i=0;i<s.size();i++)
        {
            if(s[i]==' '||i==s.size()-1)
            {
                if(i==s.size()-1)
                    s1+=s[i];
                if(s1!="") ret.push_back(s1);
                s1="";
            }
            else
                s1+=s[i];
        }
        return ret;
    }

    Eigen::Matrix4d ReadTransform(const std::string& line, bool& error)
    {
        Eigen::Matrix4d res;
        std::vector<std::string> matrix = splitSpace(line);
        if(matrix.size() != 16)
        {
            error = true;
            std::cout << "matrix does not contain 16 members" << std::endl;
        }
        for(int i =0; i<16; ++i)
        {
            char value [40];
            sscanf(matrix[i].c_str(),"%s", value);
            res(i/4, i%4) = strtod (value, NULL);
        }
        return res;
    }

    Eigen::Matrix3d Read3Transform(const std::string& line)
    {
        Eigen::Matrix3d res;
        std::vector<std::string> matrix = splitSpace(line);
        if(matrix.size() != 9)
        {
            std::cout << "matrix does not contain 9 members" << std::endl;
        }
        for(int i =0; i<9; ++i)
        {
            char value [20];
            sscanf(matrix[i].c_str(),"%s", value);
            res(i/3, i%3) = strtod (value, NULL);
        }
        return res;
    }

    planner::Sphere GetSphere(const std::string& data)
    {
        Eigen::Vector3d center;
        double radius;
        std::vector<std::string> coords = splitSpace(data);
        if(coords.size() != 4)
        {
            std::cout << "SPHERE needs 4 numbers" << std::endl;
        }
        for(int i =0; i<3; ++i)
        {
            char value [20];
            sscanf(coords[i].c_str(),"%s", value);
            center(i) = strtod (value, NULL);
        }
        char value [20];
        sscanf(coords[3].c_str(),"%s", value);
        radius = strtod (value, NULL);
        return planner::Sphere(center,radius);
    }

}




planner::CompleteScenario* planner::CompleteScenarioFromFile(const std::string& filename, double scaleEnglobing)
{
    CompleteScenario* cScenario = new CompleteScenario;
    cScenario->relocateEnglobing = false;
    bool scenario = false; bool skeleton = false; bool contraints = false;
    bool from = false; bool to = false; bool limb = false; bool initstate = false;
    bool samples = false; bool recordedstates=false;
    std::string statefilepath = "";
    std::ifstream myfile (filename);
    if (myfile.is_open())
    {
        std::string line;
        while (myfile.good())
        {
            getline(myfile, line);
            if(line.find("PRMSCENARIO file=") != string::npos)
            {
                cScenario->scenario = new planner::Scenario(ExtractQuotes(line), scaleEnglobing);
                if(cScenario->scenario )
                {
                    scenario = true;
                }
            }
            if(line.find("RELOCATE_ENGLOBING") != string::npos)
            {
                cScenario->relocateEnglobing = true;
            }
            if(line.find("ROBOT_SKELETON file=") != string::npos)
            {
                cScenario->robot = new planner::Robot(planner::LoadRobot(ExtractQuotes(line)));
                if(cScenario->robot )
                {
                    skeleton = true;
                }
            }
            if(line.find("ROBOT_CONSTRAINTS file=") != string::npos)
            {
                if(planner::LoadJointConstraints(*cScenario->robot, ExtractQuotes(line)))
                {
                    contraints = true;
                }
            }
            if(line.find("PATH_FROM matrix=") != string::npos && scenario)
            {
                Eigen::Matrix4d res = ReadTransform(ExtractQuotes(line), from);
                cScenario->from = new Model((cScenario->scenario->model_));
                cScenario->from->SetOrientation(res.block<3,3>(0,0));
                cScenario->from->SetPosition(res.block<3,1>(0,3));
                from = !from;
            }
            if(line.find("PATH_TO matrix=") != string::npos && scenario)
            {
                Eigen::Matrix4d res = ReadTransform(ExtractQuotes(line), to);
                cScenario->to = new Model((cScenario->scenario->model_));
                cScenario->to->SetOrientation(res.block<3,3>(0,0));
                cScenario->to->SetPosition(res.block<3,1>(0,3));
                to = !to;
            }
            if(line.find("STATES FILE=")!= string::npos)
            {
                recordedstates = true;
                statefilepath = ExtractQuotes(line);
            }
            if(line.find("CONSTANT_ROTATION matrix=") != string::npos && cScenario->robot)
            {
                Eigen::Matrix3d res = Read3Transform(ExtractQuotes(line));
                cScenario->robot->SetConstantRotation(res);
            }
            if(line.find("LIMB") != string::npos && cScenario->robot)
            {
                // decompose between LIMB and SPHERE
                size_t it = line.find("SPHERE");
                if(it== string::npos)
                {
                    std::cout << "no sphere for limb  "  << std::endl;
                    break;
                }
                std::string limbstring = ExtractQuotes(line);
                std::string sphereString = ExtractQuotes(line.substr(it));
                Node* res = planner::GetChild(cScenario->robot, limbstring);
                cScenario->limbRoms.push_back(GetSphere(sphereString));
                if(res)
                {
                    limb = true;
                    cScenario->limbs.push_back(res);
cScenario->limbspeed.push_back(1); // TODO HAVE ACTUAL SPEED
                }
                else
                {
                    std::cout << "no joint named " << ExtractQuotes(line) << std::endl;
                    break;
                }
            }
            if(line.find("NBSAMPLES")!= string::npos && limb)
            {
                char data[30];
                sscanf(ExtractQuotes(line).c_str(),"%s", data);
                int nb_samples = strtod(data, NULL);
                for(std::vector<Node*>::iterator it = cScenario->limbs.begin()
                    ; it!= cScenario->limbs.end(); ++it)
                {
                    cScenario->limbSamples.push_back(planner::sampling::GenerateSamples(*cScenario->robot, *it, nb_samples));
                }
                samples = true;
            }
            if(line.find("INITCONTACTS")!= string::npos && from)
            {
                std::vector<string> contacts = splitSpace(line);
                for(int i =1; i<contacts.size(); ++i)
                {
                    char data[10];
                    sscanf(contacts[i].c_str(),"%s", data);
                    cScenario->initstate.contactLimbs.push_back((int)strtod(data, NULL));
                }
            }
        }
        myfile.close();
    }
    else
    {
        std::cout << "can not found complete scenario file" << filename << std::endl;
    }

    if(scenario && skeleton && contraints && ((to && from) || recordedstates) && limb && samples )
    {
        // order correctly roms relativelty to limbs
        std::vector<std::string> limbnames;
        for(std::vector<planner::Node*>::const_iterator cit = cScenario->limbs.begin();
            cit != cScenario->limbs.end(); ++cit)
        {
            limbnames.push_back((*cit)->tag);
        }
        //cScenario->scenario->model_.SortEnglobingByName(limbnames);

        if(!recordedstates)
        {
Timer timer; timer.Start();
std::cout << " path request timer" << std::endl;
        cScenario->path = cScenario->scenario->prm->GetPath(*(cScenario->from), *(cScenario->to), 10, true, true);
std::cout << " path request end timer, time :" <<  timer.GetTime() << std::endl;
timer.Stop();
        }
        else
        {
            cScenario->states =planner::LoadStates(statefilepath, cScenario->robot);
        }
        if(cScenario->path.empty())
        {
            cScenario->path.push_back(cScenario->from);
            cScenario->path.push_back(cScenario->to);
        }
        // init first position
        //cScenario->robot->SetConfiguration(cScenario->from);
        Eigen::Vector3d direction = cScenario->path[1]->GetPosition() - cScenario->path[0]->GetPosition();
        if(direction.norm() > 0)
        {
            direction.normalize();
            for(std::vector<int>::const_iterator cit = cScenario->initstate.contactLimbs.begin();
                cit != cScenario->initstate.contactLimbs.end(); ++cit)
            {
                // assign contacts
                planner::sampling::Sample* candidate
                        = planner::GetPosturesInContact(*cScenario->robot, cScenario->limbs[*cit],
                                                        cScenario->limbSamples[*cit], cScenario->scenario->objects_, direction, *cScenario);
                /*if(candidate)
                {
                    planner::sampling::LoadSample(*candidate, cScenario->limbs[*cit]);
                }*/
            }
        }
        cScenario->initstate.value= new Robot(*cScenario->robot);
        return cScenario;
    }
    else
    {
        if(!scenario)
        {
            PrintError("PRMSCENARIO", filename);
        }
        if(!skeleton)
        {
            PrintError("ROBOT_SKELETON", filename);
        }
        if(!contraints)
        {
            PrintError("ROBOT_CONSTRAINTS", filename);
        }
        if(!to)
        {
            PrintError("PATH_FROM", filename);
        }
        if(!from)
        {
            PrintError("PATH_TO", filename);
        }
        if(!limb)
        {
            PrintError("LIMB", filename);
        }
        if(!samples)
        {
            PrintError("NBSAMPLES", filename);
        }
        return cScenario;
    }
}

/*
Structure:
nbContacts contactLimb1 ... contactLimbnbC contactLimbPositions1 ... N contactLimbPositionsNormals1 .. N \\
toujours sur la même ligne joint robots tous les robots, matrix rot, vector pos, plus values
*/

namespace
{

    void SaveVector(const Eigen::Vector3d& vec, std::stringstream& bvhStr )
    {
        for(int i=0; i<3; ++i)
        {
            bvhStr << vec(i) << " ";
        }
    }

    void SaveMatrix(const Eigen::Matrix3d& matrix, std::stringstream& bvhStr )
    {
        for(int i=0; i<3; ++i)
        {
            for(int j=0; j<3; ++j)
            {
                bvhStr << matrix(i,j) << " ";
            }
        }
    }

    void SaveNodeRec(const planner::Node* node, std::stringstream& bvhStr)
    {
        bvhStr << node->value << " ";
        for(std::vector<planner::Node*>::const_iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
            SaveNodeRec(*it, bvhStr);
        }
    }

    void SaveRobot(const planner::Robot& robot, std::stringstream& bvhStr)
    {
        //planner::sampling::RobotSample rs(robot);
        // push matrix
        SaveMatrix(robot.currentRotation, bvhStr);
        SaveVector(robot.currentPosition, bvhStr);
        SaveNodeRec(robot.node, bvhStr);
    }

    void SaveOneState(const planner::State& state, std::stringstream& bvhStr)
    {
        int nbC = state.contactLimbs.size();
        bvhStr << nbC << " ";
        for(int i=0; i < nbC; ++i)
        {
            bvhStr << state.contactLimbs[i] << " ";
        }
        for(int i=0; i < nbC; ++i)
        {
            SaveVector(state.contactLimbPositions[i], bvhStr);
        }
        for(int i=0; i < nbC; ++i)
        {
            SaveVector(state.contactLimbPositionsNormals[i], bvhStr);
        }
        //bvhStr << id << pos << normals << state.stable << " ";
        SaveRobot(*(state.value), bvhStr);
    }
}

bool planner::SaveStates(const T_State& states, const std::string& outfilename)
{
    std::stringstream sstream;
    for(planner::T_State::const_iterator it = states.begin(); it != states.end(); ++it)
    {
        SaveOneState((*it), sstream);
        sstream << "\n";
    }
    ofstream myfile;
    myfile.open (outfilename.c_str());
    if (myfile.is_open())
    {
        myfile << sstream.rdbuf();
        myfile.close();
        return true;
    }
    return false;
}

bool planner::ExportContactStates(const T_State& states, std::vector<planner::Node*>& limbs, const string &outfilename)
{
    // 0 : no contact : 1 : contact maintainance ; 2 : contact maintained
    std::stringstream sstream;
    planner::T_State::const_iterator it = states.begin();
    planner::T_State::const_iterator it2 = states.begin();
    std:vector<std::vector<int> > values;
    int nbLimbs = limbs.size();
    for(int i =0; i< nbLimbs; ++i)
    {
        std::vector<int> limbValues;
        values.push_back(limbValues);
    }
    for(; it != states.end(); ++it)
    {
        for(int i =0; i< nbLimbs; ++i)
        {
            int value = 0;
            Eigen::Vector3d targetprevious, target, normal, previousNormal;
            if((*it)->InContact(i, target, normal))
            {
                ++value;
                if(it != states.begin())
                {
                    it2 = it; --it2;
                    if((*it2)->InContact(i, targetprevious, previousNormal) && (target - targetprevious).norm() < 0.01 )
                    {
                        ++value;
                    }
                }
            }
            values[i].push_back(value);
        }
    }
    // first line denotes step number
    sstream << " ";
    for(int i = 0; i< states.size(); ++i)
    {
        sstream << "\t" << i;
    }
    sstream << " \n";
    //then write values for each limbs
    for(int i =0; i< nbLimbs; ++i)
    {
        sstream << limbs[i]->tag;
        for(std::vector<int>::const_iterator cit = values[i].begin(); cit != values[i].end();
            ++cit)
        {
            sstream << "\t" << (*cit) ;
        }
        sstream << " \n";
    }

    ofstream myfile;
    myfile.open (outfilename.c_str());
    if (myfile.is_open())
    {
        myfile << sstream.rdbuf();
        myfile.close();
        return true;
    }
    return false;
}


namespace
{
    double dfs(const std::string& s)
    {
        char value [20];
        sscanf(s.c_str(),"%s", value);
        return strtod (value, NULL);
    }

    Eigen::Vector3d readvector(const std::vector<std::string>& values, int id)
    {
         Eigen::Vector3d res;
        for(int i =0; i< 3; ++i)
        {
            res(i) = dfs(values[id +i]);
        }
        return res;
    }

    Eigen::Matrix3d readmatrix3(const std::vector<std::string>& values, int id)
    {
        Eigen::Matrix3d res;
        for(int i =0; i< 3; ++i)
        {
            for(int j =0; j< 3; ++j)
            {
                res(i,j) = dfs(values[id +i*3 + j]);
            }
        }
        return res;
    }

    void LoadNodeRec(planner::Node* node, const std::vector<std::string>& values, int& id)
    {
        node->value = dfs(values[id]);++id;
        for(std::vector<planner::Node*>::const_iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
            LoadNodeRec(*it, values, id);
        }
    }


    void LoadState(const planner::Robot* model, T_State& states, const std::string& line)
    {
        State* res = new State;
        res->value = new planner::Robot(*model);
        std::vector<std::string> values = splitSpace(line);
        // d abord nb contacts
        int nbC = (int)(dfs(values[0]));
        int c_id = 1; // current index
        // read contact index
        for(int i = 0; i!= nbC; ++i)
        {
            res->contactLimbs.push_back(dfs(values[c_id]));
            ++c_id;
        }
        // read positions of contacts
        for(int i = 0; i!= nbC; ++i)
        {
            res->contactLimbPositions.push_back(readvector(values, c_id));
            c_id += 3;
        }
        // read positions of contacts normals
        for(int i = 0; i!= nbC; ++i)
        {
            res->contactLimbPositionsNormals.push_back(readvector(values, c_id));
            c_id += 3;
        }
        // todo read balance
        // read robot rotation and position
        res->value->currentRotation = readmatrix3(values, c_id);
        c_id += 9;
        res->value->SetPosition(readvector(values, c_id)); c_id += 3;
        LoadNodeRec(res->value->node,values, c_id);
        res->value->node->Update();
        states.push_back(res);
    }
}


T_State planner::LoadStates(const std::string& infilename, const planner::Robot* model)
{
    T_State res;
    std::ifstream myfile (infilename);
    if (myfile.is_open())
    {
        std::string line;
        while (myfile.good())
        {
            getline(myfile, line);
            if(!line.empty())
            {
                LoadState(model, res, line);
            }
        }
        myfile.close();
    }
    else
    {
        std::cout << "can not find state file file " << infilename << std::endl;
    }
    return res;
}
