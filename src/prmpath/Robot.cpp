#include "Robot.h"
#include "collision/ParserObj.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <map>
#include <Eigen/Geometry>
using namespace planner;
Node::Node(const int id)
    : current(0)
    , value(0)
    , tag("")
    , parent(0)
    , toParentRotation(Eigen::Matrix3d::Identity())
    , toLocalRotation(Eigen::Matrix3d::Identity())
    , toWorldRotation(Eigen::Matrix3d::Identity())
    , position(0,0,0)
    , minAngleValue(-2*M_PI)
    , maxAngleValue(2*M_PI)
    , defaultAngleValue(0)
    , id(id)
    , axis(0,0,1)
    , offset(0,0,0)
    , effectorNormal(0,0,1)
    , permanentRotation(Eigen::Matrix3d::Identity())
    , romId(-1)
{
    //NOTHING
}
Node::Node(const Node& clone)
    : current(0)
    , value(clone.value)
    , tag(clone.tag)
    , parent(0)
    , toParentRotation(clone.toParentRotation)
    , toLocalRotation(clone.toLocalRotation)
    , toWorldRotation(clone.toWorldRotation)
    , position(clone.position)
    , minAngleValue(clone.minAngleValue)
    , maxAngleValue(clone.maxAngleValue)
    , defaultAngleValue(clone.defaultAngleValue)
    , id(clone.id)
    , axis(clone.axis)
    , offset(clone.offset)
    , effectorNormal(clone.effectorNormal)
    , permanentRotation(clone.permanentRotation)
    , romId(clone.romId)
{
    if(clone.current)
    {
        current = new Object(*(clone.current));
    }
    for(std::vector<Node*>::const_iterator cit = clone.children.begin();
        cit != clone.children.end(); ++cit)
    {
        Node * child = new Node(*(*cit));
        child->parent = this;
        children.push_back(child);
    }
}
Node::~Node()
{
    if(current) delete current;
}
void Node::free()
{
    for(std::vector<Node*>::iterator cit = children.begin();
        cit != children.end(); ++cit)
    {
        (*cit)->free();
    }
    delete this;
}
void Node::Update()
{
    toWorldRotation = /*permanentRotation **/ Eigen::AngleAxisd(value, axis).matrix();
    toParentRotation = toWorldRotation;
    if(parent)
    {
        toWorldRotation = parent->toWorldRotation * toWorldRotation;
        toParentRotation = parent->toLocalRotation * toWorldRotation;
        position = parent->toWorldRotation * offset;
        position += parent->position;
    }
    else
    {
        position = toWorldRotation * offset;
    }
    toLocalRotation = toWorldRotation;
    toLocalRotation.inverse();
    if(current)
    {
        current->SetOrientation(toWorldRotation);
        current->SetPosition(position);
    }
    for(std::vector<Node*>::iterator cit = children.begin();
        cit != children.end(); ++cit)
    {
        (*cit)->Update();
    }
}
void Node::SetRotation(double value)
{
    this->value = value;
    Update();
}
void Node::Translate(const Eigen::Vector3d& delta)
{
    offset += delta;
    Update();
}
void Node::SetTranslation(const Eigen::Vector3d& position)
{
    offset = position;
    Update();
}

int planner::GetNumChildren(const Node* node)
{
    int tmp = node->children.empty() ? 0 : 1;
    for(std::vector<Node*>::const_iterator cit = node->children.begin();
        cit != node->children.end(); ++cit)
    {
        tmp += planner::GetNumChildren(*cit);
    }
    return tmp;
}

int planner::GetNumNodes(const Node* node)
{
    int tmp = 1;
    bool ok =  node->tag.find("effector") != std::string::npos
            || node->tag.find("foot_x") != std::string::npos;
    //if(!ok)
    {
        for(std::vector<Node*>::const_iterator cit = node->children.begin();
            cit != node->children.end(); ++cit)
        {
            tmp += planner::GetNumNodes(*cit);
        }
    }
    return tmp;
}

namespace
{
    void GetEffectorsRec(Node* node, std::vector<Node*>& effectors, bool onePerLimb)
    {
        bool ok = (node->children.empty() && !onePerLimb) || node->tag.find("effector") != std::string::npos
                || (node->tag.find("Hand_x_joint") != std::string::npos && node->children.size() == 2)
                || node->tag.find("foot_x") != std::string::npos || node->tag.find("endsite") != std::string::npos;
        if(ok)
        {
            effectors.push_back(node);
        }
        else
        {
            for(std::vector<Node*>::iterator it = node->children.begin();
                it != node->children.end(); ++it)
            {
                GetEffectorsRec(*it, effectors, onePerLimb);
            }
        }
    }

    void AsVectorRec(Node* node, std::vector<Node*>& effectors)
    {
        Eigen::Vector3d zero = Eigen::Vector3d::Zero();
        if(node->offset != zero)
        {
            while(node->children.size() == 1 && node->children[0]->offset ==zero)
            {
                node = node->children[0];
            }
        }
        effectors.push_back(node);
       /* bool ok =  node->tag.find("effector") != std::string::npos
                || node->tag.find("foot_x") != std::string::npos;
        //if(ok) return;*/
        for(std::vector<Node*>::iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
            AsVectorRec(*it, effectors);
        }
    }
}

std::vector<Node*> planner::GetEffectors(Node* node, bool onePerLimb)
{
    std::vector<Node*> effectors;
    GetEffectorsRec(node, effectors, onePerLimb);
    return effectors;
}

std::vector<Node*> planner::AsVector(Node* node)
{
    std::vector<Node*> effectors;
    for(int i =0; i < 2; ++i)
    {
        node = node->children[0]; // removing rotation
    }
    for(std::vector<Node*>::iterator it = node->children.begin();
        it != node->children.end(); ++it)
    {
        AsVectorRec(*it, effectors);
    }
    return effectors;
}

planner::Node* planner::GetChild(Node* node, const std::string& tag)
{
    if(node->tag == tag) return node;
    planner::Node* res;
    for(std::vector<Node*>::iterator cit = node->children.begin();
        cit != node->children.end(); ++cit)
    {
        res = GetChild((*cit), tag);
        if(res) return res;
    }
    return 0;
}
planner::Node* planner::GetChild(Node* node, const int id)
{
    if(! node)
    {
        bool tg = false;
    }
    if(node->id == id) return node;
    planner::Node* res;
    for(std::vector<Node*>::iterator cit = node->children.begin();
        cit != node->children.end(); ++cit)
    {
        res = GetChild((*cit), id);
        if(res) return res;
    }
    return 0;
}
planner::Node* planner::GetChild(Robot* robot, const std::string& tag)
{
    return GetChild(robot->node, tag);
}
planner::Node* planner::GetChild(Robot* robot, const int id)
{
    return GetChild(robot->node, id);
}

namespace
{
    void ZeroRec(planner::Node* node)
    {
        node->value = 0;
        for(std::vector<Node*>::iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
           ZeroRec(*it);
        }
    }
}

void planner::Zero(Robot* robot)
{
    ZeroRec(robot->node);
    robot->SetRotation(Eigen::Matrix3d::Zero(),false);
    robot->currentPosition = Eigen::Vector3d::Zero();
    robot->node->offset = Eigen::Vector3d::Zero();
    robot->node->Update();
    //robot->SetPosition(Eigen::Vector3d::Zero(),true);
}

Robot::Robot(Node* root)
    : node(root)
    , constantRotation(Eigen::Matrix3d::Identity())
    , currentRotation(Eigen::Matrix3d::Identity())
    , currentPosition(Eigen::Vector3d::Zero())
{
    // NOTHING
}


Robot::Robot(const Robot& clone)
    : node(new Node (*clone.node))
    , constantRotation(clone.constantRotation)
    , currentRotation(clone.currentRotation)
    , currentPosition(clone.currentPosition)
{
    for(std::vector<NodeRom*>::const_iterator cit = clone.roms.begin();
        cit != clone.roms.end(); ++cit)
    {
        NodeRom* rom = new NodeRom((*cit)->rom);
        rom->groupId = (*cit)->groupId;
        for(int i=0; i<3; ++i)
        {
            rom->group[i] = planner::GetChild(node, (*cit)->group[i]->id);
        }
        roms.push_back(rom);
    }
    node->Update();
}

Robot::~Robot()
{
    node->free();
    for(std::vector<NodeRom*>::iterator it = roms.begin();
        it != roms.end(); ++it)
    {
        delete (*it);
    }
}
void Robot::SetConfiguration(const planner::Object* object)
{
    SetRotation(object->GetOrientation(), true);
    SetPosition(object->GetPosition(), true);
}
void Robot::SetRotation(const Eigen::Matrix3d& rotation, bool update)
{
    currentRotation = rotation * constantRotation;
    Eigen::Vector3d ea = currentRotation.eulerAngles(2,1,0);
    Node* current = node; // first node is translation
    for(int i =0; i<3; ++i)
    {
        current = current->children[0];
        current->value = ea[i];
    }
    if(update) node->Update();
}
void Robot::SetFullRotation(const Eigen::Matrix3d& rotation, bool update)
{
    currentRotation = rotation;
    Eigen::Vector3d ea = currentRotation.eulerAngles(2,1,0);
    Node* current = node; // first node is translation
    for(int i =0; i<3; ++i)
    {
        current = current->children[0];
        current->value = ea[i];
    }
    if(update) node->Update();
}
void Robot::SetConstantRotation(const Eigen::Matrix3d& rotation)
{
    constantRotation = rotation;
    currentRotation = currentRotation * constantRotation;
    Eigen::Vector3d ea = rotation.eulerAngles(2,1,0);
    Node* current = node; // first node is translation
    for(int i =0; i<3; ++i)
    {
        current = current->children[0];
        current->value = ea[i];
    }
    node->Update();
}
void Robot::SetPosition(const Eigen::Vector3d& position, bool update)
{
    // find first object
    bool found = false;
    Node* tmp = node;
    Eigen::Vector3d pos = position;
    while (!(found || tmp->children.empty()))
    {
        if(tmp->current)
        {
            found = true;
            pos = position - (tmp->current->GetPosition() - node->position);
        }
        tmp = tmp->children.front();
    }

    currentPosition = position;
    node->offset = currentPosition;
    if(update) node->Update();
}
void Robot::Translate(const Eigen::Vector3d& delta, bool update)
{
    currentPosition += delta;
    node->offset = currentPosition;
    if(update) node->Update();
}
namespace
{
struct Joint;
struct Link
{
    Link()
        : object(0)
    {
        // nothing
    }
    ~Link()
    {
        // nothing
    }
    planner::Object* object;
    std::string name;
    std::vector<Joint*> children;
};
struct Joint
{
    Joint()
        : lower(0)
        , upper(0)
        , parentLink(0)
        , childLink("")
    {
        // nothing
    }
    ~Joint()
    {
        // nothing
    }
    Eigen::Vector3d axis;
    Eigen::Vector3d offset;
    Eigen::Vector3d rpy;
    double lower;
    double upper;
    std::string name;
    Link* parentLink;
    std::string childLink;
    std::string type;
};
std::string ExtractQuotes(const std::string& line)
{
    int quoteStart = line.find("\"");
    int quoteEnd = line.find("\"", quoteStart+1);
    return line.substr(quoteStart+1, quoteEnd - quoteStart -1);
}
std::string ExtractType(const std::string& line)
{
    int quoteStart = line.find("\"");
    int quoteEnd = line.find("\"", quoteStart +1);
    quoteStart = line.find("\"", quoteEnd+1);
    quoteEnd = line.find("\"", quoteStart+1);
    return line.substr(quoteStart+1, quoteEnd - quoteStart -1);
}
Eigen::Vector3d VectorFromString(const std::string& line)
{
    char x[255],y[255],z[255];
    sscanf(line.c_str(),"%s %s %s",x,y,z);
    return Eigen::Vector3d(strtod (x, NULL), strtod(y, NULL), strtod(z, NULL));
}
double ExtractBound(const std::string& line, const bool isLower)
{
    // skiiping rpy
    int quoteStart = line.find("\"");
    int quoteEnd = line.find("\"", quoteStart +1);
    if(isLower && line.substr(0, quoteEnd).find("lower") != std::string::npos)
    {
        // NOTHING
    }
    else
    {
        quoteStart = line.find("\"", quoteEnd+1);
        quoteEnd = line.find("\"", quoteStart+1);
    }
    char x[255];
    std::string res =line.substr(quoteStart+1, quoteEnd - quoteStart -1) ;
    sscanf(res.c_str(),"%s",x);
    return strtod (x, NULL);
}
Eigen::Vector3d ExtractOffset(const std::string& line)
{
    // skiiping rpy
    int quoteStart = line.find("\"");
    int quoteEnd = line.find("\"", quoteStart +1);
    if(line.substr(0, quoteEnd).find("xyz") != std::string::npos)
    {
        return VectorFromString(line.substr(quoteStart+1, quoteEnd - quoteStart -1));
    }
    else
    {
        quoteStart = line.find("\"", quoteEnd+1);
        quoteEnd = line.find("\"", quoteStart+1);
        return VectorFromString(line.substr(quoteStart+1, quoteEnd - quoteStart -1));
    }
    /*for(int i =0; i<3; ++i)
    {
        joint->offset[i] = res(i);
    }*/
}
Eigen::Vector3d ExtractRpy(const std::string& line)
{
    // skiiping xyz
    int quoteStart = line.find("\"");
    int quoteEnd = line.find("\"", quoteStart +1);
    if(line.substr(0, quoteEnd).find("rpy") != std::string::npos)
    {
        return VectorFromString(line.substr(quoteStart+1, quoteEnd - quoteStart -1));
    }
    else
    {
        quoteStart = line.find("\"", quoteEnd+1);
        quoteEnd = line.find("\"", quoteStart+1);
        return VectorFromString(line.substr(quoteStart+1, quoteEnd - quoteStart -1));
    }
}
void ReadLink(const std::string& firstline, std::ifstream& file, std::map<std::string, Link*>& links)
{
    std::string objpath;
    Link* lk = new Link();
    lk->name = ExtractQuotes(firstline);
    bool foundMesh(false);
    std::string line;
    if(firstline.find("<link name") != std::string::npos && firstline.find("/>") == std::string::npos)
    {
        while (line.find("</link>") == std::string::npos && file.good())
        {
            getline(file, line);
            if(line.find("<mesh filename") != std::string::npos)
            {
                foundMesh = true;
                objpath = ExtractQuotes(line);
            }
        }
    }
    if(foundMesh)
    {
        Object::T_Object object = ParseObj(objpath, true);
        if(object.size() >0)
        {
            lk->object = object[0];
        }
    }
    if(links.find(lk->name) != links.end())
    {
        std::cout << "error in urdf file link redefinition" << lk->name << std::endl;
    }
    else
    {
        links.insert(std::make_pair(lk->name, lk));
    }
}
void ReadJoint(const std::string& firstline, std::ifstream& file, std::map<std::string, Joint*>& joints, std::map<std::string, Link*>& links)
{
    std::string name = ExtractQuotes(firstline);
    Joint * joint = new Joint();
    joint->name = name;
    if(firstline.find("<joint name") != std::string::npos && firstline.find("/>") == std::string::npos)
    {
        // get type
        joint->type = ExtractType(firstline);
        std::string line;
        while (line.find("</joint>") == std::string::npos && file.good())
        {
            getline(file, line);
            if(line.find("<axis") != std::string::npos)
            {
                joint->axis = VectorFromString(ExtractQuotes(line));
            }            
            if(line.find("<limit") != std::string::npos)
            {
                joint->lower = ExtractBound(line, true);
                joint->upper = ExtractBound(line, false);
            }
            if(line.find("<origin") != std::string::npos)
            {
                joint->offset =ExtractOffset(line); // * 1.2;
                joint->rpy =ExtractRpy(line);
              //  std::cout << "joint : " << name << "\n" << joint->offset(0) <<" "<<
              //             joint->offset(1) <<" "<< joint->offset(2)  << std::endl;
            }
            if(line.find("<parent link") != std::string::npos)
            {
                std::map<std::string, Link*>::iterator it = links.find(ExtractQuotes(line));
                if(it == links.end())
                {
                    std::cout << "-error in urdf file missing parent link " << ExtractQuotes(line) << std::endl;
                }
                else
                {
                    joint->parentLink = it->second;
                    it->second->children.push_back(joint);
                }
            }
            if(line.find("<child link") != std::string::npos)
            {
                joint->childLink = ExtractQuotes(line);
            }
        }
    }
    if(joints.find(name) != joints.end())
    {
        std::cout << "error in urdf file joint redefinition" << name << std::endl;
    }
    else
    {
        joints.insert(std::make_pair(name, joint));
    }
}
void MakeNodeRec(Node* node, Joint* current, std::map<std::string, Link*>& links, int& id)
{
    Link* next= links[current->childLink];
    if(current->type == "revolute")
    {
        Node* res = new Node(id++);
        res->axis = current->axis;
        res->tag = current->name;
        res->offset = current->offset;
        res->minAngleValue = current->lower;
        res->maxAngleValue = current->upper;
        res->permanentRotation = Eigen::AngleAxisd(current->rpy[0],  Eigen::Vector3d::UnitX())
                 *  Eigen::AngleAxisd(current->rpy[1],  Eigen::Vector3d::UnitY())
                 *  Eigen::AngleAxisd(current->rpy[2],  Eigen::Vector3d::UnitZ());
        res->parent = node;
        if(next->object)
            res->current = next->object;
        node->children.push_back(res);
        node = res;
    }
    for(std::vector<Joint*>::iterator it = next->children.begin();
        it != next->children.end(); ++it)
    {
        MakeNodeRec(node, *it, links, id);
    }
}
Node* MakeNode(Link* roots, std::map<std::string, Link*>& links)
{
    //create root
    int id = 3;
    Node* res = new Node(id++);
    if(roots->object) res->current = roots->object;
    for(std::vector<Joint*>::iterator it = roots->children.begin();
        it != roots->children.end(); ++it)
    {
        MakeNodeRec(res, *it, links, id);
    }
    // create 3 dummy nodes for rotation + 1 for translation
    // make X
    {
        res->tag = "base_revolute_joint_x";
        res->axis = Eigen::Vector3d::UnitX();
        res->offset = Eigen::Vector3d::Zero();
    }
    Node* ynode = new Node(2);
    {
        ynode->tag = "base_revolute_joint_y";
        ynode->axis = Eigen::Vector3d::UnitY();
        ynode->offset = Eigen::Vector3d::Zero();
        ynode->children.push_back(res);
        res->parent = ynode;
    }
    Node* znode = new Node(1);
    {
        znode->tag = "base_revolute_joint_z";
        znode->axis = Eigen::Vector3d::UnitZ();
        znode->offset = Eigen::Vector3d::Zero();
        znode->children.push_back(ynode);
        ynode->parent = znode;
    }
    Node* tnode = new Node(0);
    {
        tnode->tag = "root_translation_joint";
        tnode->axis = Eigen::Vector3d::UnitZ();
        tnode->offset = Eigen::Vector3d::Zero();
        tnode->children.push_back(znode);
        znode->parent = tnode;
    }
    tnode->Update();
    return tnode;
}
}
planner::Node* planner::LoadRobot(const std::string& urdfpath)
{
    std::map<std::string, Link*> links;
    std::map<std::string, Joint*> joints;
    //*Read all links first*/
    std::ifstream myfile (urdfpath);
    if (myfile.is_open())
    {
        std::string line;
        while (myfile.good())
        {
            getline(myfile, line);
            if(line.find("<link name") != std::string::npos)
            {
                ReadLink(line, myfile, links);
            }
        }
        myfile.close();
        std::ifstream myfile2 (urdfpath);
        while (myfile2.good())
        {
            getline(myfile2, line);
            if(line.find("<joint name") != std::string::npos)
            {
                ReadJoint(line, myfile2, joints, links);
            }
        }
        myfile2.close();
        std::map<std::string, Link*>::iterator it = links.find("root_link");
        if(it == links.end())
        {
            std::cout << "error in urdf file missing root joint" << std::endl;
        }
        else
        {
            Link* root = it->second;
            return MakeNode(root, links);
        }
    }
    return 0;
}


void GetOtherObjectsRec(planner::Node* current, planner::Node* limb, Object::T_Object& objects )
{
    if(current->tag == limb->tag || (current->tag == "LeftShoulder_z_joint")
            || (current->tag == "RightShoulder_z_joint"))
        return;
    else if(current->current)
        objects.push_back(current->current);
    for(std::vector<planner::Node*>::iterator it = current->children.begin(); it != current->children.end(); ++it)
    {
        GetOtherObjectsRec(*it, limb, objects);
    }
}

Object::T_Object GetOtherObjects(planner::Node* current, planner::Node* limb )
{
    Object::T_Object objects;
    GetOtherObjectsRec(current, limb, objects);
    return objects;
}

void GetLimbObjectsRec(planner::Node* current, Object::T_Object& objects)
{
    if(current->current)
        objects.push_back(current->current);
    for(std::vector<planner::Node*>::iterator it = current->children.begin(); it != current->children.end(); ++it)
    {
        GetLimbObjectsRec(*it, objects);
    }
}

Object::T_Object GetLimbObjects(planner::Node* limb)
{
    Object::T_Object objects;
    GetLimbObjectsRec(limb, objects);
    return objects;
}

bool planner::IsSelfColliding(planner::Robot* robot, planner::Node* limb)
{
    Object::T_Object otherObjects = GetOtherObjects(robot->node, limb);
    Object::T_Object limbObjects = GetLimbObjects(limb);
    for(Object::T_Object::iterator lit = limbObjects.begin();
        lit != limbObjects.end(); ++lit)
    {
        if((*lit)->IsColliding(otherObjects))
        {
            return true;
        }
    }
    return false;
}

double LimbLengthRec(const Node* limb)
{
    double minDistance =  (limb->children.empty()) ?  0 : std::numeric_limits<double>::max();
    double tmp;
    for(std::vector<Node*>::const_iterator cit = limb->children.begin();
        cit != limb->children.end(); ++cit)
    {
        tmp = LimbLengthRec(*cit);
        if(tmp < minDistance) minDistance = tmp;
    }
    return minDistance + (limb->position - limb->parent->position).norm();
}


bool planner::SafeTargetDistance(const planner::Node* limb, double distance, float margin)
{
    // compute distance from limb to effector with a safety margin
    double totalLength = limb->children.empty() ? 0 :  LimbLengthRec(limb->children.front());
    return distance < totalLength * margin;
}

bool planner::SafeTargetDistance(const planner::Node* limb, const Eigen::Vector3d& target, float margin)
{
    // compute distance from limb to effector with a safety margin
    double totalLength = limb->children.empty() ? 0 :  LimbLengthRec(limb->children.front());

    return (target - limb->position).norm() < totalLength * margin;
}

void GetEffectorsRec(Node* limb, std::vector<Eigen::Vector3d>& res)
{
    if(limb->children.size() != 0)
    {
        for(std::vector<Node*>::iterator cit = limb->children.begin();
            cit != limb->children.end(); ++cit)
        {
            GetEffectorsRec(*cit,res);
        }
    }
    else
    {
        res.push_back(limb->position);
    }
}

std::vector<Eigen::Vector3d> GetEffectorsRec(Node* limb)
{
    std::vector<Eigen::Vector3d> res;
    GetEffectorsRec(limb,res);
    return res;
}


Eigen::Vector3d planner::GetEffectorCenter(Node* node)
{
    std::vector<Eigen::Vector3d> res = GetEffectorsRec(node);
    Eigen::Vector3d bary(0,0,0);
    for(std::vector<Eigen::Vector3d>::iterator it = res.begin(); it != res.end();
        ++it)
    {
        bary += *it;
        //return *it;
    }
    return bary / res.size();
}

void AsConfigurationRec(planner::Node* node, Eigen::VectorXd& condiguration, std::size_t& cid)
{
    Eigen::Matrix3d rotation = Eigen::AngleAxisd(node->value, node->axis).matrix();
    while(node->children.size() == 1 && node->children[0]->offset == Eigen::Vector3d::Zero())
    {
        node = node->children.front();
        rotation *= Eigen::AngleAxisd(node->value, node->axis).matrix();
    }
    Eigen::Vector3d res = rotation.eulerAngles(2, 0, 1);
    for(int i = 0 ; i<3; ++i)
    {
        condiguration(cid) = res[i];
        ++cid;
    }
    for(std::vector<planner::Node*>::iterator it = node->children.begin();
        it != node->children.end(); ++it)
    {
        AsConfigurationRec(*it,condiguration, cid);
    }
}

Eigen::VectorXd planner::AsConfiguration(Robot* robot)
{
    std::size_t size = GetNumChildren(robot->node) + 3;
    Eigen::VectorXd res(size);
    res.block<3,1>(0,0) = robot->node->position;
    std::size_t cid = 3;
    AsConfigurationRec(robot->node->children.front(), res, cid);
    return res;
}

void AsPositionRec(planner::Node* node, Eigen::VectorXd& condiguration, std::size_t& cid, bool fantom)
{
    if(node->offset != Eigen::Vector3d::Zero())
    {
        condiguration.block<3,1>(cid,0) = node->position;
        cid += 3;
    }
    if(fantom && node->children.size() == 0 && node->tag.find("head") == std::string::npos)
    {
        condiguration.block<3,1>(cid,0) = node->position;
        cid += 3;
    }
    /*bool ok =  node->tag.find("effector") != std::string::npos
            || node->tag.find("foot_x") != std::string::npos;*/
    //if(!ok)
    {
        for(std::vector<Node*>::const_iterator cit = node->children.begin();
            cit != node->children.end(); ++cit)
        {
            AsPositionRec(*cit, condiguration, cid, fantom);
        }
    }
}

Eigen::VectorXd planner::AsPosition(Node* robot, bool fantom)
{
    std::size_t size = GetNumNodes(robot) * 3;
    if(fantom) size += 18 * 3;
    Eigen::VectorXd res(size);
    std::size_t cid = 0;
    AsPositionRec(robot, res, cid, fantom);
    return res.block(0,0,cid,1);
}
