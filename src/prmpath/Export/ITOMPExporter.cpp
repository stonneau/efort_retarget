#include "ITOMPExporter.h"

#include "prmpath/Robot.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <vector>

using namespace std;
using namespace exporter;
using namespace Eigen;

namespace
{
    const double RAD_TO_DEGREES = 180 / M_PI; // Degrees = 180 * RAD / M_PI
    const Eigen::Vector3d zero(0,0,0);
}

namespace
{
    void WriteJointNameRec(FileHandler& f, planner::Node* node)
    {
        f << node->tag << f.nl();
        for(std::vector<planner::Node*>::iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
            WriteJointNameRec(f, *it);
        }
    }



    // Blender apparently does not like it if I don't give all the rotations.
    // Therefore Adding zeros to unexisting dofs
    void WriteDofRecNoFantomJoint(planner::Node* node, stringstream& ss, bool tpose)
    {
        if(tpose)
        {
            ss << "\t" << 0;
        }
        else
        {
            /*if(node->axis == Eigen::Vector3d::UnitX() && node->parent && node->offset.norm() != 0)
            {
                ss << "\t" << node->parent->value;
            }
            else if(node->axis == Eigen::Vector3d::UnitX())
            {

            }
            else*/
            ss << "\t" << node->value;
        }
        for(std::vector<planner::Node*>::iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
            WriteDofRecNoFantomJoint(*it, ss, tpose);
        }
    }

    void WriteDofNoFantomJoint(planner::Node* node, stringstream& ss, const Eigen::Matrix3d& rot, bool tpose)
    {
        // retrieve first nodes and apply rotations
        Eigen::Matrix3d rotation = Eigen::AngleAxisd(node->value, node->axis).matrix();
        while(node->children.size() == 1 && node->children[0]->offset == Eigen::Vector3d::Zero())
        {
            node = node->children.front();
            rotation *= Eigen::AngleAxisd(node->value, node->axis).matrix();
        }
        Matrix<double,3,1> res = (rot * rotation).eulerAngles(2, 1, 0);
        ss << res[0] << "\t" << -res[2] << "\t" << res[1];
        for(std::vector<planner::Node*>::iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
            WriteDofRecNoFantomJoint(*it, ss, tpose);
        }
    }
}

namespace
{
void findstring(const std::string key, const std::vector<planner::Node*>& limbs, std::vector<int>& indexes)
{
    int i =0;
    for(std::vector<planner::Node*>::const_iterator nit = limbs.begin();
        nit != limbs.end(); ++nit, ++i)
    {
        if((*nit)->tag.find(key) != string::npos)
        {
            indexes.push_back(i);
        }
    }
}
}

ITOMPExporter::ITOMPExporter(const Eigen::Matrix3d& rotation, const Eigen::Vector3d &offset, const std::vector<planner::Node*>& limbs)
    : Exporter(rotation,offset,true)
    , limbs_(limbs)
    , nbEffectors_(limbs.size())
{
    // NOTHING
    // this must be done better
    // Currently the order I use is 'left foot' 'right foot' 'left hand' 'right hand'.
    /*find left foot*/
    findstring("upper_left_leg_z_joint", limbs, itompEffectorOrder_);
    findstring("upper_right_leg_z_joint", limbs, itompEffectorOrder_);
    findstring("upper_left_arm_z_joint", limbs, itompEffectorOrder_);
    findstring("upper_right_arm_z_joint", limbs, itompEffectorOrder_);
    if(limbs.size() != itompEffectorOrder_.size())
    {
        std::cout << "Error: could not find all itomp effectors in robot model" << std::endl;
    }
}

ITOMPExporter::~ITOMPExporter()
{
    // NOTHING
}


void ITOMPExporter::PushFrame(planner::Robot* robot, bool tpose)
{
    std::vector<int> cL;
    std::vector<Eigen::Vector3d> clp, clpn;
    PushFrame(robot, cL, clp, clpn, tpose);
}

void ITOMPExporter::PushFrame(planner::Robot* robot, const std::vector<int>& contactLimbs
               , const std::vector<Eigen::Vector3d>& contactLimbPositions
               , const std::vector<Eigen::Vector3d>& contactLimbPositionsNormals
               , bool tpose)
{
    /*Push joint values*/
    std::stringstream jframe;
     // Write translations...
    Eigen::Vector3d res = rotation_ * robot->node->position + offset_;
    Eigen::Matrix3d rot =AngleAxisd(-0.5*M_PI, Vector3d::UnitY()).matrix();
    jframe << res[0] << "\t" << res[1] << "\t" << res[2]<< "\t";
    WriteDofNoFantomJoint(robot->node->children.front(), jframe, rotation_, tpose);
    //WriteDofRecNoFantomJoint(robot->node->children.front(), frame, tpose);
    //WriteDofNoFantomJoint(robot->node->children.front(), frame, rot, tpose);
    //WriteDofRecNoFantomJoint(robot->node->children.front(), frame, tpose);
    frames_.push_back(jframe.str());


    /*Push contact values*/
    std::stringstream cframe;
    std::vector<planner::Node*>::const_iterator cit = limbs_.begin();
//for(int i = 0; i < nbEffectors_; ++i, ++cit)
    for(std::vector<int>::const_iterator contactorderit = itompEffectorOrder_.begin();
        contactorderit != itompEffectorOrder_.end(); ++contactorderit)
    {
        const int& i = *contactorderit;
        bool found = false;
        int id=0;
        for(; id < contactLimbs.size() && !found; ++id)
        {
            if(contactLimbs[id]==i)
            {
                found = true;
                break;
            }
        }
        if(found) // is in contact
        {
            cframe << 1 << "\t"; // say there s a contact
            // push position
            Eigen::Vector3d res = rotation_ * contactLimbPositions[id] + offset_;
            cframe << res[0] << "\t" << res[1] << "\t" << res[2]<< "\t";
            // push rotation
            // get Node name
            const int& id = (*cit)->id;
            // find effectors
            std::vector<planner::Node*> effectors = planner::GetEffectors(planner::GetChild(robot,id),true);
            Eigen::Quaterniond qa(rotation_ * (effectors.front()->toWorldRotation));
            cframe << qa.x() << "\t" << qa.y() << "\t" << qa.z() << "\t" << qa.w() << "\t";
        }
        else
        {
            for(int j =0; j< 8; ++j)
            {
                cframe << 0 << "\t";
            }
        }
    }
    contactFrames_.push_back(cframe.str());
}

void ITOMPExporter::PushStructure(planner::Robot* robot)
{
    f_ << "HIERARCHY" << f_.nl();
    // put translation
    f_ << "base_prismatic_joint_x" << f_.nl();
    f_ << "base_prismatic_joint_y" << f_.nl();
    f_ << "base_prismatic_joint_z" << f_.nl();

    WriteJointNameRec(f_, robot->node->children.front());
}

bool ITOMPExporter::Save(const std::string& filename)
{
    // saving frames
    f_ << f_.nl() << "MOTION\nFrames:\t" << (double)(frames_.size()) << "\nFrame Time: 1\n";
    for(std::vector<string>::const_iterator it = frames_.begin(); it!=frames_.end(); ++it)
    {
        f_ << (*it) << f_.nl();
    }
    f_ << f_.nl() << "CONTACT EFFECTORS " << nbEffectors_ << "\n";
    for(std::vector<string>::const_iterator it = contactFrames_.begin(); it!=contactFrames_.end(); ++it)
    {
        f_ << (*it) << f_.nl();
    }
    return f_.Save(filename);
}
