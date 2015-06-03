#include "Exporter.h"

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
    // Blender apparently does not like it if I don't give all the rotations.
    // Therefore Adding zeros to unexisting dofs
    void WriteDofRec(planner::Node* node, stringstream& ss, bool tpose, bool useRadians)
    {
        Eigen::Matrix3d rotation = Eigen::AngleAxisd(node->value, node->axis).matrix();
        while(node->children.size() == 1 && node->children[0]->offset == Eigen::Vector3d::Zero())
        {
            node = node->children.front();
            rotation *= Eigen::AngleAxisd(node->value, node->axis).matrix();
        }
        Matrix<double,3,1> res = rotation.eulerAngles(2, 0, 1);
        for(int i = 0 ; i<3; ++i)
        {
            if(tpose)
            {
                ss << "\t" << 0;
            }
            else
            {
                ss << "\t" << res[i] * (useRadians ? 1 : RAD_TO_DEGREES);
            }
        }
        for(std::vector<planner::Node*>::iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
            WriteDofRec(*it, ss, tpose, useRadians);
        }
    }
}

Exporter::Exporter(bool useRadians)
    : f_()
    , useRadians_(useRadians)
    , rotation_(Eigen::Matrix3d::Identity())
    , offset_(Eigen::Vector3d::Zero())
{
    //NOTHING
}

Exporter::Exporter(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &offset, bool useRadians)
    : f_()
    , useRadians_(useRadians)
    , rotation_(rotation)
    , offset_(offset)
{
    //NOTHING
}

Exporter::~Exporter()
{
    // NOTHING
}

void Exporter::PushFrame(planner::Robot *robot, bool tpose)
{
    std::stringstream frame;
     // Write translations...
    Eigen::Vector3d res = rotation_ * robot->node->position + offset_;
    frame << res[0] << "\t" << res[1] << "\t" << res[2];
    WriteDofRec(robot->node->children.front(), frame, tpose, useRadians_);
    frames_.push_back(frame.str());
}


bool Exporter::Save(const std::string& filename)
{
    // saving frames
    f_ << f_.nl() << "MOTION\nFrames:\t" << (double)(frames_.size()) << "\nFrame Time: 1\n";
    for(std::vector<string>::const_iterator it = frames_.begin(); it!=frames_.end(); ++it)
    {
        f_ << (*it) << f_.nl();
    }
    return f_.Save(filename);
}
