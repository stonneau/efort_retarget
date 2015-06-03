#include "BVHExporter.h"

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
    void WriteOffsetLine(FileHandler& f, const Eigen::Vector3d& offset)
    {
        f << "OFFSET\t" << offset(0) << "\t" << offset(1) << "\t" << offset(2);
    }

    void WriteJointOffsetRec(FileHandler& f, planner::Node* node)
    {
        //Eigen::Vector3d offset = node->evalWorldPos(zero) - node->getParentNode()->evalWorldPos(zero);
        Eigen::Vector3d offset = node->position - node->parent->position;
        while(node->children.size() == 1 && node->children[0]->offset == Eigen::Vector3d::Zero())
        {
            node = node->children.front();
        }
        f << f.nl() << "JOINT " << node->tag << f.nl() << "{";
        f.AddTab();
            WriteOffsetLine(f, offset);
            f << f.nl() << "CHANNELS 3  Zrotation Xrotation Yrotation";
            if(node->children.empty())
            {
                f << f.nl() << "End Site" << f.nl() << "{";
                f.AddTab();
                    // last offset corresponds to half length of final cube according to ParserBVH.hpp
                    offset.normalize(); offset *= 0.001 / 2;
                    WriteOffsetLine(f, offset);
                f.RemoveTab();
                f << "}";
            }
            else
            {
                for(std::vector<planner::Node*>::iterator it = node->children.begin();
                    it != node->children.end(); ++it)
                {
                    WriteJointOffsetRec(f, *it);
                }
            }
        f.RemoveTab();
        f << "}";
    }
}


BVHExporter::BVHExporter()
    : Exporter()
{
    // NOTHING
}

BVHExporter::~BVHExporter()
{
    // NOTHING
}


void BVHExporter::PushStructure(planner::Robot* robot)
{
    // just making sure that everything is set to 0
    planner::Robot skeleton(*robot);
    planner::Zero(&skeleton);
    planner::Node* node = skeleton.node;
    f_ << "HIERARCHY" << f_.nl() << "ROOT " << skeleton.node->tag << f_.nl() << "{" ;
    f_.AddTab();
    // first node istranlation, others are rotation
        WriteOffsetLine(f_, skeleton.node->position);
        f_ << f_.nl() << "CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation";
        while(node->offset == Eigen::Vector3d::Zero())
        {
            node = node->children.front();
        }
        //WriteJointOffsetRec(f_, node->children[0]);
        for(std::vector<planner::Node*>::iterator it = node->parent->children.begin();
            it != node->parent->children.end(); ++it)
        {
            WriteJointOffsetRec(f_, *it);
        }
    f_.RemoveTab();
    f_ << "}";
}
