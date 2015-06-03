/**
* \file Sample.h
* \brief Helper struct that contains angle values for given joint
* articulation.
* \author Steve T.
* \version 0.1
* \date 09/04/2014
*
*/
#ifndef _STRUCT_SAMPLE
#define _STRUCT_SAMPLE

#include <vector>
#include <Eigen/Dense>

#include "prmpath/Robot.h"
#include "prmpath/Jacobian.h"

namespace planner
{
class Node;
namespace sampling
{

struct Sample
{
     Sample(Node* root);
     Sample(const Sample& parent);
     Sample(Node* root, const std::vector<double>& values);
    ~Sample();

    const std::vector<double> values;
    const Eigen::Vector3d effectorPosition;
    Jacobian jacobian;
    const Eigen::Matrix3d jacobianProduct;
    const Eigen::Matrix3d jacobianProductInverse;
};

struct RobotSample
{
    RobotSample(const Robot& robot)
        : sample_(robot.node)
        , currentRotation_(robot.currentRotation)
        , position_(robot.currentPosition){}

    RobotSample(const RobotSample& parent)
        : sample_(parent.sample_)
        , currentRotation_(parent.currentRotation_)
        , position_(parent.position_){}

    ~RobotSample(){};
    const Sample sample_;
    const Eigen::Matrix3d currentRotation_;
    const Eigen::Vector3d position_;
};

typedef std::vector<Sample*> T_Samples;
typedef std::vector<RobotSample> T_RobotSamples;
void LoadSample(const Sample& sample, Node* root);
void LoadRobot(const RobotSample& Sample, Robot& robot);

T_Samples GenerateSamples(const planner::Robot& robot, const Node *root, int nbSamples);
double Manipulability(const Sample* sample, const Eigen::Vector3d& direction);
double ForceManipulability(const Sample* sample, const Eigen::Vector3d& direction);
double VelocityManipulability(const Sample* sample, const Eigen::Vector3d& direction);

} // namespace sampling
} // namespace planner
#endif //_STRUCT_SAMPLE
