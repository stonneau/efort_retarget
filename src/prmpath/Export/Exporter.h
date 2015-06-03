#ifndef  _EXPORTER_H_
#define  _EXPORTER_H_


#include "prmpath/Export/FileHandler.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace planner
{
class Node;
class Robot;
}

/**
*  \class Exporter
*  \brief Export A robot to a readable BVH file format
*/
namespace exporter
{
class  Exporter
{
public:
     Exporter(bool useRadians=false);
     Exporter(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& offset, bool useRadians=true);
    virtual ~Exporter();

public:
    virtual void PushFrame(planner::Robot* robot, bool tpose = false);
    virtual bool Save(const std::string& /*filename*/);
    virtual void PushStructure(planner::Robot*/*skeleton*/)=0;

private:
    Exporter(const Exporter&);
    Exporter& operator=(const Exporter&);

protected:
    FileHandler f_;
    std::vector< std::string > frames_;
    std::vector< std::string > contactFrames_;

protected:
    const bool useRadians_;
    const Eigen::Matrix3d rotation_;
    const Eigen::Vector3d offset_;
};
} //namespace exporter
#endif // _EXPORTER_H_
