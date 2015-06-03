#ifndef  _ITOMPEXPORTER_H_
#define  _ITOMPEXPORTER_H_


#include "prmpath/Export/Exporter.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace planner
{
class Node;
class Robot;
}

/**
*  \class ITOMPExporter
*  \brief Export A robot to a readable BVH file format
*/
namespace exporter
{
class  ITOMPExporter : public Exporter
{
public:
     ITOMPExporter(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &offset, const std::vector<planner::Node*>& limbs);
    ~ITOMPExporter();

public:
    virtual void PushStructure(planner::Robot*/*skeleton*/);
    virtual void PushFrame(planner::Robot * /*robot*/, bool tpose = false);
    virtual bool Save(const std::string& /*filename*/);

    void PushFrame(planner::Robot * /*robot*/, const std::vector<int>& contactLimbs
                   , const std::vector<Eigen::Vector3d>& contactLimbPositions
                   , const std::vector<Eigen::Vector3d>& contactLimbPositionsNormals
                   , bool tpose = false);

public:
     std::vector<int> contactLimbs;
     std::vector<Eigen::Vector3d> contactLimbPositions;
     std::vector<Eigen::Vector3d> contactLimbPositionsNormals;
     const std::vector<planner::Node*>& limbs_;
     const int nbEffectors_;

private:
     std::vector<int> itompEffectorOrder_;

private:
    ITOMPExporter(const ITOMPExporter&);
    ITOMPExporter& operator=(const ITOMPExporter&);
};
} //namespace exporter
#endif // _ITOMPEXPORTER_H_
