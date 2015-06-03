#ifndef  _BVHEXPORTER_H_
#define  _BVHEXPORTER_H_


#include "prmpath/Export/Exporter.h"

#include <string>
#include <vector>

namespace planner
{
class Node;
class Robot;
}

/**
*  \class BVHExporter
*  \brief Export A robot to a readable BVH file format
*/
namespace exporter
{
class  BVHExporter : public Exporter
{
public:
     BVHExporter();
    ~BVHExporter();

public:
    virtual void PushStructure(planner::Robot*/*skeleton*/);
private:
    BVHExporter(const BVHExporter&);
    BVHExporter& operator=(const BVHExporter&);
};
} //namespace exporter
#endif // _BVHEXPORTER_H_
