/**
* \file Motion.h
* \brief Structure describing the complete motion of a robot
* \author Steve T.
* \version 0.1
* \date 28/04/2015
*
*/
#ifndef _STRUCT_RETARGETER
#define _STRUCT_RETARGETER


#include <string>

#include <Eigen/Dense>
#include <memory>

namespace efort
{
struct PImpl;

struct Contact
{
    int limbIndex_;
    int startFrame_;
    int endFrame_;
    Eigen::Vector3d worldPosition_;
    Eigen::Vector3d surfaceNormal_;
    std::size_t objectId_;
    std::size_t triangleId_;
};

struct Frame
{
    const Eigen::VectorXd configuration_;
    std::vector<Contact> contacts_;
};

/*
Scenario File structure
BVH "file"
SCENARIO "file"
CONTACTS
id1 id2 id3 id4 ... // one line per frame
*/

class Retargeter
{
public:
     Retargeter(const std::string& scenario, const std::string &bvhfile);
    ~Retargeter();

public:
     Frame Update(const std::size_t frameid);

public:
    const std::vector<Frame> motion_;

private:
    std::auto_ptr<PImpl> pImpl_;

};
} //namespace efort
#endif //Retargeter
