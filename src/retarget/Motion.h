/**
* \file Motion.h
* \brief Structure describing the complete motion of a robot
* \author Steve T.
* \version 0.1
* \date 28/04/2015
*
*/
#ifndef _STRUCT_MOTION
#define _STRUCT_MOTION

#include <Eigen/Dense>
#include <memory>

#define INTERNAL 1

#if INTERNAL
#include "prmpath/CompleteScenario.h"
#endif

namespace efort
{
/// \class PImpl
/// \brief private implementation of Motion class hiding internal object representation.
struct PImpl;

/// \class Contact
/// \brief Characterization of a contact.
struct Contact
{
    int limbIndex_;
    int startFrame_;
    int endFrame_;
    Eigen::Vector3d worldPosition_;
    Eigen::Vector3d surfaceNormal_;
    //std::size_t objectId_;
    //std::size_t triangleId_;

    bool equals(const Contact& other) const
    {
        return limbIndex_ == other.limbIndex_ && startFrame_ == other.startFrame_
                && endFrame_ == other.endFrame_;
    }
};

/// \class Frame
/// \brief Characterization of a frame, to which is associated a configuration (list of all angle values
/// of all joints), and a set of contacts, related to a limb.
struct Frame
{
    Eigen::VectorXd configuration_;
    std::vector<Contact> contacts_;
};

struct ContactUpdate
{
    std::size_t initFrame_;
    std::size_t endframe_;
    std::vector<Eigen::VectorXd> positions_;
};

typedef std::vector<std::pair<std::size_t, Eigen::Vector3d> > T_PointReplacement;

/// \class Motion
/// \brief Stores a complete motion for a character. Initialized with a configuration file.
/// used to perform retargeting given environment modification.
/// of all joints), and a set of contacts, related to a limb.
struct Motion
{
    ///  \brief Performs motion retargeting at a give frame.
    /// Given the joint positions, performs generalized IK to recompute joint angles.
    /// Given environment modifications, performs contact retargetting if necessary.
    /// Reasons are: invalid configuration, contact is out of range.
    ///  \param frameid : considered frame number
    ///  \param framePositions : 3d location of each joint of the character, computed by
    /// relationship descriptors.
    /// \param objectModifications : std::vector of pair indicating for a given vertice id, its new location
    /// As first version, PQP object is recreated and retargetting is performed based on this new list.
    ///  \param return : The updated 3d joint location of each joint after retargetting if necessary.
    Eigen::VectorXd Retarget(const std::size_t frameid, const Eigen::VectorXd& framePositions, const T_PointReplacement& objectModifications) const;

    ///  \brief Performs motion retargeting for the contacts of a given frame.
    /// Given the joint positions, performs generalized IK to recompute joint angles.
    /// Given environment modifications, performs contact retargetting if necessary.
    /// repercuting contact location to all impacted frames.
    ///  \param frameid : considered frame number
    ///  \param framePositions : 3d location of each joint of the character, computed by
    /// relationship descriptors.
    /// \param objectModifications : std::vector of pair indicating for a given vertice id, its new location
    /// As first version, PQP object is recreated and retargetting is performed based on this new list.
    ///  \param return : The updated 3d joint location of each joint after retargetting if necessary.
    ContactUpdate RetargetContact(const std::size_t frameid, const Eigen::VectorXd& framePositions, const T_PointReplacement& objectModifications) const;


#if INTERNAL
    planner::Robot* RetargetInternal(const std::size_t frameid, const Eigen::VectorXd& framePositions, const T_PointReplacement& objectModifications) const;
    std::vector<planner::Robot*> RetargetContactInternal(const std::size_t frameid, const Eigen::VectorXd& framePositions, const T_PointReplacement& objectModifications) const;
#endif

    std::vector<Frame> frames_;
private:
    std::auto_ptr<PImpl> pImpl_;
    friend Motion* LoadMotion(const std::string& scenario);
#if INTERNAL
    friend Motion* LoadMotion(planner::CompleteScenario* completeScenario);
#endif
};

/// \brief Loads a motion object from a given configuration file.
/// Said file contains all information about character joint limits, Range Of Motion, obj file loaded.
Motion* LoadMotion(const std::string& scenario);


#if INTERNAL
    Motion* LoadMotion(planner::CompleteScenario* completeScenario);
#endif
} //namespace efort
#endif //_STRUCT_MOTION
