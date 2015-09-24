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
#include <stdexcept>

#define INTERNAL 1

#if INTERNAL
#include "prmpath/CompleteScenario.h"
#endif

namespace efort
{

//to pass both collision and reachability
// pass RetargetType::collision | RetargetType::reachability
enum RetargetType
{
    collision = 0x00001,
    reachability = 0x00002
};


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

// if retargeted_ is false,
// other states are undefined
struct FrameReport
{
    FrameReport(const std::size_t id, const int nbC):
        frameId_(id), retargeted_(false)
    {
        //init all contacts to false
        for(int i=0; i< nbC; ++i)
        {
            contactStates.push_back(0);
        }
    }

    std::size_t frameId_;
    Eigen::VectorXd pose_;
    // true if retargeting modified frame
    bool retargeted_;
    // if limb i in contact, contactStates[i] == 1; 0 otherwise
    std::vector<std::size_t> contactStates;
    // exhaustive info of contacts, ordered
    std::vector<Contact> contacts_;

    //returns contact associated to limb
    // throws exception if limb not in contact
    const Contact& getContact(std::size_t limbId)
    {
        if(contactStates[limbId] == 0)
            throw(std::runtime_error("No contact defined for limb " + limbId));
        std::size_t id(0);
        std::vector<Contact>::const_iterator cc =contacts_.begin();
        for(std::vector<std::size_t>::const_iterator cs = contactStates.begin();
            cs != contactStates.end(); ++cs, ++id)
        {
            if(id == limbId)
                return *cc;
            if((*cs) == 1)
            {
                ++cc;
            }
        }
    }
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
    /// \param force : if set to true, force retargetting of all limbs
    ///  \param return : The updated 3d joint location of each joint after retargetting if necessary.
    std::vector<Eigen::VectorXd> RetargetContact(const std::size_t frameid, const Eigen::VectorXd& framePositions,
                                                 const T_PointReplacement& objectModifications, bool force = false) const;

    ///  \brief Performs motion retargeting for the contacts of a given frame.
    /// Given the joint positions, performs generalized IK to recompute joint angles.
    /// Given environment modifications, performs contact retargetting if necessary.
    /// repercuting contact location to all impacted frames.
    ///  \param frameid : considered frame number
    ///  \param framePositions : 3d location of each joint of the character, computed by
    /// relationship descriptors.
    /// \param objectModifications : std::vector of pair indicating for a given vertice id, its new location
    /// As first version, PQP object is recreated and retargetting is performed based on this new list.
    /// \param forcemask : vector of bool which has the same size than each number of limbs. If a limb entry is set to
    /// true then the limb will be retargeted
    ///  \param return : The updated 3d joint location of each joint after retargetting if necessary.
    std::vector<Eigen::VectorXd> RetargetContact(const std::size_t frameid, const Eigen::VectorXd& framePositions,
                                                 const T_PointReplacement& objectModifications, const std::vector<bool>& forcemask) const;


    ///  \brief Retarget root position to collision free position.
    /// !!!! DOES NOT RETARGET. CALL RETARGET BEHIND
    ///  \param frameid : considered frame number
    ///  \param framePositions : 3d location of each joint of the character, computed by
    /// relationship descriptors.
    /// \param objectModifications : std::vector of pair indicating for a given vertice id, its new location
    /// As first version, PQP object is recreated and retargetting is performed based on this new list.
    /// \param force : if set to true, force retargetting of all limbs
    ///  \param return : The updated 3d joint location of each joint after retargetting if necessary.
    std::vector<Eigen::VectorXd> RetargetTrunk(const std::size_t frameidCurrent, const std::size_t frameidFrom, const std::size_t frameidTo,
                                               const Eigen::VectorXd& frameCurrent, const Eigen::VectorXd& frameFrom,
                                               const Eigen::VectorXd& frameTo,
                                               const T_PointReplacement& objectModifications);


    void DoRRT(const std::size_t frameidFrom, const Eigen::VectorXd& frameFrom, const Eigen::VectorXd& frameTo, const T_PointReplacement& objectModifications,
                                                 bool useSplines = true);

    void Interpolate(const std::size_t frameidFrom, const Eigen::VectorXd& frameFrom, const Eigen::VectorXd& frameTo, const int nbFrames,
                                                 bool useSplines, bool useRRT = true);

    void Interpolate(const std::size_t frameidFrom, const Eigen::VectorXd& frameFrom, const Eigen::VectorXd& frameTo,
                                                 bool useSplines, bool useRRT = true);

    ///parameter retargetType help you choose collision and or / reachability for retargetting decision
    std::vector<FrameReport> RetargetMotion(const std::vector<Eigen::VectorXd>& framePositions, const T_PointReplacement& objectModifications,
                                            const int retargetType, const std::size_t frameStart = 0, bool force = false) const;
    std::vector<FrameReport> RetargetMotion(const std::vector<Eigen::VectorXd>& framePositions, const T_PointReplacement& objectModifications,
                                            const int retargetType, const std::size_t frameStart, const std::vector<bool>& forcemask) const;

    std::vector<FrameReport> ReturnMotion() const;


#if INTERNAL
    planner::Robot* RetargetInternal(const std::size_t frameid, const Eigen::VectorXd& framePositions, const T_PointReplacement& objectModifications) const;
    std::vector<planner::Robot*> RetargetContactInternal(const std::size_t frameid, const Eigen::VectorXd& framePositions, const T_PointReplacement& objectModifications, bool force = false) const;
    std::vector<planner::Robot*> RetargetMotionInternal(const std::vector<Eigen::VectorXd>& newPositions, const T_PointReplacement& objectModifications, const std::size_t frameStart, const int retargetType, bool force = false) const;
    std::vector<planner::Robot*> RetargetTrunkInternal(const std::size_t frameidCurrent, const std::size_t frameidFrom, const std::size_t frameidTo,
                                               const Eigen::VectorXd& frameCurrent, const Eigen::VectorXd& frameFrom,
                                               const Eigen::VectorXd& frameTo,
                                               const T_PointReplacement& objectModifications);
#endif

    std::vector<Frame> frames_;
private:
    std::auto_ptr<PImpl> pImpl_;
    friend Motion* LoadMotion(const std::string& scenario);
    void ReloadMotion();
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
