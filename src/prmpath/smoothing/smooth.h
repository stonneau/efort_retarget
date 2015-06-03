/**
* \file smooth.h
* \brief Helper struct to perform path computation and smoothing
* for an manipulation task
* \author Steve T.
* \version 0.1
* \date 09/05/2014
*
*/
#ifndef _HELPER_SMOOTHING
#define _HELPER_SMOOTHING

#include "collision/Collider.h"

#include <Eigen/Dense>

namespace planner
{
struct Model;

typedef std::vector<const Model*> CT_Model;
typedef std::pair<Eigen::Vector3d, Eigen::Matrix3d> Configuration;
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> C2_Point;
typedef std::pair<double, C2_Point> MilePoint;
typedef std::vector<MilePoint> T_MilePoint;

struct ParamFunction
{
    ParamFunction(){}
    ~ParamFunction(){}
    virtual C2_Point operator()(double t) const = 0;
    virtual C2_Point max() const = 0;
    virtual double tmax() const = 0;
    Configuration Evaluate(double t) const;
};

struct InterpolatePath : public ParamFunction
{
    InterpolatePath(const CT_Model& path);
    InterpolatePath(const C2_Point& from, const C2_Point& to, const double startTime, const double endTime);
    ~InterpolatePath();
    virtual C2_Point operator()(double t) const;
    virtual C2_Point max() const;
    virtual double tmax() const;
    const T_MilePoint milePoints_;
};

struct SplinePath : public ParamFunction
{
    SplinePath(const std::vector<Eigen::Vector3d>& controlPoints, const std::vector<Eigen::Vector3d>& controlPointsRot, const std::vector<double>& knots, double scale);
   ~SplinePath();

    virtual C2_Point operator()(double t) const;
    virtual C2_Point max() const;
    virtual C2_Point min() const;
    virtual double tmax() const {return knots_.back();}
    virtual double tmin() const {return knots_.front();}
    virtual C2_Point derivative(const double t) const;
    //virtual Configuration Evaluate(double t) const;

    std::vector<Eigen::Vector3d> controlPoints_;
    std::vector<Eigen::Vector3d> controlPointsRot_;
    std::vector<double> knots_;
    double scale_;
};
// Jia Pan Collision Free and Curvature Continuous Path Smoothing in cluttered environment
SplinePath SplineFromPath(Collider& collider, CT_Model& path, double maxSpeed, double maxAcceleration, bool normalize = true);
SplinePath SplineShortCut(Collider& collider, CT_Model& path, double maxSpeed, double maxAcceleration, int nbSteps);
} //namespace planner
#endif //_HELPER_SMOOTHING
