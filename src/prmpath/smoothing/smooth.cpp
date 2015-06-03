#include "smooth.h"
#include "prm/Model.h"

#include "tools/ExpMap.h"

#include <vector>
#include <utility>
#include <random>

namespace planner
{
typedef std::vector<Model*> T_Model;
typedef T_Model::iterator IT_Model;
typedef std::vector<const Model*> CT_Model;
typedef CT_Model::const_iterator CIT_Model;

typedef std::pair<double, C2_Point> MilePoint;
typedef std::vector<MilePoint> T_MilePoint;



C2_Point MakeConfiguration(const Model* model)
{
    matrices::ExpMap emap(model->GetOrientation());
    return std::make_pair(model->GetPosition(), emap.log());
}

Eigen::Quaterniond UnitQ(const Eigen::Vector3d& v)
{
    Eigen::Vector3d unit = v;
    double norm = v.norm();
    if(norm != 0)
    {
        unit.normalize();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(norm,unit));
}

T_MilePoint CreateMilePoints(const CT_Model& path)
{
    T_MilePoint res;
    // simple distance heuristic for starters, it is not important
    std::vector<double> distances;
    double totalDistance = 0;
    CIT_Model cit = path.begin();
    CIT_Model cit2 = path.begin(); ++cit2;
    distances.push_back(0);
    for(; cit2 != path.end(); ++cit, ++cit2)
    {
        totalDistance+= ((*cit2)->GetPosition() - (*cit)->GetPosition()).norm();
        distances.push_back(totalDistance);
    }

    std::vector<double>::const_iterator dit = distances.begin();
    cit = path.begin();
    for(; cit != path.end(); ++cit, ++dit)
    {
        res.push_back(std::make_pair((*dit), MakeConfiguration(*cit)));
    }
    return res;
}

T_MilePoint CreateMilePoints(const C2_Point& from, const C2_Point& to, const double startTime, const double endTime)
{
    T_MilePoint res;
    // simple distance heuristic for starters, it is not important
    res.push_back(std::make_pair(startTime, from));
    res.push_back(std::make_pair(endTime, to));
    return res;
}

C2_Point Interpolate(const MilePoint& a, const MilePoint& b, double t)
{
    double t0 = a.first;
    double t1 = b.first;
    double tp = (t - t0) / (t1 - t0);

    const Eigen::Quaterniond qa = UnitQ(a.second.second);
    const Eigen::Quaterniond qb = UnitQ(b.second.second);

    const Eigen::Vector3d& va = a.second.first;
    const Eigen::Vector3d& vb = b.second.first;

    Eigen::Vector3d offset = va + tp * (vb - va);
    Eigen::Quaterniond qres  = qa.slerp(tp, qb);

    matrices::ExpMap emap(qres);
    return std::make_pair(offset, emap.log());
}

void InsertSorted(std::vector<double>& knots, double newValue)
{
    for(std::vector<double>::iterator it = knots.begin(); it != knots.end(); ++it)
    {
        if(newValue < *it)
        {
            knots.insert(it, newValue);
            return;
        }
    }
    knots.push_back(newValue);
}

SplinePath::SplinePath(const std::vector<Eigen::Vector3d>& controlPoints, const std::vector<Eigen::Vector3d>& controlPointsRot, const std::vector<double>& knots, double scale)
    : controlPoints_(controlPoints)
    , controlPointsRot_(controlPointsRot)
    , knots_(knots)
    , scale_(scale)
{
    // NOTHING
}

SplinePath::~SplinePath()
{
    // NOTHING
}

Eigen::Vector3d deBoor(int k, int degree, int i, double x, const std::vector<double>& knots, const std::vector<Eigen::Vector3d>& ctrlPoints)
{   // Please see wikipedia page for detail
    // note that the algorithm here kind of traverses in reverse order
    // comapred to that in the wikipedia page
    if( k == 0)
        return ctrlPoints[i];
    else
    {
        double alpha = (x-knots[i])/(knots[i+degree+1-k]-knots[i]);
        return (deBoor(k-1,degree, i-1, x, knots, ctrlPoints)*(1-alpha) + deBoor(k-1,degree, i, x, knots, ctrlPoints)*alpha );
    }
}

// implementation of de boor algorithm
Configuration ParamFunction::Evaluate(double t) const
{
    C2_Point c = this->operator ()(t);
    return std::make_pair(c.first, UnitQ(c.second).toRotationMatrix());
}

C2_Point SplinePath::max() const
{
    return std::make_pair(controlPoints_.back(), controlPointsRot_.back());
}

C2_Point SplinePath::min() const
{
    return std::make_pair(controlPoints_.front(), controlPointsRot_.front());
}

C2_Point SplinePath::derivative(const double t) const
{
    const double epsilon = 0.00001;
    if((knots_.back() == t) || (knots_.front() == t))
        return std::make_pair(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    C2_Point plus = (*this)(t-epsilon);
    C2_Point moins = (*this)(t+epsilon);
    return std::make_pair((plus.first - moins.first)/2*epsilon, (plus.second - moins.second)/2*epsilon);
}

C2_Point SplinePath::operator ()(double t) const
{
    // compute interval
    int interval =-1;
    int ti = knots_.size();
    for(int i=1;i<ti-1;i++)
    {
        if(t<knots_[i])
        {
            interval = (i-1);
            break;
        }
        else if(t == knots_[ti-1])
        {
            interval = (ti-1);
            break;
        }
    }
    //compute rotation matrix
    return std::make_pair(deBoor(3,3,interval,t,knots_,controlPoints_), deBoor(3,3,interval,t,knots_,controlPointsRot_));
}

}

using namespace planner;

InterpolatePath::InterpolatePath(const CT_Model& path)
    : milePoints_(CreateMilePoints(path))
{}

InterpolatePath::InterpolatePath(const C2_Point& from, const C2_Point& to, const double startTime, const double endTime)
    : milePoints_(CreateMilePoints(from,to, startTime, endTime))
{}

InterpolatePath::~InterpolatePath()
{
    // NOTHING
}

C2_Point InterpolatePath::operator()(double t) const
{
    //assert(0 <= t && t <= 1);
    T_MilePoint::const_iterator cit = milePoints_.begin();
    T_MilePoint::const_iterator cit2 = milePoints_.begin(); ++cit2;
    for(; cit2 != milePoints_.end(); ++cit, ++cit2)
    {
        if((*cit).first <= t && t < (*cit2).first)
        {
            return Interpolate(*cit, *cit2, t);
        }
    }
    return milePoints_.back().second;
}

C2_Point InterpolatePath::max() const
{
    return milePoints_.back().second;
}

double InterpolatePath::tmax() const {return milePoints_.back().first;}

namespace
{
    std::vector<double> UniformSample(double ta, double tb, int nbSamples)
    {
        std::vector<double> knots;
        knots.push_back(ta);
        double stepsize = 1 / nbSamples;
        double t = 0;
        for(int i = 0; i < nbSamples - 2; ++i, t = t + stepsize) // -1 because ta and tb are inserted
        {
            InsertSorted(knots, t);
        }
        knots.push_back(tb);
        return knots;
    }

    std::vector<double> RandomSample(double ta, double tb, int nbSamples)
    {
        std::vector<double> knots;
        knots.push_back(ta);
        for(int i = 0; i <nbSamples - 2; ++i) // -1 because ta and tb are inserted
        {
            double t = (tb - ta) * ((double)rand() / (double)RAND_MAX ) + ta;
            InsertSorted(knots, t);
        }
        knots.push_back(tb);
        return knots;
    }

    void ReconfigureTime(std::vector<double>& knots, double maxSpeed)
    {
        // assuming we can't go too fast all the time
        // TODO HANDLE ALL JOINT BUT F...
        maxSpeed *= 0.75;
        std::vector<double>::iterator it = knots.begin();
        for(;it!=knots.end(); ++it)
        {
            *it /= maxSpeed;
        }
    }

    struct Delta
    {
        Delta(std::vector<double> knots, int m)
            : knots_(knots)
            , m_(m)
        {
            // NOTHING
        }

        ~Delta()
        {
            // NOTHING
        }

        double operator()(int id)
        {
            if(id == -1 || id == m_) return 0;
            return knots_[id+1] - knots_[id];
        }

        std::vector<double> knots_;
        int m_;
    };

    struct Normalize
    {
        Normalize(double ta, double tb)
            : ta_(ta)
            , div_(tb - ta) {}

        ~Normalize(){}

        void operator() (double& val)
        {
            val =  (val - ta_) / div_;
        }
        double ta_, div_;
    };

    struct FirstAbove
    {
        FirstAbove(double a)
            : a_(a) {}

        ~FirstAbove(){}

        bool operator() (double val)
        {
            return a_ < val;
        }
        double a_;
    };

}

#include <omp.h>

namespace
{

    std::vector<double> CheckReachability(Collider& collider, const Model& model, const planner::SplinePath& spline)
    {
        double norm = 2 * (spline.max().first - spline.min().first).norm();
        const double step = 1 / norm; // every one unit
        bool nocollisions[ 100 ] = { true };
        for(int i=0; i< 100; ++i) nocollisions[i] = true;
        int maxIndex = (int)norm;
        #pragma omp parallel for
        for(int i =(0); i < maxIndex; ++i)
        {
            double t = i * step + spline.tmin();
            if(t < spline.tmin()) t = spline.tmin();
            if(t > spline.tmax()) t = spline.tmax();
            Model temp(model);
            Configuration conf = spline.Evaluate(t);
            temp.SetOrientation(conf.second);
            temp.SetPosition(conf.first);
            if(!temp.ReachabilityCondition(collider))
            {
                nocollisions[i] = false;
            }
        }
        std::vector<double> collisionTimes;
        for(int i =0; i< maxIndex; ++i)
        {
            if(!nocollisions[i])
            {
                double t = i * step + spline.tmin();
                if(t < spline.tmin()) t = spline.tmin();
                if(t > spline.tmax()) t = spline.tmax();
                collisionTimes.push_back(t);
            }
        }
        return collisionTimes;
    }

    planner::SplinePath createSpline(Collider& collider, const Model& model, const ParamFunction* initPath, const SplinePath* previousTrajectory, double maxSpeed, double maxAcceleration, double ta, double tb, int m, int tries, bool& success, bool normalize=false, bool shortcut = false)
    {
        while(tries > 0)
        {
            //sample m + 1 points between ta and tb
            std::vector<double> knots = RandomSample(ta, tb, m+1);
            // TODO REALIGN MAX DISTANCE TO INCLUDE THIS
            //ReconfigureTime(knots, maxSpeed);

            //Reconfigure knots so that respect max velocity and acceleration

            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m+1, m+1);
            Eigen::MatrixXd r = Eigen::MatrixXd::Zero(m+1,3);
            Eigen::MatrixXd sol = Eigen::MatrixXd::Zero(m+1,3);

            Eigen::MatrixXd r_rot = Eigen::MatrixXd::Zero(m+1,3);
            Eigen::MatrixXd sol_rot = Eigen::MatrixXd::Zero(m+1,3);

            Delta d(knots,m);

            for(int k=1; k<m;++k)
            {
                double alpha, beta, gamma;

                alpha = (d(k)*d(k))/(d(k-2)+d(k-1)+d(k));
                beta = (d(k) * (d(k-2)+d(k-1))) / (d(k-2)+d(k-1)+d(k)) +(d(k-1)*(d(k) + d(k+1)))/(d(k-1) + d(k) + d(k+1));
                gamma =(d(k-1)*d(k-1)) /(d(k-1) + d(k) + d(k+1));

                A(k,k-1) = alpha; // alphak
                A(k,k) = beta; // betak
                A(k,k+1) = gamma; // gammak

                // Assigning translation variables
                C2_Point cr = (*initPath)(knots[k]);
                r.block<1,3>(k,0) = (cr.first * (d(k-1)+d(k))).transpose();
                r_rot.block<1,3>(k,0) = (cr.second * (d(k-1)+d(k))).transpose();
            }

            if(shortcut)
            {
                A(0,0) = 1; // beta0
                A(0,1) = 0; // gamma0
                A(m,m-1) = 0; // alpham
                A(m,m) = 1; // betam
                {
                    C2_Point cr = (*previousTrajectory)(knots[0]);
                    C2_Point crd = (*previousTrajectory).derivative(knots[0]);
                    r.block<1,3>(0,0) = (cr.first + d(0) * crd.first / 3).transpose();
                    r_rot.block<1,3>(0,0) = (cr.second + d(0) * crd.second / 3).transpose();
                }
                {
                    C2_Point cr = (*previousTrajectory)(knots.back());
                    C2_Point crd = (*previousTrajectory).derivative(knots.back());
                    r.block<1,3>(m,0) = cr.first - d(m-1) * crd.first / 3;
                    r_rot.block<1,3>(m,0) = cr.second - d(m-1) * crd.second / 3;
                }
            }
            else
            {
                A(0,0) = d(0) + 2 *d(1); // beta0
                A(0,1) = -d(0); // gamma0
                A(m,m-1) = -d(m-1); // alpham
                A(m,m) = d(m-2)+2*d(m-1); // betam
                {
                    C2_Point cr = (*initPath)(knots[0]);
                    r.block<1,3>(0,0) = (cr.first * (d(0)+d(1))).transpose();
                    r_rot.block<1,3>(0,0) = (cr.second * (d(0)+d(1))).transpose();
                }
                {
                    C2_Point cr = (*initPath)(knots.back());
                    r.block<1,3>(m,0) = cr.first * (d(m-2)+d(m-1));
                    r_rot.block<1,3>(m,0) = cr.second * (d(m-2)+d(m-1));
                }
            }

            sol = A.colPivHouseholderQr().solve(r);
            sol_rot = A.colPivHouseholderQr().solve(r_rot);
            std::vector<Eigen::Vector3d> controlPoints;
            std::vector<Eigen::Vector3d> controlPointsRot;
    //std::vector<double> normalizedKnots;
            C2_Point c0 = shortcut ? (*previousTrajectory)(ta) : (*initPath)(ta);
            controlPoints.push_back(c0.first);
            controlPointsRot.push_back(c0.second);
            for(int i =0; i<sol.rows();++i)
            {
                controlPoints.push_back(sol.block<1,3>(i,0));
                controlPointsRot.push_back(sol_rot.block<1,3>(i,0));
            }
            C2_Point cEnd = shortcut ? previousTrajectory->max() : initPath->max();
            controlPoints.push_back(cEnd.first);
            controlPointsRot.push_back(cEnd.second);
            /*multiplicity at start and end*/
            for(int i =0; i<3;++i)
            {
                knots.insert(knots.begin(), ta);
            }
            for(int i =0; i<3;++i)
            {
                knots.push_back(tb);
            }
            //normalizedKnots.push_back(1);
            if(normalize)
            {
                Normalize norm (ta, tb);
                std::for_each(knots.begin(), knots.end(), norm);
            }
            SplinePath test(controlPoints, controlPointsRot, knots,knots.back());
            success = CheckReachability(collider, model, test).empty() ;
            if(success || tries == 1) // TODO BE SMARTER
            {
                return test;
            }
            else
            {
                m = m*2;
                tries--;
            }
        }
        //throw "TODO";
    }

    planner::SplinePath MergeSpline(const planner::SplinePath& s0, const planner::SplinePath& s1, const double ta, const double tb, bool normalize)
    {
        std::vector<double> knots;
        std::vector<Eigen::Vector3d> controlPoints;
        std::vector<Eigen::Vector3d> controlPointsRot;
        const int multipliedPoints = 3;

        std::vector<double>::const_iterator tait = std::find_if(s0.knots_.begin(),s0.knots_.end(),FirstAbove(ta));
        int distance = std::distance(s0.knots_.begin(), tait) - multipliedPoints; //3 first points are repeated

        controlPoints.insert(controlPoints.end(), s0.controlPoints_.begin(), s0.controlPoints_.begin()+distance);
        controlPointsRot.insert(controlPointsRot.end(), s0.controlPointsRot_.begin(), s0.controlPointsRot_.begin()+distance);
        knots.insert(knots.end(), s0.knots_.begin(), tait);

        controlPoints.insert(controlPoints.end(), s1.controlPoints_.begin() + 1, s1.controlPoints_.end() -1);
        controlPointsRot.insert(controlPointsRot.end(), s1.controlPointsRot_.begin() + 1, s1.controlPointsRot_.end() -1);
        knots.insert(knots.end(), s1.knots_.begin() + multipliedPoints -1 , s1.knots_.end() -multipliedPoints + 1); //3 first ad last points are repeated

        std::vector<double>::const_iterator tbit = std::find_if(s0.knots_.begin(),s0.knots_.end(),FirstAbove(tb));
        distance = std::distance(s0.knots_.begin(), tbit) - multipliedPoints; //3 first points are repeated

        controlPoints.insert(controlPoints.end(), s0.controlPoints_.begin() + distance, s0.controlPoints_.end());
        controlPointsRot.insert(controlPointsRot.end(), s0.controlPointsRot_.begin() + distance, s0.controlPointsRot_.end());
        knots.insert(knots.end(), tbit, s0.knots_.end());
        if(normalize)
        {
            Normalize norm(s0.knots_.front(), s0.knots_.back());
            std::for_each(knots.begin(), knots.end(), norm);
        }
        return SplinePath(controlPoints, controlPointsRot, knots, knots.back());
    }

}

planner::SplinePath planner::SplineFromPath(Collider& collider, CT_Model& path, double maxSpeed, double maxAcceleration, bool normalize)
{
    int m = path.size();
    InterpolatePath initPath(path);

    //sample m + 1 points along the linear trajectory
    double ta = 0;  double tb = initPath.tmax();
    bool success = false;
    return createSpline(collider, *(path.front()),&initPath, 0, maxSpeed, maxAcceleration, ta, tb, m, 5, success, normalize);
}

planner::SplinePath planner::SplineShortCut(Collider& collider, CT_Model& path, double maxSpeed, double maxAcceleration, int nbSteps)
{
    // first iteration, create initial spline.
    // 2 kind of methods
    // straight line, or using the guide spline. As the steps increase, there are less and less chances of
    // having a straight line working, so proba increases as we go
    int currentStep=0;
    float proba;
    // true if straightLine method is chosen, false if spline is followed

    planner::SplinePath currentSpline = SplineFromPath(collider, path, maxSpeed, maxAcceleration, nbSteps == 0);
    planner::ParamFunction* paramFunction(0);
    while(nbSteps > currentStep)
    {
        //select two random knots
        // sample 2 points t_a et t_b
        double ta, tb;
        do
        {
            ta = (double)std::rand() / (double)RAND_MAX * currentSpline.tmax();
            tb = (double)std::rand() / (double)RAND_MAX * currentSpline.tmax();
            if(tb < ta)
            {
                double tmp = ta; tb = ta; ta = tmp;
            }
        } while(std::abs(ta - tb) < 0.01 || ta==0 || tb== currentSpline.tmax()); // check this to faciliate merge

        InterpolatePath interpolatePath(currentSpline(ta), currentSpline(tb), ta, tb);
        proba = (float)currentStep / (float)nbSteps;
        int m = currentSpline.controlPoints_.size();
        if(((float)std::rand() / (float)RAND_MAX) > proba) // straightline
        {
            paramFunction = &interpolatePath;
            m = 20;
        }
        else
        {
            paramFunction = &currentSpline;
        }
        bool success = false;
        planner::SplinePath subSpline = createSpline(collider,  *(path.front()), paramFunction, &currentSpline, maxSpeed, maxAcceleration, ta, tb, m, 3, success, false, true);
        if(success) currentSpline = MergeSpline(currentSpline,subSpline, ta, tb, currentStep == nbSteps);
        currentStep++;
        if(currentStep == nbSteps)
        {
            std::vector<double> knots = currentSpline.knots_;
            Normalize norm (0, 1);
            std::for_each(knots.begin(), knots.end(), norm);
            currentSpline = SplinePath(currentSpline.controlPoints_, currentSpline.controlPointsRot_, knots, knots.back());
        }
    }
    return currentSpline;
}
