/**
* \file ExpMap.h
* \brief Utility class implementing exponential maps according to Grassia paper:
* "Pratical parameterization of Rotations Using the Exponential Map."
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/
#include <Eigen/Dense>
#include <tools/MatrixDefs.h>

#ifndef _EXPMAP
#define _EXPMAP


#if (USEFLOAT)
    typedef float numeric;
    typedef Eigen::Quaternionf Quat;
    typedef Eigen::AngleAxisf AngleAx;
#else
    typedef double numeric;
    typedef Eigen::Quaterniond Quat;
typedef Eigen::AngleAxisd AngleAx;
#endif



namespace matrices
{
const numeric ExpMap_MINANGLE (1e-7);

class ExpMap {
public:
    ExpMap() {}
    ExpMap(const Vector3& v) : m_v(v) { angleUpdated(); }

    ExpMap(numeric x, numeric y, numeric z) :
        m_v(x, y, z) { angleUpdated(); }

    ExpMap(const Quat &q)
    {
        setRotation(q);
    }

    ExpMap(const Matrix3 &m)
    {
        setRotation(Quat(m));
    }

    const Vector3& vector() const
    {
        return m_v;
    }

    void setRotation(const Quat &q)
    {
         // ok first normalize the quaternion
         // then compute theta the axis-angle and the normalized axis v
         // scale v by theta and that's it hopefully!

        m_q = q; m_q.normalize();
        m_v = Vector3(m_q.x(), m_q.y(), m_q.z());

        numeric cosp = m_q.w();
        m_sinp = m_v.norm();
        if(m_sinp != 0)
        {
            m_v /= m_sinp;
            m_theta = atan2(double(m_sinp),double(cosp));
            m_v *= m_theta;
        }
     }

    const Quat& getRotation() const;

    Matrix3 getMatrix() const;

    void update(const Vector3& dv);

    void partialDerivatives(Matrix3& dRdx, Matrix3& dRdy, Matrix3& dRdz) const;

    Vector3 log() const{
        // v = log(q) = 2 * cos-1 (q_w) / |qv| * qv
        return m_v.norm() == 0 ? m_v : (2 * std::acos(m_q.w())) / (m_q.vec().norm()) * m_q.vec();
        /*numeric theta = v.norm(); if(v.norm() != 0) v.normalize();
        return AngleAx(theta,v);*/
    }


 private :
     // m_v contains the exponential map, the other variables are
     // cached for efficiency

     Vector3 m_v;
     numeric m_theta, m_sinp;
     Quat m_q;

     // private methods
     // Compute partial derivatives dR (3x3 rotation matrix) / dVi (EM vector)
     // given the partial derivative dQ (Quaternion) / dVi (ith element of EM vector)

    void compute_dRdVi(const Quat &dQdV, Matrix3 & dRdVi) const;

     // compute partial derivatives dQ/dVi

    void compute_dQdVi(Quat *dQdX) const;

     // reparametrize away from singularity

    void reParametrize();

     // (re-)compute cached variables

    void angleUpdated();
 };

} //namespace matrices

#endif //_EXPMAP
