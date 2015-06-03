#include "ExpMap.h"

using namespace matrices;

void ExpMap::setRotation(const Quat &q)
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

const Quat& ExpMap::getRotation() const
{
     return m_q;
}

Matrix3 ExpMap::getMatrix() const
{
    return m_q.toRotationMatrix();
}

void ExpMap:: update(const Vector3& dv)
{
     m_v += dv;
     angleUpdated();
}

void ExpMap::partialDerivatives(Matrix3& dRdx,Matrix3& dRdy,Matrix3& dRdz) const
{
     Quat dQdx[3];
     compute_dQdVi(dQdx);
     compute_dRdVi(dQdx[0], dRdx);
     compute_dRdVi(dQdx[1], dRdy);
     compute_dRdVi(dQdx[2], dRdz);
 }

void ExpMap::compute_dRdVi(const Quat &dQdvi, Matrix3 & dRdvi) const
{
     numeric  prod[9];

     /* This efficient formulation is arrived at by writing out the
      * entire chain rule product dRdq * dqdv in terms of 'q' and
      * noticing that all the entries are formed from sums of just
      * nine products of 'q' and 'dqdv' */

     prod[0] = -numeric(4)*m_q.x()*dQdvi.x();
     prod[1] = -numeric(4)*m_q.y()*dQdvi.y();
     prod[2] = -numeric(4)*m_q.z()*dQdvi.z();
     prod[3] = numeric(2)*(m_q.y()*dQdvi.x() + m_q.x()*dQdvi.y());
     prod[4] = numeric(2)*(m_q.w()*dQdvi.z() + m_q.z()*dQdvi.w());
     prod[5] = numeric(2)*(m_q.z()*dQdvi.x() + m_q.x()*dQdvi.z());
     prod[6] = numeric(2)*(m_q.w()*dQdvi.y() + m_q.y()*dQdvi.w());
     prod[7] = numeric(2)*(m_q.z()*dQdvi.y() + m_q.y()*dQdvi.z());
     prod[8] = numeric(2)*(m_q.w()*dQdvi.x() + m_q.x()*dQdvi.w());

     /* first row, followed by second and third */
     dRdvi(0,0) = prod[1] + prod[2];
     dRdvi(0,1) = prod[3] - prod[4];
     dRdvi(0,2) = prod[5] + prod[6];

     dRdvi(1,0) = prod[3] + prod[4];
     dRdvi(1,1) = prod[0] + prod[2];
     dRdvi(1,2) = prod[7] - prod[8];

     dRdvi(2,0) = prod[5] - prod[6];
     dRdvi(2,1) = prod[7] + prod[8];
     dRdvi(2,2) = prod[0] + prod[1];
 }

 // compute partial derivatives dQ/dVi

namespace
{
    numeric& qid(Quat& q, int i)
    {
        switch(i)
        {
            case 0:
            return q.x();

            case 1:
            return q.y();

            case 2:
            return q.z();

            default:
            return q.w(); // should Never happen
        }
    }
}

void ExpMap::compute_dQdVi(Quat *dQdX) const
{

     /* This is an efficient implementation of the derivatives given
      * in Appendix A of the paper with common subexpressions factored out */
     numeric sinc, termCoeff;

     if (m_theta < ExpMap_MINANGLE)
     {
         sinc = 0.5 - m_theta*m_theta/48.0;
         termCoeff = (m_theta*m_theta/40.0 - 1.0)/24.0;
     }
     else
     {
         numeric cosp = m_q.w();
         numeric ang = 1.0/m_theta;

         sinc = m_sinp*ang;
         termCoeff = ang*ang*(0.5*cosp - sinc);
     }

     for (int i = 0; i < 3; i++)
     {
         Quat& dQdx = dQdX[i];
         int i2 = (i+1)%3;
         int i3 = (i+2)%3;

         numeric term = m_v[i]*termCoeff;

         qid(dQdx,i)  = term*m_v[i] + sinc;
         qid(dQdx,i2) = term*m_v[i2];
         qid(dQdx,i3) = term*m_v[i3];
         dQdx.w() = -0.5*m_v[i]*sinc;
     }
 }

 // reParametize away from singularity, updating
 // m_v and m_theta

void ExpMap::reParametrize()
{
    if (m_theta > M_PI)
    {
        numeric scl = m_theta;
        if (m_theta > 2*M_PI)
        { /* first get theta into range 0..2PI */
            m_theta = numeric(fmod(m_theta, 2*M_PI));
            scl = m_theta/scl;
            m_v *= scl;
        }
        if (m_theta > M_PI)
        {
            scl = m_theta;
            m_theta = 2*M_PI - m_theta;
            scl = numeric(1.0) - 2*M_PI/scl;
            m_v *= scl;
        }
    }
}

 // compute cached variables
void ExpMap::angleUpdated()
{
     m_theta = m_v.norm();

     reParametrize();

     // compute quaternion, sinp and cosp

     if (m_theta < ExpMap_MINANGLE)
     {
         m_sinp = numeric(0.0);

         /* Taylor Series for sinc */
         Vector3 temp = m_v * numeric(numeric(.5) - m_theta*m_theta/numeric(48.0));
         m_q.x() = temp.x();
         m_q.y() = temp.y();
         m_q.z() = temp.z();
         m_q.w() = numeric(1.0);
     }
     else
     {
         m_sinp = numeric(sin(.5*m_theta));

         /* Taylor Series for sinc */
         Vector3 temp = m_v * (m_sinp/m_theta);
         m_q.x() = temp.x();
         m_q.y() = temp.y();
         m_q.z() = temp.z();
         m_q.w() = numeric(cos(.5*m_theta));
     }
 }

Vector3 ExpMap::log() const
{
    // v = log(q) = 2 * cos-1 (q_w) / |qv| * qv
    return m_v.norm() == 0 ? m_v : (2 * std::acos(m_q.w())) / (m_q.vec().norm()) * m_q.vec();
    /*numeric theta = v.norm(); if(v.norm() != 0) v.normalize();
    return AngleAx(theta,v);*/
}
