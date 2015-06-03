
#include "MatrixDefs.h"

#ifndef _MATRIXDEFSINTERNAL
#define _MATRIXDEFSINTERNAL

namespace matrices
{

#if (USEFLOAT)
    typedef Eigen::MatrixXf MatrixX;
#else
    typedef Eigen::MatrixXd MatrixX;
#endif

//REF: boulic et al An inverse kinematics architecture enforcing an arbitrary numeric of strict priority levels
template<typename _Matrix_Type_>
void PseudoInverseDLS(_Matrix_Type_& pinvmat,  numeric lambda)
{
	Eigen::JacobiSVD<_Matrix_Type_> svd(pinvmat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    VectorX m_sigma = svd.singularValues();
// temp computation foireuse pour lambda
// REF: must apply numerical filtering for the operation of robotic manipulators through kinematically singular ...
	bool found = false; int i = m_sigma.rows() -1;
	numeric val = 0;
	while (!found && i >= 0)
	{
		val = m_sigma(i);
		found = m_sigma(i) > 0;
		if (found) lambda = val  / 1.f;
		i--;
	}
//end tmp

    numeric  pinvtoler= numeric(1.e-6); // choose your tolerance widely!
	numeric lambda2 = lambda * lambda;

	MatrixX m_sigma_inv = MatrixX::Zero(pinvmat.cols(),pinvmat.rows());
	for (int i=0; i<m_sigma.rows(); ++i)
	{
		if (m_sigma(i) > pinvtoler)
			m_sigma_inv(i,i)=m_sigma(i)/(m_sigma(i) * m_sigma(i) + lambda2);
	}
	pinvmat = (svd.matrixV()*m_sigma_inv*svd.matrixU().transpose());
}

template<typename _Matrix_Type_>
void PseudoInverseSVDDLS(_Matrix_Type_& pinvmat, Eigen::JacobiSVD<_Matrix_Type_>& svd, _Matrix_Type_& dest, numeric lambda = 0.f)
{
	VectorX m_sigma = svd.singularValues();
		
	// temp computation foireuse pour lambda
	// REF: must apply numerical filtering for the operation of robotic manipulators through kinematically singular ...
	bool found = false; int i = m_sigma.rows() -1;
	numeric val = 0;
	while (!found && i >= 0)
	{
		val = m_sigma(i);
		found = m_sigma(i) > 0;
		if (found) lambda = val / 1.f;
		i--;
	}
	//end tmp
	numeric  pinvtoler = numeric(lambda != 0 ? 0 : 1.e-6); // choose your tolerance widely!
	numeric lambda2 = lambda * lambda;
		
	MatrixX m_sigma_inv = MatrixX::Zero(pinvmat.cols(),pinvmat.rows());
	for (int i=0; i<m_sigma.rows(); ++i)
	{
		if (m_sigma(i) > pinvtoler)
			m_sigma_inv(i,i)=m_sigma(i)/(m_sigma(i) * m_sigma(i) + lambda2);
			//m_sigma_inv(i,i)=1.0/m_sigma(i);
	}
	dest= (svd.matrixV()*m_sigma_inv*svd.matrixU().transpose());
}

//REF: boulic et al An inverse kinematics architecture enforcing an arbitrary numeric of strict priority levels
template<typename _Matrix_Type_>
void PseudoInverse(_Matrix_Type_& pinvmat)
{
	Eigen::JacobiSVD<_Matrix_Type_> svd(pinvmat, Eigen::ComputeFullU | Eigen::ComputeFullV);
	VectorX m_sigma = svd.singularValues();

	numeric  pinvtoler= 1.e-6; // choose your tolerance widely!

	MatrixX m_sigma_inv = MatrixX::Zero(pinvmat.cols(),pinvmat.rows());
	for (long i=0; i<m_sigma.rows(); ++i)
	{
		if (m_sigma(i) > pinvtoler)
			m_sigma_inv(i,i)=1.0/m_sigma(i);
	}
	pinvmat = (svd.matrixV()*m_sigma_inv*svd.matrixU().transpose());
}

}//namespace matrices

#endif //_MATRIXDEFSINTERNAL
