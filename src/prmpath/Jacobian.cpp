
#include "Jacobian.h"
#include "prmpath/Robot.h"

#include "tools/MatrixDefsInternal.h"

using namespace planner;
using namespace matrices;
using namespace Eigen;

Jacobian::Jacobian(Node* root, double lambdaInv, bool full)
    : lambdaInv_(lambdaInv)
{
    ComputeJacobian(root, full);
}

Jacobian::~Jacobian()
{
	//NOTHING
}

void Jacobian::SetJacobian(const MatrixX& jacobian)
{
	jacobian_ = jacobian;
	Invalidate();
}

void Jacobian::Invalidate()
{
	computeInverse_ = true; computeProduct_ = true; computeProductInverse_ = true;
	computeJacSVD_ = true; computeNullSpace_ = true;
}

namespace
{
    Eigen::Vector3d ComputeRotationAxis(Node* node, Node* root)
    {
        Node* y = node->parent;
        Eigen::Vector3d w_ = node->axis;							// Initialize to local rotation axis
        while (y) {
            Rotate(y->axis, w_, y->value);
            //if(y->id == root->id) break;
            y = y->parent;
        }
        return w_;
    }
}

void Jacobian::ComputeJacobian(Node* root, bool full)
{
    /*Eigen::Matrix4d toRootCoordinates = Eigen::Matrix4d::Identity();
    toRootCoordinates.block<3,3>(0,0) = root->toLocalRotation;
    toRootCoordinates.block<3,1>(0,3) = -root->position;
    Eigen::Matrix4d toWorldCoordinates = toRootCoordinates;
    toWorldCoordinates.inverse();*/
	Invalidate();
    int dim = full ? planner::GetNumNodes(root)  :  planner::GetNumChildren(root);
    std::vector<Node*> effectors = full ? planner::AsVector(root) : planner::GetEffectors(root, true);
    jacobian_ = MatrixX(3 * effectors.size(), dim);
    jacobian_.setZero(3 * effectors.size(),dim);
    // Traverse this to find all end effectors
    for(int i=0; i!=effectors.size(); ++i)
    {
        Node* nodeEffector = effectors[i];
        Node* currentNode = nodeEffector;
        Eigen::Vector3d effectorPos = nodeEffector->position;

        while(currentNode->id != root->id)
        {
            currentNode = currentNode->parent;
            Eigen::Vector3d siMinuspj = effectorPos -
                    currentNode->position; //);
            Eigen::Vector3d vj = ComputeRotationAxis(currentNode, root);//root->toLocalRotation * currentNode->axis; // ComputeRotationAxis(currentNode); //currentNode->toWorldRotation * currentNode->axis;
            jacobian_.block<3,1>(3*i,currentNode->id - root->id) = vj.cross(siMinuspj);
        }
    }
}

void OnePositionJacobian(planner::Node* root, planner::Node* position, int i)
{
    /*Node* currentNode = position;
    Eigen::Vector3d effectorPos = nodeEffector->position;
    while(currentNode->id != root->id)
    {
        currentNode = currentNode->parent;
        Eigen::Vector3d siMinuspj = effectorPos -
                currentNode->position; //);
        Eigen::Vector3d vj = ComputeRotationAxis(currentNode, root);//root->toLocalRotation * currentNode->axis; // ComputeRotationAxis(currentNode); //currentNode->toWorldRotation * currentNode->axis;
        jacobian_.block<3,1>(3*i,currentNode->id - root->id) = vj.cross(siMinuspj);
    }*/
}

void Jacobian::GetEllipsoidAxes(Eigen::Vector3d &u1, Eigen::Vector3d &u2, Eigen::Vector3d &u3)
{
	GetJacobianProduct();
	svdProduct_ = Eigen::JacobiSVD<MatrixX>(jacobianProduct_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	u1 = svdProduct_.matrixU().block(0,0,3,1) / (svdProduct_.singularValues()(0) + 0.000000000000000000000000001);
	u2 = svdProduct_.matrixU().block(0,1,3,1) / (svdProduct_.singularValues()(1) + 0.000000000000000000000000001);
	u3 = svdProduct_.matrixU().block(0,2,3,1) / (svdProduct_.singularValues()(2) + 0.000000000000000000000000001);
	// TODO
}

void Jacobian::GetEllipsoidAxes(matrices::Vector3& u1, matrices::Vector3& u2, matrices::Vector3& u3, double& sig1, double& sig2, double& sig3)
{
	GetJacobianProduct();
	svdProduct_ = Eigen::JacobiSVD<MatrixX>(jacobianProduct_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	u1 = svdProduct_.matrixU().block(0,0,3,1);
	u2 = svdProduct_.matrixU().block(0,1,3,1);
	u3 = svdProduct_.matrixU().block(0,2,3,1);
	sig1 = 1. / (svdProduct_.singularValues()(0) + 0.000000000000000000000000001);
	sig2 = 1. / (svdProduct_.singularValues()(1) + 0.000000000000000000000000001);
	sig3 = 1. / (svdProduct_.singularValues()(2) + 0.000000000000000000000000001);
}


void Jacobian::ComputeAll(Node *root)
{
    ComputeJacobian(root);
	ComputeAll();
}
void Jacobian::ComputeAll()
{
	ComputeSVD();
	GetJacobianInverse();
	GetJacobianProduct();
	GetJacobianProductInverse();
	GetNullspace();
}

const MatrixX& Jacobian::GetJacobian()
{
	return jacobian_;
}

const MatrixX& Jacobian::GetJacobianInverse()
{
	if(computeInverse_)
	{
		computeInverse_ = false;
		jacobianInverse_ = jacobian_;
        PseudoInverseDLS(jacobianInverse_, lambdaInv_); // tmp while figuring out how to chose lambda
	}
	return jacobianInverse_;
}

const MatrixX& Jacobian::GetNullspace()
{
	if(computeNullSpace_)
	{
        computeNullSpace_ = false;
		MatrixX id = MatrixX::Identity(jacobian_.cols(), jacobian_.cols());
        ComputeSVD();
		MatrixX res = MatrixX::Zero(id.rows(), id.cols());
		for(int i =0; i < svd_.matrixV().cols(); ++ i)
		{
			VectorX v = svd_.matrixV().col(i);
			res += v * v.transpose();
		}
        Identitymin_ = id - res;
	}
	return Identitymin_;
}

void Jacobian::GetNullspace(const MatrixX pseudoId, MatrixX& result)
{
		GetNullspace(); // computing inverse jacobian

		MatrixX id = MatrixX::Identity(Identitymin_.rows(), Identitymin_.cols());
        result = pseudoId - (id + Identitymin_);
}

const Eigen::MatrixXd& Jacobian::GetJacobianProduct()
{
	if(computeProduct_)
	{
		computeProduct_ = false;
		jacobianProduct_ = jacobian_ * jacobian_.transpose();
	}
	return jacobianProduct_;
}

const Eigen::MatrixXd &Jacobian::GetJacobianProductInverse()
{
	if(computeProductInverse_)
	{
		computeProductInverse_ = false;
        Eigen::JacobiSVD<MatrixX> svd = Eigen::JacobiSVD<MatrixX>(jacobianProduct_, Eigen::ComputeFullU | Eigen::ComputeFullV);
		PseudoInverseSVDDLS(jacobianProduct_, svd, jacobianProductInverse_);
	}
	return jacobianProductInverse_;
}

void Jacobian::ComputeSVD()
{
	if(computeJacSVD_)
	{
		computeJacSVD_ = false;
		svd_ = Eigen::JacobiSVD<MatrixX>(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	}
}

