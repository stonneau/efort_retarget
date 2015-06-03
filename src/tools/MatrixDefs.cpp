#include "MatrixDefs.h"
#include "math.h"

using namespace matrices;

void matrices::Matrix3ToMatrix4(const Matrix3& from, Matrix4& to)
{
	to.block(0,0,3,3) = from;
	to(3,3) = 1;
}

void matrices::Matrix4ToMatrix3(const Matrix4& from, Matrix3& to)
{
    to = from.block(0,0,3,3);
}

/* Rotates rotated around axis by angle theta, and returns it */
Matrix4 matrices::Translate(numeric x, numeric y, numeric z)
{								
	Matrix4 mat = Matrix4::Zero();				
	mat(3,0) = x;
	mat(3,1) = y;
	mat(3,2) = z;
	mat(3,3) = 1;
	return mat;
}
Matrix4 matrices::Translate(Vector3 vector)
{								
	Matrix4 mat = Matrix4::Zero();				
	mat(0,3) = vector(0);
	mat(1,3) = vector(1);
	mat(2,3) = vector(2);
	mat(3,3) = 1;
	return mat;
}
Matrix4 matrices::Rotx4(numeric theta)
{
	numeric c = cos(theta);								
	numeric s = sin(theta);								
	Matrix4 mat = Matrix4::Zero();				
	mat(0,0) = 1;					//	1 	0	0
	mat(1, 1) = c; mat(1, 2) = -s;	//	0	cos -sin
	mat(2, 1) = s; mat(2, 2) = c;	//	0	sin	cos	
	return mat;
}
Matrix4 matrices::Roty4(numeric theta)
{
	numeric c = cos(theta);										
	numeric s = sin(theta);										
	Matrix4 mat = Matrix4::Zero();		
	mat(0,0) = c; mat(0,2) = s;		//	cos	 0	sin
	mat(1, 1) = 1;					//	0	 1	0
	mat(2,0) = -s; mat(2, 2) = c;	//	-sin 0	cos
	return mat;
}
Matrix4 matrices::Rotz4(numeric theta)
{
	numeric c = cos(theta);										
	numeric s = sin(theta);										
	Matrix4 mat = Matrix4::Zero();		
	mat(0,0) = c; mat(0,1) = -s ;	//	cos	-sin 0
	mat(1, 0) = s; mat(1, 1) = c;	//	sin	cos	 0
	mat(2, 2) = 1;					//	0	0	 1
	return mat;
}

Matrix3 matrices::Rotx3(numeric theta)
{
	numeric c = cos(theta);								
	numeric s = sin(theta);								
	Matrix3 mat = Matrix3::Zero();				
	mat(0,0) = 1;					//	1 	0	0
	mat(1,1) = c; mat(1, 2) = -s;	//	0	cos -sin
	mat(2, 1) = s; mat(2, 2) = c;	//	0	sin	cos	
	return mat;
}
Matrix3 matrices::Roty3(numeric theta)
{
	numeric c = cos(theta);										
	numeric s = sin(theta);										
	Matrix3 mat = Matrix3::Zero();		
	mat(0,0) = c; mat(0,2) = s;		//	cos	 0	sin
	mat(1, 1) = 1;					//	0	 1	0
	mat(2,0) = -s; mat(2, 2) = c;	//	-sin 0	cos
	return mat;
}
Matrix3 matrices::Rotz3(numeric theta)
{
	numeric c = cos(theta);										
	numeric s = sin(theta);										
	Matrix3 mat = Matrix3::Zero();		
	mat(0,0) = c; mat(0,1) = -s ;	//	cos	-sin 0
	mat(1, 0) = s; mat(1, 1) = c;	//	sin	cos	 0
	mat(2, 2) = 1;					//	0	0	 1
	return mat;
}

// project vector onto another and returns cosinnus angle
double matrices::Project(const Vector3& from, const Vector3& to)
{
	double fromNorm = from.norm();
	if(fromNorm == 0) return 0.;
	Vector3 toNormalized = to; toNormalized.normalize();
	double cosA = (from(0) * toNormalized(0) + from(1) * toNormalized(1) + from(2) * toNormalized(2) ) / fromNorm;
	return cosA;
}

Vector3 matrices::ProjectOnPlan(const Vector3& normalAxis, const Vector3& vector)
{
	return normalAxis.cross(vector.cross(normalAxis));
}

bool matrices::NotZero(const Vector3& vect)
{
	return (!(vect(0) == 0 && vect(1) == 0 && vect(2) == 0));
}

/*Tu considères u,v deux vecteurs de norme L2 = 1 dans R^3
Tu cherches la rotation R, telle que Ru=v.
R = cos theta * I + (I x [a,a,a])^T * sin theta + (1 - cos theta) * a*a^T
avec :
cos theta = u . v
sin theta = ||u x v||
a=u x v / sin theta
I étant l'identité, * le produit matriciel, x le cross product et ^T la transposée.
http://fr.wikipedia.org/wiki/Rotation_vectorielle
Dérivée de la formule de Rodriguez*/
void matrices::GetRotationMatrix(const Vector3& from, const Vector3& to, Matrix3& result)
{
	Vector3 u, v, uXv, a;
	u = from; u.normalize();
	v = to  ; v.normalize();
	uXv = u.cross(v);
	numeric sinTheta = uXv.norm();
	if (sinTheta == 0) // angle is 0
	{
		result = Matrix3::Identity();
	}
	else
	{
		numeric cosTheta = u.dot(v);
		a = uXv / sinTheta;

		Matrix3 I = Matrix3::Identity();

		Matrix3 Iaaa = Matrix3::Zero();

		Iaaa(0,1) = - a(2); Iaaa(0,2) =  a(1); //  0  -z   y
		Iaaa(1,0) =   a(2); Iaaa(1,2) = -a(0); //  z   0  -x
		Iaaa(2,0) = - a(1); Iaaa(2,1) =  a(0); // -y   x   0

		result = I * cosTheta + sinTheta * Iaaa + (1 - cosTheta) * (a*a.transpose());
	}
}

Matrix3 matrices::RotationMatrixFromNormal(const Vector3& normal)
{
    int imin = 0;
    for(int i=0; i<3; ++i)
        if(std::abs(normal[i]) < std::abs(normal[imin]))
            imin = i;

    Vector3 v2(0,0,0);
    float dt    = normal[imin];

    v2[imin] = 1;
    for(int i=0;i<3;i++)
        v2[i] -= dt*normal[i];

    v2.normalize();
    Vector3 v3 = normal.cross(v2);
    Matrix3 res;
    res.block(0,0,3,1) = v3;
    res.block(0,1,3,1) = normal;
    res.block(0,2,3,1) = v2;
    return res;
}

// TODO forward dec
Vector3& matrices::Rotate(const Vector3& axis, Vector3& rotated, numeric theta)
{
	numeric x  = rotated.x(); numeric y  = rotated.y(); numeric z = rotated.z();
	numeric x1 = axis.x(); numeric y1 = axis.y(); numeric z1 = axis.z();

	numeric c = cos(theta);
	numeric s = sin(theta);
	numeric dotw = (x*x1 + y*y1 + z*z1);
	numeric v0x = dotw*x1;
	numeric v0y = dotw*y1;		// v0 = provjection onto axis
	numeric v0z = dotw*z1;
	numeric v1x = x-v0x;
	numeric v1y = y-v0y;			// v1 = projection onto plane normal to axis
	numeric v1z = z-v0z;
	numeric v2x = y1*v1z - z1*v1y;
	numeric v2y = z1*v1x - x1*v1z;	// v2 = axis * v1 (cross product)
	numeric v2z = x1*v1y - y1*v1x;
		
	rotated(0) = v0x + c*v1x + s*v2x;
	rotated(1) = v0y + c*v1y + s*v2y;
	rotated(2) = v0z	+ c*v1z + s*v2z;
	return rotated;
}


void matrices::vect4ToVect3(const VectorX& from, Vector3& to)
{
	to = from.block(0,0,3,1);
}

void matrices::vect3ToVect4(const Vector3& from, VectorX& to)
{
	to.block(0,0,3,1) = from;
	to(3) = 1;
}

Vector3 matrices::matrix4TimesVect3(const Matrix4& mat4, const Vector3& vect3)
{
	VectorX vect4(4);
	Vector3 res;
	vect3ToVect4(vect3, vect4);
	vect4 = mat4 * vect4;
	vect4ToVect3(vect4, res);
	return res;
}

Vector3 matrices::matrix4TimesVect4(const Matrix4& mat4, const VectorX& vect4)
{
	Vector3 vect3; VectorX res(4);
	res = mat4 * vect4;
	vect4ToVect3(res, vect3);
	return vect3;
}
