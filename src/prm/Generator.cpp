#include "Generator.h"
#include "MatrixDefsInternal.h"

#include <time.h>
#include <math.h>

namespace
{
    static bool generatorInit = false;

    void DoubleArrayToVector(Eigen::Vector3d& to, const PQP_REAL* from)
    {
        for(int i=0; i<3; ++i)
        {
            to(i) = from[i];
        }
    }
}

namespace
{
    double Length(const double a[3], const double b[3])
    {
        double res = 0;
        for(int i=0; i<3; ++i)
        {
            res += (b[i] - a [i]) *(b[i] - a [i]);
        }
        return sqrt(res);
    }
    // Heron formula
    double TriangleArea(const Tri& tri)
    {
        double a, b, c;
        a = Length(tri.p1, tri.p2);
        b = Length(tri.p2, tri.p3);
        c = Length(tri.p3, tri.p1);
        double s = 0.5 * (a + b + c);
        return sqrt(s * (s-a) * (s-b) * (s-c));
    }

    void copy(const std::vector<size_t>& from, std::vector<size_t>& to)
    {
        for(std::vector<size_t>::const_iterator cit = from.begin();
            cit != from.end(); ++cit)
        {
            to.push_back(*cit);
        }
    }
}

using namespace planner;

Generator::Generator(Object::T_Object &objects, Object::T_Object &collisionObjects, const Model &model)
    : model_(model)
    , objects_(objects)
    , contactObjects_(collisionObjects)
    , collider_(collisionObjects)
{
    if(! ::generatorInit)
    {
        ::generatorInit = true;
        srand((unsigned int)(time(0))); //Init Random generation
    }
    InitWeightedTriangles();
}

Generator::~Generator()
{
    // NOTHING
}

Model* Generator::operator()()
{
   std::vector<std::size_t> limbs;
   return this->operator ()(limbs);
}

Model* Generator::operator()(std::vector<std::size_t>& limbs)
{
    // en v0, juste les positions
    int limit = 10000;

    Eigen::Vector3d pos;
    while(limit > 0)
    {
        -- limit;
        Model configuration(model_);
        // pick one object randomly
        SampleTriangle sampled;
        double r = ((double) rand() / (RAND_MAX));
        if(r > 0.3)
            sampled = RandomPointIntriangle();
        else
            sampled = WeightedTriangles();
        const Tri& triangle = *(sampled.second.second);
        const Eigen::Vector3d& triangleNormal = (sampled.second.first);
        //http://stackoverflow.com/questions/4778147/sample-random-point-in-triangle
        Eigen::Vector3d A, B, C;
        DoubleArrayToVector(A, triangle.p1);
        DoubleArrayToVector(B, triangle.p2);
        DoubleArrayToVector(C, triangle.p3);
        double r1, r2;
        r1 = ((double) rand() / (RAND_MAX)); r2 = ((double) rand() / (RAND_MAX));
        Eigen::Vector3d P = (1 - sqrt(r1)) * A + (sqrt(r1) * (1 - r2)) * B + (sqrt(r1) * r2) * C;
if(P.y() > -0.3)
		{
			configuration.SetPosition(P);
			// random rotation
            double rx, rz; double ry = ((double) rand() / (RAND_MAX))  * M_PI *2;
            matrices::Matrix3 tranform = matrices::Rotz3(ry);
            matrices::Matrix3 tranformComplete = tranform;
            //asserting that x is not pointing upward
            matrices::Vector3 y = matrices::Vector3(0,1,0);
            matrices::Vector3 x = matrices::Vector3(1,0,0);
            matrices::Vector3 z = matrices::Vector3(0,0,1);
            do
            {
                rx = ((double) rand() / (RAND_MAX)) * M_PI * 2;
                rz = ((double) rand() / (RAND_MAX)) * M_PI *2;
// ry = ((double) rand() / (RAND_MAX))  * M_PI *2;
                 ry = 0;
//                ry = ((double) rand() / (RAND_MAX))  * M_PI / 8; // climb
                tranformComplete = matrices::Rotz3(rz);
                tranformComplete*= matrices::Roty3(ry);
                tranformComplete*= matrices::Rotx3(rx);
            }
            //while( (false // && // torso not facing upward
            while( !(y.dot(tranformComplete.block<3,1>(0,0)) < 0 // && // torso not facing upward
                    // y.dot(tranformComplete.block<3,1>(0,1)) > 0.3 &&
                     //&& x.dot(tranformComplete.block<3,1>(0,1)) < -0.1
                   )); // head not pointing too down
           // while( y.dot(tranformComplete.block<3,1>(0,0)) < 0.9);
            //while(false);
			// find random direction
			int limit2 = 100;
			int limitstraight = 2;
            // first try with straight form
            configuration.SetOrientation(tranformComplete);
			while (limitstraight >0)
			{
                Eigen::Vector3d dir((double) rand() / (double)(RAND_MAX) - 0.5, (double) rand() / (double)(RAND_MAX) - 0.5, (double) rand() / (double)(RAND_MAX) - 0.5);
                // check that direction is somewhat colinear to normal
                if(!sampled.first->normals_.empty())
                {
                    /*while(dir.dot(triangleNormal) <= 0)
                    {
                        dir = Eigen::Vector3d((double) rand() / (RAND_MAX) - 0.5, (double) rand() / (RAND_MAX) - 0.5, (double) rand() / (RAND_MAX) - 0.5);
                        if(dir.norm() != 0)
                        {
                            dir.normalize();
                        }
                    }*/
                }
                if(dir.norm() == 0) break;
                dir.normalize();
                // add random direction and check for collision
                //configuration.SetPosition(configuration.GetPosition() + (double) rand() / (RAND_MAX) * dir);
                std::vector<size_t> collisions = configuration.EnglobingCollisionClimb(contactObjects_, 0.3);
//collisions = configuration.EnglobingCollisionGround(sampled.first);
                int maxStep = 5;
//while(collisions.size()>0 && maxStep >0)
                while(collisions.size()>0)
				{
					if(!collider_.IsColliding(configuration.englobed))
					{
//if(configuration.GetPosition().y() > 0 && configuration.GetPosition().y() < 5 && std::abs(configuration.GetPosition().z()) < 1.3 ) // && std::abs(configuration.GetPosition().x()) < 1)
//if(configuration.GetPosition().y() > -0.3 )
                            copy(collisions, limbs);
                            return new Model(configuration);
						break;
					}
                    configuration.SetPosition(configuration.GetPosition() + (double) rand() / (RAND_MAX) / 8 * dir);
                    collisions = configuration.EnglobingCollisionClimb(contactObjects_, 0.3);
//collisions = configuration.EnglobingCollisionGround(sampled.first);
                    maxStep--;
                }
				--limitstraight;
			}
            /*tranform*= matrices::Roty3(ry);
            tranform*= matrices::Rotx3(rx);*/
            configuration.SetOrientation(tranformComplete);
			while (limit2 >0)
			{
                Eigen::Vector3d dir((double) rand() / (double) (RAND_MAX) -0.5, (double) rand() / (double) (RAND_MAX) -0.5, (double) rand() / (double) (RAND_MAX) -0.5);
				// if normal check colinearity
                if(sampled.first->normals_.size() > sampled.second.second->id)
				{
                    Eigen::Vector3d normal = sampled.first->normals_[sampled.second.second->id];
					normal.normalize();
					dir.normalize();
					int i = 1000;
                    /*while((normal.dot(dir) < 0 && dir.dot(normal) < 0) && i > 0)
					{
						dir= Eigen::Vector3d((double) rand() / (RAND_MAX) -0.5, (double) rand() / (RAND_MAX) -0.5, (double) rand() / (RAND_MAX) -0.5);
						dir.normalize();
						--i;
                    }*/
				}
				else
				{
					if(dir.norm() == 0) break;
					dir.normalize();
				}
				// add random direction and check for collision

                std::vector<size_t> collisions = configuration.EnglobingCollision(sampled.first);
//while(collisions.size()>0)
                while(collisions.size()>0) // || (configuration.GetPosition().y() > 3.7 && collisions.size()>0))
                {
					if(!collider_.IsColliding(configuration.englobed))
					{
//if(configuration.GetPosition().y() > 0  && configuration.GetPosition().y() < 5 && std::abs(configuration.GetPosition().z()) < 1.3 ) ; //&& std::abs(configuration.GetPosition().x()) < 1)
//if(configuration.GetPosition().y() > -0.3)
                            copy(collisions, limbs);
                            return new Model(configuration);
						break;
					}
                    configuration.SetPosition(configuration.GetPosition() + (double) rand() / (double)(RAND_MAX)  * dir);
                    collisions = configuration.EnglobingCollisionGround(sampled.first);
				}
				--limit2;
			}
		}
    }
    return 0;
}


Generator::SampleTriangle Generator::RandomPointIntriangle()
{
    Object* sampled = contactObjects_[rand() % (contactObjects_.size())];
    int i =rand() % (sampled->GetModel()->num_tris);
    Eigen::Vector3d normal = (sampled->normals_.empty()) ? Eigen::Vector3d(0,0,0) : sampled->normals_[i];
    return std::make_pair(sampled, std::make_pair(normal, &(sampled->GetModel()->tris[i])));
}

const Generator::SampleTriangle &Generator::WeightedTriangles()
{
    double r = ((double) rand() / (RAND_MAX));
    std::vector<SampleTriangle >::const_iterator trit = triangles_.begin();
    for(std::vector<float>::iterator wit = weights_.begin();
        wit != weights_.end();
        ++wit, ++trit)
    {
        if(*wit <= r)
        {
            return *trit;
        }
    }
    return triangles_[triangles_.size()-1]; // not supposed to happen
}

void Generator::InitWeightedTriangles()
{
    float sum = 0;
    for(Object::T_Object::iterator objit = contactObjects_.begin();
        objit != contactObjects_.end(); ++objit)
    {
        for(int i =0; i < (*objit)->GetModel()->num_tris; ++i)
        {
            const Tri* pTri = &((*objit)->GetModel()->tris[i]);
            float weight = TriangleArea(*pTri);
            sum += weight;
            weights_.push_back(weight);
            if((*objit)->normals_.empty())
            {
                triangles_.push_back(std::make_pair(*objit, std::make_pair(Eigen::Vector3d(0,1,0),pTri)));
            }
            else
            {
                triangles_.push_back(std::make_pair(*objit, std::make_pair((*objit)->normals_[i],pTri)));
            }
        }
        float previousWeight = 0;
        for(std::vector<float>::iterator wit = weights_.begin();
            wit != weights_.end();
            ++wit)
        {
            previousWeight += (*wit) / sum;
            (*wit) = previousWeight;
        }
    }
}
