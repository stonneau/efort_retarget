#include "MatrixDefs.h"

#include "create_obj_box.h"

#include <vector>
#include <iostream>
#include <fstream>



namespace
{

    std::string from_double(double dbl)
    {
        std::ostringstream strs;
        strs << dbl;
        return strs.str();
    }

    std::vector<Eigen::Vector3d> get_points()
    {
        std::vector<Eigen::Vector3d> res;
        /*res.push_back(Eigen::Vector3d(-0.057418, -0.046388, -0.023488));
        res.push_back(Eigen::Vector3d(0.069137, -0.046388, -0.023488));
        res.push_back(Eigen::Vector3d(0.069137, 0.053561, -0.023488));
        res.push_back(Eigen::Vector3d(-0.057417, -0.046388, -0.328301));
        res.push_back(Eigen::Vector3d(0.069137, -0.046388, -0.328302));
        res.push_back(Eigen::Vector3d(-0.057417, 0.053561, -0.328301));
        res.push_back(Eigen::Vector3d(0.069137, 0.053561, -0.328302));
        res.push_back(Eigen::Vector3d(-0.057418, 0.053561, -0.023488));*/
        res.push_back(Eigen::Vector3d(0.026257 , -0.412667, -0.026257));
        res.push_back(Eigen::Vector3d(0.033335 , -0.362913, -0.033335));
        res.push_back(Eigen::Vector3d(-0.033335, -0.362913, -0.033335));
        res.push_back(Eigen::Vector3d(-0.026257, -0.412667, -0.026257));
        res.push_back(Eigen::Vector3d(0.026257 , -0.020650,  0.026257));
        res.push_back(Eigen::Vector3d(0.033335 , -0.070403,  0.033335));
        res.push_back(Eigen::Vector3d(-0.033335, -0.070403, 0.033335));
        res.push_back(Eigen::Vector3d(-0.026257, -0.020650, 0.026257));
        res.push_back(Eigen::Vector3d(0.012549 , -0.006463,  0.012549));
        res.push_back(Eigen::Vector3d(-0.012549, -0.006463, 0.012549));
        res.push_back(Eigen::Vector3d(-0.012549, -0.006463, -0.012549));
        res.push_back(Eigen::Vector3d(0.012549 , -0.006463,  -0.012549));
        res.push_back(Eigen::Vector3d(0.026257 , -0.412667, 0.026257));
        res.push_back(Eigen::Vector3d(0.033335 , -0.362913, 0.033335));
        res.push_back(Eigen::Vector3d(-0.012549, -0.426853, 0.012549));
        res.push_back(Eigen::Vector3d(0.012549 , -0.426853, 0.012549));
        res.push_back(Eigen::Vector3d(0.012549 , -0.426853, -0.012549));
        res.push_back(Eigen::Vector3d(-0.012549, -0.426853, -0.012549));
        res.push_back(Eigen::Vector3d(-0.033335, -0.070403, -0.033335));
        res.push_back(Eigen::Vector3d(-0.026257, -0.020650, -0.026257));
        res.push_back(Eigen::Vector3d(0.033335 , -0.070403,  -0.033335));
        res.push_back(Eigen::Vector3d(0.026257 , -0.020650,  -0.026257));
        res.push_back(Eigen::Vector3d(-0.026257, -0.412667, 0.026257));
        res.push_back(Eigen::Vector3d(-0.033335, -0.362913, 0.033335));
        res.push_back(Eigen::Vector3d(-0.034375, -0.216658, -0.034375));
        res.push_back(Eigen::Vector3d(0.034375 , -0.216658, -0.034375));
        res.push_back(Eigen::Vector3d(-0.034375, -0.216658, 0.034375));
        res.push_back(Eigen::Vector3d(0.034375 , -0.216658, 0.034375));
        return res;
    }

    std::vector<Eigen::Vector3d> get_normals()
    {
        std::vector<Eigen::Vector3d> res;


        /*res.push_back(Eigen::Vector3d(0.000000, -0.000000, 1.000000));
        res.push_back(Eigen::Vector3d(-0.000000, -1.000000, -0.000000));
        res.push_back(Eigen::Vector3d(-0.000000, 0.000000, -1.000000));
        res.push_back(Eigen::Vector3d(0.000000, 1.000000, 0.000000));
        res.push_back(Eigen::Vector3d(1.000000, 0.000000, 0.000000));
        res.push_back(Eigen::Vector3d(-1.000000, -0.000000, -0.000000));*/
        res.push_back(Eigen::Vector3d(0.000000, 0.140800, -0.990000));
        res.push_back(Eigen::Vector3d(-0.000000, -0.140800, 0.990000));
        res.push_back(Eigen::Vector3d(0.000000, -1.000000, 0.000000));
        res.push_back(Eigen::Vector3d(0.990000, 0.140800, -0.000000));
        res.push_back(Eigen::Vector3d(0.000000, 1.000000, 0.000000));
        res.push_back(Eigen::Vector3d(-0.990000, -0.140800, 0.000000));
        res.push_back(Eigen::Vector3d(0.000000, -0.140800, -0.990000));
        res.push_back(Eigen::Vector3d(-0.000000, 0.140800, 0.990000));
        res.push_back(Eigen::Vector3d(0.990000, -0.140800, 0.000000));
        res.push_back(Eigen::Vector3d(-0.990000, 0.140800, 0.000000));
        res.push_back(Eigen::Vector3d(0.000000, -0.007100, -1.000000));
        res.push_back(Eigen::Vector3d(0.000000, 0.007100, 1.000000));
        res.push_back(Eigen::Vector3d(1.000000, -0.007100, 0.000000));
        res.push_back(Eigen::Vector3d(-1.000000, 0.007100, -0.000000));
        res.push_back(Eigen::Vector3d(0.000000, 0.007100, -1.000000));
        res.push_back(Eigen::Vector3d(-0.000000, -0.007100, 1.000000));
        res.push_back(Eigen::Vector3d(1.000000, 0.007100, -0.000000));
        res.push_back(Eigen::Vector3d(-1.000000, -0.007100, 0.000000));
        res.push_back(Eigen::Vector3d(0.000000, -0.694900, 0.719100));
        res.push_back(Eigen::Vector3d(-0.719100, -0.694900, 0.000000));
        res.push_back(Eigen::Vector3d(0.000000, -0.694900, -0.719100));
        res.push_back(Eigen::Vector3d(0.719100, -0.694900, 0.000000));
        res.push_back(Eigen::Vector3d(0.000000, 0.694900, 0.719100));
        res.push_back(Eigen::Vector3d(0.719100, 0.694900, 0.000000));
        res.push_back(Eigen::Vector3d(0.000000, 0.694900, -0.719100));
        res.push_back(Eigen::Vector3d(-0.719100, 0.694900, 0.000000));
        return res;
    }

    typedef std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d> > T_Object;
    T_Object createAndRotate(const Eigen::Vector3d& axis)
    {
        //compute rotation matrix around axis
        Eigen::Matrix3d rotation; // = matrices::Rotx3(M_PI);
        Eigen::Vector3d nAxis = axis;
        nAxis.normalize();
        /*weird magic rotations that fit the object. To be honest
        I don't really know why I need the additional pi rotation*/
        matrices::GetRotationMatrix(nAxis, Eigen::Vector3d(0,1,0), rotation);
        double isX = std::abs(nAxis.dot(Eigen::Vector3d(1,0,0)));
        double isZ = std::abs(nAxis.dot(Eigen::Vector3d(0,0,1)));
        if(isX > isZ)
        {
            rotation = matrices::Rotx3(M_PI) * rotation;
        }
        else
        {
            rotation = matrices::Rotz3(M_PI) * rotation;
        }
        std::vector<Eigen::Vector3d> oPoints = get_points();
        std::vector<Eigen::Vector3d> oNnormals = get_normals();
        std::vector<Eigen::Vector3d> points, normals;
        for(std::vector<Eigen::Vector3d>::const_iterator cit = oPoints.begin();
            cit != oPoints.end(); ++cit)
        {
            points.push_back(rotation * (*cit));
        }
        for(std::vector<Eigen::Vector3d>::const_iterator cit = oNnormals.begin();
            cit != oNnormals.end(); ++cit)
        {
            normals.push_back(rotation * (*cit));
        }
        return std::make_pair(points, normals);
    }
}

bool objects::create_obj_box(const std::string& outfilename, const Eigen::Vector3d& axis)
{
    //create box

    std::stringstream outstream;

    //dump data
    T_Object res = createAndRotate(axis);
    std::vector<Eigen::Vector3d>& points = res.first;
    std::vector<Eigen::Vector3d>& normals = res.second;
    outstream << "mtllib " << outfilename <<".mtl" <<'\n';
    outstream << "o " << outfilename  <<'\n';
    for(std::vector<Eigen::Vector3d>::const_iterator cit = points.begin();
        cit != points.end(); ++cit)
    {
        double x = (*cit)(0) * axis.norm() / 0.42; double y = (*cit)(1) * axis.norm() / 0.42; double z = (*cit)(2) * axis.norm() / 0.42;
        outstream << "v " << from_double(x) << " " << from_double(y) << " " << from_double(z) <<'\n';
    }
    for(std::vector<Eigen::Vector3d>::const_iterator cit = normals.begin();
        cit != normals.end(); ++cit)
    {
        double x = (*cit)(0); double y = (*cit)(1); double z = (*cit)(2);
        outstream << "vn " << from_double(x) << " " << from_double(y) << " " << from_double(z) <<'\n';
    }
    outstream << "usemtl None.081" <<'\n';
    outstream << "s off" <<'\n';
    outstream << "f 1//1 2//1 3//1" <<'\n';
    outstream << "f 4//1 1//1 3//1" <<'\n';
    outstream << "f 5//2 6//2 7//2" <<'\n';
    outstream << "f 7//2 8//2 5//2" <<'\n';
    outstream << "f 9//3 10//3 11//3" <<'\n';
    outstream << "f 11//3 12//3 9//3" <<'\n';
    outstream << "f 13//4 14//4 2//4" <<'\n';
    outstream << "f 2//4 1//4 13//4" <<'\n';
    outstream << "f 15//5 16//5 17//5" <<'\n';
    outstream << "f 17//5 18//5 15//5" <<'\n';
    outstream << "f 8//6 7//6 19//6" <<'\n';
    outstream << "f 19//6 20//6 8//6" <<'\n';
    outstream << "f 21//7 22//7 20//7" <<'\n';
    outstream << "f 20//7 19//7 21//7" <<'\n';
    outstream << "f 14//8 13//8 23//8" <<'\n';
    outstream << "f 23//8 24//8 14//8" <<'\n';
    outstream << "f 6//9 5//9 22//9" <<'\n';
    outstream << "f 22//9 21//9 6//9" <<'\n';
    outstream << "f 24//10 23//10 4//10" <<'\n';
    outstream << "f 4//10 3//10 24//10" <<'\n';
    outstream << "f 21//11 19//11 25//11" <<'\n';
    outstream << "f 25//11 26//11 21//11" <<'\n';
    outstream << "f 14//12 24//12 27//12" <<'\n';
    outstream << "f 27//12 28//12 14//12" <<'\n';
    outstream << "f 6//13 21//13 26//13" <<'\n';
    outstream << "f 26//13 28//13 6//13" <<'\n';
    outstream << "f 24//14 3//14 25//14" <<'\n';
    outstream << "f 25//14 27//14 24//14" <<'\n';
    outstream << "f 25//15 3//15 2//15" <<'\n';
    outstream << "f 2//15 26//15 25//15" <<'\n';
    outstream << "f 27//16 7//16 6//16" <<'\n';
    outstream << "f 6//16 28//16 27//16" <<'\n';
    outstream << "f 26//17 2//17 14//17" <<'\n';
    outstream << "f 14//17 28//17 26//17" <<'\n';
    outstream << "f 25//18 19//18 7//18" <<'\n';
    outstream << "f 7//18 27//18 25//18" <<'\n';
    outstream << "f 5//19 8//19 10//19" <<'\n';
    outstream << "f 10//19 9//19 5//19" <<'\n';
    outstream << "f 8//20 20//20 11//20" <<'\n';
    outstream << "f 11//20 10//20 8//20" <<'\n';
    outstream << "f 20//21 22//21 12//21" <<'\n';
    outstream << "f 12//21 11//21 20//21" <<'\n';
    outstream << "f 22//22 5//22 9//22" <<'\n';
    outstream << "f 9//22 12//22 22//22" <<'\n';
    outstream << "f 23//23 13//23 16//23" <<'\n';
    outstream << "f 16//23 15//23 23//23" <<'\n';
    outstream << "f 13//24 1//24 17//24" <<'\n';
    outstream << "f 17//24 16//24 13//24" <<'\n';
    outstream << "f 1//25 4//25 18//25" <<'\n';
    outstream << "f 17//25 1//25 18//25" <<'\n';
    outstream << "f 4//26 23//26 15//26" <<'\n';
    outstream << "f 15//26 18//26 4//26" <<'\n';
    /*outstream << "f 1//1 2//1 3//1" <<'\n';
    outstream << "f 4//2 5//2 2//2" <<'\n';
    outstream << "f 4//3 6//3 7//3" <<'\n';
    outstream << "f 8//4 3//4 7//4" <<'\n';
    outstream << "f 2//5 5//5 7//5" <<'\n';
    outstream << "f 4//6 1//6 8//6" <<'\n';
    outstream << "f 8//1 1//1 3//1" <<'\n';
    outstream << "f 1//2 4//2 2//2" <<'\n';
    outstream << "f 5//3 4//3 7//3" <<'\n';
    outstream << "f 6//4 8//4 7//4" <<'\n';
    outstream << "f 3//5 2//5 7//5" <<'\n';
    outstream << "f 6//6 4//6 8//6" <<'\n';*/
    outstream << "" <<'\n';

    std::ofstream outfile;
    outfile.open((outfilename + ".obj").c_str());
    if (outfile.is_open())
    {
        outfile << outstream.rdbuf();
        outfile.close();

        std::ofstream mtlfile;
        mtlfile.open((outfilename + ".mtl").c_str());
        if (mtlfile.is_open())
        {
            std::stringstream mtlfstream;
            mtlfstream << "newmtl None.081" <<'\n';
            mtlfstream << "Ns 0.000000" <<'\n';
            mtlfstream << "Ka 0.000000 0.000000 0.000000" <<'\n';
            mtlfstream << "Kd 0.512000 0.512000 0.512000" <<'\n';
            mtlfstream << "Ks 0.800000 0.800000 0.800000" <<'\n';
            mtlfstream << "Ni 1.000000" <<'\n';
            mtlfstream << "d 1.000000" <<'\n';
            mtlfstream << "illum 2" <<'\n';
            mtlfile << mtlfstream.rdbuf();
            mtlfile.close();
            return true;
        }
    }
    std::cout << "Can not create objfile " << outfilename << std::endl;
    return false;
}
