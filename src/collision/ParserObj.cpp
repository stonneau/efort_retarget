#include "ParserObj.h"
#include "Object.h"
#include "MatrixDefsInternal.h"
#include "PQP/PQP.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>

using namespace std;
using namespace matrices;

namespace
{
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > T_Vector3;
typedef std::vector<PQP_Model*> T_PQPModel;

string doubleSlash(const string& s)
{
    //Remplace "//" par "/1/".
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(i<s.size()-1&&s[i]=='/'&&s[i+1]=='/')
        {
            s1+="/1/";
            i++;
        }
        else
            s1+=s[i];
    }
    return s1;
}

string remplacerSlash(const string& s)
{
    //Remplace les '/' par des espaces.
    string ret="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]=='/')
            ret+=' ';
        else
            ret+=s[i];
    }
    return ret;
}

vector<string> splitSpace(const string& s)
{
    //Eclate une chaîne au niveau de ses espaces.
    vector<string> ret;
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]==' '||i==s.size()-1)
        {
            if(i==s.size()-1)
                s1+=s[i];
            ret.push_back(s1);
            s1="";
        }
        else
            s1+=s[i];
    }
    return ret;
}
}

namespace
{
	double isLeft( const VectorX& P0, const VectorX& P1, const Vector3& P2 )
	{
		return ( (P1.x() - P0.x()) * (P2.y() - P0.y())
				- (P2.x() - P0.x()) * (P1.y() - P0.y()) );
	}

	const Vector3& LeftMost(const T_Vector3& points)
	{
		unsigned int i = 0;
		for(unsigned int j = 1; j < points.size(); ++j)
		{
			if(points[j].x() < points[i].x())
			{
				i = j;
			}
		}
		return points[i];
	}

	T_Vector3 ConvexHull(const T_Vector3& points)
	{
		T_Vector3 res;
		Vector3 pointOnHull = LeftMost(points);	
		Vector3 endPoint;
		int i = 0;
		do
		{
			++i;
			VectorX pi = pointOnHull;
			endPoint = points[0];
			for(unsigned int j = 1; j < points.size(); ++j)
			{
				if((endPoint == pointOnHull) || (isLeft(pi, endPoint, points[j]) > 0))
				{
					endPoint = points[j];
				}
				
				if( i > 10000 )
				{
                    std::cout << " WTF " << std::endl << points[j] << std::endl;
				}
			}
			res.push_back(pi);
			pointOnHull = endPoint;
		} while(endPoint != res[0]);
		res.push_back(endPoint);
		return res;
    }

    void Vector3toArray(const Vector3& vect, double* arr)
    {
        for(int i=0; i<3; ++i)
        {
            arr[i] = vect[i];
        }
    }
}


namespace planner
{
	struct ParserPImpl
	{

        ParserPImpl(planner::ObjectDictionary& dic)
            : modelOn_(false)
            , currentModel_(0)
            , currentIndex_(-1)
            , dic_(dic)
        {
            // NOTHING
        }

        void CreatePQPObstacle(const std::string& line)
        {
            string ligne = doubleSlash(line);
            ligne=remplacerSlash(ligne); //On remplace les '/' par des espaces, ex : pour "f 1/2/3 4/5/6 7/8/9" on obtiendra "f 1 2 3 4 5 6 7 8 9"
            vector<string> termes=splitSpace(ligne.substr(2)); //On éclate la chaîne en ses espaces (le substr permet d'enlever "f ")
            int ndonnees=(int)termes.size()/3;
            // start just with triangles
            if(ndonnees >= 3 && ndonnees <=4) // only triangles or quad
            {
                PQP_REAL points [4][3];
                ObjectTri objtri;
                for(int i=0; i< ndonnees; ++i)
                {
                    long int idx = (strtol(termes[i*3].c_str(),NULL, 10)) - 1;
                    Vector3toArray(points_[(int)idx], points[i]);
                    dic_.objectsLinkedToVertex[idx].push_back(dic_.dic.size());
                    objtri.points[i] = idx;
                }
                currentModel_->AddTri(points[0], points[1], points[2], ++currentIndex_);
                dic_.dic.back().triangles_.push_back(objtri);
                if(ndonnees == 4)
                {
                    currentModel_->AddTri(points[0], points[2], points[3], ++currentIndex_);
                }
                long int idx = (strtol(termes[2].c_str(),NULL, 10)) - 1;
                currentNormals_.push_back(normals_[idx]);
                dic_.dic.back().normals_.push_back(currentNormals_.back());
            }
        }

        T_Vector3 points_;
		T_Vector3 normals_;
        T_Vector3 currentNormals_;
        std::vector<T_Vector3> objectnormals;
        T_PQPModel models_;
        bool modelOn_;
        PQP_Model* currentModel_;
        long int currentIndex_;

        //dictionnary
        planner::ObjectDictionary& dic_;
	};
}

namespace
{
std::string ExtractObjName(const std::string& line)
{
    std::vector<string> res  = splitSpace(line);
    if(res.size()>1) return res.back();
}
}

using namespace planner;

Object::T_Object planner::ParseObj(const std::string& filename, const bool asOneObject, double scaleEnglobing)
{
    Object::T_Object objects;
    ParseObj(filename, objects, asOneObject, scaleEnglobing);
    return objects;
}


void planner::ParseObj(const std::string& filename, std::vector<Object*>& objects, const bool asOneObject, double scaleEnglobing)
{
    ObjectDictionary dic;
    ParseObj(filename, objects,dic,asOneObject,scaleEnglobing);
}

void planner::ParseObj(const std::string& filename, std::vector<Object*>& objects, ObjectDictionary& dic, const bool asOneObject, double scaleEnglobing)
{
    ParserPImpl pImpl(dic);
    string line;
    ifstream myfile (filename);
    std::vector<std::string> lines;
    std::vector<std::string> names;
    bool firstInit = false;
    std::string current_name = "foo";
    if (myfile.is_open())
    {
        while ( myfile.good() )
        {
            getline (myfile, line);
            // new model
            if((line.find("o ") == 0 || line.find("g ") == 0) && (!asOneObject || !firstInit))
            {
                PQP_Model* m = new PQP_Model;
                dic.dic.push_back(ObjectData());
                if(pImpl.modelOn_)
                {
                    // TODO ADD MATRIX !
                    names.push_back(current_name); current_name = ExtractObjName(line);
                    pImpl.currentModel_->EndModel();
                    pImpl.objectnormals.push_back(pImpl.currentNormals_);
                    pImpl.currentNormals_.clear();
                }
                else
                {
                    current_name = ExtractObjName(line);
                    pImpl.modelOn_ = true;
                    firstInit = true;
                }
                pImpl.currentIndex_ = -1;
                pImpl.models_.push_back(m);
                pImpl.currentModel_ = m;
                pImpl.currentModel_->BeginModel();
            }
            if(line.find("v ") == 0)
            {
                char x[255],y[255],z[255];
                sscanf(line.c_str(),"v %s %s %s",x,y,z);
                pImpl.points_.push_back(scaleEnglobing * Vector3(strtod (x, NULL), strtod(y, NULL), strtod(z, NULL)));
                dic.points.push_back(pImpl.points_.back());
                dic.objectsLinkedToVertex.push_back(std::vector<std::size_t>());
            }
            if(line.find("vn ") == 0)
            {
                char x[255],y[255],z[255];
                sscanf(line.c_str(),"vn %s %s %s",x,y,z);
                pImpl.normals_.push_back(Vector3(strtod (x, NULL), strtod(y, NULL), strtod(z, NULL)));
            }
            if(line.find("f ") == 0)
            {
                pImpl.CreatePQPObstacle(line);
            }
        }
        myfile.close();
        if(pImpl.modelOn_)
        {
            pImpl.currentModel_->EndModel();
            pImpl.objectnormals.push_back(pImpl.currentNormals_);
            pImpl.currentNormals_.clear();
            pImpl.modelOn_ = false;
            names.push_back(current_name); current_name = "";
        }
        // now create Objects
        int i =0;
        for(T_PQPModel::iterator it = pImpl.models_.begin();
            it != pImpl.models_.end();
            ++it, ++i)
        {
            objects.push_back(new Object(*it, pImpl.objectnormals[i], names[i]));
        }
    }
    else
    {
        std::cout << "file not found:" << filename << std::endl;
    }
}

