#ifndef  _FileHandler_H_
#define  _BVHEXPORTER_H_


#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>

/**
*  \class BVHExporter
*  \brief Export A robot to a readable BVH file format
*/

namespace exporter
{
class  FileHandler
{
public:
     FileHandler();
    ~FileHandler();
	
private:
    FileHandler(const FileHandler&);
    FileHandler& operator=(const FileHandler&);


public:
    bool Save(const std::string& /*filename*/);

    const std::string AddTab(bool newLine=true); // add indentation
    const std::string RemoveTab(bool newLine=true); // remove indentation

    FileHandler& operator<< (const std::string& val); // push string to buffer
    FileHandler& operator<< (const double& val); // push double to buffer

    const std::string nl() const; // return a new line string

private:
    std::stringstream bvhStr_; // To write the new bvh
    std::string tabs_;
    int depth_;
};
} //namespace exporter
#endif // _BVHEXPORTER_H_
