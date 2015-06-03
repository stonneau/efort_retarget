#include "Export/FileHandler.h"

using namespace exporter;
using namespace std;

FileHandler::FileHandler()
    : bvhStr_()
    , tabs_("")
    , depth_(0)
{
    // NOTHING
}

FileHandler::~FileHandler()
{
    // NOTHING
}

bool FileHandler::Save(const string & filename)
{
    ofstream myfile;
    myfile.open (filename.c_str());
    if (myfile.is_open())
    {
        myfile << bvhStr_.rdbuf();
        myfile.close();
        return true;
    }
    return false;
}

const std::string FileHandler::AddTab(bool newLine)
{
    tabs_ += "\t";
    ++depth_;
    if (newLine) bvhStr_ << nl();
    return tabs_;
}

FileHandler& FileHandler::operator<< (const std::string& val)
{
    bvhStr_ << val;
    return *this;
}

FileHandler& FileHandler::operator<< (const double& val)
{
    bvhStr_ << val;
    return *this;
}

const std::string FileHandler::nl() const
{
    return std::string( "\n" + tabs_);
}

const std::string FileHandler::RemoveTab(bool newLine)
{
    if(depth_>0)
    {
        depth_--;
        tabs_.clear();
        for(int i=0; i<depth_; ++i)
        {
            tabs_ += "\t";
        }
    }
    if (newLine) bvhStr_ << nl();
    return tabs_;
}
