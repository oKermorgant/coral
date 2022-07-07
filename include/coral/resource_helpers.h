#ifndef CORAL_RESOURCE_HELPERS_H
#define CORAL_RESOURCE_HELPERS_H

#include <osgDB/Registry>

namespace coral
{

inline void addResourcePath(const std::string &path)
{  
  static auto &paths(osgDB::Registry::instance()->getDataFilePathList());
  if(std::find(paths.begin(), paths.end(), path) == paths.end())
    paths.push_back(path);
}

void initCoralResources();

// resolve the path (package:// - compatible) and returns the corresponding node
osg::ref_ptr<osg::Node> extractMesh(const std::string &path);

}

#endif // CORAL_RESOURCE_HELPERS_H
