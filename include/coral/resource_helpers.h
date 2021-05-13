#ifndef CORAL_RESOURCE_HELPERS_H
#define CORAL_RESOURCE_HELPERS_H

#include <osgDB/Registry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace coral
{

inline void addResourcePath(const std::string &path)
{
  static auto &paths(osgDB::Registry::instance()->getDataFilePathList());
  if(std::find(paths.begin(), paths.end(), path) == paths.end())
    paths.push_back(path);
}

inline void initCoralResources()
{
  const auto coral_resources(ament_index_cpp::get_package_share_directory("coral") + "/resources");
  addResourcePath(coral_resources);
  for(auto &p: std::filesystem::directory_iterator(coral_resources))
  {
    if(p.is_directory())
      addResourcePath(p.path());
  }
}


// resolve the path (package:// included) and create an osg-compatible mesh
osg::Node *extractMesh(const std::string &path);

}

#endif // CORAL_RESOURCE_HELPERS_H
