#include <ament_index_cpp/get_package_share_directory.hpp>
#include <coral/resource_helpers.h>
#include <filesystem>
#include <osgDB/ReadFile>

namespace coral
{

namespace fs = std::filesystem;

inline void addResourcePath(const std::string &path)
{
  static auto &paths(osgDB::Registry::instance()->getDataFilePathList());
  if(std::find(paths.begin(), paths.end(), path) == paths.end())
    paths.insert(paths.begin(), path);
}

void initCoralResources()
{
  // shaders and textures from Coral
  const auto coral_resources(ament_index_cpp::get_package_share_directory("coral") + "/resources");
  addResourcePath(coral_resources);
  for(const auto &p: fs::directory_iterator(coral_resources))
  {
    if(p.is_directory())
      addResourcePath(p.path());
  }
}


fs::path resolvePath(const std::string &file)
{
  const std::string file_prefix("file://");
  const std::string pkg_prefix("package://");

  std::string abs_path(file);

  if(file.find(pkg_prefix) == file.npos)
  {
    if(file.find(file_prefix) == 0)
      abs_path = file.substr(file_prefix.size());
  }
  else
  {
    const auto slash = file.find_first_of('/', pkg_prefix.size());
    const auto package(file.substr(pkg_prefix.size(), slash-pkg_prefix.size()));
    const auto pkg_path(ament_index_cpp::get_package_share_directory(package));
    abs_path = pkg_path + file.substr(slash);
  }
  return abs_path;
}
}

