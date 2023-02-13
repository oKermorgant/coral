#include <ament_index_cpp/get_package_share_directory.hpp>
#include <coral/resource_helpers.h>
#include <filesystem>
#include <osgDB/ReadFile>


namespace coral
{

namespace fs = std::filesystem;

// try to find similar mesh file with OSG extension
fs::path toOSGMesh(const fs::path &path)
{
  const std::vector<std::string> osg_ext{".ive", ".osg",".osgb", ".osgt"};

  // already OSG mesh?
  if(std::find(osg_ext.begin(), osg_ext.end(), path.extension())
     != osg_ext.end())
    return path;

  // find if any
  auto new_path{path};

  for(const auto &ext: osg_ext)
  {
    new_path.replace_extension(ext);
    if(fs::exists(new_path))
      return new_path;
  }
  return path;
}


void initCoralResources()
{
  // shaders and textures from Coral
  const auto coral_resources(ament_index_cpp::get_package_share_directory("coral") + "/resources");
  addResourcePath(coral_resources);
  for(auto &p: fs::directory_iterator(coral_resources))
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

osg::ref_ptr<osg::Node> extractMesh(const std::string &mesh)
{
  const auto fullpath{toOSGMesh(resolvePath(mesh))};
  const auto filename(fullpath.filename());
  const auto path(fullpath.parent_path());

  addResourcePath(path);
  osg::ref_ptr<osg::Node> node{osgDB::readNodeFile(filename)};

  if(!node.valid())
  {
    OSG_FATAL << "Cannot find mesh file '"
              << fullpath << "'\n";
    return nullptr;
  }
  return node.release();
}
}
