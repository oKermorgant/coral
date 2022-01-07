#include <coral/resource_helpers.h>
#include <filesystem>
#include <osgDB/ReadFile>

namespace coral
{

namespace fs = std::filesystem;

// try to find similar mesh file with OSG extension
fs::path toOSGMesh(fs::path path)
{
  const std::vector<std::string> osg_ext{".ive", ".osg",".osgb", ".osgt"};

  // already OSG mesh?
  if(std::find(osg_ext.begin(), osg_ext.end(), path.extension())
     != osg_ext.end())
     return path;

  // find if any
  const std::string base{path.stem().string()};
  for(const auto &ext: osg_ext)
  {
    if(fs::exists(base + ext))
      return base+ext;
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

  // also textures from uuv_gazebo_worlds, if present
  try
  {
    const auto uuv_resources(ament_index_cpp::get_package_share_directory("uuv_gazebo_worlds") + "/media/materials/textures");
    addResourcePath(uuv_resources);
  }
  catch (...) {}
  
}


fs::path resolvePath(const std::string &file)
{
  static const std::string file_prefix("file://");
  static const std::string pkg_prefix("package://");

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

osg::Node * extractMesh(const std::string &mesh)
{
  // do not deal with surface meshes from Plankton / UUV
  if(mesh.find("sea_surface") != mesh.npos)
    return nullptr;

  const auto fullpath(toOSGMesh(resolvePath(mesh)));
  const auto filename(fullpath.filename());
  const auto path(fullpath.parent_path());

  // TODO detect and add texture to sea bottom meshes from Plankton / UUV    

  addResourcePath(path);

  auto node = osgDB::readNodeFile(filename);

  if(node == nullptr)
  {
    OSG_FATAL << "Cannot find mesh file '"
              << fullpath << "'\n";
  }
  return node;
}
}
