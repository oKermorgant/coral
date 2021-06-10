#include <coral/resource_helpers.h>
#include <filesystem>
#include <osgDB/ReadFile>

namespace coral
{

namespace fs = std::filesystem;

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
  const auto fullpath(resolvePath(mesh));
  const auto filename(fullpath.filename());
  const auto path(fullpath.parent_path());

  if(!fs::exists(fullpath))
  {
    OSG_FATAL << "Cannot find mesh file '"
              << fullpath << "'\n";
  }

  addResourcePath(path);

  auto node = osgDB::readNodeFile(filename);
  if (node->asGroup() == nullptr)
  {
    osg::Node * aux = node;
    node = new osg::Group();
    node->asGroup()->addChild(aux);
  }
  return node;
}
}
