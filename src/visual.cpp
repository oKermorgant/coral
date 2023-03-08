#include <coral/visual.h>
#include <coral/resource_helpers.h>
#include <coral/transforms.h>
#include <coral/osg_make_ref.h>
#include <osg/Texture2D>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <urdf/model.h>
#include <filesystem>
#include <osgDB/ReadFile>

namespace coral
{

namespace fs = std::filesystem;

/// wraps a material (color + texture) to osg::StateSet
struct Material
{
  std::string texture;
  osg::Vec4f rgba;
  explicit Material(osg::Vec4f rgba, const std::string &texture = "") : texture{texture}
  {
    for(size_t i = 0; i < 4; ++i)
      this->rgba[i] = std::clamp(rgba[i], 0.f, 1.f);
  }
  explicit Material(const urdf::Color &color, const std::string &texture = "") : Material({color.r,color.g,color.b,color.a}, texture) {}
  explicit Material(const urdf::Material &mat) : Material(mat.color, mat.texture_filename) {}
  // poor man's hash
  std::string serialize() const
  {
    if(!texture.empty())
      return texture;
    return std::to_string(rgba.asRGBA());
  }

  osg::StateSet* toStateSet() const
  {
    static std::map<std::string, osg::ref_ptr<osg::StateSet>> cache;
    auto &stateset{cache[serialize()]};

    if(stateset.valid())
      return stateset.get();

    stateset = osg::make_ref<osg::StateSet>();
    if(!texture.empty())
    {
      auto image = osgDB::readImageFile(texture);
      auto texture2d = osg::make_ref<osg::Texture2D>(image);
      texture2d->setFilter(osg::Texture2D::FilterParameter::MIN_FILTER,osg::Texture2D::FilterMode::LINEAR);
      texture2d->setFilter(osg::Texture2D::FilterParameter::MAG_FILTER,osg::Texture2D::FilterMode::LINEAR);

      stateset->setTextureAttribute(0,texture2d,osg::StateAttribute::OVERRIDE);
      stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
      stateset->setTextureMode(0,GL_TEXTURE_GEN_S,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
      stateset->setTextureMode(0,GL_TEXTURE_GEN_T,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
    }
    else
    {
      auto material = osg::make_ref<osg::Material>();
      material->setDiffuse(osg::Material::FRONT_AND_BACK, rgba);
      stateset->setAttribute(material);
      if (rgba.a() < 1)
      {
        stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
      }
    }
    return stateset.get();
  }
};


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

osg::ref_ptr<osg::Node> extractMesh(const std::string &mesh)
{
  const auto fullpath{toOSGMesh(resolvePath(mesh))};
  const auto filename(fullpath.filename());
  const auto path(fullpath.parent_path());

  addResourcePath(path);
  osgDB::Registry::instance()->getOptions()->setObjectCacheHint(osgDB::Options::CACHE_ALL);
  auto node{osgDB::readNodeFile(filename)};

  if(!node)
  {
    OSG_FATAL << "Cannot find mesh file '"
              << fullpath << "'\n";
    return nullptr;
  }
  return node;
}

// util function to build or reuse osg::Shapes from urdf::Geometry
osg::ref_ptr<osg::Shape> shapeCache(urdf::Geometry const* geometry, const Material &mat)
{
  // extract info
  enum class Type {SPHERE, BOX, CYLINDER} type;
  double radius, length;
  urdf::Vector3 box;
  std::string name;

  if(geometry->type == geometry->BOX)
  {
    type = Type::BOX;
    box = static_cast<urdf::Box const *>(geometry)->dim;
    name = "box_"+std::to_string(box.x)+std::to_string(box.y)+std::to_string(box.z);
  }
  else if(geometry->type == geometry->SPHERE)
  {
    type = Type::SPHERE;
    radius = static_cast<urdf::Sphere const *>(geometry)->radius;
    name = "sph_"+std::to_string(radius);
  }
  else
  {
    type = Type::CYLINDER;
    const auto info = static_cast<urdf::Cylinder const *>(geometry);
    radius = info->radius;
    length = info->length;
    name = "cyl_"+std::to_string(radius)+std::to_string(length);
  }
  name += mat.serialize();

  // to shape
  static std::map<std::string, osg::ref_ptr<osg::Shape>> cache;
  auto &cached{cache[name]};
  if(cached.valid())
    return cached;

  if(type == Type::BOX)
    cached = osg::make_ref<osg::Box>(osg::Vec3d{}, box.x, box.y, box.z);
  else if(type == Type::SPHERE)
    cached = osg::make_ref<osg::Sphere>(osg::Vec3d{}, radius);
  else if(type == Type::CYLINDER)
    cached = osg::make_ref<osg::Cylinder>(osg::Vec3d{}, radius, length);
  return cached;
}


// factories
namespace detail
{

Visual fromMesh(osg::ref_ptr<osg::Node> mesh, const osg::Matrix &M, osg::StateSet* stateset = nullptr)
{
  if(stateset)
    mesh->setStateSet(stateset);
  return Visual(M, mesh);
}
}

std::optional<Visual> Visual::fromURDF(const urdf::Visual &visual, const osg::Matrix &M)
{
  const auto mat(visual.material.get());
  const auto geometry{visual.geometry.get()};

  if(geometry->type == geometry->MESH)
  {
    const auto mesh_info = static_cast<urdf::Mesh*>(geometry);
    const auto mesh{extractMesh(mesh_info->filename)};
    if(!mesh)
      return {};
    const auto Mv(osgMatFrom(visual.origin.position,visual.origin.rotation, mesh_info->scale)*M);
    if(mat)
      return detail::fromMesh(mesh, Mv, Material(*mat).toStateSet());
    return detail::fromMesh(mesh, Mv);
  }

  // we do not know how to display a geometric shape without material
  if(!mat)
    return {};
  const auto material{Material(*mat)};

  const auto Mv(osgMatFrom(visual.origin.position, visual.origin.rotation) *M);
  return fromShapes({shapeCache(geometry, material)}, Mv, material.toStateSet());
}

Visual Visual::fromShapes(const Visual::Shapes &shapes, const osg::Matrix &M, osg::StateSet* stateset)
{
  auto geode = osg::make_ref<osg::Geode>();
  for(auto &shape: shapes)
    geode->addDrawable(osg::make_ref<osg::ShapeDrawable>(shape));
  return detail::fromMesh(geode, M, stateset);
}

// return the stateset corresponding to the texture or color, with cache
osg::StateSet* Visual::makeStateSet(osg::Vec4f rgba, const std::string &texture)
{
  return Material(rgba, texture).toStateSet();
}

}
