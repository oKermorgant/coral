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
#include <osgUtil/Optimizer>

namespace coral
{

namespace
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
      auto texture2d = osg::make_ref<osg::Texture2D>(osgDB::readImageFile(texture));
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

osg::ref_ptr<osg::Group> extractMesh(const std::string &mesh)
{
  const auto fullpath{toOSGMesh(resolvePath(mesh))};
  const auto filename(fullpath.filename());
  const auto path(fullpath.parent_path());

  addResourcePath(path);
  static auto opt = []()
  {
    auto opt{osg::make_ref<osgDB::Options>()};
    opt->setObjectCacheHint(osgDB::Options::CACHE_ALL);
    opt->setPrecisionHint(osgDB::Options::FLOAT_PRECISION_ALL);
    return opt;
  }();

  auto node{osgDB::readNodeFile(filename.string(), opt)};

  if(!node)
  {
    OSG_FATAL << "Cannot find mesh file "
              << fullpath.string() << "\n";
    return nullptr;
  }
  return node->asGroup();
}

// util function to build or reuse osg::Shapes from urdf::Geometry
osg::ref_ptr<osg::Shape> shapeCache(urdf::Geometry const* geometry, const Material &mat)
{
  // extract info
  std::vector<double> params;

  if(geometry->type == geometry->BOX)
  {
    const auto box{static_cast<urdf::Box const *>(geometry)->dim};
    params = {box.x, box.y, box.z};
  }
  else if(geometry->type == geometry->SPHERE)
  {
    params = {static_cast<urdf::Sphere const *>(geometry)->radius};
  }
  else
  {
    const auto cyl{static_cast<urdf::Cylinder const *>(geometry)};
    params = {cyl->radius, cyl->length};
  }

  const auto concat = [](const std::string &prev, double v){return prev + std::to_string(v);};
  const auto name = std::accumulate(params.begin(), params.end(), std::string{}, concat) + mat.serialize();

  // to shape
  static std::map<std::string, osg::ref_ptr<osg::Shape>> cache;
  auto &cached{cache[name]};
  if(cached.valid())
    return cached;

  if(geometry->type == geometry->BOX)
    cached = osg::make_ref<osg::Box>(osg::Vec3d{}, params[0], params[1], params[2]);
  else if(geometry->type == geometry->SPHERE)
    cached = osg::make_ref<osg::Sphere>(osg::Vec3d{}, params[0]);
  else if(geometry->type == geometry->CYLINDER)
    cached = osg::make_ref<osg::Cylinder>(osg::Vec3d{}, params[0], params[1]);
  return cached;
}
}

Visual::Visual(osg::ref_ptr<osg::Group> mesh, const std::optional<osg::Matrix> &M)
{
  if(!mesh.valid())
    return;

  if(M.has_value())
  {
    base = osg::make_ref<osg::MatrixTransform>(M.value());
    base->addChild(mesh);
  }
  else
  {
    base = mesh;
  }
}

// factories

std::optional<Visual> Visual::fromURDF(const urdf::Visual &urdf, const osg::Matrix &M)
{
  const auto mat(urdf.material.get());
  const auto geometry{urdf.geometry.get()};

  if(geometry->type == geometry->MESH)
  {
    const auto mesh_info{static_cast<urdf::Mesh*>(geometry)};
    const auto mesh{extractMesh(mesh_info->filename)};
    if(!mesh)
      return {};
    const auto Mv(osgMatFrom(urdf.origin.position,urdf.origin.rotation, mesh_info->scale)*M);
    if(mat)
      mesh->setStateSet(Material(*mat).toStateSet());
    return Visual(mesh, Mv);
  }

  // we do not know how to display a geometric shape without material
  if(!mat)
    return {};
  const auto material{Material(*mat)};
  const auto Mv(osgMatFrom(urdf.origin.position, urdf.origin.rotation) *M);
  return fromShapes({shapeCache(geometry, material)}, material.toStateSet(), Mv);
}

Visual Visual::fromShapes(const Visual::Shapes &shapes, osg::StateSet* stateset, const std::optional<osg::Matrix> &M)
{
  auto geode = osg::make_ref<osg::Geode>();
  for(auto &shape: shapes)
    geode->addDrawable(osg::make_ref<osg::ShapeDrawable>(shape));
  geode->setStateSet(stateset);
  return Visual(geode, M);
}

// return the stateset corresponding to the texture or color, with cache
osg::StateSet* Visual::makeStateSet(const osg::Vec4 &rgba, const std::string &texture)
{
  return Material(rgba, texture).toStateSet();
}

void Visual::attachTo(osg::Group *parent, bool moving, bool seen_by_cameras)
{
  base->setDataVariance(moving ? osg::Object::DYNAMIC : osg::Object::STATIC);
  base->setNodeMask(Mask::getMask(seen_by_cameras));
  static osgUtil::Optimizer optim;
  optim.optimize(base, optim.ALL_OPTIMIZATIONS);
  parent->addChild(base);
}

}
