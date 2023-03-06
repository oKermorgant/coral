#include <coral/link.h>
#include <coral/resource_helpers.h>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <filesystem>
#include <algorithm>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <coral/OceanScene.h>
#include <coral/osg_make_ref.h>

// util struct to build or reuse osg::Shapes
class ShapeWrapper
{
  enum class Type {SPHERE, BOX, CYLINDER} type;
  //Type type;
  urdf::Vector3 box;
  double radius, length;
  std::string name;

  inline static std::string hash(urdf::MaterialConstSharedPtr mat)
  {
    if(!mat->texture_filename.empty())
      return mat->texture_filename;

    const uint8_t r(mat->color.r*255);
    const uint8_t g(mat->color.g*255);
    const uint8_t b(mat->color.b*255);
    const uint8_t a(mat->color.a*255);
    return std::to_string(r)+"."+std::to_string(g)+"."+std::to_string(b)+"."+std::to_string(a);
  }

  osg::ref_ptr<osg::Shape> create() const
  {
    switch(type)
    {
      case Type::BOX:
        return osg::make_ref<osg::Box>(osg::Vec3d{}, box.x, box.y, box.z);
      case Type::SPHERE:
        return osg::make_ref<osg::Sphere>(osg::Vec3d{}, radius);
      case Type::CYLINDER:
        return osg::make_ref<osg::Cylinder>(osg::Vec3d{}, radius, length);
    }
    return {};
  }

public:

  explicit ShapeWrapper(urdf::GeometryConstSharedPtr geometry, urdf::MaterialConstSharedPtr mat)
  {
    if(geometry->type == geometry->BOX)
    {
      type = Type::BOX;
      box = static_cast<urdf::Box const *>(geometry.get())->dim;
      name = "box_"+std::to_string(box.x)+std::to_string(box.y)+std::to_string(box.z);
    }
    else if(geometry->type == geometry->SPHERE)
    {
      type = Type::SPHERE;
      radius = static_cast<urdf::Sphere const *>(geometry.get())->radius;
      name = "sph_"+std::to_string(radius);
    }
    else
    {
      type = Type::CYLINDER;
      const auto info = static_cast<urdf::Cylinder const *>(geometry.get());
      radius = info->radius;
      length = info->length;
      name = "cyl_"+std::to_string(radius)+std::to_string(length);
    }
    name += hash(mat);
  }

  // cache
  static osg::ref_ptr<osg::Shape>& createOrReuse(const ShapeWrapper &shape)
  {
    static std::map<std::string, osg::ref_ptr<osg::Shape>> cache;
    auto &cached{cache[shape.name]};
    if(!cached.valid())
      cached = shape.create();
    return cached;
  }

  static osg::ref_ptr<osg::StateSet> & StateSet(urdf::MaterialConstSharedPtr mat)
  {
    static std::map<std::string, osg::ref_ptr<osg::StateSet>> cache;
    auto &stateset{cache[hash(mat)]};

    if(!stateset.valid())
    {
      stateset = osg::make_ref<osg::StateSet>();
      if(!mat->texture_filename.empty())
      {
        auto image = osgDB::readImageFile(mat->texture_filename);

        auto texture = osg::make_ref<osg::Texture2D>(image);
        texture->setFilter(osg::Texture2D::FilterParameter::MIN_FILTER,osg::Texture2D::FilterMode::LINEAR);
        texture->setFilter(osg::Texture2D::FilterParameter::MAG_FILTER,osg::Texture2D::FilterMode::LINEAR);

        stateset->setTextureAttribute(0,texture,osg::StateAttribute::OVERRIDE);
        stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
        stateset->setTextureMode(0,GL_TEXTURE_GEN_S,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
        stateset->setTextureMode(0,GL_TEXTURE_GEN_T,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
      }
      else
      {
          auto material = osg::make_ref<osg::Material>();
          material->setDiffuse(
                osg::Material::FRONT_AND_BACK,
                {mat->color.r, mat->color.g,mat->color.b, mat->color.a});
          stateset->setAttribute(material);
          if (mat->color.a < 1)
          {
            stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
          }
        }
    }
    return stateset;
  }
};


namespace coral
{

Link::Visual::Visual(osg::ref_ptr<osg::Node> mesh, const osg::Matrixd &M)
  : mesh(mesh), pose(new osg::MatrixTransform(M))
{
  pose->setDataVariance(osg::Object::STATIC);
  pose->addChild(this->mesh);
}

void Link::addVisual(urdf::VisualSharedPtr visual, const osg::Matrixd &M)
{
  const auto mat(visual->material);

  if(visual->geometry->type == visual->geometry->MESH)
  {
    const auto mesh_info = static_cast<urdf::Mesh*>(visual->geometry.get());
    auto Mv(osgMatFrom(visual->origin.position,visual->origin.rotation, mesh_info->scale)*M);
    addVisualNode(extractMesh(mesh_info->filename), Mv, mat);
    return;
  }

  // we do not know how to display a geometric shape without material
  if(mat == nullptr)
    return;

  const auto Mv(osgMatFrom(visual->origin.position, visual->origin.rotation) *M);
  const ShapeWrapper shape(visual->geometry, mat);
  addVisualShape(ShapeWrapper::createOrReuse(shape), Mv, mat);
}

void Link::addVisualShape(osg::ref_ptr<osg::Shape> shape, const osg::Matrixd &M, urdf::MaterialConstSharedPtr mat)
{
  auto drawable = osg::make_ref<osg::ShapeDrawable>(shape);
  auto geode = osg::make_ref<osg::Geode>();
  geode->addDrawable(drawable);
  addVisualNode(geode, M, mat);
}

void Link::addVisualNode(osg::ref_ptr<osg::Node> node, const osg::Matrixd &M, urdf::MaterialConstSharedPtr mat)
{
  if(mat)
    node->setStateSet(ShapeWrapper::StateSet(mat));

  auto &visual{visuals.emplace_back(node, M)};
  pose->addChild(visual.pose);
  OceanScene::setupMeshNode(visual.mesh);
}
}
