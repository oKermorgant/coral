#include <coral/link.h>
#include <coral/resource_helpers.h>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <filesystem>
#include <algorithm>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <coral/OceanScene.h>
#include <coral/osg_make_ref.h>


template<>
struct std::less<urdf::Color>
{
  inline bool operator()(const urdf::Color &c1, const urdf::Color &c2) const
  {
    if(c1.r != c2.r)
      return c1.r < c2.r;
    if(c1.g != c2.g)
      return c1.g < c2.g;
    if(c1.b != c2.b)
      return c1.b < c2.b;
    return c1.a < c2.a;
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
  const auto mat(visual->material.get());

  if(visual->geometry->type == visual->geometry->MESH)
  {
    const auto mesh_info = static_cast<urdf::Mesh*>(visual->geometry.get());
    auto Mm(osgMatFrom(visual->origin.position,visual->origin.rotation, mesh_info->scale)*M);
    addVisualNode(extractMesh(mesh_info->filename), Mm, mat);
    return;
  }

  // we do not know how to display a geometric shape without material
  if(mat == nullptr)
    return;

  const auto Mm(osgMatFrom(visual->origin.position, visual->origin.rotation) *M);

  if(visual->geometry->type == visual->geometry->BOX)
  {
    const auto info = static_cast<urdf::Box*>(visual->geometry.get());
    addVisualShape(osg::make_ref<osg::Box>(osg::Vec3d{}, info->dim.x, info->dim.y, info->dim.z),
                   Mm, mat);
  }
  else if(visual->geometry->type == visual->geometry->SPHERE)
  {
    const auto info = static_cast<urdf::Sphere*>(visual->geometry.get());
    addVisualShape(osg::make_ref<osg::Sphere>(osg::Vec3d{}, info->radius),
                   Mm, mat);
  }
  else
  {
    const auto info = static_cast<urdf::Cylinder*>(visual->geometry.get());
    addVisualShape(osg::make_ref<osg::Cylinder>(osg::Vec3d{}, info->radius, info->length),
                   Mm, mat);
  }
}

void Link::addVisualShape(osg::ref_ptr<osg::Shape> shape, const osg::Matrixd &M, const urdf::Material *mat)
{
  auto drawable = osg::make_ref<osg::ShapeDrawable>(shape);
  auto geode = osg::make_ref<osg::Geode>();
  geode->addDrawable(drawable);
  addVisualNode(geode, M, mat);
}

void Link::addVisualNode(osg::ref_ptr<osg::Node> node, const osg::Matrixd &M, const urdf::Material *mat)
{
  if(mat)
  {
    if(!mat->texture_filename.empty())
    {
      auto image = osgDB::readImageFile(mat->texture_filename);

      auto texture = osg::make_ref<osg::Texture2D>(image);
      texture->setFilter(osg::Texture2D::FilterParameter::MIN_FILTER,osg::Texture2D::FilterMode::LINEAR);
      texture->setFilter(osg::Texture2D::FilterParameter::MAG_FILTER,osg::Texture2D::FilterMode::LINEAR);

      auto stateset = osg::make_ref<osg::StateSet>();
      stateset->setTextureAttribute(0,texture,osg::StateAttribute::OVERRIDE);
      stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
      stateset->setTextureMode(0,GL_TEXTURE_GEN_S,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
      stateset->setTextureMode(0,GL_TEXTURE_GEN_T,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
      node->setStateSet(stateset);
    }
    else
    {
      static std::map<urdf::Color, osg::ref_ptr<osg::StateSet>> rgbaCache;
      auto& stateset = rgbaCache[mat->color];
      if(!stateset.valid())
      {
        stateset = osg::make_ref<osg::StateSet>();
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
      node->setStateSet(stateset);
    }
  }
  auto &visual{visuals.emplace_back(node, M)};
  pose->addChild(visual.pose);
  OceanScene::setupMeshNode(visual.mesh);
}
}
