#include <coral/link.h>
#include <coral/resource_helpers.h>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <filesystem>
#include <algorithm>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <coral/OceanScene.h>
#include <coral/osg_make_ref.h>

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
    addVisualBox(osgVecFrom(info->dim), Mm, mat);
  }
  else if(visual->geometry->type == visual->geometry->SPHERE)
  {
    const auto info = static_cast<urdf::Sphere*>(visual->geometry.get());
    addVisualSphere(info->radius, Mm, mat);
  }
  else
  {
    const auto info = static_cast<urdf::Cylinder*>(visual->geometry.get());
    addVisualCylinder(info->radius, info->length, Mm, mat);
  }
}

void Link::addVisualBox(const osg::Vec3d &dim, const osg::Matrixd &M, const urdf::Material* mat)
{
  auto box = osg::make_ref<osg::Box>(osg::Vec3d{}, dim.x(), dim.y(), dim.z());
  auto shape = osg::make_ref<osg::ShapeDrawable>(box);
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  geode->addDrawable(shape);

  addVisualNode(geode, M, mat);
}

void Link::addVisualSphere(double radius, const osg::Matrixd &M, const urdf::Material *mat)
{
  osg::Sphere *sphere = new osg::Sphere({}, radius);

  osg::ShapeDrawable *shape = new osg::ShapeDrawable(sphere);
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  geode->addDrawable(shape);
  addVisualNode(geode, M, mat);
}

void Link::addVisualCylinder(double radius, double length, const osg::Matrixd &M, const urdf::Material* mat)
{
  osg::Cylinder *cylinder = new osg::Cylinder({}, radius, length);

  osg::ShapeDrawable *shape = new osg::ShapeDrawable(cylinder);
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  geode->addDrawable(shape);
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
      auto stateset = osg::make_ref<osg::StateSet>();
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
      node->setStateSet(stateset);
    }
  }
  auto &visual{visuals.emplace_back(node, M)};
  pose->addChild(visual.pose);
  OceanScene::setupMeshNode(visual.mesh);
}
}
