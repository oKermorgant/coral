#include <coral/link.h>
#include <coral/resource_helpers.h>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <filesystem>
#include <algorithm>
#include <osg/Material>
#include <osg/ShapeDrawable>



namespace coral
{

Link::Visual::Visual(osg::Node *mesh, const osg::Matrixd &M)
  : mesh(mesh), pose(new osg::MatrixTransform(M))
{
  pose->setDataVariance(osg::Object::STATIC);
  pose->addChild(this->mesh);
}

osg::Vec3 osgVecFrom(const std::vector<double> &xyz)
{
  return osg::Vec3(xyz[0], xyz[1], xyz[2]);
}
osg::Vec3 osgVecFrom(const urdf::Vector3 &t)
{
  return osg::Vec3(t.x, t.y, t.z);
}

osg::Matrixd Link::osgMatFrom(const std::vector<double> &xyz, const std::vector<double> &rpy, const std::vector<double> &scale)
{
  static const osg::Vec3d X(1,0,0);
  static const osg::Vec3d Y(0,1,0);
  static const osg::Vec3d Z(0,0,1);
  osg::Matrixd M(-osg::Quat(rpy[0], X, rpy[1], Y, rpy[2], Z));
  M.setTrans(osgVecFrom(xyz));
  M.preMultScale(osgVecFrom(scale));
  return M;
}

osg::Matrixd Link::osgMatFrom(const urdf::Vector3 &t, const urdf::Rotation &q, const urdf::Vector3 &scale)
{
  osg::Matrixd M(-osg::Quat{q.x, q.y, q.z, q.w});
  M.setTrans(osgVecFrom(t));
  M.preMultScale(osgVecFrom(scale));
  return M;
}


void Link::addVisual(urdf::VisualSharedPtr visual, const osg::Matrixd &M)
{
  const auto mat(visual->material.get());

  if(visual->geometry->type == visual->geometry->MESH)
  {
    const auto mesh_info = static_cast<urdf::Mesh*>(visual->geometry.get());
    auto Mm(osgMatFrom(visual->origin.position,visual->origin.rotation, mesh_info->scale)*M);

    addVisualMesh(mesh_info->filename, Mm, mat);
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

void Link::addVisualMesh(const std::string &mesh, const osg::Matrixd &M, const urdf::Material* mat = nullptr)
{
  addVisualNode(extractMesh(mesh), M, mat);
}

void Link::addVisualBox(const osg::Vec3d &dim, const osg::Matrixd &M, const urdf::Material* mat)
{
  osg::Box *box = new osg::Box({}, dim.x(), dim.y(), dim.z());

  osg::ShapeDrawable *shape = new osg::ShapeDrawable(box);
  osg::Geode *geode = new osg::Geode();
  geode->addDrawable(shape);
  addVisualNode(geode, M, mat);
}

void Link::addVisualSphere(double radius, const osg::Matrixd &M, const urdf::Material *mat)
{
  osg::Sphere *sphere = new osg::Sphere({}, radius);

  osg::ShapeDrawable *shape = new osg::ShapeDrawable(sphere);
  osg::Geode *geode = new osg::Geode();
  geode->addDrawable(shape);
  addVisualNode(geode, M, mat);
}

void Link::addVisualCylinder(double radius, double length, const osg::Matrixd &M, const urdf::Material* mat)
{
  osg::Cylinder *cylinder = new osg::Cylinder({}, radius, length);

  osg::ShapeDrawable *shape = new osg::ShapeDrawable(cylinder);
  osg::Geode *geode = new osg::Geode();
  geode->addDrawable(shape);
  addVisualNode(geode, M, mat);
}

void Link::addVisualNode(osg::Node* node, const osg::Matrixd &M, const urdf::Material *mat)
{
  if(mat)
  {
    if(!mat->texture_filename.empty())
    {
      auto image = osgDB::readImageFile(mat->texture_filename);

      auto texture = new osg::Texture2D(image);
      texture->setFilter(osg::Texture2D::FilterParameter::MIN_FILTER,osg::Texture2D::FilterMode::LINEAR);
      texture->setFilter(osg::Texture2D::FilterParameter::MAG_FILTER,osg::Texture2D::FilterMode::LINEAR);

      auto stateset = new osg::StateSet;
      stateset->setTextureAttribute(0,texture,osg::StateAttribute::OVERRIDE);
      stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
      stateset->setTextureMode(0,GL_TEXTURE_GEN_S,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
      stateset->setTextureMode(0,GL_TEXTURE_GEN_T,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
      node->setStateSet(stateset);
    }
    else if(mat->color.b + mat->color.r + mat->color.g > 0.f)
    {
      osg::ref_ptr < osg::StateSet > stateset = new osg::StateSet();
      osg::ref_ptr < osg::Material > material = new osg::Material();
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

  if(node->asGroup() == nullptr)
  {
    osg::Node * aux = node;
    node = new osg::Group();
    node->asGroup()->addChild(aux);
  }

  visuals.emplace_back(node, M);
}

urdf::MaterialSharedPtr Link::uuvMaterial(const std::string &mesh_file)
{
  auto mat(std::make_shared<urdf::Material>());

  // TODO automagically resolve mesh dependency on a texture
  // will have to identify the mesh is part of a Gazebo model and parse the sdf file
  const std::vector<std::string> sanded_meshes
  {"LakeBottom.dae", "herkules_seabed.dae", "heightmap.dae", "seabed.dae"};

  if(mesh_file == "" ||
     std::any_of(sanded_meshes.begin(), sanded_meshes.end(), [&mesh_file](const auto &name)
  {return mesh_file.find(name) != mesh_file.npos && mesh_file.find("uuv_gazebo_worls") != mesh_file.npos;}))
    mat->texture_filename = "soil_sand_0045_01.jpg";

  return mat;
}




}
