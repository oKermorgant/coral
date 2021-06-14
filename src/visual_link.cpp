#include <coral/visual_link.h>
#include <coral/resource_helpers.h>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <filesystem>

namespace fs = std::filesystem;

namespace coral
{

VisualLink::Visual::Visual(osg::Node *mesh, const osg::Matrixd &M)
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

osg::Matrixd VisualLink::osgMatFrom(const std::vector<double> &xyz, const std::vector<double> &rpy, const std::vector<double> &scale)
{
  static const osg::Vec3d X(1,0,0);
  static const osg::Vec3d Y(0,1,0);
  static const osg::Vec3d Z(0,0,1);
  osg::Matrixd M(-osg::Quat(rpy[0], X, rpy[1], Y, rpy[2], Z));
  M.setTrans(osgVecFrom(xyz));
  M.preMultScale(osgVecFrom(scale));
  return M;
}

osg::Matrixd VisualLink::osgMatFrom(const urdf::Vector3 &t, const urdf::Rotation &q, const urdf::Vector3 &scale)
{
  osg::Matrixd M(-osg::Quat{q.x, q.y, q.z, q.w});
  M.setTrans(osgVecFrom(t));
  M.preMultScale(osgVecFrom(scale));
  return M;
}

VisualLink::VisualLink(std::string name, urdf::LinkSharedPtr link) : name(name)
{
  if(link == nullptr)
    return;

  for(const auto &visual: link->visual_array)
    addVisual(visual);
}

void VisualLink::addVisual(urdf::VisualSharedPtr visual)
{
  // for now only considers meshed visuals

  if(visual->geometry->type == visual->geometry->MESH)
  {
    const auto mesh_info = static_cast<urdf::Mesh*>(visual->geometry.get());
    addVisualMesh(mesh_info->filename,
                  osgMatFrom(visual->origin.position,visual->origin.rotation,mesh_info->scale)
                  );
    return;
  }

  auto mat(visual->material);
  if(mat == nullptr)
    return;

  const auto pose(osgMatFrom(visual->origin.position, visual->origin.rotation));

  if(visual->geometry->type == visual->geometry->BOX)
  {
    const auto info = static_cast<urdf::Box*>(visual->geometry.get());
    addVisualBox(osgVecFrom(info->dim), pose, *mat);
  }
  else if(visual->geometry->type == visual->geometry->SPHERE)
  {
    const auto info = static_cast<urdf::Sphere*>(visual->geometry.get());
    addVisualSphere(info->radius, pose, *mat);
  }
  else
  {
    const auto info = static_cast<urdf::Cylinder*>(visual->geometry.get());
    addVisualCylinder(info->radius, info->length, pose, *mat);
  }
}


void VisualLink::addVisualMesh(const std::string &mesh, const osg::Matrixd &M)
{
  addVisualNode(extractMesh(mesh), M);
}

void VisualLink::addVisualBox([[maybe_unused]] const osg::Vec3d &dim,
[[maybe_unused]] const osg::Matrixd &M,
[[maybe_unused]] const urdf::Material &mat)
{
  // TODO
}

void VisualLink::addVisualSphere([[maybe_unused]] double radius, [[maybe_unused]] const osg::Matrixd &M, [[maybe_unused]] const urdf::Material &mat)
{
  // TODO
}

void VisualLink::addVisualCylinder([[maybe_unused]] double radius,
[[maybe_unused]] double length,
[[maybe_unused]] const osg::Matrixd &M,
[[maybe_unused]] const urdf::Material &mat)
{
  // TODO
}


void VisualLink::attachTo(coral::Scene *scene)
{
  // build tree from parent to visuals
  pose = new osg::MatrixTransform();
  pose->setDataVariance(osg::Object::DYNAMIC);

  for(const auto &visual: visuals)
  {
    pose->addChild(visual.pose);
    scene->setupMeshNode(visual.mesh.get());

    if(visual.mesh->getStateSet() == nullptr)
    {
      std::cout << name << ": no stateset" << std::endl;
    }
  }
  scene->lock();
  scene->oceanScene()->addChild(pose);
  scene->unlock();
}

void VisualLink::refreshFrom(tf2_ros::Buffer &tf_buffer, const std::vector<std::string> &tf_links)
{
  if(std::find(tf_links.begin(), tf_links.end(), name) == tf_links.end())
    return;

  static geometry_msgs::msg::TransformStamped tr;
  static osg::Matrixd M;
  static const std::string world("world");
  static const auto timeout(std::chrono::milliseconds(10));

  if(!tf_buffer.canTransform(world, name, tf2::TimePointZero, timeout))
    return;

  tr = tf_buffer.lookupTransform(world, name, tf2::TimePointZero, timeout);
  const auto &t(tr.transform.translation);
  const auto &q(tr.transform.rotation);

  M.setRotate({q.x, q.y, q.z, q.w});
  M.setTrans(t.x, t.y, t.z);

  pose->setMatrix(M);
}

}
