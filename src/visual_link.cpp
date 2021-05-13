#include <coral/visual_link.h>
#include <coral/resource_helpers.h>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <filesystem>

namespace fs = std::filesystem;

namespace coral
{

osg::Vec3d osgVecFrom(const std::vector<double> &xyz)
{
  return {xyz[0], xyz[1], xyz[2]};
}

osg::Vec3d osgVecFrom(const urdf::Vector3 &v)
{
  return {v.x, v.y, v.z};
}

osg::Quat osgQuatFrom(const std::vector<double> &rpy)
{
  osg::Matrixd Rx, Ry, Rz;
  Rx.makeRotate(rpy[0], 1, 0, 0);
  Ry.makeRotate(rpy[1], 0, 1, 0);
  Rz.makeRotate(rpy[2], 0, 0, 1);
  return (Rz * Ry * Rx).getRotate();
}

osg::Quat osgQuatFrom(const urdf::Rotation &q)
{
  return {q.x, q.y, q.z, q.w};
}

VisualLink::Visual::Visual(const std::string &mesh, const osg::Matrixd &M)
  : mesh(extractMesh(mesh)), pose(new osg::MatrixTransform(M))
{
  pose->addChild(this->mesh);
}


VisualLink::VisualLink(std::string name, urdf::LinkSharedPtr link) : name(name)
{
  if(link == nullptr)
    return;

  for(const auto &visual: link->visual_array)
    parseVisual(visual);
}

void VisualLink::parseVisual(urdf::VisualSharedPtr visual)
{
  // only considers meshes visuals
  // TODO deal with geometric ones (one day...)
  if(visual->geometry->type == visual->geometry->MESH)
  {
    const auto mesh_info = static_cast<urdf::Mesh*>(visual->geometry.get());
    addVisual(mesh_info->filename,
              osgVecFrom(visual->origin.position),
              osgQuatFrom(visual->origin.rotation),
              osgVecFrom(mesh_info->scale));
  }
}

void VisualLink::addVisual(const std::string &mesh, const std::vector<double> &xyz, const std::vector<double> &rpy, const std::vector<double> &scale)
{
  addVisual(mesh,
            osgVecFrom(xyz),
            osgQuatFrom(rpy),
            osgVecFrom(scale));
}

void VisualLink::addVisual(const std::string &mesh, osg::Vec3d t, osg::Quat q, osg::Vec3d scale)
{
  osg::Matrixd M(q);
  M.setTrans(t);
  M.preMultScale(scale);
  visuals.emplace_back(mesh, M);
}

void VisualLink::attachTo(osg::Group * parent)
{
  // build tree from parent to visuals
  pose = new osg::MatrixTransform();

  for(const auto &visual: visuals)
    pose->addChild(visual.pose);

  parent->addChild(pose);
}

void VisualLink::refreshFrom(tf2_ros::Buffer &tf_buffer)
{
  static geometry_msgs::msg::TransformStamped tr;
  static const std::string world("world");
  static const auto timeout(std::chrono::milliseconds(10));

  if(!tf_buffer.canTransform(world, name, tf2::TimePointZero, timeout))
    return;

  tr = tf_buffer.lookupTransform(world, name, tf2::TimePointZero, timeout);
  const auto &t(tr.transform.translation);
  const auto &q(tr.transform.rotation);

  osg::Matrixd M(osg::Quat{q.x, q.y, q.z, q.w});
  M.setTrans(t.x, t.y, t.z);

  pose->setMatrix(M);
}

}
