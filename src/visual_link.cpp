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
  static const osg::Vec3d X(1,0,0);
  static const osg::Vec3d Y(0,1,0);
  static const osg::Vec3d Z(0,0,1);
  return {rpy[0], X, rpy[1], Y, rpy[2], Z};
}

osg::Quat osgQuatFrom(const urdf::Rotation &q)
{
  return {q.x, q.y, q.z, q.w};
}

VisualLink::Visual::Visual(const std::string &mesh, const osg::Matrixd &M)
  : mesh(extractMesh(mesh)), pose(new osg::MatrixTransform(M))
{
  pose->setDataVariance(osg::Object::STATIC);
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
  // for now only considers meshed visuals
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
  osg::Matrixd M(-q);
  M.setTrans(t);
  M.preMultScale(scale);
  visuals.emplace_back(mesh, M);
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
