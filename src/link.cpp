#include <coral/link.h>

namespace coral
{

Link::Link(std::string name, urdf::LinkSharedPtr link) : name(name)
{
  if(link == nullptr)
    return;

  for(const auto &visual: link->visual_array)
    addVisual(visual);
}

void Link::attachTo(coral::Scene *scene)
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


void Link::refreshFrom(tf2_ros::Buffer &tf_buffer, const std::vector<std::string> &tf_links)
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
