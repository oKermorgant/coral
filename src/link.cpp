#include <coral/link.h>

using std::vector, std::string;

namespace coral
{

void Link::attachTo(coral::Scene *scene) const
{
  // build tree from parent to visuals
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

  if(parent == "world")
  {
    scene->lock();
    scene->oceanScene()->addChild(pose);
    scene->unlock();
  }
}


void Link::refreshFrom(tf2_ros::Buffer &tf_buffer)
{
  static geometry_msgs::msg::TransformStamped tr;

  static const auto timeout(std::chrono::milliseconds(10));

  if(!tf_buffer.canTransform(parent, name, tf2::TimePointZero, timeout))
    return;

  tr = tf_buffer.lookupTransform(parent, name, tf2::TimePointZero, timeout);

  setPose(osgMatFrom(tr.transform.translation, tr.transform.rotation));
}

}
