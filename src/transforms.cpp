#include <coral/transforms.h>
#include <tf2_ros/transform_listener.h>


coral::Buffer::Buffer(rclcpp::Node* spinning_node) : tf2_ros::Buffer(spinning_node->get_clock())
{
  static tf2_ros::TransformListener listener(*this, spinning_node);
}

osg::Matrix coral::osgMatFrom(const urdf::Vector3 &t, const urdf::Rotation &q, const urdf::Vector3 &scale)
{
  osg::Matrix M(-osg::Quat{q.x, q.y, q.z, q.w});
  M.setTrans(osgVecFrom(t));
  M.preMultScale(osgVecFrom(scale));
  return M;
}
