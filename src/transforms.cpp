#include <coral/transforms.h>
#include <tf2_ros/transform_listener.h>


coral::Buffer::Buffer(rclcpp::Clock::SharedPtr clock) : tf2_ros::Buffer(clock)
{
  static tf2_ros::TransformListener listener(*this);
}

osg::Matrix coral::osgMatFrom(const urdf::Vector3 &t, const urdf::Rotation &q, const urdf::Vector3 &scale)
{
  osg::Matrix M(-osg::Quat{q.x, q.y, q.z, q.w});
  M.setTrans(osgVecFrom(t));
  M.preMultScale(osgVecFrom(scale));
  return M;
}