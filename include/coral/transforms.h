#ifndef CORAL_TRANSFORMS_H
#define CORAL_TRANSFORMS_H

#include <urdf_model/pose.h>
#include <osg/MatrixTransform>
#include <tf2_ros/buffer.h>


namespace coral
{

class Buffer : public tf2_ros::Buffer
{
  constexpr static auto timeout{std::chrono::milliseconds(10)};
  constexpr static auto world{"world"};
public:
  explicit Buffer(rclcpp::Node *spinning_node);
  inline std::optional<geometry_msgs::msg::Transform> lookup(const std::string &frame,const std::string &reference = world) const
  {
    if(canTransform(reference, frame, tf2::TimePointZero, timeout))
      return lookupTransform(reference, frame, tf2::TimePointZero).transform;
    return {};
  }
  inline bool ready() const
  {
    static auto ok{false};
    if(ok)  return true;
    return ok = _frameExists(world);
  }
  inline std::optional<std::string> getParent(const std::string &frame) const
  {
    std::string parent;
    if(_getParent(frame, tf2::TimePointZero, parent))
      return parent;
    return {};
  }
};

// some conversions
template <class Translation>
inline osg::Matrix osgMatFrom(const Translation &t, const geometry_msgs::msg::Quaternion &q)
{
  osg::Matrix M(osg::Quat{q.x, q.y, q.z, q.w});
  M.setTrans(t.x, t.y, t.z);
  return M;
}

osg::Matrix osgMatFrom(const urdf::Vector3 &t,
                        const urdf::Rotation &q,
                        const urdf::Vector3 &scale = {1,1,1});

template <class Translation>
inline osg::Vec3 osgVecFrom(const Translation &t)
{
  return osg::Vec3(t.x, t.y, t.z);
}

}

#endif // CORAL_TRANSFORMS_H
