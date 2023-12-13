#ifndef CORAL_TRANSFORMS_H
#define CORAL_TRANSFORMS_H

#include <urdf_model/pose.h>
#include <osg/MatrixTransform>
#include <tf2_ros/buffer.h>


namespace coral
{

constexpr auto WORLD_NAME = "world";

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

class Buffer : public tf2_ros::Buffer
{
  constexpr static auto timeout{std::chrono::milliseconds(10)};
public:
  explicit Buffer(rclcpp::Node *spinning_node);
  inline std::optional<osg::Matrix> lookup(const std::string &frame,const std::string &reference = WORLD_NAME) const
  {
    if(canTransform(reference, frame, tf2::TimePointZero, timeout))
    {
      const auto tf{lookupTransform(reference, frame, tf2::TimePointZero).transform};
      return osgMatFrom(tf.translation, tf.rotation);
    }
    return {};
  }
  inline bool ready() const
  {
    static auto ok{false};
    if(ok)  return true;
    return ok = _frameExists(WORLD_NAME);
  }
  inline std::optional<std::string> getParent(const std::string &frame) const
  {
    std::string parent;
    if(_getParent(frame, tf2::TimePointZero, parent))
      return parent;
    return {};
  }
};



}

#endif // CORAL_TRANSFORMS_H
