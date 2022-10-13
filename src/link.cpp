#include <coral/link.h>

using std::vector, std::string;

namespace coral
{


void Link::refreshFrom(const tf2_ros::Buffer &tf_buffer)
{
  static geometry_msgs::msg::TransformStamped tr;

  static const auto timeout(std::chrono::milliseconds(10));

  if(!tf_buffer.canTransform(parent, name, tf2::TimePointZero, timeout))
    return;

  tr = tf_buffer.lookupTransform(parent, name, tf2::TimePointZero, timeout);

  setPose(osgMatFrom(tr.transform.translation, tr.transform.rotation));
}
}
