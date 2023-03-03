#include <coral/link.h>

using namespace std::chrono_literals;
using std::vector, std::string;

constexpr auto timeout{10ms};

namespace coral
{

void Link::refreshFrom(const tf2_ros::Buffer &tf_buffer)
{
  static geometry_msgs::msg::TransformStamped tr;
  if(!tf_buffer.canTransform(parent, name, tf2::TimePointZero, timeout))
    return;

  tr = tf_buffer.lookupTransform(parent, name, tf2::TimePointZero, timeout);

  setPending(osgMatFrom(tr.transform.translation, tr.transform.rotation));
}

}
