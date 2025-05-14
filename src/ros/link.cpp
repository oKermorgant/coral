#include <coral/link.h>

namespace coral
{

void Link::refreshFrom(const Buffer &buffer)
{
  if(pose_msg.has_value())
    M_pending = osgMatFrom(pose_msg->position, pose_msg->orientation);
  else if(const auto pose{buffer.lookup(name_, parent->name_)}; pose.has_value())
      M_pending = pose.value();
}


void Link::addElements(const urdf_parser::LinkInfo &info)
{
  for(const auto &[urdf,M]: info.visuals)
  {
    if(auto visual{Visual::fromURDF(*urdf, M)}; visual.has_value())
      visual->attachTo(pose, true);
  }
}


}
