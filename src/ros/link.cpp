#include <coral/link.h>

namespace coral
{

void Link::refreshFrom(const Buffer &buffer)
{
  if(pose_msg.has_value())
  {
    setPending(osgMatFrom(pose_msg->position, pose_msg->orientation));
  }
  else
  {
    const auto pose{buffer.lookup(name_, parent->name_)};
    if(pose.has_value())
      setPending(pose.value());
  }
}


void Link::addElements(const urdf_parser::LinkInfo &info)
{
  for(const auto &[urdf,M]: info.visuals)
  {
    auto visual{Visual::fromURDF(*urdf, M)};
    if(visual.has_value())
      visual->attachTo(pose, true);
  }
}


}
