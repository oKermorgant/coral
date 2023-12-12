#include <coral/link.h>

namespace coral
{

void Link::refreshFrom(const Buffer &buffer)
{
  if(last.has_value())
  {
    setPending(osgMatFrom(last->position, last->orientation));
  }
  else
  {
    const auto pose{buffer.lookup(name_, parent->name_)};
    if(pose.has_value())
      setPending(osgMatFrom(pose->translation, pose->rotation));
  }
}


void Link::addElements(const urdf_parser::LinkInfo &info)
{
  for(const auto &[urdf,M]: info.visuals)
  {
    auto visual{Visual::fromURDF(*urdf, M)};
    if(visual.has_value())
      visual->attachTo(pose);
  }
}


}
