#include <coral/link.h>
#include <coral/camera.h>

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
    const auto pose{buffer.lookup(name, parent)};
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

  if(info.cameras.empty())
    return;

  Camera::addCameras(pose, info.cameras);

}


}
