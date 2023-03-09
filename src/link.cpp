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
    const auto pose{buffer.lookup(name, parent)};
    if(pose.has_value())
      setPending(osgMatFrom(pose->translation, pose->rotation));
  }
}

}
