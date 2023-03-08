#ifndef CORAL_MASKS_H
#define CORAL_MASKS_H

#include <osg/Node>

namespace coral
{

namespace Mask
{
constexpr static unsigned int reflection  {0x1};
constexpr static unsigned int refraction  {0x2};
constexpr static unsigned int heightmap   {0x20};
constexpr static unsigned int surface     {0x8};
constexpr static unsigned int normal      {0x4};
constexpr static unsigned int silt        {0x10};
constexpr static unsigned int marker      {0x32};

inline auto getMask(bool render = true)
{
  if(render)
    return normal | reflection | refraction;
  return marker;
}

}
}

#endif // CORAL_MASKS_H
