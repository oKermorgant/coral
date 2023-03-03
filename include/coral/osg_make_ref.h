#ifndef CORAL_OSG_MAKE_REF_H
#define CORAL_OSG_MAKE_REF_H

#include <osg/Referenced>
#include <osg/ref_ptr>
#include <type_traits>
#include <utility>

// wrap creation of a osg::ref_ptr
namespace osg
{
template <class osgPointee, typename... Args>
inline auto make_ref( Args&&... args )
{
  static_assert (std::is_base_of<osg::Referenced, osgPointee>::value,
      "osg::make_ref should only be used on osg::Referenced derived classes");
  return osg::ref_ptr<osgPointee>(new osgPointee(std::forward<Args>(args)...));
}
}


#endif // CORAL_OSG_MAKE_REF_H
