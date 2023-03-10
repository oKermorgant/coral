#ifndef CORAL_VISUAL_H
#define CORAL_VISUAL_H

#include <memory>
#include <osg/Node>
#include <osg/MatrixTransform>
#include <coral/masks.h>
#include <optional>

namespace urdf
{
class Visual;
}

namespace coral
{

class Visual
{
protected:

  osg::ref_ptr<osg::Group> base;

public:

  using Shapes = std::vector<osg::ref_ptr<osg::Shape>>;

  Visual(osg::ref_ptr<osg::Group> mesh = {}, const std::optional<osg::Matrix> &M = std::nullopt);
  void configure(bool moving = false, bool seen_by_cameras = true);

  inline auto frame() {return base.get();}

  // factories
  static std::optional<Visual> fromURDF(const urdf::Visual &urdf, const osg::Matrix &M);
  static Visual fromShapes(const Shapes &shapes, osg::StateSet *stateset, const std::optional<osg::Matrix> &M = std::nullopt);

  // utilities and cache
  // return the stateset corresponding to the texture or color, with cache
  static osg::StateSet* makeStateSet(const osg::Vec4 &rgba, const std::string &texture="");
  static inline auto makeStateSet(const std::array<float, 3> &rgb)
  {
    return makeStateSet({rgb[0],rgb[1],rgb[2],1.f});
  }
};

}

#endif // CORAL_RESOURCE_HELPERS_H
