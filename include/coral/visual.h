#ifndef CORAL_VISUAL_H
#define CORAL_VISUAL_H

#include <memory>
#include <osg/ref_ptr>
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

  osg::ref_ptr<osg::Group> pose;

public:

  using Shapes = std::vector<osg::ref_ptr<osg::Shape>>;

  Visual() = default;
  Visual(const osg::Matrix &M, osg::ref_ptr<osg::Node> mesh = {}) : pose{new osg::MatrixTransform(M)}
  {
    if(mesh)
      pose->addChild(mesh);
  }

  void configure(bool render, decltype (osg::Object::STATIC) variance);

  inline auto frame() {return pose.get();}

  // factories
  static std::optional<Visual> fromURDF(const urdf::Visual &visual, const osg::Matrix &M);
  static Visual fromShapes(const Shapes &shapes, const osg::Matrix &M, osg::StateSet *stateset);

  // utilities and cache
  // return the stateset corresponding to the texture or color, with cache
  static osg::StateSet* makeStateSet(osg::Vec4f rgba, const std::string &texture="");
};

}

#endif // CORAL_RESOURCE_HELPERS_H
