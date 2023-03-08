#ifndef CORAL_MARKER_H
#define CORAL_MARKER_H

#include <nav_msgs/msg/path.hpp>
#include <osg/Vec3d>
#include <osg/MatrixTransform>
#include <coral/transforms.h>
#include <urdf/model.h>
#include <coral/visual.h>

namespace coral
{

class Marker : public Visual
{
  friend class Path;
  static osg::Group* world;
public:  
  Marker(const Visual &visual, bool moving = false);
  Marker(const Shapes &shapes, const osg::Matrix &M, osg::StateSet *stateset, bool moving = false)
    : Marker(fromShapes(shapes, M, stateset), moving) {}
  inline static void setWorld(osg::Group* world)
  {
    Marker::world = world;
  }
  inline ~Marker()
  {
    world->removeChild(pose);
  }
};


class Goal : public Marker
{
  constexpr static auto length{1.f};
  constexpr static auto radius{.1f};
  constexpr static auto head{.2f};

private:

  inline void hide()
  {
    pose->setMatrix(osg::Matrix::translate(0,0,-1000));
  }

public:
  Goal(const osg::Vec4 &rgba = {1.f, 0.f, 0.f, 1.f});
  void update(const Buffer &buffer, const geometry_msgs::msg::PoseStamped &msg);
};


class Path
{
  // how much we approximate a sequence of point with cylinders
  constexpr static float radius{0.02f};
  osg::StateSet* color;

private:
  std::vector<osg::Vec3d> points;
  std::vector<Marker> segments;

  void reset(size_t dim = 0);

public:
  Path(const osg::Vec4 &rgba = {.2, .7, .7, 1.})
  {
    color = Visual::makeStateSet(rgba);
  }
  void update(const Buffer &buffer, const nav_msgs::msg::Path &path);

};


}

#endif // CORAL_MARKER_H
