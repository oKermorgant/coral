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
protected:
  static osg::Group* world;
  static rclcpp::Node* node;
  static Buffer* buffer;
  rclcpp::SubscriptionBase::SharedPtr sub;
  std::string frame_id;
public:
  inline virtual ~Marker()
  {
    if(base.valid())
      world->removeChild(base);
  }
  virtual void refresh() = 0;
  inline void configure(bool moving)
  {
    Visual::configure(moving, false);
    world->addChild(base);
  }
  inline auto topic() const {return sub->get_topic_name();}
  static void spawnThrough(rclcpp::Node* node, osg::Group* world, Buffer* buffer);
};

namespace markers
{

class Pose : public Marker
{
  constexpr static auto length{1.f};
  constexpr static auto radius{.1f};
  constexpr static auto head{.2f};

private:

  Pose(const std::string &topic, const std::array<float,3> &rgb, bool pose_stamped = false);

  inline void setMatrix(const osg::Matrix &M)
  {
    base->asTransform()->asMatrixTransform()->setMatrix(M);
  }

  inline void hide()
  {
    setMatrix(osg::Matrix::translate(0,0,-1000));
   // pose->asTransform().setMatrix(osg::Matrix::translate(0,0,-1000));
  }

  std::optional<geometry_msgs::msg::Pose> pending;

public:
  Pose(const osg::Vec4 &rgba = {1.f, 0.f, 0.f, 1.f});
  void refresh() override;
};


class Path : public Marker
{
  constexpr static float radius = 0.02f;

  // how much we approximate a sequence of point with cylinders  
  osg::StateSet* color;

private:
  std::vector<osg::Vec3d> points;
  std::optional<std::vector<geometry_msgs::msg::PoseStamped>> pending;

public:
  Path(const std::string &topic, const std::array<float,3> &rgb);
  void refresh() override;
};

}
}

#endif // CORAL_MARKER_H
