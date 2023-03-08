#ifndef CORAL_VISUAL_LINK_H
#define CORAL_VISUAL_LINK_H

#include <string>
#include <coral/urdf_parser.h>
#include <coral/visual.h>
#include <geometry_msgs/msg/pose.hpp>

namespace coral
{

class Link
{
public:
  Link(const std::string &name) : name{name}
  {
    pose->setDataVariance(osg::Object::STATIC);
  }
  Link(const urdf_parser::LinkInfo &info) : name{info.name}
  {
    pose->setDataVariance(osg::Object::DYNAMIC);
    addElements(info);
  }

  void addElements(const urdf_parser::LinkInfo &info)
  {
    for(const auto &[urdf,M]: info.visuals)
    {
      auto visual{Visual::fromURDF(*urdf, M)};
      if(visual.has_value())
      {
        visual->configure(true, osg::Object::STATIC);
        pose->addChild(visual->frame());
      }
    }
    //for(auto &cam: info.cameras)
    //pose->addChild(cam.pose);
  }

  inline auto frame() const {return pose.get();}
  inline std::string getName() const {return name;}
  void refreshFrom(const Buffer &buffer);
  inline void setPending(const osg::Matrix &M)
  {
    M_pending = M;
  }
  inline void applyNewPose()
  {
    pose->setMatrix(M_pending);
  }

  inline bool operator==(const std::string &name) const
  {
    return this->name == name;
  }

  inline std::string describe() const
  {
    return name + " (" + parent + ")";
  }

  inline void setParent(const Link &root)
  {
    root.pose->addChild(pose);
    parent = root.name;
  }

  inline auto poseCallback()
  {
    last.emplace();
    return [&](geometry_msgs::msg::Pose::SharedPtr msg)
    {
      setPending(osgMatFrom(msg->position, msg->orientation));
    };
  }

  inline void ignoreTF() {last.emplace();}

private:

  const std::string name;
  std::string parent;
  osg::Matrix M_pending;
  osg::ref_ptr <osg::MatrixTransform> pose = new osg::MatrixTransform;

  std::optional<geometry_msgs::msg::Pose> last;

};

}
#endif
