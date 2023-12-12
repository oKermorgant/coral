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
  Link(const std::string &name = "") : name_{name}
  {
    pose->setDataVariance(osg::Object::STATIC);
  }
  Link(const urdf_parser::LinkInfo &info)
  {
    name_ = info.name;
    pose->setDataVariance(osg::Object::DYNAMIC);
    addElements(info);
  }

  inline auto frame() const {return pose.get();}
  inline auto name() const {return name_;}

  void addElements(const urdf_parser::LinkInfo &info);
  void refreshFrom(const Buffer &buffer);

  inline void setPending(const osg::Matrix &M)
  {
    M_pending = M;
  }
  inline void applyNewPose()
  {
    pose->setMatrix(M_pending);
  }

 /* inline bool operator==(const std::string &name) const
  {
    return this->name_ == name;
  }*/

  inline void setParent(const Link &root)
  {
    root.pose->addChild(pose);
    parent = &root;
  }

  /// inits last pose received (disables pose from tf) and returns the callback to update this pose
  inline auto poseCallback()
  {
    last.emplace();
    return [&](geometry_msgs::msg::Pose::SharedPtr msg)
    {
      last = *msg;
    };
  }

private:

  std::string name_;
  Link const * parent = nullptr;
  osg::Matrix M_pending;
  osg::ref_ptr <osg::MatrixTransform> pose = new osg::MatrixTransform;

  std::optional<geometry_msgs::msg::Pose> last;

};

}
#endif
