#ifndef CORAL_VISUAL_LINK_H
#define CORAL_VISUAL_LINK_H

#include <string>
#include <tf2_ros/buffer.h>
#include <coral/urdf_parser.h>
#include <geometry_msgs/msg/pose.hpp>

namespace coral
{

template <class Translation>
inline osg::Matrixd osgMatFrom(const Translation &t, const geometry_msgs::msg::Quaternion &q)
{
  osg::Matrixd M(osg::Quat{q.x, q.y, q.z, q.w});
  M.setTrans(t.x, t.y, t.z);
  return M;
}

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
    for(const auto &[visual,M]: info.visuals)
      addVisual(visual, M);
    //for(auto &cam: info.cameras)
      //pose->addChild(cam.pose);
  }

  auto frame() const {return pose.get();}
  inline std::string getName() const {return name;}
  void refreshFrom(const tf2_ros::Buffer &tf_buffer);
  inline void refreshFromTopic()
  {
    setPending(osgMatFrom(last.position, last.orientation));
  }
  inline void setPending(const osg::Matrixd &M)
  {
    M_pending = M;
  }
  inline void applyNewPose()
  {
    pose->setMatrix(M_pending);
  }

  // visual things
  struct Visual
  {
    osg::ref_ptr<osg::Node> mesh;
    osg::ref_ptr<osg::MatrixTransform> pose;
    Visual(osg::ref_ptr<osg::Node> mesh, const osg::Matrixd &M);
  };

  inline bool hasVisuals() const {return !visuals.empty();}

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

  auto poseCallback()
  {
      pose_from_topic = true;
      return [&](geometry_msgs::msg::Pose::SharedPtr msg){last = *msg;};
  }

  inline void ignoreTF() {pose_from_topic = true;}
  inline bool updatedFromTF() const {return !pose_from_topic;}

private:

  const std::string name;
  std::string parent;
  osg::Matrixd M_pending;
  osg::ref_ptr <osg::MatrixTransform> pose = new osg::MatrixTransform;
  geometry_msgs::msg::Pose last;

  bool pose_from_topic{false};
  void addVisual(urdf::VisualSharedPtr visual, const osg::Matrixd &M);
  void addVisualShape(osg::ref_ptr<osg::Shape> shape, const osg::Matrixd &M, const urdf::Material* mat);
  inline void addVisualNode(osg::ref_ptr<osg::Node> frame, const osg::Matrixd &M, const urdf::Material* mat);
  std::vector<Visual> visuals;

};

}
#endif

