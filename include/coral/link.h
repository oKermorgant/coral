#ifndef CORAL_VISUAL_LINK_H
#define CORAL_VISUAL_LINK_H

#include <string>
#include <osg/Vec3d>
#include <osg/Quat>
#include <osg/MatrixTransform>
#include <urdf_model/link.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <coral/Scene.h>

namespace coral
{

class Link
{
public:
  // some conversions
  static osg::Matrixd osgMatFrom(const std::vector<double> &xyz,
                                 const std::vector<double> &rpy,
                                 const std::vector<double> &scale = {1,1,1});
  static osg::Matrixd osgMatFrom(const urdf::Vector3 &t,
                                 const urdf::Rotation &q,
                                 const urdf::Vector3 &scale = {1,1,1});
  template <class Translation>
  inline static osg::Matrixd osgMatFrom(const Translation &t, const geometry_msgs::msg::Quaternion &q)
  {
    osg::Matrixd M(osg::Quat{q.x, q.y, q.z, q.w});
    M.setTrans(t.x, t.y, t.z);
    return M;
  }

  Link(const std::string &name, osg::MatrixTransform* pose = new osg::MatrixTransform) : name(name), pose(pose)
  {
    pose->setDataVariance(osg::Object::DYNAMIC);
  }

  auto frame() const {return pose.get();}
  inline std::string getName() const {return name;}
  void refreshFrom(const tf2_ros::Buffer &tf_buffer);

  inline void setPose(const osg::Matrixd &M)
  {
    pose->setMatrix(M);    
  }

  // visual things
  struct Visual
  {
    osg::ref_ptr<osg::Node> mesh;
    osg::ref_ptr <osg::MatrixTransform > pose;
    Visual(osg::ref_ptr<osg::Node> mesh, const osg::Matrixd &M);
  };

  inline bool hasVisuals() const {return !visuals.empty();}
  void addVisual(urdf::VisualSharedPtr visual, const osg::Matrixd &M);
  void addVisualBox(const osg::Vec3d &dim, const osg::Matrixd &M, const urdf::Material* mat);
  void addVisualSphere(double radius, const osg::Matrixd &M, const urdf::Material* mat);
  void addVisualCylinder(double radius, double length, const osg::Matrixd &M, const urdf::Material* mat);

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

  inline void ignoreTF() {pose_from_topic = true;}
  inline bool updatedFromTF() const {return !pose_from_topic;}

private:

  const std::string name;
  std::string parent;
  osg::ref_ptr <osg::MatrixTransform> pose;
  bool pose_from_topic{false};

  inline void addVisualNode(osg::ref_ptr<osg::Node> frame, const osg::Matrixd &M, const urdf::Material* mat);

  std::vector<Visual> visuals;

};

}
#endif

