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

  Link(const std::string &name, osg::MatrixTransform* pose = new osg::MatrixTransform) : name(name), pose(pose) {}


  void attachTo(coral::Scene *scene) const;
  inline std::string get_name() const {return name;}
  void refreshFrom(tf2_ros::Buffer &tf_buffer);
  auto frame() const {return pose.get();}

  inline void setPose(const osg::Matrixd &M)
  {
    pose->setMatrix(M);    
  }

  // visual things
  struct Visual
  {
    osg::ref_ptr<osg::Node> mesh;
    osg::ref_ptr <osg::MatrixTransform > pose;
    Visual(osg::Node *mesh, const osg::Matrixd &M);
  };

  inline bool hasVisuals() const {return !visuals.empty();}
  static urdf::MaterialSharedPtr uuvMaterial(const std::string &mesh_file = "");
  void addVisual(urdf::VisualSharedPtr visual, const osg::Matrixd &M);
  void addVisualMesh(const std::string &mesh, const osg::Matrixd &M, const urdf::Material* mat);
  void addVisualBox(const osg::Vec3d &dim, const osg::Matrixd &M, const urdf::Material* mat);
  void addVisualSphere(double radius, const osg::Matrixd &M, const urdf::Material* mat);
  void addVisualCylinder(double radius, double length, const osg::Matrixd &M, const urdf::Material* mat);

  inline bool operator==(const std::string &name) const
  {
    return this->name == name;
  }

private:

  const std::string name;
  std::string parent = "world";
  osg::ref_ptr < osg::MatrixTransform > pose;

  inline void addVisualNode(osg::Node* frame, const osg::Matrixd &M, const urdf::Material* mat);

  std::vector<Visual> visuals;

};

}
#endif

