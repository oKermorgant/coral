#ifndef CORAL_VISUAL_LINK_H
#define CORAL_VISUAL_LINK_H

#include <string>
#include <osg/Vec3d>
#include <osg/Quat>
#include <osg/MatrixTransform>
#include <urdf_model/link.h>
#include <tf2_ros/buffer.h>

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

  Link(std::string name, urdf::LinkSharedPtr link = nullptr);
  void attachTo(coral::Scene *scene);
  void refreshFrom(tf2_ros::Buffer &tf_buffer, const std::vector<std::string> &tf_links);
  auto frame() const {return pose.get();}

  // visual things
  struct Visual
  {
    osg::ref_ptr<osg::Node> mesh;
    osg::ref_ptr <osg::MatrixTransform > pose;
    Visual(osg::Node *mesh, const osg::Matrixd &M);
  };
  inline bool hasVisuals() const {return !visuals.empty();}
  static urdf::MaterialSharedPtr uuvMaterial(const std::string &mesh_file = "");
  void addVisualMesh(const std::string &mesh, const osg::Matrixd &M, const urdf::Material* mat);
  void addVisualBox(const osg::Vec3d &dim, const osg::Matrixd &M, const urdf::Material* mat);
  void addVisualSphere(double radius, const osg::Matrixd &M, const urdf::Material* mat);
  void addVisualCylinder(double radius, double length, const osg::Matrixd &M, const urdf::Material* mat);

private:

  const std::string name;
  osg::ref_ptr < osg::MatrixTransform > pose;

  void addVisual(urdf::VisualSharedPtr visual);
  inline void addVisualNode(osg::Node* frame, const osg::Matrixd &M, const urdf::Material* mat);

  std::vector<Visual> visuals;

};

}
#endif

