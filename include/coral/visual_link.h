#ifndef CORAL_VISUAL_LINK_H
#define CORAL_VISUAL_LINK_H

#include <string>
#include <osg/Vec3d>
#include <osg/Quat>
#include <osg/MatrixTransform>
#include <urdf_model/link.h>
#include <tf2_ros/buffer.h>

namespace coral
{

class VisualLink
{
public:

  struct Visual
  {    
    osg::ref_ptr<osg::Node> mesh;
    osg::ref_ptr <osg::MatrixTransform > pose;
    Visual(const std::string &mesh, const osg::Matrixd &M);
  };

  VisualLink(std::string name, urdf::LinkSharedPtr link = nullptr);


  void addVisual(const std::string &mesh, osg::Vec3d t, osg::Quat q, osg::Vec3d scale);
  void addVisual(const std::string &mesh, const std::vector<double> &xyz, const std::vector<double> &rpy, const std::vector<double> &scale);

  inline bool hasVisuals() const {return !visuals.empty();}
  void attachTo(osg::Group * parent);

  void refreshFrom(tf2_ros::Buffer &tf_buffer);

private:

  const std::string name;
  osg::ref_ptr < osg::MatrixTransform > pose;

  void parseVisual(urdf::VisualSharedPtr visual);
  std::vector<Visual> visuals;
  //  S, T, Rx, Ry, Rz, transform;
};

}
#endif
