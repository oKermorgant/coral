#ifndef CORAL_URDF_PARSER_H
#define CORAL_URDF_PARSER_H

#include <urdf/model.h>
#include <osg/MatrixTransform>
#include <optional>

struct TiXmlElement;

namespace coral
{

// some conversions
osg::Matrixd osgMatFrom(const std::vector<double> &xyz,
                        const std::vector<double> &rpy,
                        const std::vector<double> &scale = {1,1,1});
osg::Matrixd osgMatFrom(const urdf::Vector3 &t,
                        const urdf::Rotation &q,
                        const urdf::Vector3 &scale = {1,1,1});
inline osg::Vec3 osgVecFrom(const std::vector<double> &xyz)
{
  return osg::Vec3(xyz[0], xyz[1], xyz[2]);
}
inline osg::Vec3 osgVecFrom(const urdf::Vector3 &t)
{
  return osg::Vec3(t.x, t.y, t.z);
}

namespace urdf_parser
{

// temporary structure to extract the camera info before building an actual simulated camera
struct CameraInfo
{
  int height, width;
  double fov;
  double clip_near, clip_far;
  std::string topic;
  std::string link_name;
  int period_ms;
  osg::ref_ptr<osg::MatrixTransform> pose = new osg::MatrixTransform;

  CameraInfo(std::string link, const TiXmlElement* sensor_elem);
  inline CameraInfo(const CameraInfo &src, const osg::Matrix &M) : CameraInfo{src}
  {
    // copy with change in the matrix
    pose->setMatrix(M * pose->getMatrix());
  }

  static std::vector<CameraInfo> extractFrom(const std::string &description);
};



// helper struct to simplify kinematic chains



struct LinkInfo
{
  using Visual = std::pair<urdf::VisualSharedPtr, osg::Matrixd>;
  std::string name;
  LinkInfo* parent = nullptr;
  std::vector<LinkInfo*> children;

  std::vector<Visual> visuals;
  std::vector<CameraInfo> cameras;

  std::optional<osg::Matrixd> pose = std::nullopt;

  explicit LinkInfo(const std::string &name) : name{name} {}

  inline bool operator==(const std::string &name) const
  {
    return name == this->name;
  }

  inline bool isFloating() const
  {
    return !pose.has_value();
  }

  inline void setParent(LinkInfo *parent)
  {
    this->parent = parent;
    parent->children.push_back(this);
  }

  void mergeIntoParent();

  // static functions for algorithms
  static bool canBeMerged(const LinkInfo &link);

  static bool isRoot(const LinkInfo &link)
  {
    return link.parent == nullptr;
  }
};

struct Tree : public std::list<LinkInfo>
{  
  explicit Tree(const std::string &description, const bool keep_thrusters);
  inline LinkInfo& add(const std::string &name)
  {
    return emplace_back(name);
  }
  auto find(const std::string &name);
  auto simplify();
};
}

}

#endif // CORAL_URDF_PARSER_H
