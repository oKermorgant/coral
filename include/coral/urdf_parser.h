#ifndef CORAL_URDF_PARSER_H
#define CORAL_URDF_PARSER_H

#include <urdf/model.h>
#include <coral/transforms.h>
#include <osg/MatrixTransform>
#include <optional>

namespace tinyxml2
{
struct XMLElement;
}

namespace coral
{

namespace urdf_parser
{

// temporary structure to extract the camera info before building an actual simulated camera
struct CameraInfo
{
  int height{600}, width{800};
  double fov{60};
  double clip_near{0.1}, clip_far;
  std::string topic;
  std::string frame_id;
  int rate{20};
  osg::ref_ptr<osg::MatrixTransform> pose = new osg::MatrixTransform;

  CameraInfo(const std::string &link, const tinyxml2::XMLElement* sensor_elem);
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
  using Visual = std::pair<urdf::VisualSharedPtr, osg::Matrix>;
  std::string name;
  LinkInfo* parent = nullptr;
  std::vector<LinkInfo*> children;

  std::vector<Visual> visuals;
  std::vector<CameraInfo> cameras;

  std::optional<osg::Matrix> pose = std::nullopt;

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

  bool isAncestorOf(const LinkInfo &link) const
  {
    if(parent == nullptr || link.parent == this)
      return true;
    if(link.parent == nullptr)
      return false;
    return parent->isAncestorOf(link);
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
