#include <coral/urdf_parser.h>
#include <urdf_parser/urdf_parser.h>

using std::vector, std::string, std::map;

namespace coral
{
namespace urdf_parser
{
using Visual = std::pair<urdf::VisualSharedPtr, osg::Matrixd>;

enum class LinkBase{FIXED, FLOATING, MERGED};

// helper struct to resolve constant kinematic chains
struct LinkInfo
{
  std::string parent;
  std::vector<Visual> visuals;
  std::vector<CameraInfo> cameras;
  bool has_children = false;
  LinkBase base = LinkBase::FLOATING;
  osg::MatrixTransform* pose = new osg::MatrixTransform;

  LinkInfo *merged = nullptr;

  void mergeInto(LinkInfo &parent)
  {
    merged = &parent;
    if(parent.merged)
    {
      mergeInto(*parent.merged);
      return;
    }

    parent.has_children |= has_children;

    for(const auto &[visual, M]: visuals)
      parent.visuals.push_back({visual, pose->getMatrix()*M});

    for(auto &cam: cameras)
    {
      cam.pose->setMatrix(pose->getMatrix() * cam.pose->getMatrix());
      parent.cameras.push_back(cam);
    }

    base = LinkBase::MERGED;
  }

  bool relative() const {return !parent.empty() && parent != "world";}

  bool empty() const
  {
    return base != LinkBase::FLOATING || (!has_children && visuals.empty() && cameras.empty());
  }
};


std::tuple<string, vector<Link>, vector<CameraInfo>> parse(const string &description, bool with_thrusters)
{
  const auto cameras(CameraInfo::extractFrom(description));

  const auto model(urdf::parseURDF(description));
  map<string, LinkInfo> tree;

  // extract all links
  for(const auto &[name, link]: model->links_)
  {

    if(!with_thrusters && name.find("thruster_") != name.npos)
      continue;

    auto &info(tree[name]);
    for(const auto &visual: link->visual_array)
    {
      info.visuals.push_back({visual, {}});
    }

    for(const auto &cam: cameras)
    {
      if(cam.link_name == name)
        info.cameras.push_back(cam);
    }
  }

  // parse joint structure
  for(const auto &[name,joint]: model->joints_)
  {
    auto &link{tree[joint->child_link_name]};
    link.parent = joint->parent_link_name;
    tree[link.parent].has_children = true;
    link.pose->setMatrix(Link::osgMatFrom(joint->parent_to_joint_origin_transform.position, joint->parent_to_joint_origin_transform.rotation));
    auto limits(joint->limits);
    if(joint->type == urdf::Joint::FIXED || (limits && limits->lower == limits->upper))
      link.base = LinkBase::FIXED;
    else
      link.base = LinkBase::FLOATING;
  }

  // recursive merge until no more fixed links
  while(true)
  {
    const auto link = std::find_if(tree.begin(), tree.end(), [](const auto &link){return link.second.base == LinkBase::FIXED;});
    if(link == tree.end())
      break;
    link->second.mergeInto(tree[link->second.parent]);
  }

  // build actual osg links from floating joints with either visuals or cameras
  vector<Link> links;
  for(auto &[name, info]: tree)
  {
    if(name == "world" || info.empty())
      continue;

    auto link = info.relative() ? Link(name, tree[info.parent].pose) : Link(name);

    for(const auto &[visual,M]: info.visuals)
      link.addVisual(visual, M);
    for(auto &cam: info.cameras)
      link.frame()->addChild(cam.pose);

    links.push_back(link);
  }

  return {model->getRoot()->name, links, cameras};
}

}
}
