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
  osg::ref_ptr <osg::MatrixTransform> pose = new osg::MatrixTransform;

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

  static bool isFixed(const std::pair<std::string, LinkInfo> &info)
  {
    return info.second.base == LinkBase::FIXED;
  }

  Link toNamedLink(const std::string &name, osg::MatrixTransform* pose) const
  {

    auto link{Link(name, pose)};

    for(const auto &[visual,M]: visuals)
      link.addVisual(visual, M);
    for(auto &cam: cameras)
      link.frame()->addChild(cam.pose);

    return link;
  }
};


std::tuple<vector<Link>, vector<CameraInfo>> parse(const string &description, bool with_thrusters)
{
  const auto cameras(CameraInfo::extractFrom(description));

  const auto model(urdf::parseURDF(description));
  map<string, LinkInfo> tree;

  // extract all links
  for(const auto &[name, link]: model->links_)
  {
    if(name == "world")
      continue;

    if(!with_thrusters && name.find("thruster") != name.npos)
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
    if(joint->parent_link_name == "world")
      continue;
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
    const auto link = std::find_if(tree.begin(), tree.end(), LinkInfo::isFixed);
    if(link == tree.end())
      break;
    link->second.mergeInto(tree[link->second.parent]);
  }

  // build actual osg links from floating joints with either visuals or cameras
  // find a root link to attach others
  // in standard robots there should be only 1
  vector<Link> links;
  const auto root_frame{std::find_if(tree.begin(), tree.end(), [](const auto &elem)
    {
      return elem.first != "world" && !elem.second.empty() && !elem.second.relative();
    })};
  const auto root_link = root_frame == tree.end() ? "" : root_frame->first;

  // root link is the first in the list
  if(!root_link.empty())
  {
    links.push_back(root_frame->second.toNamedLink(root_link, new osg::MatrixTransform));
  }

  for(const auto &[name, info]: tree)
  {
    if(name == "world" || info.empty() || name == root_link)
      continue;

    links.push_back(info.toNamedLink(name, info.relative() ? tree[info.parent].pose.get() : new osg::MatrixTransform));
  }

  // register all links with regards to root
  for(auto &link: links)
  {
    if(!(link == root_link))
      link.setParent(links.front());
  }

  return {links, cameras};
}

}
}
