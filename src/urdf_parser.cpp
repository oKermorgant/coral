#include <coral/urdf_parser.h>
#include <urdf_parser/urdf_parser.h>

using std::vector, std::string, std::map;

enum class LinkBase{FIXED, FLOATING, MERGED};

std::ostream& operator<<(std::ostream &os, const LinkBase &base)
{
  if(base == LinkBase::FIXED)
    os << "FIXED";
  else if(base == LinkBase::FLOATING)
    os << "FLOATING";
  else
     os << "MERGED";
  return os;
}

namespace coral
{
namespace urdf_parser
{
using Visual = std::pair<urdf::VisualSharedPtr, osg::Matrixd>;

// helper struct to resolve constant kinematic chains
struct LinkInfo
{
  std::string name;
  LinkInfo* parent = nullptr;
  std::vector<LinkInfo*> children;

  std::vector<Visual> visuals;
  std::vector<CameraInfo> cameras;

  LinkBase base = LinkBase::FLOATING;
  osg::ref_ptr <osg::MatrixTransform> pose = new osg::MatrixTransform;

  explicit LinkInfo(const std::string &name) : name{name} {}

  inline bool operator==(const std::string &name) const
  {
    return name == this->name;
  }

  void setParent(LinkInfo *parent)
  {
    this->parent = parent;
    parent->children.push_back(this);
  }

  void mergeIntoParent()
  {
    //std::transform(children.begin(), children.end(), std::back_inserter(parent.children),
    //               SubLink::merge);

    for(auto child: children)
    {
      child->pose->setMatrix(pose->getMatrix() * child->pose->getMatrix());
      child->setParent(parent);
    }

    for(const auto &[visual, M]: visuals)
      parent->visuals.push_back({visual, pose->getMatrix()*M});

    for(auto &cam: cameras)
    {
      cam.pose->setMatrix(pose->getMatrix() * cam.pose->getMatrix());
      parent->cameras.push_back(cam);
    }
    base = LinkBase::MERGED;
  }

  inline bool isUseful() const
  {
    return name != "world" && base == LinkBase::FLOATING &&
        (!children.empty() || !visuals.empty() || !cameras.empty());
  }

  static bool canBeMerged(const LinkInfo &link)
  {
    if(link.base == LinkBase::MERGED) return false;
    return link.parent && (link.base == LinkBase::FIXED || (link.cameras.empty() && link.visuals.empty()));
  }

  Link toOSGLink()
  {
    // find a parent link that will be kept
    if(parent)
    {
      while(parent->parent && !parent->isUseful())
        parent = parent->parent;
    }

    auto link{Link(name)}; //, parent ? parent->pose.get() : new osg::MatrixTransform)};

    for(const auto &[visual,M]: visuals)
      link.addVisual(visual, M);
    for(auto &cam: cameras)
      link.frame()->addChild(cam.pose);

    return link;
  }

  void toWorldLink(Link &world) const
  {
    for(const auto &[visual,M]: visuals)
      world.addVisual(visual, M);
    for(auto &cam: cameras)
      world.frame()->addChild(cam.pose);
  }
};


std::tuple<vector<Link>, vector<CameraInfo>> parse(const string &description, bool with_thrusters, Link &world_link)
{
  const auto cameras(CameraInfo::extractFrom(description));
  const auto model(urdf::parseURDF(description));

  std::vector<LinkInfo> tree;
  // ensure pointers stay valid along the way
  tree.reserve(model->links_.size());
  // find-shortcut, will always have a good find
  const auto findInTree = [&](const std::string &name) {return std::find(tree.begin(), tree.end(), name);};

  // extract all links
  for(const auto &[name, link]: model->links_)
  {
    if(!with_thrusters && name.find("thruster") != name.npos)
      continue;

    auto &info(tree.emplace_back(name));
    std::transform(link->visual_array.begin(), link->visual_array.end(), std::back_inserter(info.visuals),
                   [](const auto &visual){return Visual{visual, {}};});

    for(const auto &cam: cameras)
    {
      if(cam.link_name == name)
        info.cameras.push_back(cam);
    }
  }

  // parse joint structure
  for(const auto &[name,joint]: model->joints_)
  {
    auto link{findInTree(joint->child_link_name)};
    link->setParent(findInTree(joint->parent_link_name).base());
    auto limits(joint->limits);
    if(joint->type == urdf::Joint::FIXED || (limits && limits->lower == limits->upper))
    {
      link->base = LinkBase::FIXED;
      link->pose->setMatrix(Link::osgMatFrom(joint->parent_to_joint_origin_transform.position, joint->parent_to_joint_origin_transform.rotation));
    }
    else
    {
      // pose is not important, it will change
      link->base = LinkBase::FLOATING;      
    }
  }

  // top-down merge
  while(true)
  {
    auto link{std::find_if(tree.begin(), tree.end(), LinkInfo::canBeMerged)};
    if(link == tree.end())
      break;
    link->mergeIntoParent();
  }


  // build actual osg links from floating joints with either visuals or cameras  
  vector<Link> links;  
  links.reserve(tree.size());
  // add all floating links except world
  for(auto &link: tree)
  {
    if(link.isUseful())
      links.push_back(link.toOSGLink());
  }

  // register parent link for the new links, will be used for tf lookups to update the links
  for(auto &link: links)
  {
    const auto &parent{findInTree(link.getName())->parent};
    if(!parent || parent->name == "world")
    {
      link.setParent(world_link);
    }
    else
    {
      auto parent_link{std::find(links.begin(), links.end(), parent->name)};
      link.setParent(*parent_link);
    }
  }

  // transfer visuals to world link if any fixed ones
  if(const auto root{findInTree(model->getRoot()->name)}; root->name == "world")
    root->toWorldLink(world_link);

  return {links, cameras};
}

}
}
