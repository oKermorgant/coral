#include <coral/urdf_parser.h>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>

#include <coral/debug_msg.h>

using std::vector, std::string;

namespace coral
{

namespace urdf_parser
{

struct NestedXML
{
  const tinyxml2::XMLElement* root;
  explicit NestedXML(const tinyxml2::XMLElement* root) : root(root) {}

  static const tinyxml2::XMLElement* getNested(const tinyxml2::XMLElement* root, const vector<string> &keys)
  {
    if(keys.size() == 0)  return root;

    const auto child = root->FirstChildElement(keys[0].c_str());
    if(child == nullptr)  return nullptr;

    return getNested(child, {keys.begin()+1, keys.end()});
  }

  bool read(const vector<string> &keys, string &value) const
  {
    const auto elem(getNested(root, keys));
    if(!elem) return false;
    value = elem->GetText();
    return true;
  }

  bool read(const vector<string> &keys, double &value) const
  {
    const auto elem(getNested(root, keys));
    if(!elem) return false;
    value = atof(elem->GetText());
    return true;
  }

  bool read(const vector<string> &keys, int &value) const
  {
    const auto elem(getNested(root, keys));
    if(!elem) return false;
    value = atoi(elem->GetText());
    return true;
  }
};

// camera parser
vector<CameraInfo> CameraInfo::extractFrom(const string &description)
{
  vector<CameraInfo> cameras;

  // tediously parse the description XML to extract gazebo sensors and plugins
  tinyxml2::XMLDocument doc;
  doc.Parse(description.c_str());
  auto root = doc.RootElement();

  for(auto gazebo_elem = root->FirstChildElement("gazebo");
      gazebo_elem != nullptr;
      gazebo_elem = gazebo_elem->NextSiblingElement("gazebo"))
  {
    if(!gazebo_elem->Attribute("reference"))
      continue;

    const string link(gazebo_elem->Attribute("reference"));
    for(auto sensor = gazebo_elem->FirstChildElement("sensor");
        sensor != nullptr;
        sensor = sensor->NextSiblingElement("sensor"))
    {
      if(sensor->Attribute("type") != std::string{"camera"})
        continue;
      cameras.emplace_back(link, sensor);
    }
  }
  return cameras;
}

CameraInfo::CameraInfo(const std::string &link, const tinyxml2::XMLElement* sensor_elem)
  : frame_id(link)
{
  const auto sensor(NestedXML{sensor_elem});
  const auto cam(NestedXML{sensor_elem->FirstChildElement("camera")});

  cam.read({"horizontal_fov"}, fov);
  cam.read({"image", "width"}, width);
  cam.read({"image", "height"}, height);
  cam.read({"clip", "near"}, clip_near);
  cam.read({"clip", "far"}, clip_far);

  int rate = 30;
  sensor.read({"update_rate"}, rate);
  period_ms = 1000/rate;

  // gz topic
  sensor.read({"topic"}, topic);
  if(topic[0] != '/')
    topic = "/" + topic;
}


// link parser
bool LinkInfo::canBeMerged(const LinkInfo &link)
{
  if(isRoot(link))
    return false;

  // you can only merge links that are fixed or carry nothing
  if(link.isFloating() && (!link.cameras.empty() || !link.visuals.empty()))
    return false;

  // you can only merge links with floating children
  return std::all_of(link.children.begin(), link.children.end(), [](auto child)
    {return child->isFloating();});
}

void LinkInfo::mergeIntoParent()
{
  parent->children.erase(std::find(parent->children.begin(),
                                   parent->children.end(),
                                   this));

  for(auto child: children)
  {
    if(!child->isFloating())
      child->pose.value() = child->pose.value() * pose.value();
    child->setParent(parent);
  }

  std::transform(visuals.begin(), visuals.end(), std::back_inserter(parent->visuals),
                 [&](const auto &visual) -> Visual
  {return {visual.first,visual.second*pose.value()};});

  std::transform(cameras.begin(), cameras.end(), std::back_inserter(parent->cameras),
                 [&](const auto &cam)
  {return CameraInfo(cam, pose.value());});
}


// a whole kinematic tree
inline auto Tree::find(const std::string &name)
{
  const auto elem{std::find(begin(), end(), name)};
  return elem == end() ? nullptr : &*elem;
}

auto Tree::simplify()
{
  while(true)
  {
    auto link{std::find_if(begin(), end(), LinkInfo::canBeMerged)};
    if(link == end())
      return;
    link->mergeIntoParent();
    erase(link);
  }
}

urdf::ModelInterfaceConstSharedPtr loadModel(const std::string &description)
{
  const auto urdf{description.find("<robot")};
  const auto sdf{description.find("<sdf>")};

  if(urdf < sdf)
    return urdf::parseURDF(description);

  auto model = std::make_shared<urdf::Model>();
  model->initString(description);
  return model;
}

// main function
Tree::Tree(const string &description, const bool keep_thrusters)
{
  const auto cameras(CameraInfo::extractFrom(description));
  const auto model{loadModel(description)};

  // extract all links
  for(const auto &[name, link]: model->links_)
  {
    if(!keep_thrusters && name.find("thruster") != name.npos)
      continue;

    auto &info{add(name)};
    std::transform(link->visual_array.begin(), link->visual_array.end(), std::back_inserter(info.visuals),
                   [](const auto &visual){return LinkInfo::Visual{visual, osg::Matrix::identity()};});
    std::copy_if(cameras.begin(), cameras.end(), std::back_inserter(info.cameras),
                 [&name=name](const auto &cam){return cam.frame_id == name;});
  }

  // build hierarchy from model joints
  for(const auto &[name,joint]: model->joints_)
  {
    auto link{find(joint->child_link_name)};
    if(link)
    {
      link->setParent(find(joint->parent_link_name));
      auto limits(joint->limits);
      if(joint->type == urdf::Joint::FIXED || (limits && limits->lower == limits->upper))
        link->pose = osgMatFrom(joint->parent_to_joint_origin_transform.position,
                                joint->parent_to_joint_origin_transform.rotation);
    }
  }

  // merge / remove merged links
  simplify();

  // reorder so that children are always behind their parents
  // induces that root link is at front
  const auto cmp = [](const LinkInfo &link1, const LinkInfo &link2)
  {
    if(link1.isAncestorOf(link2))
      return true;
    return !link2.isAncestorOf(link1);
  };
  sort(cmp);
}

}
}
