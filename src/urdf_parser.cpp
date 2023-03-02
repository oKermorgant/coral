#include <coral/urdf_parser.h>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>

#include <coral/debug_msg.h>

using std::vector, std::string;

namespace coral
{

osg::Matrixd osgMatFrom(const std::vector<double> &xyz, const std::vector<double> &rpy, const std::vector<double> &scale)
{
  const osg::Vec3d X(1,0,0);
  const osg::Vec3d Y(0,1,0);
  const osg::Vec3d Z(0,0,1);
  osg::Matrixd M(-osg::Quat(rpy[0], X, rpy[1], Y, rpy[2], Z));
  M.setTrans(osgVecFrom(xyz));
  M.preMultScale(osgVecFrom(scale));
  return M;
}

osg::Matrixd osgMatFrom(const urdf::Vector3 &t, const urdf::Rotation &q, const urdf::Vector3 &scale)
{
  osg::Matrixd M(-osg::Quat{q.x, q.y, q.z, q.w});
  M.setTrans(osgVecFrom(t));
  M.preMultScale(osgVecFrom(scale));
  return M;
}

namespace urdf_parser
{

struct NestedXML
{
  const TiXmlElement* root;
  explicit NestedXML(const TiXmlElement* root) : root(root) {}

  static const TiXmlElement* getNested(const TiXmlElement* root, const vector<string> &keys)
  {
    if(keys.size() == 0)  return root;

    const auto child = root->FirstChildElement(keys[0]);
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
  TiXmlDocument doc;
  doc.Parse(description.c_str());
  auto root = doc.RootElement();

  for(auto gazebo_elem = root->FirstChildElement("gazebo");
      gazebo_elem != nullptr;
      gazebo_elem = gazebo_elem->NextSiblingElement("gazebo"))
  {
    const auto sensor(gazebo_elem->FirstChildElement("sensor"));

    if(sensor != nullptr &&
       std::string(sensor->Attribute("type")) == "camera" &&
       sensor->FirstChildElement("plugin") != nullptr)
    {
      const string link(gazebo_elem->Attribute("reference"));

      if(link.find("right") == link.npos && link.find("left") == link.npos)
      {
        cameras.emplace_back(link, sensor);
      }
    }
  }
  return cameras;
}

CameraInfo::CameraInfo(std::string link, const TiXmlElement* sensor_elem)
  : link_name(link)
{

  std::cout << "Adding camera @ " << link << std::endl;

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

  // topic namespace
  sensor.read({"plugin", "camera_name"}, topic);
  if(topic[0] != '/')
    topic = "/" + topic;

  // image topic in namespace
  string im_topic = "image_raw";
  /*const bool remapped = sensor.read({"plugin", "ros", "remapping"}, im_topic);
  if(remapped)
  {
    const auto idx(im_topic.find('='));
    if(idx != im_topic.npos)
      im_topic = im_topic.substr(idx+1);
  }*/
  topic += '/' + im_topic;
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
  if(std::any_of(link.children.begin(), link.children.end(),
                 [](LinkInfo* child)
  {return !child->isFloating();}))
    return false;

  return true;
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
      break;
    link->mergeIntoParent();
    erase(link);
  }
}

// main function
Tree::Tree(const string &description, const bool keep_thrusters)
{
  const auto cameras(CameraInfo::extractFrom(description));
  const auto model(urdf::parseURDF(description));

  // extract all links
  for(const auto &elem: model->links_)
  {
    const auto &name{elem.first};
    const auto &link{elem.second};
    if(!keep_thrusters && name.find("thruster") != name.npos)
      continue;

    auto &info{add(name)};
    std::transform(link->visual_array.begin(), link->visual_array.end(), std::back_inserter(info.visuals),
                   [](const auto &visual){return LinkInfo::Visual{visual, osg::Matrixd::identity()};});
    std::copy_if(cameras.begin(), cameras.end(), std::back_inserter(info.cameras),
                 [&](const auto &cam){return cam.link_name == name;});
  }

  // build hierarchy from model joints
  for(const auto &[name,joint]: model->joints_)
  {
    auto link{find(joint->child_link_name)};
    link->setParent(find(joint->parent_link_name));
    auto limits(joint->limits);
    if(joint->type == urdf::Joint::FIXED || (limits && limits->lower == limits->upper))
      link->pose = osgMatFrom(joint->parent_to_joint_origin_transform.position,
                              joint->parent_to_joint_origin_transform.rotation);
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
