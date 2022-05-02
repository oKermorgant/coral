#include <coral/camera.h>
#include <coral/coral_node.h>
#include <tinyxml.h>

using namespace std::chrono_literals;
using std::vector, std::string;

namespace coral
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


Camera::Camera(CoralNode* node, const CameraInfo &info)
{
  // init image
  image = new osg::Image;
  image->allocateImage(info.width, info.height, 1, GL_RGB, GL_UNSIGNED_BYTE);

  // init cam projection
  cam = new osg::Camera;
  cam->setReferenceFrame(osg::Transform::RELATIVE_RF);
  cam->setClearMask(GL_COLOR_BUFFER_BIT);
  cam->setViewport(0, 0, info.width, info.height);
  cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
  cam->attach(osg::Camera::COLOR_BUFFER, image.get());
  cam->setName(info.topic);
  cam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
  cam->setProjectionMatrixAsPerspective(info.fov,
                                        static_cast<double>(info.width)/info.height,
                                        info.clip_near,
                                        info.clip_far);

  // transform
  pose = info.pose; // TODO check if Gazebo and OSG use same convention for cameras
  pose->addChild(cam);


  // ROS part
  msg.header.frame_id = info.link_name;
  msg.width = info.width;
  msg.height = info.height;
  msg.encoding = "rgb8";
  const auto size(image->getTotalSizeInBytes());
  msg.step = size/info.height;
  msg.data.resize(size);
  publisher = node->image_transport().advertise(info.topic, 1);
  publish_timer = node->create_wall_timer(std::chrono::milliseconds(info.period_ms),
                                          [&](){publish(node);});
}

void Camera::publish([[maybe_unused]] CoralNode *node)
{
  //msg.header.stamp = node->now();
  const auto &height(msg.height);
  const auto data(static_cast<unsigned char*>(image->data()));

  for(uint row = 0; row < height; ++row)
  {
    const auto src_row = data + row + msg.step*3;
    const auto dst_row = msg.data.data() + (height-row-1) * msg.step;

    for(uint col = 0; col < msg.step; ++col)
      dst_row[col] = src_row[col];
  }

  publisher.publish(msg);
}


}
