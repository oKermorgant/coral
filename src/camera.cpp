#include <coral/camera.h>
#include <coral/osg_make_ref.h>

using namespace std::chrono_literals;
using std::vector, std::string;

namespace coral
{
std::unique_ptr<image_transport::ImageTransport> Camera::transport;
rclcpp::Node::SharedPtr Camera::node;
std::vector<Camera> Camera::cameras;

void Camera::addCameras(osg::Group* link, const std::vector<urdf_parser::CameraInfo> &infos)
{
  if(!transport)
  {
    node = std::make_unique<rclcpp::Node>("coral_camera_transport");
    transport = std::make_unique<image_transport::ImageTransport>(node);
  }

  const auto current_topics{node->get_topic_names_and_types()};

  cameras.reserve(cameras.size() + infos.size());
  for(auto &cam: infos)
  {

    if(current_topics.find(cam.topic) != current_topics.end())
      RCLCPP_WARN(node->get_logger(), "Image topic %s seems already advertized", cam.topic.c_str());

    cam.pose->setDataVariance(osg::Object::STATIC);
    link->addChild(cam.pose);
    cameras.push_back(Camera(cam));
  }
}

Camera::Camera(const urdf_parser::CameraInfo &info)
{




  // init image
  image = osg::make_ref<osg::Image>();
  image->allocateImage(info.width, info.height, 1, GL_RGB, GL_UNSIGNED_BYTE);

  // init cam projection
  cam = osg::make_ref<osg::Camera>();
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
  msg.header.frame_id = info.frame_id;
  msg.width = info.width;
  msg.height = info.height;
  msg.encoding = "rgb8";
  const auto size(image->getTotalSizeInBytes());
  msg.step = size/info.height;
  msg.data.resize(size);
  publisher = transport->advertise(info.topic, 1);
  publish_timer = node->create_wall_timer(std::chrono::milliseconds(info.period_ms),
                                          [&](){publish();});
}

void Camera::publish()
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
