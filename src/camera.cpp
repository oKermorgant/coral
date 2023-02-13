#include <coral/camera.h>
#include <coral/coral_node.h>
#include <tinyxml.h>

using namespace std::chrono_literals;
using std::vector, std::string;

namespace coral
{


Camera::Camera(CoralNode* node, const urdf_parser::CameraInfo &info)
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
