#include <coral/camera.h>
#include <coral/osg_make_ref.h>
#include <osg/Switch>
#include <osgOcean/ShaderManager>
#include <coral/masks.h>

using namespace std::chrono_literals;
using std::vector, std::string;

namespace coral
{

// Callback functors

//Custom node tracking
class CamTrackerCallback : public osg::NodeTrackerCallback
{
  osg::Camera* cam;
  uint frame{0};

  void operator()(osg::Node *node, osg::NodeVisitor *nv)
  {
    const auto stamp{nv->getFrameStamp()->getFrameNumber()};
    if(stamp == frame)
      return;
    frame = stamp;

    auto M{osg::computeWorldToLocal(nv->getNodePath())};
    traverse(node, nv);		

	// Gazebo / CV and OSG differ from pi rotation around X-axis for cameras
	M.preMult(osg::Matrix(osg::Quat(1., 0., 0., 0.)));

	cam->setViewMatrix(M);

	const auto t{M.getTrans()};

	//std::cout << "Cam pose: " << t.x() << "," << t.y() << "," << t.z() << std::endl;

  }
public:
  CamTrackerCallback(osg::Camera *cam) : cam{cam} {}  
};

//Updates camera buffer when publisher is not publishing
class CameraBufferCallback : public osg::Camera::DrawCallback
{
public:
  virtual void operator () (const osg::Camera&) const
  {
    cam->saveBuffer();
  }
  CameraBufferCallback(Camera *camera) : cam{camera} {}
private:
  Camera *cam;
};

class UpdateUnderWater : public osg::Uniform::Callback
{
public:
  UpdateUnderWater(osg::Camera* camera) : cam(camera) {}
  virtual void operator ()(osg::Uniform* u, osg::NodeVisitor*)
  {
    u->set(true); //TODO: Should check waterheight!!
  }

protected:
  osg::Camera* cam;
};

class UpdateEye : public osg::Uniform::Callback
{
public:
  UpdateEye(osg::Camera* camera) : cam(camera) {}
  virtual void operator ()(osg::Uniform* u, osg::NodeVisitor*)
  {
    osg::Vec3d eye, center, up;
    cam->getViewMatrixAsLookAt(eye, center, up);
    u->set(eye);
  }

protected:
  osg::Camera* cam;
};

class UpdateVMI : public osg::Uniform::Callback
{
public:
  UpdateVMI(osg::Camera* camera) : cam(camera) {}
  virtual void operator ()(osg::Uniform* u, osg::NodeVisitor*)
  {
    u->set(cam->getInverseViewMatrix());
  }

protected:
  osg::Camera* cam;
};



void Camera::addCameras(osg::Group* link, const std::vector<urdf_parser::CameraInfo> &infos)
{
  if(link == nullptr || infos.empty())
    return;

  if(!transport)
  {
    node = std::make_unique<rclcpp::Node>("camera_transport", "coral");
    //node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    transport = std::make_unique<image_transport::ImageTransport>(node);

	cam_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	cam_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
	cam_executor->add_node(node);

    static auto cam_thread{std::thread([&](){cam_executor->spin();})};
  }

  cameras.reserve(cameras.size() + infos.size());

  for(auto &cam: infos)
  {
    if(!node->get_publishers_info_by_topic(cam.topic).empty())
    {
      RCLCPP_WARN(node->get_logger(), "Image topic %s seems already advertized", cam.topic.c_str());
    }
    //cam.pose->setDataVariance(osg::Object::DYNAMIC);
    link->addChild(cam.pose);
    cameras.emplace_back(new Camera(cam));
  }
}

void Camera::loadShaders()
{
  if(!scene)
    return;

  auto program = osgOcean::ShaderManager::instance().createProgram("object_shader",
                                                                   Shader::ocean_scene_vert_file,
                                                                   Shader::ocean_scene_frag_file, "", "");

  cam->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);

  const auto &[params, water, weather] = scene->getFullParams(); {}

  cam->getStateSet()->addUniform(new osg::Uniform("osgOcean_EnableGlare", params.glare));
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableUnderwaterScattering", params.underwaterScattering) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableDOF", params.underwaterDOF) );

  float UWFogDensity= water.fogDensity;
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterFogDensity", -UWFogDensity*UWFogDensity*1.442695f) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterFogColor", water.fogColor ) );

  float AWFogDensity= weather.fogDensity;
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_AboveWaterFogDensity", -AWFogDensity*AWFogDensity*1.442695f ) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_AboveWaterFogColor", weather.fogColor ) );

  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Near",  scene->getDOFNear() ) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Far",  scene->getDOFFar() ) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Focus",  scene->getDOFFocalDistance() ) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Clamp",  scene->getDOFFarClamp() ) );

  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_WaterHeight", float(scene->getOceanSurfaceHeight()) ) );

  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterAttenuation", water.attenuation ) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterDiffuse", water.diffuse ) );

  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableHeightmap", false));
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableReflections", false));
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableRefractions", false));
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableCrestFoam", false));
  /*cam->getStateSet()->addUniform(new osg::Uniform("uOverlayMap", 1));
            cam->getStateSet()->addUniform(new osg::Uniform("uNormalMap", 2));
            cam->getStateSet()->addUniform(new osg::Uniform("SLStex", 3));
            cam->getStateSet()->addUniform(new osg::Uniform("SLStex2", 4));*/

  cam->getStateSet()->addUniform(new osg::Uniform("offsets", osg::Vec4f(1,2,3,4)));
  cam->getStateSet()->addUniform( new osg::Uniform("stddev", 0.0f ) );
  cam->getStateSet()->addUniform( new osg::Uniform("mean", 0.0f ) );

  /*osg::Fog *fog = new osg::Fog();
  fog->setMode(osg::Fog::EXP2);
  fog->setFogCoordinateSource(osg::Fog::FRAGMENT_DEPTH);
  fog->setDensity(water.fogDensity);
  fog->setColor(water.fogColor);
  cam->getStateSet()->setAttributeAndModes(fog, osg::StateAttribute::ON);*/
}

Camera::Camera(const urdf_parser::CameraInfo &info)
{
  RCLCPP_INFO(node->get_logger(), "Rendering images for camera @ %s", info.frame_id.c_str());

  // transform
  pose = info.pose;

  // init image
  buffer = osg::make_ref<osg::Image>();
  buffer->allocateImage(info.width, info.height, 1, GL_RGB, GL_UNSIGNED_BYTE);

  // init cam projection
  cam = osg::make_ref<osg::Camera>();
  cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
  cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  cam->setViewport(0, 0, info.width, info.height);
  cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER);
  cam->setRenderOrder(osg::Camera::PRE_RENDER);
  cam->attach(osg::Camera::COLOR_BUFFER, buffer.get());
  cam->setName(info.topic);
  cam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
  cam->setProjectionMatrixAsPerspective(info.fov,
                                        static_cast<double>(info.width)/info.height,
                                        info.clip_near,
                                        info.clip_far);

  // callbacks
  pose->setEventCallback(new CamTrackerCallback(cam));

  // Uniforms for independence from main camera (underwater effects on shaders)
  osg::Uniform* u = new osg::Uniform("osgOcean_EyeUnderwater", true);
  u->setUpdateCallback(new UpdateUnderWater(cam));
  cam->getOrCreateStateSet()->addUniform(u);

  osg::Vec3d eye, center, up;
  cam->getViewMatrixAsLookAt(eye, center, up);

  osg::Uniform* u2 = new osg::Uniform("osgOcean_Eye", eye);
  u2->setUpdateCallback(new UpdateEye(cam));
  cam->getStateSet()->addUniform(u2);

  osg::Uniform* u3 = new osg::Uniform(osg::Uniform::FLOAT_MAT4,"osg_ViewMatrixInverse");
  u3->setUpdateCallback(new UpdateVMI(cam));
  cam->getStateSet()->addUniform(u3);

  //cam->setCullMask(~Mask::marker);
  loadShaders();

  cam->setPostDrawCallback(new CameraBufferCallback(this));

  //scene->connect(cam.get());

  // ROS part
  msg.header.frame_id = info.frame_id;
  msg.width = info.width;
  msg.height = info.height;
  msg.encoding = "rgb8";
  msg.is_bigendian = 0;
  const auto size(buffer->getTotalSizeInBytes());
  msg.step = size/info.height;
  msg.data.resize(size);
  publisher = transport->advertise(info.topic, 1);

  publish_timer = node->create_wall_timer(std::chrono::milliseconds(info.period_ms),
                                          std::bind(&Camera::publish, this),
                                          cam_cb_group);
}

void Camera::publish()
{
  if(!image || image->getTotalSizeInBytes() == 0)
  {
    RCLCPP_WARN(node->get_logger(), "Image not generated for %s", msg.header.frame_id.c_str());
    return;
  }

  {
    const auto lock{std::lock_guard(mtx)};
    const auto data(static_cast<unsigned char*>(image->data()));
    if(data == nullptr)
    {
      RCLCPP_WARN(node->get_logger(), "Image not generated for %s", msg.header.frame_id.c_str());
      return;
    }

    msg.header.stamp = node->now();

	for (uint row = 0; row < msg.height; row++)
	{
	  const auto srcRow = data + row * msg.step;
	  const auto dstRow = msg.data.data() + (msg.height - row - 1) * msg.step;
	  for (uint chan = 0; chan < msg.step; chan++)
	  {
		dstRow[chan] = srcRow[chan];
	  }
	}
  }
  publisher.publish(msg);
}
}
