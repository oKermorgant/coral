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
  OceanScene* ocean_scene{nullptr};
  uint frame{0};
  osg::Uniform* u_eyeUnderwater;

  void operator()(osg::Node *node, osg::NodeVisitor *nv)
  {
    const auto stamp{nv->getFrameStamp()->getFrameNumber()};
    if(stamp == frame)
      return;
    frame = stamp;

	auto M{osg::computeWorldToLocal(nv->getNodePath())};
	traverse(node, nv);

	// Gazebo / CV and OSG differ from pi rotation around X-axis for cameras
	M.postMult(osg::Matrix(osg::Quat(1., 0., 0., 0.)));

	cam->setViewMatrix(M);
	M = cam->getInverseViewMatrix();

	// Update surface uniform
	//u_eyeUnderwater->set(!ocean_scene->isEyeAboveWater(M.getTrans()));


	const auto t{M.getTrans()};
	const auto q{M.getRotate()};
	std::cout << "Cam tran: " << t.x() << "," << t.y() << "," << t.z() << std::endl;
	std::cout << "    quat: " << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;

  }
public:
  CamTrackerCallback(osg::Camera *cam,
                     OceanScene* ocean_scene,
                     osg::Uniform* u_surface)
      : cam{cam}, ocean_scene{ocean_scene}, u_eyeUnderwater{u_surface}
  {}
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
  UpdateUnderWater(osg::Camera* camera, OceanScene* scene ) : cam(camera), scene{scene} {}
  virtual void operator ()(osg::Uniform* u, osg::NodeVisitor*)
  {

    std::cout << " cam Z " << cam->getViewMatrix().getTrans().z() << std::endl;

//	scene->isEyeAboveWater(cam->getViewMatrix().getTrans().z())

    //u->set(cam->getViewMatrix().getTrans().z() < scene->getOceanSurfaceHeight()); //TODO: Should check waterheight!!
    u->set(true);
  }

protected:
  osg::Camera* cam;
  OceanScene* scene{nullptr};
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

void Camera::addCameras(osg::Group* link, const std::vector<urdf_parser::CameraInfo> &infos, Viewer* viewer)
{
  if(link == nullptr || infos.empty())
    return;

  if(scene == nullptr || !node || !transport)
  {
    RCLCPP_ERROR(node->get_logger(), "Virtual Cameras: Scene not initialized");
    return;
  }

  cameras.reserve(cameras.size() + infos.size());

  for(auto &cam: infos)
  {
    if(!node->get_publishers_info_by_topic(cam.topic).empty())
    {
      RCLCPP_WARN(node->get_logger(), "Image topic %s seems already advertized", cam.topic.c_str());
    }
    cam.pose->setDataVariance(osg::Object::DYNAMIC);
    link->addChild(cam.pose);
    auto &new_cam{cameras.emplace_back(new Camera(cam))};
    viewer->addCamWidget(new_cam->getWidgetWindow(), cam.width);
  }
}

void Camera::loadShaders()
{
  //if(!scene)
  return;

  auto ocean_scene{scene->getOceanScene()};
  const auto &params{ocean_scene->params};
  const auto &water{ocean_scene->water};
  const auto &weather{ocean_scene->weather};

  cam->getStateSet()->addUniform(new osg::Uniform("osgOcean_EnableGlare", params.glare));
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableUnderwaterScattering", params.underwaterScattering) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableDOF", params.underwaterDOF));

  const auto UWFogDensity{water.fogDensity};
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterFogDensity", -UWFogDensity*UWFogDensity*1.442695f) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterFogColor", water.fogColor ) );

  const auto AWFogDensity{weather.fogDensity};
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_AboveWaterFogDensity", -AWFogDensity*AWFogDensity*1.442695f ) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_AboveWaterFogColor", weather.fogColor ) );

  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Near",  ocean_scene->getDOFNear() ) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Far",  ocean_scene->getDOFFar() ) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Focus",  ocean_scene->getDOFFocalDistance() ) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Clamp",  ocean_scene->getDOFFarClamp() ) );

  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_WaterHeight", float(ocean_scene->getOceanSurfaceHeight()) ) );

  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterAttenuation", water.attenuation ) );
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterDiffuse", water.diffuse ) );

  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableHeightmap", params.heightmap));
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableReflections", params.reflections));
  cam->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableRefractions", params.refractions));
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
  RCLCPP_INFO(node->get_logger(), "Rendering images for camera '%s' @ %i ms ", info.frame_id.c_str(), info.rate);

  last_save = node->now();

  // image
  image = osg::make_ref<osg::Image>();
  image->allocateImage(info.width, info.height, 1, GL_RGB, GL_UNSIGNED_BYTE);

  // camera model
  cam = osg::make_ref<osg::Camera>();
  cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
  cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  cam->setViewport(0, 0, info.width, info.height);
  cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
  cam->setRenderOrder(osg::Camera::PRE_RENDER);
  cam->attach(osg::Camera::COLOR_BUFFER, image.get());
  cam->setName(info.frame_id);
  cam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
  cam->setProjectionMatrixAsPerspective(info.fov,
                                        static_cast<double>(info.width)/info.height,
                                        info.clip_near,
                                        info.clip_far);

  // camera transform
  pose = info.pose;  

  // Uniforms for independence from main camera (underwater effects on shaders)
  osg::Uniform* u = new osg::Uniform("osgOcean_EyeUnderwater", true);
  //u->setUpdateCallback(new UpdateUnderWater(cam, scene->getOceanScene()));
  cam->getOrCreateStateSet()->addUniform(u);

  osg::Vec3d eye, center, up;
  cam->getViewMatrixAsLookAt(eye, center, up);
  osg::Uniform* u2 = new osg::Uniform("osgOcean_Eye", eye);
  u2->setUpdateCallback(new UpdateEye(cam));
  cam->getStateSet()->addUniform(u2);

  osg::Uniform* u3 = new osg::Uniform(osg::Uniform::FLOAT_MAT4,"osg_ViewMatrixInverse");
  u3->setUpdateCallback(new UpdateVMI(cam));
  cam->getStateSet()->addUniform(u3);

  pose->setEventCallback(new CamTrackerCallback(cam, scene->getOceanScene(), u));

  cam->setCullMask(~Mask::marker);

  // main shader
  cam->getOrCreateStateSet()->setAttributeAndModes(Shader::create("cam_shader"), osg::StateAttribute::ON);

  loadShaders();

  cam->setPostDrawCallback(new CameraBufferCallback(this));

  cam->addChild(scene->getOceanScene());
  scene->getRoot()->addChild(cam.get());

  // ROS part
  msg.header.frame_id = info.frame_id;
  msg.width = info.width;
  msg.height = info.height;
  msg.encoding = "rgb8";
  msg.is_bigendian = 0;
  msg.step = 3*info.width;
  msg.data.resize(3*info.width*info.height);
  publisher = transport->advertise(info.topic, 1);
  period_s = 1./info.rate;
  cam_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  publish_timer = node->create_wall_timer(std::chrono::milliseconds(int(1000./info.rate)),
                                          [&](){publish();},
                                          cam_cb_group);
}

void Camera::saveBuffer()
{
  const auto now{node->now()};
  const auto delay{now - last_save};

  if(delay.seconds() < period_s)
    return;

  const auto lock{std::lock_guard(mtx)};
  msg.header.stamp = last_save = now;

  auto srcRow{image->data()};
  auto dstRow{msg.data.data() + (msg.height-1)*msg.step};

  for (uint row = 0; row < msg.height; row++)
  {
    std::memcpy(dstRow, srcRow, msg.step);
    srcRow += msg.step;
    dstRow -= msg.step;
  }
}

void Camera::publish()
{
  const auto lock{std::lock_guard(mtx)};
  publisher.publish(msg);
}
}
