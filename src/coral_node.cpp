#include <coral/coral_node.h>
#include <coral/urdf_parser.h>
#include <coral/scene_lock.h>
#include <coral/camera.h>
#include <coral/link.h>
#include <coral/marker.h>
#include <coral/OceanScene.h>
#include <coral/viewer.h>
#include <std_srvs/srv/empty.hpp>
#include <functional>

#ifdef CORAL_CUSTOM_SCENE
#include <coral/srv/scene_color.hpp>
#include <coral/custom_scene.h>
#endif

#ifdef CORAL_SYNC_WAVES
#include <ros_gz_interfaces/msg/param_vec.hpp>
#include <std_msgs/msg/float32.hpp>
#endif

using namespace coral;
using namespace std::chrono_literals;
using std::vector, std::string;
using namespace std::placeholders;

namespace
{
const std::string coral_cam_link = "coral_cam_view";
}

CoralNode::CoralNode() : rclcpp::Node("coral")
{
  display_thrusters = declare_parameter("with_thrusters", true);

  clock_sub = create_subscription<rosgraph_msgs::msg::Clock>("/clock", 1, [&]([[maybe_unused]] rosgraph_msgs::msg::Clock::SharedPtr msg)
                                                             {
                                                               // use_sim_time as soon as 1 message is received here
                                                               set_parameter(rclcpp::Parameter("use_sim_time", true));
                                                               clock_sub.reset();
                                                             });
}


void CoralNode::manage(Scene& scene, Viewer & viewer)
{
  this->scene = &scene;
  this->viewer = &viewer;
  //Marker::spawnThrough(this, scene.get(), &tf_buffer);

  // add localized world
  scene.getOceanScene()->addChild(world_link.frame());
  Camera::observe(shared_from_this(), this->scene);

  if(const auto delay{get_parameter("spawn_auto").as_int()}; delay > 0)
  {
    [[maybe_unused]] static auto future = std::async([=]()
                                                     {
                                                       std::this_thread::sleep_for(std::chrono::seconds(delay));
                                                       findModels();
                                                     });
  }

  static auto spawn_srv = create_service<Spawn>
      ("/coral/spawn",
       [&](const Spawn::Request::SharedPtr request, [[maybe_unused]] Spawn::Response::SharedPtr response)
       {
         if(request->robot_namespace.empty() && request->world_model.empty())
           findModels();
         else
           spawnModel(request->robot_namespace, request->pose_topic, request->world_model);
       });

  static auto pose_update_timer = create_wall_timer(50ms, [&](){refreshLinkPoses();});

#ifdef CORAL_CUSTOM_SCENE
  static auto color_sub = create_service<coral::srv::SceneColor>("/coral/scene_color", colorCallback(scene));
#endif
  /*
#ifdef CORAL_SYNC_WAVES  
  static auto wave_sub = create_subscription<ros_gz_interfaces::msg::ParamVec>("/coral/waves", 1,
                                                                               [&](ros_gz_interfaces::msg::ParamVec::SharedPtr msg)
  {scene->setWavesParams(msg->params);});
  static auto wind_speed_sub = create_subscription<std_msgs::msg::Float32>("/vrx/debug/wind/speed", 1,
                                                                           [&](std_msgs::msg::Float32::SharedPtr msg)
  {scene->setWindSpeed(msg->data);});
  static auto wind_dir_sub = create_subscription<std_msgs::msg::Float32>("/vrx/debug/wind/direction", 1,
                                                                         [&](std_msgs::msg::Float32::SharedPtr msg)
  {scene->setWindDirection(msg->data);});
#endif*/
}

SceneParams CoralNode::parameters()
{
  SceneParams params;

  // display
  declareParamDescription("gui.width", params.width, "Width of the window");
  declareParamDescription("gui.height", params.height, "Height of the window");
  declareParamDescription("gui.enable_stats", params.stats_keys,"Allow to show stats");
  declareParamDescription("gui.enable_stateset", params.stateset_keys,"Allow to show 3D states");
  auto cam(params.asVector(params.initialCameraPosition));
  declareParamDescription("gui.camera", cam,"Initial camera position");
  params.initialCameraPosition.set(cam[0], cam[1], cam[2]);

  // sky
  declareParamDescription("scene_type", params.scene_type,"Weather");
  this->declareParamDescription("sun.azimuth", params.azim, -180., 180.,
                                "Sun azimuth [deg]");
  this->declareParamDescription("sun.elevation", params.elev, 0., 90.,
                                "Sun elevation [deg]");

  // wind
  auto wind(params.asVector(params.windDirection));
  declareParamDescription("wind.direction", wind);
  params.windDirection.set(wind[0], wind[1]);
  declareParamDescription("wind.speed", params.windSpeed);
  declareParamDescription("wave.scale", params.waveScale);
  declareParamDescription("wave.choppy_factor", params.choppyFactor);
  declareParamDescription("wave.foam_height", params.crestFoamHeight);

  // underwater
  declareParamDescription("ocean.depth", params.depth);
  this->declareParamDescription("ocean.jerlov", params.jerlov, 0., 1.,
                                "Jerlov water type");
  this->declareParamDescription("ocean.fog_density", params.fogDensity, 0., 0.01f,
                                "Water fog density");


  // ocean surface params
  declareParamDescription("surface.reflection_damping", params.reflectionDamping);

  // vfx
  declareParamDescription("vfx.godrays", params.godrays);
  declareParamDescription("vfx.glare", params.glare);
  declareParamDescription("vfx.underwaterDof", params.underwaterDOF);

  if(!has_parameter("spawn_auto"))
    declare_parameter("spawn_auto", 2);

  if(!param_change)
    param_change = add_on_set_parameters_callback(std::bind(&CoralNode::parametersCallback,
                                                            this,
                                                            _1));

  return params;
}

SetParametersResult CoralNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
  auto result{SetParametersResult().set__successful(true)};
#ifdef ALLOW_CUSTOM_COLORS
  for(const auto &param: parameters)
  {
    if(param.get_name() == "ocean.jerlov")
    {
      scene->setJerlov(param.as_double());
    }
    else if(param.get_name() == "ocean.fog_density")
    {
      scene->setFogDensity(param.as_double());
    }
    else if(param.get_name() == "sun.azimuth")
    {
      scene->setSunAzimuth(param.as_double());
    }
    else if(param.get_name() == "sun.elevation")
    {
      scene->setSunElevation(param.as_double());
    }
    else
    {
      result.successful = false;
    }
  }
#endif
  return result;
}

bool CoralNode::updateViewerPose()
{
  if(!tf_buffer.frameExists(coral_cam_link))
    return false;

  // get the frame we are observing
  Link* cam_parent = nullptr;
  auto parent{tf_buffer.getParent(coral_cam_link)};

  while(parent.has_value())
  {
    // if we have reached the world frame
    if(parent.value() == world_link.name())
    {
      cam_parent = &world_link;
      break;
    }
    // if we have reached a link that moves without TF knowing
    if(auto root{findLink(parent.value())}; root)
    {
      cam_parent = root;
      break;
    }
    // continue parenting
    parent = tf_buffer.getParent(parent.value());
  }

  if(cam_parent == nullptr)
    return false;

  const auto tr{tf_buffer.lookupTransform(cam_parent->name(), coral_cam_link, 10ms)};

  if((now() - tr.header.stamp).seconds() > 1)
    return false;

  // add 180 deg yaw for intuitivity
  static const osg::Matrix M180(-osg::Quat{0,0,1,0});

  auto M = osgMatFrom(tr.transform.translation, tr.transform.rotation) * M180;

  if(cam_parent->name() != WORLD_NAME)
    M = M*cam_parent->frame()->getMatrix();
  viewer->lockCamera(M);
  return true;
}

void CoralNode::refreshLinkPoses()
{
  if(tf_buffer.ready())
  {
    const auto size{[&](){const auto lock{scene_lock()};return links.size();}()};
    // cache retrieval of pending new poses
    for(size_t link = 0; link < size; ++link)
      links[link]->refreshFrom(tf_buffer);
  }

  {
    // locked while forwarding poses to scene
    [[maybe_unused]] const auto lock{scene_lock()};
    for(auto &link: links)
      link->applyNewPose();
  }

  if(!updateViewerPose())
    viewer->freeCamera();
}

void CoralNode::findModels()
{
  const auto topics{get_topic_names_and_types()};
  if(topics.empty())
    return;
  const std::string description{"robot_description"};
  const auto isDescription{[description](const std::string &topic)
                           {
                             return topic.size() >= description.size()
                                    && 0 == topic.compare(topic.size()-description.size(), description.size(), description);
                           }};

  // find all robot_description's
  for(const auto &[topic, msg]: topics)
  {
    if(!isDescription(topic))
      continue;

	const auto ns{topic.substr(0, topic.size() - description.size()-1)};
	const auto isSameNS{[&ns](const std::string &topic)
						{
						  return topic.size() >= ns.size() && 0 == topic.compare(0, ns.size(), ns);
						}};

	// pose_topic should be a geometry_msgs/Pose, published by Gazebo as ground truth
	const auto pose_topic{std::find_if(topics.begin(), topics.end(), [&](const auto &elem)
									   {
										 return isSameNS(elem.first) && elem.second[0] == "geometry_msgs/msg/Pose";
									   })};
	if(pose_topic == topics.end())
	  spawnModel(ns);
	else
	  spawnModel(ns, pose_topic->first.substr(ns.size()+1));
  }
}

void CoralNode::spawnModel(const std::string &model_ns,
						   const std::string &pose_topic,
						   const std::string &world_model)
{
  if(!world_model.empty())
  {
    std::ifstream urdf{world_model};
    if(!urdf)
    {
      RCLCPP_WARN(get_logger(), "cannot open file %s", world_model.c_str());
      return;
    }
    ScopedTimer("Loading world model from " + world_model, this);
    using Buffer = std::istreambuf_iterator<char>;
    addModel({(Buffer(urdf)), Buffer()});
    return;
  }

  // model from robot_description
  if(model_ns.empty() || hasModel(model_ns))
    return;

  // retrieve full model through robot_description param
  const auto rsp_node{std::make_shared<Node>("coral_rsp_client", model_ns)};
  const auto rsp_param_srv{std::make_shared<rclcpp::SyncParametersClient>
                           (rsp_node, model_ns + "/robot_state_publisher")};
  rsp_param_srv->wait_for_service();
  if(!rsp_param_srv->has_parameter("robot_description"))
  {
    // cannot get the model anyway
    RCLCPP_WARN(get_logger(), "cannot get model %s", model_ns.substr(1).c_str());
    return;
  }

  ScopedTimer("Loading model from " + model_ns + "/robot_state_publisher", this);
  const auto root{addModel(rsp_param_srv->get_parameter<string>("robot_description"))};

  if(!pose_topic.empty() && root)
  {
    RCLCPP_INFO(get_logger(), "%s seems to have its pose published on %s/%s for frame %s",
                model_ns.substr(1).c_str(),
                model_ns.c_str(),
                pose_topic.c_str(),
                root->name().c_str());

    pose_subs.push_back(create_subscription<Pose>(model_ns + "/" + pose_topic, 1, root->poseCallback()));
  }
  known_model_namespaces.push_back(model_ns);
}

Link* CoralNode::addModel(const string &description)
{
  const auto tree{urdf_parser::Tree(description, display_thrusters)};
  Link* root{};

  // lock during add to scene
  const auto lock{scene_lock()};

  for(const auto &link_info: tree)
  {
    if(link_info.name == WORLD_NAME)
    {
      world_link.addElements(link_info);
      Camera::addCameras(world_link.frame(), link_info.cameras, viewer);
    }
    else
    {
      auto &last_link{links.emplace_back(std::make_unique<Link>(link_info))};
      if(!root)
        root = last_link.get();
      // find the parent if any, was already added
      if(!link_info.parent || link_info.parent->name == WORLD_NAME)
      {
        RCLCPP_INFO(get_logger(), "  Got frame %s", link_info.name.c_str());
        last_link->setParent(world_link);
      }
      else
      {
        last_link->setParent(*findLink(link_info.parent->name));
      }
      Camera::addCameras(last_link->frame(), link_info.cameras, viewer);
    }
  }
  return root;
}
