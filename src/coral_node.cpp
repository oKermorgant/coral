#include <coral/coral_node.h>
#include <coral/urdf_parser.h>
#include <coral/scene_lock.h>
#include <coral/camera.h>
#include <coral/link.h>
#include <coral/marker.h>
#include <coral/OceanScene.h>
#include <coral/viewer.h>
#include <std_srvs/srv/empty.hpp>

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

void CoralNode::manage(osg::ref_ptr<OceanScene> scene, Viewer & viewer)
{
  this->scene = scene.get();
  this->viewer = &viewer;
  Marker::spawnThrough(this, scene.get(), &tf_buffer);

  scene->addChild(world_link.frame());
  Camera::observe(scene.get());

  if(const auto delay = declare_parameter("spawn_auto", 2); delay > 0)
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
    [[maybe_unused]] const auto lock{coral_lock()};
    if(request->robot_namespace.empty() && request->world_model.empty())
      findModels();
    else
      spawnModel(request->robot_namespace, request->pose_topic, request->world_model);
  });

  static auto pose_update_timer = create_wall_timer(50ms, [&](){refreshLinkPoses();});

#ifdef CORAL_CUSTOM_SCENE
  static auto color_sub = create_service<coral::srv::SceneColor>("/coral/scene_color", colorCallback(scene));
#endif

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
#endif
}

SceneParams CoralNode::parameters()
{
  SceneParams params;

  const auto updateParam = [&](const string &description, auto & val)
  {
    if(has_parameter(description))
      get_parameter(description, val);
    else
      val = declare_parameter(description, val);
  };

  // display
  updateParam("gui.width", params.width);
  updateParam("gui.height", params.height);
  auto cam(params.asVector(params.initialCameraPosition));
  updateParam("gui.camera", cam);
  params.initialCameraPosition.set(cam[0], cam[1], cam[2]);

  // weather
  updateParam("scene_type", params.scene_type);
  auto wind(params.asVector(params.windDirection));
  updateParam("wind.direction", wind);
  params.windDirection.set(wind[0], wind[1]);
  updateParam("wind.speed", params.windSpeed);
  updateParam("wave.scale", params.waveScale);
  updateParam("wave.choppy_factor", params.choppyFactor);
  updateParam("wave.foam_height", params.crestFoamHeight);

  // underwater
  updateParam("ocean.depth", params.depth);
  updateParam("ocean.attenuation", params.depth_attn);

  // ocean surface params
  updateParam("surface.reflection_damping", params.reflectionDamping);

  // vfx
  updateParam("vfx.godrays", params.godrays);
  updateParam("vfx.glare", params.glare);
  updateParam("vfx.underwaterDof", params.underwaterDOF);

  return params;
}

Link* CoralNode::getKnownCamParent()
{
  Link* prev_link = nullptr;
  auto parent{tf_buffer.getParent(coral_cam_link)};

  while(parent.has_value())
  {
    // if we have reached the world frame
    if(parent.value() == world_link.name())
    {
      prev_link = &world_link;
      break;
    }
    // if we have reached a link that moves without TF knowing
    if(auto root{findLink(parent.value())}; root)
    {
      prev_link = root;
      break;
    }
    // continue parenting
    parent = tf_buffer.getParent(parent.value());
  }
  return prev_link;
}

void CoralNode::refreshLinkPoses()
{
  if(tf_buffer.ready())
  {
    // cache retrieval of pending new poses
    for(auto &link: links)
      link->refreshFrom(tf_buffer);
  }

  {
    // locked while forwarding poses to scene
    [[maybe_unused]] const auto lock{coral_lock()};
    for(auto &link: links)
      link->applyNewPose();
  }

  if(tf_buffer.frameExists(coral_cam_link))
  {
    const auto parent{getKnownCamParent()};

    if(parent == nullptr)
      return;

    const auto tr{tf_buffer.lookupTransform(parent->name(), coral_cam_link, 10ms)};
    const auto delay{(now() - tr.header.stamp).seconds()};
    if(delay < 1 || delay > 1e8)
    {
      auto M = osgMatFrom(tr.transform.translation, tr.transform.rotation);

      if(parent->name() != WORLD_NAME)
        M = M*parent->frame()->getMatrix();
      viewer->lockCamera(M);
    }
    else
    {
      viewer->freeCamera();
    }
  }
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
    parseModel({(Buffer(urdf)), Buffer()});
    return;
  }

  // model from robot_description
  if(model_ns.empty() || hasModel(model_ns))
    return;
  // retrieve full model through robot_state_publisher
  const auto rsp_node(std::make_shared<Node>("coral_rsp"));
  const auto rsp_param_srv = std::make_shared<rclcpp::SyncParametersClient>
      (rsp_node, model_ns + "/robot_state_publisher");
  rsp_param_srv->wait_for_service();
  if(!rsp_param_srv->has_parameter("robot_description"))
  {
    // cannot get the model anyway
    RCLCPP_WARN(get_logger(), "cannot get model %s", model_ns.substr(1).c_str());
    return;
  }

  ScopedTimer("Loading model from " + model_ns + "/robot_state_publisher", this);
  const auto root{parseModel(rsp_param_srv->get_parameter<string>("robot_description"))};

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

Link* CoralNode::parseModel(const string &description)
{
  const auto tree{urdf_parser::Tree(description, display_thrusters)};
  Link* root{};

  for(const auto &link: tree)
  {
    if(link.name == WORLD_NAME)
    {
      world_link.addElements(link);
      Camera::addCameras(world_link.frame(), link.cameras);      
    }
    else
    {
      auto &last{links.emplace_back(std::make_unique<Link>(link))};
      if(!root)
        root = last.get();
      // find the parent if any, was already added
      if(!link.parent || link.parent->name == WORLD_NAME)
      {
        RCLCPP_INFO(get_logger(), "  Got frame %s", link.name.c_str());
        last->setParent(world_link);
      }
      else
      {
        last->setParent(*findLink(link.parent->name));
      }
      Camera::addCameras(last->frame(), link.cameras);
    }
  }
  return root;
}
