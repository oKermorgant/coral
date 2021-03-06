#include <coral/coral_node.h>
#include <coral/urdf_parser.h>
#include <std_srvs/srv/empty.hpp>

using namespace coral;
using namespace std::chrono_literals;
using std::vector, std::string;

CoralNode::CoralNode()
  : rclcpp::Node("coral"), tf_buffer(get_clock()), tf_listener(tf_buffer)
{  
  pose_update_timer = create_wall_timer(50ms, [&](){refreshLinkPoses();});

  display_thrusters = declare_parameter("with_thrusters", false);

  spawn_srv = create_service<Spawn>
              ("/coral/spawn",
               [&](const Spawn::Request::SharedPtr request, [[maybe_unused]] Spawn::Response::SharedPtr response)
  {
    if(request->robot_namespace.empty())
      findModels();
    else
      spawnModel(request->robot_namespace, request->pose_topic);
  });

  surface_srv = create_service<Surface>("/coral/surface",
                                        [&](const Surface::Request::SharedPtr req, Surface::Response::SharedPtr res)
  {
    computeSurface(*req, *res);
  });

  clock_pub = create_subscription<rosgraph_msgs::msg::Clock>("/clock", 1, [&]([[maybe_unused]] rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
    // use_sim_time as soon as 1 message is received here
    set_parameter(rclcpp::Parameter("use_sim_time", true));
    clock_pub.reset();
  });
}

SceneParams CoralNode::parameters()
{
  SceneParams params;

  auto updateParam = [&](const string &description,
                     auto & val)
  {val = declare_parameter(description, val);};

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

  // ocean surface params
  updateParam("surface.reflection_damping", params.reflectionDamping);

  // vfx
  updateParam("vfx.vbo", params.useVBO);
  updateParam("vfx.godrays", params.godrays);
  updateParam("vfx.glare", params.glare);
  updateParam("vfx.underwaterDof", params.underwaterDof);

  return params;
}

Link* CoralNode::getKnownCamParent()
{
  static Link* prev_link{};

  std::string parent;
  tf_buffer._getParent(coral_cam_link, tf2::TimePointZero, parent);
  if(prev_link != nullptr && parent == prev_link->getName())
    return prev_link;

  prev_link = nullptr;

  // have to find it
  while(true)
  {
    // if we have reached the world frame
    if(parent == world_link.getName())
    {
      prev_link = &world_link;
      break;
    }
    // if we have reached a link that moves without TF knowing
    const auto root{std::find(links.begin(), links.end(), parent)};
    if(root != links.end())
    {
      prev_link = root.base();
      break;
    }
    // continue parenting
    if(!tf_buffer._getParent(parent, tf2::TimePointZero, parent))
      break;
  }
  return prev_link;
}

void CoralNode::refreshLinkPoses()
{
  vector<string> tf_links;
  tf_buffer._getFrameStrings(tf_links);
  for(auto &link: links)
  {
    if(hasLink(link.getName(), tf_links))
      link.refreshFrom(tf_buffer);
  }

  if(hasLink("world", tf_links) && hasLink(coral_cam_link, tf_links))
  {
    const auto parent{getKnownCamParent()};

    if(parent == nullptr)
      return;

    const auto tr = tf_buffer.lookupTransform(parent->getName(), coral_cam_link, tf2::TimePointZero, 10ms);
    const auto delay{(now() - tr.header.stamp).seconds()};
    if(delay < 1 || delay > 1e8)
    {
      auto M = Link::osgMatFrom(tr.transform.translation, tr.transform.rotation);

      if(parent->getName() != "world")
      {
        M = M*parent->frame()->getMatrix();
      }
      viewer->lockCamera(M);
    }
    else
      viewer->freeCamera();
  }
}

void CoralNode::findModels()
{
  const auto topics{get_topic_names_and_types()};
  const std::string description{"robot_description"};
  const auto isDescription{[description](const std::string &topic)
    {
      return topic.size() >= description.size()
          && 0 == topic.compare(topic.size()-description.size(), description.size(), description);
    }};

  // find all robot_description's
  for(const auto &[topic, msg]: topics)
  {
    if(isDescription(topic))
    {
      const auto ns{topic.substr(0, topic.size() - description.size()-1)};
      const auto isSameNS{[&ns](const std::string &topic)
        {
          return topic.size() >= ns.size() && 0 == topic.compare(0, ns.size(), ns);
        }};

      // pose_topic should be a geometry_msgs/Pose, published by Gazebo as ground truth
      std::string pose_topic;
      for(const auto &[topic, msg]: topics)
      {
        if(isSameNS(topic) && msg[0] == "geometry_msgs/msg/Pose")
        {
          pose_topic = topic.substr(ns.size()+1);
          break;
        }
      }
      spawnModel(ns, pose_topic);
    }
  }
}

void CoralNode::spawnModel(const std::string &model_ns, const std::string &pose_topic)
{
  if(hasModel(model_ns))
    return;
  // retrieve full model through robot_state_publisher
  const auto rsp_node(std::make_shared<Node>("coral_rsp"));
  const auto rsp_param_srv = std::make_shared<rclcpp::SyncParametersClient>
                             (rsp_node, model_ns + "/robot_state_publisher");
  rsp_param_srv->wait_for_service();
  if(!rsp_param_srv->has_parameter("robot_description"))
  {
    // cannot get the model anyway
    std::cout << "cannot get model " << model_ns << std::endl;
    return;
  }

  const auto root_link_idx{links.size()};

  parseModel(rsp_param_srv->get_parameter<string>("robot_description"));
  if(!pose_topic.empty())
  {
    if(root_link_idx == links.size())
    {
      // no new links were added for this robot
      RCLCPP_WARN(get_logger(),
                  "No link was found in description from namespace %s",
                  model_ns.c_str());
    }
    else
    {
      links[root_link_idx].updatedFromTopic();
      std::cout << model_ns << " seems to have its pose published on " << pose_topic << std::endl;
      pose_subs.push_back(create_subscription<Pose>(model_ns + "/" + pose_topic, 1, [&,root_link_idx](Pose::SharedPtr msg)
      {
        links[root_link_idx].setPose(Link::osgMatFrom(msg->position, msg->orientation));
      }));
    }
  }
  models.push_back(model_ns);
}

void CoralNode::parseModel(const string &description)
{
  const auto [new_links, new_cams] = urdf_parser::parse(description, display_thrusters); {}

  // add cameras
  if(!new_cams.empty())
  {
    if(!it) it = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    // check if Gazebo is already publishing images, if any
    const auto current_topics{get_topic_names_and_types()};
    for(const auto &cam: new_cams)
    {

      if(current_topics.find(cam.topic) != current_topics.end())
        RCLCPP_WARN(get_logger(),
                    "Image topic %s seems already advertized by Gazebo, use `unset DISPLAY` in the Gazebo terminal and run without GUI",
                    cam.topic.c_str());

      cameras.emplace_back(this, cam);
    }
  }

  // add links
  for(const auto &link: new_links)
  {
    link.attachTo(scene);
    links.push_back(link);
  }
}

void CoralNode::computeSurface(const Surface::Request &req, Surface::Response &res)
{
  const auto surface{scene->oceanSurface()};

  // compute local frame
  res.dim = req.size/req.resolution + 1;
  res.surface.reserve(res.dim*res.dim);
  const auto c{cos(req.theta)};
  const auto s{sin(req.theta)};
  const auto start{-req.size/2};

  for(int ix = 0; ix < res.dim; ix++)
  {
    const auto dx{start+ix*req.resolution};

    for(int iy = 0; iy < res.dim; iy++)
    {
      const auto dy{start+iy*req.resolution};
      res.surface.push_back(
            surface->getSurfaceHeightAt(
              req.x + dx*c - dy*s,
              req.y + dx*s + dy*c));
    }
  }
}
