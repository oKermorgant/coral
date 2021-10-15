#include <coral/coral_node.h>
#include <coral/urdf_parser.h>
#include <std_srvs/srv/empty.hpp>

using namespace coral;
using namespace std::chrono_literals;
using std::vector, std::string;

CoralNode::CoralNode()
  : rclcpp::Node("coral"), tf_buffer(get_clock()), tf_listener(tf_buffer),
    world_link("world")
{  
  world_sim_timer = create_wall_timer(1s, [&](){refreshWorldParams();});
  pose_update_timer = create_wall_timer(50ms, [&](){refreshLinkPoses();});

  display_thrusters = declare_parameter("with_thrusters", false);

  spawn_srv = create_service<Spawn>
              ("/coral/spawn", [&](const Spawn::Request::SharedPtr request, Spawn::Response::SharedPtr response)
  {
    spawnModel(request->robot_namespace, request->pose_topic);
    (void) response;
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

void CoralNode::odomCallback(const std::string &link_name, const geometry_msgs::msg::Pose &pose)
{
  for(auto &link: abs_links)
  {
    if(link.get_name() == link_name)
    {
      link.setPose(Link::osgMatFrom(pose.position, pose.orientation));
      return;
    }
  }
}

void CoralNode::refreshWorldParams()
{
  if(scene == nullptr || viewer == nullptr)
    return;

  const auto gazebo_detected{get_parameter("use_sim_time").as_bool()};
  if(!gazebo_detected)
  {
    const auto gz_physics = create_client<std_srvs::srv::Empty>("/gazebo/pause_physics");
    if(gz_physics->service_is_ready())
      set_parameter(rclcpp::Parameter("use_sim_time", true));
  }

  if(!world_is_parsed)
    parseWorld();

  if(gazebo_detected && world_is_parsed)
    world_sim_timer.reset();
}

Link* CoralNode::getKnownCamParent()
{
  static Link* prev_link{};

  std::string parent;
  tf_buffer._getParent(coral_cam_link, tf2::TimePointZero, parent);
  if(prev_link != nullptr && parent == prev_link->get_name())
    return prev_link;

  prev_link = nullptr;

  // have to find it
  while(true)
  {
    // if we have reached the world frame
    if(parent == world_link.get_name())
    {
      prev_link = &world_link;
      break;
    }
    // if we have reached a link that moves without TF knowing
    const auto root{std::find(abs_links.begin(), abs_links.end(), parent)};
    if(root != abs_links.end())
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
  for(auto &link: rel_links)
  {
    if(hasLink(link.get_name(), tf_links))
      link.refreshFrom(tf_buffer);
  }

  if(hasLink(world_link.get_name(), tf_links) && hasLink(coral_cam_link, tf_links))
  {
    const auto parent{getKnownCamParent()};

    if(parent == nullptr)
      return;

    const auto tr = tf_buffer.lookupTransform(parent->get_name(), coral_cam_link, tf2::TimePointZero, 10ms);
    if((now() - tr.header.stamp).seconds() < 1)
    {
      auto M = Link::osgMatFrom(tr.transform.translation, tr.transform.rotation);

      if(parent->get_name() != world_link.get_name())
        M = parent->frame()->getMatrix() * M;
      viewer->lockCamera(M);
    }
    else
      viewer->freeCamera();
  }
}

void CoralNode::spawnModel(const std::string &model_ns, const std::string &pose_topic)
{
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

  const auto moving{!pose_topic.empty()};

  parseModel(rsp_param_srv->get_parameter<string>("robot_description"), moving);

  if(moving)
  {
    odom_subs.push_back(create_subscription<Odometry>(model_ns + "/" + pose_topic, 1, [&](Odometry::UniquePtr msg)
    {odomCallback(msg->child_frame_id, msg->pose.pose);}));
  }
}

void CoralNode::parseModel(const string &description, bool moving)
{
  const auto [root_link, new_links, new_cams] = urdf_parser::parse(description, display_thrusters); {}

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
                    "Image topic " + cam.topic + " seems already advertized by Gazebo, use `unset DISPLAY` in the Gazebo terminal and run without GUI");

      cameras.emplace_back(this, cam);
    }
  }

  // add links
  for(const auto &link: new_links)
  {
    link.attachTo(scene);
    if(moving && link == root_link)
      abs_links.push_back(link);
    else
      rel_links.push_back(link);
  }
}

void CoralNode::parseWorld()
{
  auto pwm(std::make_shared<Node>("coral_pwm"));
  auto pwm_client = std::make_shared<rclcpp::SyncParametersClient>
                    (pwm, "publish_world_models");

  if(!pwm_client->service_is_ready())
    return;

  world_is_parsed = true;
  const auto params(pwm_client->list_parameters({"meshes"}, 0).names);
  const auto len_begin(string("meshes.").size());

  const vector<string> ignored{"north", "south", "east", "west", "sea_surface"};

  const std::map<GeometryType, string> extension
  {{GeometryType::MESH, ".mesh"}, {GeometryType::BOX, ".plane"}};

  // extract all relevant world visual links
  for(const auto &[geometry, ext]: extension)
  {
    const auto len_end(ext.size());
    for(const auto &param: params)
    {
      if(0 == param.compare(param.size()-len_end, len_end, ext))
      {
        const auto name{param.substr(len_begin, param.size()-len_begin-len_end)};
        if(std::find(ignored.begin(), ignored.end(), name) == ignored.end())
          addWorldVisual(pwm_client, name, geometry);
      }
    }
  }

  if(world_link.hasVisuals())
    world_link.attachTo(scene);
}

void CoralNode::addWorldVisual(rclcpp::SyncParametersClient::SharedPtr pwm_client, const std::string &name, GeometryType geometry)
{
  const auto getParam([&name,&pwm_client](string info, auto default_val)
  {return pwm_client->get_parameter("meshes." + name + info, default_val);});

  const auto xyz(getParam(".pose.position", vector<double>{0,0,0}));
  const auto rpy(getParam(".pose.orientation", vector<double>{0,0,0}));

  if(geometry == GeometryType::MESH)
  {
    const auto scale(getParam(".scale", vector<double>{1,1,1}));
    const auto mesh_file(getParam(".mesh", std::string{}));
    world_link.addVisualMesh(mesh_file,
                             Link::osgMatFrom(xyz, rpy, scale),
                             Link::uuvMaterial(mesh_file).get());
    return;
  }
  else if(geometry == GeometryType::BOX)
  {
    const auto dim(getParam("", vector<double>{0,0,0}));
    // uuvMaterial defaults to sand
    world_link.addVisualBox({dim[0], dim[1], dim[2]},
                            Link::osgMatFrom(xyz, rpy),
                            Link::uuvMaterial().get());
  }
}


