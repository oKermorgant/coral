#include <coral/coral_node.h>
#include <coral/link.h>
#include <urdf_parser/urdf_parser.h>
#include <std_srvs/srv/empty.hpp>

using namespace coral;
using namespace std::chrono_literals;
using std::vector;
using std::string;

CoralNode::CoralNode()
  : rclcpp::Node("coral"), tf_buffer(get_clock()), tf_listener(tf_buffer),
    world_link("world")
{
  tree_parser_timer = create_wall_timer(1s, [&](){parseTFTree();});
  pose_update_timer = create_wall_timer(50ms, [&](){refreshLinkPoses();});
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



void CoralNode::parseTFTree()
{
  if(scene == nullptr || viewer == nullptr)
    return;

  if(!get_parameter("use_sim_time").as_bool())
    guessSimTime();

  vector<string> tf_links;
  tf_buffer._getFrameStrings(tf_links);

  for(const auto &link: tf_links)
  {
    if(parsed_links.find(link) == parsed_links.end())
      parseModelFromLink(link);
  }

  if(!world_is_parsed)
    parseWorld();
}

void CoralNode::guessSimTime()
{
 const auto gz_physics = create_client<std_srvs::srv::Empty>("/gazebo/pause_physics");
 if(gz_physics->service_is_ready())
   set_parameter(rclcpp::Parameter("use_sim_time", true));
}

void CoralNode::refreshLinkPoses()
{
  vector<string> tf_links;
  tf_buffer._getFrameStrings(tf_links);
  for(auto &link: links)
    link.refreshFrom(tf_buffer, tf_links);

  if(has_cam_view && tf_buffer.canTransform("world", "coral_cam_view", tf2::TimePointZero, 10ms))
  {
    auto tr = tf_buffer.lookupTransform("world", "coral_cam_view", tf2::TimePointZero, 10ms);    
    if((now() - tr.header.stamp).seconds() < 1)
    {
      const auto &t(tr.transform.translation);
      const auto &q(tr.transform.rotation);
      viewer->lockCamera({t.x,t.y,t.z}, {q.x,q.y,q.z,q.w});
    }
    else
      viewer->freeCamera();
  }
}


void CoralNode::parseModelFromLink(const string &link)
{
  parsed_links.insert(link);

  // by convention link is shaped as "namespace/link" else we do not care
  const auto slash = link.find('/');
  if(slash == link.npos)
  {
    // not an object link, maybe camera setpoint?
    if(link == coral_cam_link)
      has_cam_view = true;
    return;
  }

  // retrieve full model through robot_state_publisher
  auto rsp_node(std::make_shared<Node>("coral_rsp"));
  auto rsp_param_srv = std::make_shared<rclcpp::SyncParametersClient>
                       (rsp_node, link.substr(0, slash) + "/robot_state_publisher");
  if(!rsp_param_srv->service_is_ready() || !rsp_param_srv->has_parameter("robot_description"))
  {
    // cannot get the model anyway
    return;
  }

  parseModel(rsp_param_srv->get_parameter<string>("robot_description"));
}

void CoralNode::parseModel(const string &description)
{
  const auto model(urdf::parseURDF(description));
  const auto cameras(CameraInfo::extractFrom(description));

  // check if Gazebo is already publishing images, if any
  const auto current_topics = cameras.empty()
                              ? decltype (get_topic_names_and_types()){}
                              : get_topic_names_and_types();

  for(const auto &[name,urdf]: model->links_)
  {
    parsed_links.insert(name);

    Link link(name, urdf);

    bool has_cam(false);
    for(const auto &cam: cameras)
    {
      if(cam.link_name == name)
      {
        if(current_topics.find(cam.topic) != current_topics.end())
          RCLCPP_WARN(get_logger(), "Image topic " + cam.topic + " seems already advertized by Gazebo, use `unset DISPLAY` in the Gazebo terminal");
        has_cam = true;
        if(!it) it = std::make_unique<image_transport::ImageTransport>(shared_from_this());
        this->cameras.emplace_back(this, cam.attachedTo(link.frame()));
      }
    }

    if(link.hasVisuals() || has_cam)
    {
      link.attachTo(scene);
      links.push_back(link);
    }
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


