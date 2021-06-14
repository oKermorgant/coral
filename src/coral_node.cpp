#include <coral/coral_node.h>
#include <coral/visual_link.h>
#include <urdf_parser/urdf_parser.h>

#include <osgUtil/Optimizer>

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

void CoralNode::parseTFTree()
{
  if(scene == nullptr || viewer == nullptr)
    return;

  tf_buffer._getFrameStrings(tf_links);
  for(const auto &link: tf_links)
  {
    if(std::find(parsed_links.begin(), parsed_links.end(), link) == parsed_links.end())
      parseLink(link);
  }

  if(!world_is_parsed)
    parseWorld();
}

void CoralNode::refreshLinkPoses()
{
  tf_buffer._getFrameStrings(tf_links);
  for(auto &link: visual_links)
    link.refreshFrom(tf_buffer, tf_links);

  if(has_cam_view)
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


void CoralNode::parseLink(const string &link)
{
  // by convention link == "namespace/link" else we do not care
  const auto slash = link.find('/');
  if(slash == link.npos)
  {
    // not a visual link
    parsed_links.push_back(link);

    if(link == coral_cam_link)
      has_cam_view = true;
    return;
  }

  // go up full model through robot_state_publisher
  auto rsp_node(std::make_shared<Node>("coral_rsp"));
  auto rsp_param_srv = std::make_shared<rclcpp::SyncParametersClient>
                       (rsp_node, link.substr(0, slash) + "/robot_state_publisher");
  if(!rsp_param_srv->service_is_ready() || !rsp_param_srv->has_parameter("robot_description"))
  {
    // not a robot_state_publisher
    parsed_links.push_back(link);
    return;
  }

  parseModel(rsp_param_srv->get_parameter<string>("robot_description"));
}

void CoralNode::parseModel(const string &description)
{
  const auto model(urdf::parseURDF(description));

  for(const auto &[name,link]: model->links_)
  {
    parsed_links.push_back(name);

    VisualLink visual(name, link);

    if(visual.hasVisuals())
    {
      visual.attachTo(scene);
      visual_links.push_back(visual);
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
    world_link.addVisualMesh(mesh_file, VisualLink::osgMatFrom(xyz, rpy, scale));
    return;
  }
  else if(geometry == GeometryType::BOX)
  {
    const auto dim(getParam("", vector<double>{0,0,0}));
    // world boxes are sand
    urdf::Material mat;
    mat.texture_filename = "soil_sand_0045_01.jpg";
    world_link.addVisualBox({dim[0], dim[1], dim[2]},
                            VisualLink::osgMatFrom(xyz, rpy),
                            mat);
  }
}


