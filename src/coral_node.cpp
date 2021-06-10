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

SceneParams CoralNode::parameters()
{
  SceneParams params;

  auto updateParam = [&](const std::string &description,
                   auto & val)
  {val = declare_parameter(description, val);};

  // display
  updateParam("width", params.width);
  updateParam("height", params.height);
  auto cam(params.asVector(params.initialCameraPosition));
  updateParam("camera", cam);
  params.initialCameraPosition.set(cam[0], cam[1], cam[2]);

  auto wind(params.asVector(params.windDirection));
  updateParam("windDirection", wind);
  params.windDirection.set(wind[0], wind[1]);
  updateParam("windSpeed", params.windSpeed);

  updateParam("depth", params.depth);
  updateParam("reflectionDamping", params.reflectionDamping);

  // ocean surface params
  updateParam("waveScale", params.waveScale);
  updateParam("choppyFactor", params.choppyFactor);
  updateParam("crestFoamHeight", params.crestFoamHeight);

  // vfx
  updateParam("useVBO", params.useVBO);
  updateParam("godrays", params.godrays);
  updateParam("glare", params.glare);
  updateParam("underwaterDof", params.underwaterDof);

  return params;
}

void CoralNode::parseTFTree()
{
  if(scene == nullptr || viewer == nullptr)
    return;

  tf_buffer._getFrameStrings(tf_links);
  bool new_meshes(false);

  for(const auto &link: tf_links)
  {
    if(std::find(parsed_links.begin(), parsed_links.end(), link) == parsed_links.end())
      new_meshes = new_meshes || parseLink(link);
  }

  if(!world_is_parsed)
    new_meshes = new_meshes || parseWorld();

  if(new_meshes)
  {
    // optimize graph / meshes at some point?
  }
}

bool CoralNode::parseLink(const std::string &link)
{
  // by convention link == "namespace/link" else we do not care
  const auto slash = link.find('/');
  if(slash == link.npos)
  {
    // not a meshed link
    parsed_links.push_back(link);

    if(link == coral_cam_link)
      has_cam_view = true;
    return false;
  }

  // go up full model through robot_state_publisher
  auto rsp_node(std::make_shared<Node>("coral_rsp"));
  auto rsp_param_srv = std::make_shared<rclcpp::SyncParametersClient>
                       (rsp_node, link.substr(0, slash) + "/robot_state_publisher");
  if(!rsp_param_srv->service_is_ready() || !rsp_param_srv->has_parameter("robot_description"))
  {
    // not a robot_state_publisher
    parsed_links.push_back(link);
    return false;
  }

  return parseModel(rsp_param_srv->get_parameters({"robot_description"})[0].as_string());
}


bool CoralNode::parseModel(const std::string &description)
{
  const auto model(urdf::parseURDF(description));
  bool has_mesh(false);

  for(const auto &[name,link]: model->links_)
  {
    parsed_links.push_back(name);

    VisualLink visual(name, link);

    if(visual.hasVisuals())
    {
      has_mesh = true;
      visual.attachTo(scene);
      visual_links.push_back(visual);
    }
  }
  return has_mesh;
}


bool CoralNode::parseWorld()
{
  auto pwm(std::make_shared<Node>("coral_pwm"));
  auto pwm_param_srv = std::make_shared<rclcpp::SyncParametersClient>
                       (pwm, "publish_world_models");

  if(!pwm_param_srv->service_is_ready())
    return false;

  world_is_parsed = true;
  const auto params(pwm_param_srv->list_parameters({"meshes"}, 0).names);

  // params are meshes.<name>. mesh
  //                          pose.position
  //                          pose.orientation
  //                          scale

  vector<string> names;
  const auto len_begin(std::string("meshes.").size());
  const std::string mesh_end(".mesh");
  const auto len_end(mesh_end.size());
  for(const auto &param: params)
  {
    // keep name if ends with ".mesh"
    if(0 == param.compare(param.size()-len_end, len_end, mesh_end))
      names.push_back(param.substr(len_begin, param.size()-len_begin-len_end));
  }

  if(names.empty())
    return false;

  for(const auto &name: names)
  {
    const auto getParam([pwm_param_srv,&name](const std::string info, auto default_val)
    {return pwm_param_srv->get_parameter("meshes." + name + "." + info, default_val);});

    const auto mesh(getParam("mesh", std::string{}));

    if(mesh.empty())
      continue;

    world_link.addVisual(mesh,
                         getParam("pose.position", std::vector<double>{0,0,0}),
                         getParam("pose.orientation", std::vector<double>{0,0,0}),
                         getParam("scale", std::vector<double>{1,1,1}));
  }

  if(!world_link.hasVisuals())
    return false;

  world_link.attachTo(scene);
  return true;
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
