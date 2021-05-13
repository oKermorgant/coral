#include <coral/coral_node.h>
#include <coral/visual_link.h>
#include <urdf_parser/urdf_parser.h>

using namespace coral;
using namespace std::chrono_literals;

using Strings = std::vector<std::string>;

CoralNode::CoralNode() : rclcpp::Node("coral"), tf_buffer(get_clock()), tf_listener(tf_buffer)
{
  tree_parser_timer = create_wall_timer(1s, [&](){parseTFTree();});
  pose_update_timer = create_wall_timer(50ms, [&](){refreshLinkPoses();});
}

void CoralNode::setInterface(SceneInterface &scene)
{
  this->scene = &scene;
}

SceneParams CoralNode::nodeParams()
{
  SceneParams params;

  const auto wind(declare_parameter("windDirection", std::vector<double>{1.1,1.1}));
  params.windDirection.x() = static_cast<float>(wind[0]);
  params.windDirection.y() = static_cast<float>(wind[1]);
  params.windSpeed = declare_parameter<float>("windSpeed", 12);

  params.depth = declare_parameter("depth", 1000.f);
  params.reflectionDamping = declare_parameter("reflectionDamping", 0.35f);

  params.waveScale = declare_parameter("scale", 1e-8f);
  params.choppyFactor = declare_parameter("choppyFactor", 2.5f);
  params.isChoppy = std::abs(params.choppyFactor) > 1e-3;
  params.crestFoamHeight = declare_parameter("crestFoamHeight", 2.2f);

  params.useVBO = declare_parameter("useVBO", false);

  const auto cam(declare_parameter("camera", std::vector<double>{0.,0.,20.}));
  params.initialCameraPosition = {static_cast<float>(cam[0]), static_cast<float>(cam[1]), static_cast<float>(cam[2])};

  // godrays do not work on Intel GPUs, default to false
  params.godrays = declare_parameter<bool>("godrays", false);

  return params;
}

void CoralNode::parseTFTree()
{
  static Strings links;
  tf_buffer._getFrameStrings(links);

  for(const auto &link: links)
  {
    if(std::find(parsed_links.begin(), parsed_links.end(), link) == parsed_links.end())
      parseLink(link);
  }
}

void CoralNode::parseLink(const std::string &link)
{
  // by convention link == "namespace/link" else we do not care
  const auto slash = link.find('/');
  if(slash == link.npos)
  {
    // not a meshed link
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

  parseModel(rsp_param_srv->get_parameters({"robot_description"})[0].as_string());
}


void CoralNode::parseModel(const std::string &description)
{
  const auto model(urdf::parseURDF(description));

  for(const auto &[name,link]: model->links_)
  {
    parsed_links.push_back(name);

    VisualLink visual(name, link);

    if(visual.hasVisuals())
    {
      visual.attachTo(scene->oceanScene());
      links.push_back(visual);
    }
  }
}

void CoralNode::refreshLinkPoses()
{
  for(auto &link: links)
    link.refreshFrom(tf_buffer);

  if(has_cam_view)
  {
    auto tr = tf_buffer.lookupTransform("world", "coral_cam_view", tf2::TimePointZero, 10ms);
    if((now() - tr.header.stamp).seconds() < 1)
    {
      // TODO send this cam view to scene
    }
  }

  if(!world_is_parsed)
    parseWorld();
}

void CoralNode::parseWorld()
{
  auto pwm(std::make_shared<Node>("coral_pwm"));
  auto pwm_param_srv = std::make_shared<rclcpp::SyncParametersClient>
                       (pwm, "publish_world_models");
  if(pwm_param_srv->service_is_ready())
  {
    world_is_parsed = true;
    const auto params(pwm_param_srv->list_parameters({"meshes"}, 0).names);

    // params are meshes.<name>. mesh
    //                          pose.position
    //                          pose.orientation
    //                          scale

    Strings names;
    const auto len_begin(std::string("meshes.").size());
    const std::string mesh_end(".mesh");
    const auto len_end(mesh_end.size());
    for(const auto &param: params)
    {
      // keep name if ends with ".mesh"
      if(0 == param.compare(param.size()-len_end, len_end, mesh_end))
        names.push_back(param.substr(len_begin, param.size()-len_end));
    }

    if(names.empty())
      return;

    VisualLink world_visuals("world");

    for(const auto &name: names)
    {
      const auto getParam([pwm_param_srv,&name](const std::string info, auto default_val)
      {return pwm_param_srv->get_parameter("meshes." + name + "." + info, default_val);});

      const auto mesh(getParam("mesh", std::string{}));

      if(mesh.empty())
        continue;

      world_visuals.addVisual(mesh,
                              getParam("pose.position", std::vector<double>{0,0,0}),
                              getParam("pose.orientation", std::vector<double>{0,0,0}),
                              getParam("scale", std::vector<double>{1,1,1}));

      std::cout << name << std::endl;
    }

    world_visuals.attachTo(scene->oceanScene());
  }
}
