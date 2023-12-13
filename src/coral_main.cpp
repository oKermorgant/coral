#include <coral/coral_node.h>
#include <coral/OceanScene.h>
#include <coral/viewer.h>

using namespace coral;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  osg::setNotifyLevel(osg::WARN);

  auto node{std::make_shared<CoralNode>()};
  auto scene = osg::make_ref<OceanScene>(node->parameters());
  auto viewer = Viewer(scene);

  node->manage(scene, viewer);

#ifndef CORAL_NO_THREADS
  std::thread ros([&](){rclcpp::spin(node);});
#endif

  while(!viewer.done() && rclcpp::ok())
  {
#ifdef CORAL_NO_THREADS
    rclcpp::spin_some(node);
#endif
    viewer.frame();
  }

  if(rclcpp::ok())
    rclcpp::shutdown();
#ifndef CORAL_NO_THREADS
  ros.join();
#endif

  return 0;
}
