
#include <coral/coral_node.h>

using namespace coral;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);


  auto node(std::make_shared<CoralNode>());
  //SceneInterface interface(node->nodeParams());
  //node->setInterface(interface);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while(rclcpp::ok())
  {
    exec.spin_once();
    //viewer.frame();
  }

  if(rclcpp::ok())
    rclcpp::shutdown();

  return 0;
}
