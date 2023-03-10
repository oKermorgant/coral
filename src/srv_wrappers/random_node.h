#include <rclcpp/node.hpp>
#include <random>

auto random_node(std::string base)
{
  const std::string chars = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
  std::random_device rd;
  std::default_random_engine g{rd()};

  // The uniform_int_distribution takes a closed interval of [a, b]; since that
  // would include the \0, we remove one from our string length.
  static std::uniform_int_distribution<std::string::size_type> pick(0, chars.length() - 1);

  const auto orig_length{base.length()};
  base.resize(orig_length + 16);
  for(auto c = base.begin()+orig_length; c != base.end(); ++c)
    *c = chars[pick(g)];

  return rclcpp::Node::make_shared(base);
}
