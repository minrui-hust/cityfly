#include "common/node.h"

using namespace cityfly::common;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  Node test_node("test_node");
  test_node.start();
  return 0;
}
