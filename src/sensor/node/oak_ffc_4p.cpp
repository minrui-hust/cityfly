#include "sensor/oak_ffc_4p.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  cityfly::sensor::OakFfc4p node;
  node.start();
  return 0;
}
