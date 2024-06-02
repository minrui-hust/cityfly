#include "common/log.h"

#include "sensor/oak_ffc_4p.h"

int main(int argc, char **argv) {
  cityfly::common::InitLogging("oak_ffc_4p", "/tmp", INFO, true);

  rclcpp::init(argc, argv);
  cityfly::sensor::OakFfc4p node;
  node.start();

  cityfly::common::FinitLogging();

  return 0;
}
