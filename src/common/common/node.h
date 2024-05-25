#pragma once

#include "rclcpp/node.hpp"
#include <rclcpp/executors.hpp>

namespace cityfly::common {

struct Node {
  Node(const std::string &name) {
    node_ = std::shared_ptr<rclcpp::Node>(new rclcpp::Node(
        name, rclcpp::NodeOptions()
                  .allow_undeclared_parameters(true)
                  .automatically_declare_parameters_from_overrides(true)));
  }

  void start();

protected:
  void run();

  virtual void configure() {}
  virtual void preRun() {}
  virtual void runOnline();
  virtual void runOffline() {}
  virtual void postRun() {}
  virtual void clean() {}

protected:
  std::shared_ptr<rclcpp::Node> node_;

  bool offline_ = false;
};

inline void Node::start() {
  configure();
  preRun();
  run();
  postRun();
  clean();
}

inline void Node::run() {
  if (offline_) {
    runOffline();
  } else {
    runOnline();
  }
}

inline void Node::runOnline() { rclcpp::spin(node_); }

} // namespace cityfly::common
