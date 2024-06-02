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
  virtual bool getParameters() { return true; }
  virtual bool configure() { return true; }
  virtual bool connect() { return true; }
  virtual bool preRun() { return true; }
  virtual bool runOnline();
  virtual bool runOffline() { return true; }
  virtual bool postRun() { return true; }
  virtual bool clean() { return true; }

  bool run();

protected:
  std::shared_ptr<rclcpp::Node> node_;

  bool offline_ = false;
};

inline void Node::start() {
  if (!getParameters()) {
    return;
  }

  if (!configure()) {
    return;
  }

  if (!connect()) {
    return;
  }

  if (!preRun()) {
    return;
  }

  if (!run()) {
    return;
  }

  if (!postRun()) {
    return;
  }

  if (!clean()) {
    return;
  }
}

inline bool Node::run() {
  if (offline_) {
    return runOffline();
  } else {
    return runOnline();
  }
}

inline bool Node::runOnline() {
  rclcpp::spin(node_);
  return true;
}

} // namespace cityfly::common
