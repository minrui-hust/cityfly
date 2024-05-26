#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "depthai/depthai.hpp"

#include "common/node.h"

namespace cityfly::sensor {
using namespace rclcpp;
using namespace dai;
using namespace common;

struct OakFfc4pConfig {
  uint32_t imu_rate = 100;
  float cam_fps = 20;
};

struct OakFfc4p : public Node {
  OakFfc4p();

protected:
  void getParameters() override;
  void configure() override;
  void connect() override;
  void runOnline() override;
  void clean() override;

  Pipeline constructPipeline();

  void createImu(Pipeline &pipeline);
  void createCamera(Pipeline &pipeline, const std::string &name,
                    const CameraBoardSocket &socket_id,
                    const CameraControl::FrameSyncMode &sync_mode);

  void pollImu();
  void pollImage(const std::string &name);

protected:
  OakFfc4pConfig cfg_;
  std::shared_ptr<Device> device_;
  std::map<std::string, std::thread> threads_;

  Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::map<std::string, Publisher<sensor_msgs::msg::Image>::SharedPtr>
      img_pubs_;
};

} // namespace cityfly::sensor
