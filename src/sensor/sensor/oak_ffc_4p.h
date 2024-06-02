#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "depthai/depthai.hpp"

#include "common/interpolater.h"
#include "common/node.h"

namespace cityfly::sensor {
using namespace rclcpp;
using namespace dai;
using namespace common;

struct OakFfc4pConfig {
  int imu_rate = 100;
  float cam_fps = 10;
};

struct OakFfc4p : public Node {
  OakFfc4p();

protected:
  bool getParameters() override;
  bool configure() override;
  bool connect() override;
  bool runOnline() override;
  bool clean() override;

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

  Interpolater<Vector3> acc_interp_{100000, 0};
};

} // namespace cityfly::sensor
