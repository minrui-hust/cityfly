#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "depthai/depthai.hpp"

#include "common/interpolater.h"
#include "common/node.h"

#include "msync/supported_policies/linear_interpolater.h"
#include "msync/supported_storages/map_storage.h"
#include "msync/syncronizer.h"

namespace cityfly::sensor {
using namespace rclcpp;
using namespace dai;
using namespace common;
using namespace msync;

using Policy = LinearInterpolatePolicy<Vector3, MapStorage<Vector3>>;
using Sync = SyncronizerMasterSlave<Policy, Policy>;

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

  std::shared_ptr<image_transport::ImageTransport> img_transport_;

  Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::map<std::string, image_transport::Publisher> img_transport_pubs_;

  Sync imu_sync_{Policy{NANOS_PER_MILLI * 100, 0, kNormal},
                 Policy{NANOS_PER_MILLI * 100, 0, kMaster}};
};

} // namespace cityfly::sensor
