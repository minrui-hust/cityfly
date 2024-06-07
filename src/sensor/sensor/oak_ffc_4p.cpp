#include "cv_bridge/cv_bridge.h"

#include "common/log.h"

#include "oak_ffc_4p.h"

namespace cityfly::sensor {

namespace {

Stamp FromTimePoint(
    const std::chrono::time_point<std::chrono::steady_clock,
                                  std::chrono::steady_clock::duration> &tp) {
  return tp.time_since_epoch().count();
}

} // namespace

OakFfc4p::OakFfc4p() : Node("oak_ffc_4p") {}

bool OakFfc4p::getParameters() {
  cfg_.imu_rate = node_->get_parameter_or("imu_rate", 100);
  cfg_.cam_fps = node_->get_parameter_or("cam_fps", 10.0f);

  return true;
}

bool OakFfc4p::configure() {

  try { // try to open device
    SINFO << "Opening oak device...";
    device_ = std::make_shared<Device>();
  } catch (const std::exception &) {
    SINFO << "Failed to open device";
    return false;
  }

  // try start pipeline
  if (!device_->startPipeline(constructPipeline())) {
    SINFO << "Failed to start pipeline";
    return false;
  }

  return true;
}

bool OakFfc4p::connect() {
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu", 1);

  // clang-format off
  img_pubs_["cam_front_l"] = node_->create_publisher<sensor_msgs::msg::Image>("cam_front_l", 3);
  img_pubs_["cam_front_r"] = node_->create_publisher<sensor_msgs::msg::Image>("cam_front_r", 3);
  img_pubs_["cam_down_l"]  = node_->create_publisher<sensor_msgs::msg::Image>("cam_down_l" , 3);
  img_pubs_["cam_down_r"]  = node_->create_publisher<sensor_msgs::msg::Image>("cam_down_r" , 3);
  // clang-format on

  return true;
}

bool OakFfc4p::runOnline() {
  threads_["imu"] = std::thread([this]() { pollImu(); });
  threads_["cam_front_l"] = std::thread([this]() { pollImage("cam_front_l"); });
  threads_["cam_front_r"] = std::thread([this]() { pollImage("cam_front_r"); });
  threads_["cam_down_l"] = std::thread([this]() { pollImage("cam_down_l"); });
  threads_["cam_down_r"] = std::thread([this]() { pollImage("cam_down_r"); });

  SINFO << "Driver running...";

  // wait for all thread to join
  for (auto &[_, thread] : threads_) {
    thread.join();
  }

  return true;
}

bool OakFfc4p::clean() {
  SINFO << "closing device";
  device_->close();
  return true;
}

Pipeline OakFfc4p::constructPipeline() {
  Pipeline pipeline;

  // clang-format off
  createImu(pipeline);
  createCamera(pipeline, "cam_front_l", CameraBoardSocket::CAM_D, CameraControl::FrameSyncMode::INPUT );
  createCamera(pipeline, "cam_front_r", CameraBoardSocket::CAM_A, CameraControl::FrameSyncMode::OUTPUT);
  createCamera(pipeline, "cam_down_l" , CameraBoardSocket::CAM_C, CameraControl::FrameSyncMode::OFF   );
  createCamera(pipeline, "cam_down_r" , CameraBoardSocket::CAM_B, CameraControl::FrameSyncMode::OFF   );
  // clang-format on

  return pipeline;
}

void OakFfc4p::createImu(Pipeline &pipeline) {
  auto imu_sensors = std::vector<dai::IMUSensor>{
      dai::IMUSensor::ACCELEROMETER, dai::IMUSensor::GYROSCOPE_UNCALIBRATED};
  auto imu = pipeline.create<node::IMU>();
  imu->enableIMUSensor(imu_sensors, cfg_.imu_rate);
  imu->setBatchReportThreshold(1);
  imu->setMaxBatchReports(10);

  auto imu_link = pipeline.create<node::XLinkOut>();
  imu_link->setStreamName("imu");

  imu->out.link(imu_link->input);
}

void OakFfc4p::createCamera(Pipeline &pipeline, const std::string &name,
                            const CameraBoardSocket &socket_id,
                            const CameraControl::FrameSyncMode &sync_mode) {
  auto cam = pipeline.create<dai::node::MonoCamera>();
  cam->setBoardSocket(socket_id);
  cam->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
  cam->setFps(cfg_.cam_fps);

  cam->initialControl.setFrameSyncMode(sync_mode);

  auto cam_link = pipeline.create<dai::node::XLinkOut>();
  cam_link->setStreamName(name);

  cam->out.link(cam_link->input);
}

void OakFfc4p::pollImu() {
  auto queue = device_->getOutputQueue("imu", 10, false);
  bool timeout;
  int last_acc_stamp = -1;
  int last_gyr_stamp = -1;

  imu_sync_.registerCallback([&](const Time stamp,
                                 const std::pair<Vector3, bool> &acc,
                                 const std::pair<Vector3, bool> &gyr) {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = ToRosTime(stamp);

    imu_msg.linear_acceleration.x = acc.first.x();
    imu_msg.linear_acceleration.y = acc.first.y();
    imu_msg.linear_acceleration.z = acc.first.z();

    imu_msg.angular_velocity.x = gyr.first.x();
    imu_msg.angular_velocity.y = gyr.first.y();
    imu_msg.angular_velocity.z = gyr.first.z();

    imu_pub_->publish(imu_msg);
  });

  while (rclcpp::ok()) {
    auto imu_data = queue->get<IMUData>(std::chrono::seconds(1), timeout);
    if (timeout) {
      SWARN << "Poll imu data timeout";
      continue;
    }

    for (const auto &imu_pkt : imu_data->packets) {
      const auto &acc = imu_pkt.acceleroMeter;
      const auto &gyr = imu_pkt.gyroscope;
      auto acc_stamp = FromTimePoint(acc.getTimestamp());
      auto gyr_stamp = FromTimePoint(gyr.getTimestamp());

      if (acc_stamp != last_acc_stamp) {
        imu_sync_.push<0>(acc_stamp, {acc.x, acc.y, acc.z});
        last_acc_stamp = acc_stamp;
      }

      if (gyr_stamp != last_gyr_stamp) {
        imu_sync_.push<1>(gyr_stamp, {gyr.x, gyr.y, gyr.z});
        last_gyr_stamp = gyr_stamp;
      }
    }
  }
}

void OakFfc4p::pollImage(const std::string &name) {
  auto queue = device_->getOutputQueue(name, 2, false);
  auto &pub = img_pubs_.at(name);
  bool timeout;

  while (rclcpp::ok()) {
    auto img = queue->get<ImgFrame>(std::chrono::seconds(1), timeout);
    if (timeout) {
      SWARN << "Poll for image " << name << " timeout";
      continue;
    }
    auto stamp = FromTimePoint(img->getTimestamp(CameraExposureOffset::MIDDLE));

    std_msgs::msg::Header header;
    header.frame_id = name;
    header.stamp = ToRosTime(stamp);

    auto img_cv = img->getFrame(false);

    auto msg = cv_bridge::CvImage(header, "mono8", img_cv).toImageMsg();
    pub->publish(*msg);
  }
}

} // namespace cityfly::sensor
