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
  img_pubs_["cam_front_l"] = node_->create_publisher<sensor_msgs::msg::Image>("cam_front_l", 1);
  img_pubs_["cam_front_r"] = node_->create_publisher<sensor_msgs::msg::Image>("cam_front_r", 1);
  img_pubs_["cam_down_l"]  = node_->create_publisher<sensor_msgs::msg::Image>("cam_down_l" , 1);
  img_pubs_["cam_down_r"]  = node_->create_publisher<sensor_msgs::msg::Image>("cam_down_r" , 1);
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
  createCamera(pipeline, "cam_front_l", CameraBoardSocket::CAM_A, CameraControl::FrameSyncMode::OUTPUT);
  createCamera(pipeline, "cam_front_r", CameraBoardSocket::CAM_D, CameraControl::FrameSyncMode::INPUT );
  createCamera(pipeline, "cam_down_l" , CameraBoardSocket::CAM_B, CameraControl::FrameSyncMode::OFF   );
  createCamera(pipeline, "cam_down_r" , CameraBoardSocket::CAM_C, CameraControl::FrameSyncMode::OFF   );
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
  Vector3 acc_interp;
  int imu_seq = -1;

  while (rclcpp::ok()) {
    auto imu_data = queue->get<IMUData>();
    for (const auto imu_pkt : imu_data->packets) {
      const auto &acc = imu_pkt.acceleroMeter;
      const auto &gyr = imu_pkt.gyroscope;
      auto acc_stamp = FromTimePoint(acc.getTimestamp());
      auto gyr_stamp = FromTimePoint(gyr.getTimestamp());

      acc_interp_.put(acc_stamp, {acc.x, acc.y, acc.z});

      if (gyr.sequence != imu_seq) {
        imu_seq = gyr.sequence;

        acc_interp_.get(gyr_stamp, &acc_interp);

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.frame_id = "imu";
        imu_msg.header.stamp = ToRosTime(gyr_stamp);

        imu_msg.linear_acceleration.x = acc_interp.x();
        imu_msg.linear_acceleration.y = acc_interp.y();
        imu_msg.linear_acceleration.z = acc_interp.z();

        imu_msg.angular_velocity.x = gyr.x;
        imu_msg.angular_velocity.y = gyr.y;
        imu_msg.angular_velocity.z = gyr.z;

        imu_pub_->publish(imu_msg);
      } else {
        SDEBUG << "Drop duplicated imu pkt";
      }
    }
  }
}

void OakFfc4p::pollImage(const std::string &name) {
  auto queue = device_->getOutputQueue(name, 2, false);
  auto pub = img_pubs_.at(name);

  while (rclcpp::ok()) {
    auto img = queue->get<ImgFrame>();
    auto stamp = FromTimePoint(img->getTimestamp(CameraExposureOffset::MIDDLE));

    std_msgs::msg::Header header;
    header.frame_id = name;
    header.stamp = ToRosTime(stamp);

    auto img_cv = img->getFrame(false);
    auto img_msg = cv_bridge::CvImage(header, "mono8", img_cv).toImageMsg();

    pub->publish(*img_msg);
  }
}

} // namespace cityfly::sensor
