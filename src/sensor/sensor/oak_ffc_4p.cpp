#include "cv_bridge/cv_bridge.h"
#include "depthai/utility/Clock.hpp"

#include "oak_ffc_4p.h"

namespace cityfly::sensor {

namespace {

int64_t ToStamp(
    const std::chrono::time_point<std::chrono::steady_clock,
                                  std::chrono::steady_clock::duration> &tp) {
  return tp.time_since_epoch().count();
}

} // namespace

OakFfc4p::OakFfc4p() : Node("oak_ffc_4p") {}

void OakFfc4p::getParameters() {}

void OakFfc4p::configure() {
  device_ = std::make_shared<Device>(constructPipeline());
}

void OakFfc4p::connect() {
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu", 1);

  // clang-format off
  img_pubs_["cam_front_l"] = node_->create_publisher<sensor_msgs::msg::Image>("cam_front_l", 1);
  img_pubs_["cam_front_r"] = node_->create_publisher<sensor_msgs::msg::Image>("cam_front_r", 1);
  img_pubs_["cam_down_l"]  = node_->create_publisher<sensor_msgs::msg::Image>("cam_down_l" , 1);
  img_pubs_["cam_down_r"]  = node_->create_publisher<sensor_msgs::msg::Image>("cam_down_r" , 1);
  // clang-format on
}

void OakFfc4p::runOnline() {
  threads_["imu"] = std::thread([this]() { pollImu(); });
  threads_["cam_front_l"] = std::thread([this]() { pollImage("cam_front_l"); });
  threads_["cam_front_r"] = std::thread([this]() { pollImage("cam_front_r"); });
  threads_["cam_down_l"] = std::thread([this]() { pollImage("cam_down_l"); });
  threads_["cam_down_r"] = std::thread([this]() { pollImage("cam_down_r"); });

  std::cout << "Driver running..." << std::endl;

  // wait for all thread to join
  for (auto &[_, thread] : threads_) {
    thread.join();
  }
}

void OakFfc4p::clean() {
  std::cout << "closing device" << std::endl;
  device_->close();
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
  auto imu = pipeline.create<node::IMU>();
  imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
  imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
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
  while (rclcpp::ok()) {
    auto imu_data = queue->get<IMUData>();
    for (const auto imu_pkt : imu_data->packets) {
      auto &acc = imu_pkt.acceleroMeter;
      auto &gyr = imu_pkt.gyroscope;

      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.frame_id = "imu";
      imu_msg.header.stamp.sec = acc.timestamp.sec;
      imu_msg.header.stamp.nanosec = acc.timestamp.nsec;

      imu_msg.linear_acceleration.x = acc.x;
      imu_msg.linear_acceleration.y = acc.y;
      imu_msg.linear_acceleration.z = acc.z;

      imu_msg.angular_velocity.x = gyr.x;
      imu_msg.angular_velocity.y = gyr.y;
      imu_msg.angular_velocity.z = gyr.z;

      imu_pub_->publish(imu_msg);
    }
  }
}

void OakFfc4p::pollImage(const std::string &name) {
  auto queue = device_->getOutputQueue(name, 2, false);
  auto pub = img_pubs_.at(name);

  while (rclcpp::ok()) {
    auto img = queue->get<ImgFrame>();

    int64_t stamp = ToStamp(img->getTimestamp());

    std_msgs::msg::Header header;
    header.frame_id = name;
    header.stamp.sec = stamp / 1000000000;
    header.stamp.nanosec = stamp % 1000000000;

    auto img_cv = img->getFrame(false);
    auto img_msg = cv_bridge::CvImage(header, "mono8", img_cv).toImageMsg();

    pub->publish(*img_msg);
  }
}

} // namespace cityfly::sensor
