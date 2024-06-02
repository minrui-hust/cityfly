#pragma once

#include <cstdint>

#include <rclcpp/time.hpp>

namespace cityfly::common {

using Stamp = int64_t; // nanosec
using Time = Stamp;    // for backward compatbility

constexpr int64_t MILLIS_PER_SECOND = 1000;

constexpr int64_t MICROS_PER_MILLI = 1000;
constexpr int64_t MICROS_PER_SECOND = 1000000;

constexpr int64_t NANOS_PER_MICRO = 1000;
constexpr int64_t NANOS_PER_MILLI = 1000000;
constexpr int64_t NANOS_PER_SECOND = 1000000000;

inline rclcpp::Time ToRosTime(const Stamp &stamp) {
  return rclcpp::Time(stamp / NANOS_PER_SECOND, stamp % NANOS_PER_SECOND);
}

} // namespace cityfly::common
