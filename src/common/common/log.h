#pragma once

#include <iostream>
#include <string>

#include "g3log/g3log.hpp"
#include "g3log/loglevels.hpp"
#include "g3log/logmessage.hpp"
#include "g3log/logworker.hpp"
#include "g3sinks/LogRotate.h"

namespace cityfly::common {

namespace {
inline std::string Formatting(const g3::LogMessage &msg) {
  return msg.timestamp() + " [" + msg.level() + "]: ";
}
} // namespace

struct LogStdout {
  // Linux xterm color
  // http://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
  enum FG_Color { YELLOW = 33, RED = 31, GREEN = 32, WHITE = 37 };

  FG_Color GetColor(const LEVELS level) const {
    if (level.value == WARNING.value) {
      return YELLOW;
    }
    if (level.value == DEBUG.value) {
      return GREEN;
    }
    if (g3::internal::wasFatal(level)) {
      return RED;
    }

    return WHITE;
  }

  void reset() {
    std::cout << "\033[" << WHITE << "m" << " " << "\033[m" << std::endl;
  }

  void ReceiveLogMessage(g3::LogMessageMover logEntry) {
    auto level = logEntry.get()._level;
    auto color = GetColor(level);

    std::cout << "\033[" << color << "m" << logEntry.get().toString(&Formatting)
              << "\033[m" << std::flush;
  }
};

inline void InitLogging(const std::string &prefix, const std::string &dir,
                        const LEVELS &level = INFO,
                        const bool to_stdout = false) {
  g3::logLevel(level);

  static std::unique_ptr<g3::LogWorker> logger{
      g3::LogWorker::createLogWorker()};

  logger->addSink(std::make_unique<LogRotate>(prefix, dir), &LogRotate::save);

  if (to_stdout) {
    logger->addSink(std::make_unique<LogStdout>(),
                    &LogStdout::ReceiveLogMessage);
  }

  g3::initializeLogging(logger.get());
}

inline void FinitLogging() { g3::internal::shutDownLogging(); }

// stream version
#define SDEBUG LOG(DEBUG)
#define SDEBUG_IF(condition) LOG_IF(DEBUG, condition)

#define SINFO LOG(INFO)
#define SINFO_IF(condition) LOG_IF(INFO, condition)

#define SWARN LOG(WARNING)
#define SWARN_IF(condition) LOG_IF(WARNING, condition)

#define SERROR LOG(FATAL)
#define SERROR_IF(condition) LOG_IF(FATAL, condition)

#define SCHECK CHECK

// formatted version
#define FDEBUG(message, ...) LOGF(DEBUG, message, ##__VA_ARGS__)
#define FDEBUG_IF(condition, message, ...)                                     \
  LOGF_IF(DEBUG, (condition), message, ##__VA_ARGS__)

#define FINFO(message, ...) LOGF(INFO, message, ##__VA_ARGS__)
#define FINFO_IF(condition, message, ...)                                      \
  LOGF_IF(INFO, (condition), message, ##__VA_ARGS__)

#define FWARN(message, ...) LOGF(WARNING, message, ##__VA_ARGS__)
#define FWARN_IF(condition, message, ...)                                      \
  LOGF_IF(WARNING, (condition), message, ##__VA_ARGS__)

#define FERROR(message, ...) LOGF(FATAL, message, ##__VA_ARGS__)
#define FERROR_IF(condition, message, ...)                                     \
  LOGF_IF(FATAL, (condition), message, ##__VA_ARGS__)

#define FCHECK(condition, message, ...)                                        \
  CHECKF((condition), message, ##__VA_ARGS__)

} // namespace cityfly::common
