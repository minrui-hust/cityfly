#include "common/node.h"

namespace cityfly::sensor {
using namespace common;

struct OakFfc4p : public Node {
  OakFfc4p();

protected:
  void configure() override;
  void preRun() override;
  void postRun() override;
  void clean() override;
};

} // namespace cityfly::sensor
