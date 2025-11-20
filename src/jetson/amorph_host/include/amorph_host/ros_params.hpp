
#pragma once

#include <string>

namespace amorph {

struct RosParams {
  struct {
    std::string can_port;
    int mosi_frame_id;
    int miso_frame_id;
  } rpc;
};

}  // namespace amorph