#pragma once

#include <librm.hpp>

struct GlobalConfig {
  constexpr static rm::device::RefereeRevision kRefereeRev =
      rm::device::RefereeRevision::kNewV120;  ///< 使用的裁判系统通信协议版本
};
