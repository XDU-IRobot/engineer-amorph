#pragma once

#include <librm.hpp>
#include <etl/message_router.h>

struct GlobalConfig {
  constexpr static rm::device::RefereeRevision kRefereeRev =
      rm::device::RefereeRevision::kNewV120;  ///< 使用的裁判系统通信协议版本

  constexpr static etl::message_router_id_t kLeftArmMessageRouterId = 1;
  constexpr static etl::message_router_id_t kRightArmMessageRouterId = 2;
  constexpr static etl::message_router_id_t kGimbalMessageRouterId = 3;
  constexpr static etl::message_router_id_t kChassisMessageRouterId = 4;
  constexpr static etl::message_router_id_t kOrchMessageRouterId = 5;
};
