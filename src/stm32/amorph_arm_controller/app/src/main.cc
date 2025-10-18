
#include "main.h"

#include <chrono>

#include <librm.hpp>

#include "mpc_solver.hpp"
#include "pbf_bridge.hpp"

extern "C" {

[[noreturn]] void AppMain(void) {
  rm::device::Referee<rm::device::RefereeRevision::kV170> ref;
  for (;;) {
    HAL_Delay(HAL_MAX_DELAY);
  }
}
}