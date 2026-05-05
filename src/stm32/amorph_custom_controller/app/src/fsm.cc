#include "fsm.hpp"

namespace fsm {
CcArm cc_arm;
namespace state {
NoForce no_force;
LockPosition lock_position;
Enabled enabled;
}  // namespace state

void Init() {
  static etl::ifsm_state *state_list[] = {
      &state::no_force,       //
      &state::lock_position,  //
      &state::enabled,        //
  };
  cc_arm.set_states(state_list, std::size(state_list));
  cc_arm.start();
}
}  // namespace fsm