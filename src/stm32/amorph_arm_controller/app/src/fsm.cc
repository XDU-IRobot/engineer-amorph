
#include "fsm.hpp"

// fsm&state instances

namespace fsm {
Arm arm;
namespace state {
static NoForce no_force;
static Manual manual;
static Idle idle;
static Executing executing;
static J6Calibrating j6_calibrating;
static Fault fault;
}  // namespace state

void Init() {
  static etl::ifsm_state *state_list[] = {
      &state::no_force,        //
      &state::manual,          //
      &state::idle,            //
      &state::executing,       //
      &state::j6_calibrating,  //
      &state::fault,           //
  };
  arm.set_states(state_list, std::size(state_list));
  arm.start();
}

}  // namespace fsm