
#include "fsm_chassis.hpp"
#include "fsm_gimbal.hpp"

// fsm&state instances
namespace fsm_chassis {
Chassis chassis;
namespace state {
static NoForce no_force;
static Normal normal;
static Fault fault;
}  // namespace state

void Init() {
  static etl::ifsm_state *state_list[] = {
      &state::no_force,  //
      &state::normal,    //
      &state::fault,     //
  };
  chassis.set_states(state_list, std::size(state_list));
  chassis.start();
}

}  // namespace fsm_chassis

namespace fsm_gimbal {
Gimbal gimbal;
namespace state {
static NoForce no_force;
static Normal normal;
static Manual manual;
static Fault fault;
}  // namespace state

void Init() {
  static etl::ifsm_state *state_list[] = {
      &state::no_force,  //
      &state::normal,    //
      &state::manual,    //
      &state::fault,     //
  };
  gimbal.set_states(state_list, std::size(state_list));
  gimbal.start();
}
}  // namespace fsm_gimbal