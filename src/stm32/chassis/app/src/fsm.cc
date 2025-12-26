
#include "fsm.hpp"

// fsm&state instances
namespace fsm {
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

}  // namespace fsm