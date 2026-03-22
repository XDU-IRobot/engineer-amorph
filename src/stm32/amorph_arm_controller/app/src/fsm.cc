
#include "fsm.hpp"

// fsm&state instances
namespace fsm {
Arm arm;
namespace state {
static NoForce no_force;
static Manual manual;
static Idle idle;
static HostControl host_control;
static CustomCtrl custom_ctrl;
static CartesianCustomCtrl cartesian_custom_ctrl;
static LongJointCalibrating long_joint_calibrating;
static Fault fault;
}  // namespace state

void Init() {
  static etl::ifsm_state *state_list[] = {
      &state::no_force,                //
      &state::manual,                  //
      &state::idle,                    //
      &state::host_control,            //
      &state::custom_ctrl,             //
      &state::cartesian_custom_ctrl,   //
      &state::long_joint_calibrating,  //
      &state::fault,                   //
  };
  arm.set_states(state_list, std::size(state_list));
  arm.start();
}

}  // namespace fsm