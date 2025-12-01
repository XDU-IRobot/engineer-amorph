
#include "fsm.hpp"

// state instances

namespace fsm::state {
NoForce no_force;
Manual manual;
Idle idle;
Executing executing;
J6Calibrating j6_calibrating;
Fault fault;
}  // namespace fsm::state