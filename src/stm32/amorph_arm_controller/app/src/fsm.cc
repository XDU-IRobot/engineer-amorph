
#include "fsm.hpp"

// state instances

namespace fsm::state {
NoForce no_force;
Manual manual;
Idle idle;
Executing executing;
Fault fault;
}  // namespace fsm::state