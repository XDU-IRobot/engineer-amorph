
#pragma once

#include <librm.hpp>

#include "rpc/typedefs.hpp"

#include "mpc_solver.hpp"
#include "dynamics.hpp"
#include "pid_motor_controller.hpp"

class ArmController {
 public:
  ArmController() = default;

  void Init() {}
  void ExecuteTrajectory(const JointTrajectory &traj) {}

 private:
};