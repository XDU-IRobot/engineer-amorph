
#pragma once

#include <librm.hpp>

#include "rpc/typedefs.hpp"

#include "dynamics.hpp"
#include "trajectory_tracker.hpp"

class ArmController {
 public:
  ArmController() = default;

  void Init() {}
  void ExecuteTrajectory(const JointTrajectory &traj) {}

 private:
};