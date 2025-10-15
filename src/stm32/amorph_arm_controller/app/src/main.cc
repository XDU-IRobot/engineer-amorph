
#include "main.h"

#include <chrono>

#include "mpc_solver.hpp"
#include "tinympc/tiny_api.hpp"

int runtime = 0;

extern "C" {

void AppMain(void) {
  int exitflag = 1;
  MpcSolver *solver = new MpcSolver;
  // solver->instance.work->Adyn = tinyMatrix::Random(12, 12);

  const auto start = HAL_GetTick();
  // for (;;) {
  for (int i = 0; i < 1000; ++i) {
    tiny_precompute_and_set_cache(solver->instance.cache, solver->instance.work->Adyn, solver->instance.work->Bdyn,
                                  solver->instance.work->Q.asDiagonal(), solver->instance.work->R.asDiagonal(),
                                  solver->instance.work->nx, solver->instance.work->nu, solver->instance.cache->rho, 0);
    tinyMatrix x0 = tinyMatrix::Zero(12, 1);
    tiny_set_x0(&solver->instance, x0);
    exitflag = tiny_solve(&solver->instance);
    HAL_GPIO_WritePin(CAMERA_TRIG_GPIO_Port, CAMERA_TRIG_Pin, exitflag ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
  runtime = HAL_GetTick() - start;

  for (;;) {
    HAL_Delay(HAL_MAX_DELAY);
  }
}
}