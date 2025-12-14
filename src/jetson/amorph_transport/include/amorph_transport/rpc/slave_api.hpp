
#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <rpc_core.hpp>

#include "typedefs.hpp"

namespace slave {

extern std::shared_ptr<rpc_core::rpc> rpc;

void Init(const std::shared_ptr<rpc_core::rpc> &rpc_instance);

inline bool ExecuteTrajectory(JointTrajectory &traj) {
  std::atomic<bool> done{false};
  std::atomic<bool> ret{false};
  std::mutex mtx;
  std::condition_variable cv;

  rpc->cmd("execute_trajectory")
      ->msg(traj)
      ->timeout_ms(1000)
      ->timeout([&] { ret = false; })
      ->rsp([&](bool ret_val) { ret = ret_val; })
      ->finally([&] {
        done = true;
        cv.notify_one();
      })
      ->call();

  std::unique_lock<std::mutex> lock(mtx);
  cv.wait(lock, [&done]() { return done.load(); });
  return ret.load();
}

template <size_t N>
void GoTo(const JointPosition<N> &jp) {
  rpc->cmd("goto")->msg(jp)->call();
}
template <size_t N>
void GoTo(const JointPosition<N> &&jp) {
  rpc->cmd("goto")->msg(std::forward<const JointPosition<N>>(jp))->call();
}

}  // namespace slave