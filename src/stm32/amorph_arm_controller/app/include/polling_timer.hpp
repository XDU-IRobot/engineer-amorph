#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <vector>

class PollingTimer {
 public:
  using Clock = std::chrono::steady_clock;

  void Poll() {
    auto now = Clock::now();
    // A simple way to handle re-entrant calls to DeferredCall is to copy tasks to be run
    // and then clear them from the main list. This avoids iterator invalidation issues.
    std::vector<std::function<void()>> to_run;

    tasks_.erase(std::remove_if(tasks_.begin(), tasks_.end(),
                                [&](const Task& task) {
                                  if (task.due_time <= now) {
                                    to_run.push_back(task.func);
                                    return true;  // Remove from tasks_
                                  }
                                  return false;
                                }),
                 tasks_.end());

    for (auto& func : to_run) {
      func();
    }
  }

  void DeferredCall(std::function<void()> fn, std::chrono::milliseconds delay) {
    tasks_.push_back({Clock::now() + delay, std::move(fn)});
  }

 private:
  struct Task {
    Clock::time_point due_time;
    std::function<void()> func;
  };

  std::vector<Task> tasks_;
};