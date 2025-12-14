#pragma once

#include <algorithm>
#include <chrono>
#include <functional>

#include <etl/vector.h>

/**
 * @brief   基于轮询的定时器，用于在裸机环境下实现延时调用功能
 *
 * 该类通过轮询的方式检查预定的任务是否到期，并在到期时执行相应的回调函数。
 * 适用于没有操作系统支持的嵌入式系统，可以实现类似于定时器中断的功能。
 */
class PollingTimer {
 public:
  using Clock = std::chrono::steady_clock;

  /**
   * @brief 轮询一次，执行所有到期的任务
   */
  void Poll() {
    auto now = Clock::now();
    // A simple way to handle re-entrant calls to DeferredCall is to copy tasks to be run
    // and then clear them from the main list. This avoids iterator invalidation issues.
    etl::vector<std::function<void()>, 10> to_run;

    etl::erase_if(tasks_, [&](const Task& task) {
      if (task.due_time <= now) {
        to_run.push_back(task.func);
        return true;  // Remove from tasks_
      }
      return false;
    });

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

  etl::vector<Task, 10> tasks_;
};