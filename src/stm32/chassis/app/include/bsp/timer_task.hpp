#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>

#include <etl/vector.h>
#include <etl/delegate.h>
#include <etl/forward_list.h>

#include "tim.h"

/**
 * @brief 基于 STM32 定时器的多频率任务调度器，内置 DWT 性能统计。
 *
 * 构造时自动从 HAL TIM 句柄推算基础中断频率（支持 STM32H7 APB1/APB2）。
 * 通过 AddTask() 注册不同频率的子任务，调度器自动均摊各任务的执行相位
 * 以避免计算堆叠，并借助 Cortex-M DWT 周期计数器统计每个任务的耗时。
 *
 * 性能统计读取示例：
 * @code
 *   for (size_t i = 0; i < sched.task_count(); ++i) {
 *     const auto& s = sched.task_stats(i);
 *     float avg_us = sched.cycles_to_us(s.ewma_cycles);
 *   }
 *   const auto& ts = sched.tick_stats();
 *   float load_pct = 100.f * ts.ewma_tick_cycles / sched.tick_budget_cycles();
 * @endcode
 */
class TimerTaskScheduler {
 public:
  /** @brief 单个任务的性能统计（由 ISR 更新，主线程只读）。所有时间单位均为 µs。 */
  struct TaskStats {
    const char* name{nullptr};  ///< 任务名（AddTask 可选传入）
    uint32_t exec_count{0};     ///< 累计执行次数
    float last_us{0.f};         ///< 最近一次执行耗时（µs）
    float max_us{0.f};          ///< 历史峰值耗时（µs）
    float min_us{1e9f};         ///< 历史最低耗时（µs），初始为大值，首次执行后覆盖
    float ewma_us{0.f};         ///< 指数加权移动平均耗时（µs，α = 1/16）
    uint32_t overrun_count{0};  ///< 单次执行超过 1 tick 周期（1/base_freq µs）的次数
  };

  /** @brief 调度器整体的 per-tick 统计（由 ISR 更新，主线程只读）。所有时间单位均为 µs。 */
  struct TickStats {
    uint32_t tick_count{0};   ///< 总 tick 数（自 Start() 或 reset_stats() 后）
    float last_tick_us{0.f};  ///< 本次 tick 中所有任务的总耗时（µs）
    float max_tick_us{0.f};   ///< 历史峰值 tick 总耗时（µs）
    float ewma_tick_us{0.f};  ///< tick 总耗时 EWMA（µs，α = 1/16）
  };

  explicit TimerTaskScheduler(TIM_HandleTypeDef* htim = nullptr) : htim_(htim) {
    if (htim_) {
      base_frequency_ = ComputeTimerFrequency(htim_);
      cpu_hz_ = HAL_RCC_GetHCLKFreq();
      HAL_TIM_RegisterCallback(htim_, HAL_TIM_PERIOD_ELAPSED_CB_ID, &TimerTaskScheduler::HALTimerCallback);
    }
    Register(this);
  }

  ~TimerTaskScheduler() { Unregister(this); }

  TimerTaskScheduler(const TimerTaskScheduler&) = delete;
  TimerTaskScheduler& operator=(const TimerTaskScheduler&) = delete;

  /**
   * @brief 注册以指定频率运行的任务（链式调用）。
   *
   * @param frequency 期望频率（Hz），须 ≤ base_frequency()
   * @param func      任务回调（etl::delegate，无堆分配）
   * @param name      可选任务名，用于统计输出（传 nullptr 则不命名）
   */
  TimerTaskScheduler& AddTask(float frequency, etl::delegate<void()> func, const char* name = nullptr) {
    uint32_t period = (base_frequency_ > 0.f) ? static_cast<uint32_t>(std::round(base_frequency_ / frequency)) : 1u;
    if (period == 0u) period = 1u;
    const uint32_t offset = (period > 1u) ? FindBestOffset(period) : 0u;
    tasks_.push_back({std::move(func), period, offset, {name}});
    return *this;
  }

  bool Start() {
    if (!htim_) return false;
    EnableDWT();
    return HAL_TIM_Base_Start_IT(htim_) == HAL_OK;
  }

  bool Stop() {
    if (!htim_) return false;
    return HAL_TIM_Base_Stop_IT(htim_) == HAL_OK;
  }

  /** @brief 手动覆盖基础频率（用于 TIMPRE=1 等特殊时钟配置）。 */
  void set_base_frequency(float hz) { base_frequency_ = hz; }

  /** @brief 任务数量。 */
  size_t task_count() const { return tasks_.size(); }

  /**
   * @brief 获取指定任务的性能统计（下标与 AddTask 调用顺序对应）。
   * @warning 在主线程读取时若 ISR 恰好写入，32-bit 字段单个读取是原子的；
   *          但多字段联合读取不保证一致性，仅供监控使用。
   */
  const TaskStats& task_stats(size_t index) const { return tasks_[index].stats; }

  /** @brief 获取调度器整体 per-tick 统计。 */
  const TickStats& tick_stats() const { return tick_stats_; }

  /** @brief CPU 主频（Hz）。 */
  uint32_t cpu_hz() const { return cpu_hz_; }

  /**
   * @brief 1 tick 的时间预算（µs），即 1 / base_frequency × 1e6。
   * tick_stats().ewma_tick_us / tick_period_us() 即为调度器 CPU 占用比。
   */
  float tick_period_us() const { return (base_frequency_ > 0.f) ? 1e6f / base_frequency_ : 0.f; }

  /** @brief 清零所有统计计数（不影响任务注册和运行）。 */
  void reset_stats() {
    for (auto& t : tasks_) {
      t.stats.exec_count = 0;
      t.stats.last_us = 0.f;
      t.stats.max_us = 0.f;
      t.stats.min_us = 1e9f;
      t.stats.ewma_us = 0.f;
      t.stats.overrun_count = 0;
    }
    tick_stats_ = {};
  }

  static void HandleIRQ(TIM_HandleTypeDef* htim) {
    for (auto* s : instances_) {
      if (s && s->htim_ == htim) s->Tick();
    }
  }

 private:
  struct TaskEntry {
    etl::delegate<void()> func;
    uint32_t period;  ///< 每隔多少 tick 执行一次
    uint32_t offset;  ///< 触发条件：tick_count % period == offset
    TaskStats stats;
  };

  TIM_HandleTypeDef* htim_{nullptr};
  float base_frequency_{0.f};
  uint32_t cpu_hz_{0};
  etl::vector<TaskEntry, 10> tasks_;
  volatile uint32_t tick_count_{0};
  TickStats tick_stats_{};

  static etl::forward_list<TimerTaskScheduler*, 8> instances_;

  void Tick() {
    const uint32_t tick_start = DWT->CYCCNT;
    // 预计算 cycles → µs 换算因子，避免在每次任务测量中重复除法
    const float to_us = (cpu_hz_ > 0u) ? 1e6f / static_cast<float>(cpu_hz_) : 0.f;
    const float period_us = tick_period_us();

    const uint32_t tick = tick_count_;
    tick_count_ = tick + 1u;

    for (auto& t : tasks_) {
      if (tick % t.period != t.offset) continue;

      const uint32_t t_start = DWT->CYCCNT;
      t.func();
      const float elapsed_us = static_cast<float>(DWT->CYCCNT - t_start) * to_us;

      TaskStats& s = t.stats;
      s.last_us = elapsed_us;
      s.exec_count++;
      if (elapsed_us > s.max_us) s.max_us = elapsed_us;
      if (elapsed_us < s.min_us) s.min_us = elapsed_us;
      // EWMA α = 1/16
      s.ewma_us = (s.exec_count == 1u) ? elapsed_us : s.ewma_us + (elapsed_us - s.ewma_us) * (1.f / 16.f);
      if (period_us > 0.f && elapsed_us > period_us) s.overrun_count++;
    }

    // 整体 tick 耗时
    const float tick_elapsed_us = static_cast<float>(DWT->CYCCNT - tick_start) * to_us;
    TickStats& ts = tick_stats_;
    ts.last_tick_us = tick_elapsed_us;
    ts.tick_count++;
    if (tick_elapsed_us > ts.max_tick_us) ts.max_tick_us = tick_elapsed_us;
    ts.ewma_tick_us =
        (ts.tick_count == 1u) ? tick_elapsed_us : ts.ewma_tick_us + (tick_elapsed_us - ts.ewma_tick_us) * (1.f / 16.f);
  }

  uint32_t FindBestOffset(uint32_t new_period) const {
    std::vector<uint32_t> score(new_period, 0u);
    for (uint32_t candidate = 0; candidate < new_period; ++candidate) {
      for (const auto& t : tasks_) {
        const uint32_t g = std::gcd(new_period, t.period);
        const uint32_t diff = (candidate % t.period + t.period - t.offset % t.period) % t.period;
        if (diff % g == 0u) ++score[candidate];
      }
    }
    return static_cast<uint32_t>(std::min_element(score.begin(), score.end()) - score.begin());
  }

  static float ComputeTimerFrequency(TIM_HandleTypeDef* htim) {
    if (!htim) return 0.f;

    RCC_ClkInitTypeDef clk{};
    uint32_t flash_latency{};
    HAL_RCC_GetClockConfig(&clk, &flash_latency);

    const bool is_apb2 = (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM15 ||
                          htim->Instance == TIM16 || htim->Instance == TIM17);

    uint32_t timer_clk_hz;
    if (is_apb2) {
      const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
      timer_clk_hz = (clk.APB2CLKDivider != RCC_APB2_DIV1) ? pclk2 * 2u : pclk2;
    } else {
      const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
      timer_clk_hz = (clk.APB1CLKDivider != RCC_APB1_DIV1) ? pclk1 * 2u : pclk1;
    }

    const uint64_t psc = static_cast<uint64_t>(htim->Init.Prescaler) + 1u;
    const uint64_t arr = static_cast<uint64_t>(htim->Init.Period) + 1u;
    return static_cast<float>(timer_clk_hz) / static_cast<float>(psc * arr);
  }

  static void EnableDWT() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能 trace
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // 启动周期计数器
  }

  static void Register(TimerTaskScheduler* s) {
    if (s) instances_.push_front(s);
  }
  static void Unregister(TimerTaskScheduler* s) {
    if (s) instances_.remove(s);
  }
  static void HALTimerCallback(TIM_HandleTypeDef* htim) { HandleIRQ(htim); }
};

inline etl::forward_list<TimerTaskScheduler*, 8> TimerTaskScheduler::instances_;
