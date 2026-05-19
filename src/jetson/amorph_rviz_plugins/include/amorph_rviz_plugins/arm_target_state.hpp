/**
 * @file arm_target_state.hpp
 * @brief 单臂目标状态 —— 纯数据 + 简单状态机
 *
 * 不依赖 Qt，不依赖 ROS，仅持有：
 * - 目标能量单元 frame_id
 * - lift 方向
 * - 是否处于选点模式
 *
 * OperatorPanel 持有两个实例（左/右），通过此类的 API 修改状态。
 */

#pragma once

#include <string>
#include <functional>

namespace amorph {

/// 选点模式
enum class SelectionMode { None, Left, Right };

/// 单臂的目标分配状态
class ArmTargetState {
 public:
  /// 当目标被成功分配时的回调（frame_id 为新分配的目标）
  using AssignCallback = std::function<void(const std::string& frame_id)>;

  void setAssignCallback(AssignCallback cb) { assign_cb_ = std::move(cb); }

  // ── 查询 ──
  const std::string& targetFrameId() const { return target_frame_id_; }
  bool hasTarget() const { return !target_frame_id_.empty(); }
  bool liftUpward() const { return lift_upward_; }

  // ── 修改方向（不改变目标） ──
  void setLiftUpward(bool upward) { lift_upward_ = upward; }

  // ── 分配目标 ──
  void assignTarget(const std::string& frame_id) {
    target_frame_id_ = frame_id;
    if (assign_cb_) assign_cb_(frame_id);
  }

  // ── 清除目标 ──
  void clearTarget() { target_frame_id_.clear(); }

 private:
  std::string target_frame_id_;
  bool lift_upward_ = false;
  AssignCallback assign_cb_;
};

}  // namespace amorph
