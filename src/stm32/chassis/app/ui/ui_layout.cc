#include "layout.hpp"

#include <Eigen/Dense>

#include "TaskScheduler.hpp"
#include "protocol_user.hpp"
#include "referee_user.hpp"

#include "shared_resources.hpp"
#include "global_objects.hpp"

namespace ui {

struct Point {
  int x, y;
};

struct Line {
  Line() = default;
  Line(int p1x, int p1y, int p2x, int p2y) : p1{p1x, p1y}, p2{p2x, p2y} {}
  Point p1;
  Point p2;
};

static uint8_t robot_id_ = 102;
static uint8_t ui_buf_[128];

void SetRobotId(uint8_t robot_id) { robot_id_ = robot_id; }

template <typename UiFigureType>
void PushBuffer(const UiFigureType &ui_figure) {
  const auto dataLen = rm::device::Referee0x301Prepare(ui_buf_, 0, ui_figure, robot_id_, robot_id_ + 256);
  SharedResources::GetInstance().no_dtcm->pm_link_serial.WriteAsync(ui_buf_, dataLen, [] {});
}

static Eigen::Vector2i CalcLiftPos(float pos) {
  using namespace rm::modules::angle_literals;
  constexpr float kTrackRadius = 390;
  const float mapped = rm::modules::Map(pos, 0.f, M_PIf / 2.f, (0_deg).rad(), (80_deg).rad());
  const float rotated = rm::modules::Wrap((80_deg).rad() - mapped - (40_deg).rad(), 0.f, 2.f * M_PIf);
  return {960 + std::cos(rotated) * kTrackRadius, 540 + std::sin(rotated) * kTrackRadius};
}

void InitRefPoints() {
  rm::device::UIFigure7 init;
  init.figure1.fillRound("r11", rm::device::UIFigure::Operation::Add, 1, rm::device::UIFigure::Color::Pink, 8, 741,
                         1080 - 400 + 36, 4);
  init.figure2.fillRound("r12", rm::device::UIFigure::Operation::Add, 1, rm::device::UIFigure::Color::Pink, 8, 1174,
                         1080 - 630 + 36, 4);
  init.figure3.fillRound("r21", rm::device::UIFigure::Operation::Add, 1, rm::device::UIFigure::Color::Pink, 8, 755,
                         1080 - 650 + 36, 4);
  init.figure4.fillRound("r22", rm::device::UIFigure::Operation::Add, 1, rm::device::UIFigure::Color::Pink, 8, 1175,
                         1080 - 392 + 36, 4);
  init.figure5.fillRound("r31", rm::device::UIFigure::Operation::Add, 1, rm::device::UIFigure::Color::Pink, 8, 957,
                         1080 - 268 + 36, 4);
  init.figure6.fillRound("r32", rm::device::UIFigure::Operation::Add, 1, rm::device::UIFigure::Color::Pink, 8, 964,
                         1080 - 753 + 36, 4);
  PushBuffer(init);
}

void InitRefLines() {
  rm::device::UIFigure5 init;
  init.figure1.fillLine("l1", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::White, 1, 741,
                        1080 - 400 + 36, 1174, 1080 - 630 + 36);
  init.figure2.fillLine("l2", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::White, 1, 755,
                        1080 - 650 + 36, 1175, 1080 - 392 + 36);
  init.figure3.fillLine("l3", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::White, 1, 957,
                        1080 - 268 + 36, 964, 1080 - 753 + 36);
  PushBuffer(init);
}

void DelRefPoints() {
  rm::device::UIFigure7 init;
  init.figure1.fillRound("r11", rm::device::UIFigure::Operation::Delete, 1, rm::device::UIFigure::Color::Pink, 0, 741,
                         1080 - 400, 4);
  init.figure2.fillRound("r12", rm::device::UIFigure::Operation::Delete, 1, rm::device::UIFigure::Color::Pink, 0, 1174,
                         1080 - 630, 4);
  init.figure3.fillRound("r21", rm::device::UIFigure::Operation::Delete, 1, rm::device::UIFigure::Color::Pink, 0, 753,
                         1080 - 400, 4);
  init.figure4.fillRound("r22", rm::device::UIFigure::Operation::Delete, 1, rm::device::UIFigure::Color::Pink, 0, 1175,
                         1080 - 392, 4);
  init.figure5.fillRound("r31", rm::device::UIFigure::Operation::Delete, 1, rm::device::UIFigure::Color::Pink, 0, 957,
                         1080 - 268, 4);
  init.figure6.fillRound("r32", rm::device::UIFigure::Operation::Delete, 1, rm::device::UIFigure::Color::Pink, 0, 964,
                         1080 - 753, 4);
  PushBuffer(init);
}

void DelRefLines() {
  rm::device::UIFigure5 init;
  init.figure1.fillLine("l1", rm::device::UIFigure::Operation::Delete, 0, rm::device::UIFigure::Color::White, 1, 741,
                        1080 - 400, 1174, 1080 - 630);
  init.figure2.fillLine("l2", rm::device::UIFigure::Operation::Delete, 0, rm::device::UIFigure::Color::White, 1, 753,
                        1080 - 400, 1175, 1080 - 392);
  init.figure3.fillLine("l3", rm::device::UIFigure::Operation::Delete, 0, rm::device::UIFigure::Color::White, 1, 957,
                        1080 - 268, 964, 1080 - 753);
  PushBuffer(init);
}

void HideCenter() {
  rm::device::UIFigure7 init;
  // 底盘圈背景
  init.figure1.fillArc("ceb", rm::device::UIFigure::Operation::Delete, 0, rm::device::UIFigure::Color::Black, 2, 960,
                       540, 0, 360, 80, 80);
  // 底盘朝向圆弧
  init.figure2.fillArc("coi", rm::device::UIFigure::Operation::Delete, 1, rm::device::UIFigure::Color::Pink, 10, 960,
                       540, 330, 30, 80, 80);
  // 左j3力矩
  init.figure3.fillFloat("ltq", rm::device::UIFigure::Operation::Delete, 0, rm::device::UIFigure::Color::Pink, 5, 750,
                         760, 30, 0.f);
  // 右j3力矩
  init.figure4.fillFloat("rtq", rm::device::UIFigure::Operation::Delete, 0, rm::device::UIFigure::Color::Pink, 5, 1070,
                         760, 30, 0.f);
  PushBuffer(init);
}

void Init1() {
  rm::device::UIFigure7 init;
  // 底盘圈背景
  init.figure1.fillArc("ceb", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::Black, 2, 960, 540,
                       0, 360, 80, 80);
  // 底盘朝向圆弧
  init.figure2.fillArc("coi", rm::device::UIFigure::Operation::Add, 1, rm::device::UIFigure::Color::Pink, 10, 960, 540,
                       330, 30, 80, 80);
  // 杆圆弧背景
  init.figure3.fillArc("lpb", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::White, 4, 960, 540,
                       45, 135, 390, 390);
  // 杆位置
  init.figure4.fillRound("lps", rm::device::UIFigure::Operation::Add, 1, rm::device::UIFigure::Color::Pink, 0, 0.f, 0.f,
                         4);
  // 杆目标位置
  init.figure5.fillRound("ltg", rm::device::UIFigure::Operation::Add, 1, rm::device::UIFigure::Color::Pink, 0, 0.f, 0.f,
                         4);
  // 左测距
  init.figure6.fillFloat("dl", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::Black, 5, 750, 200,
                         30, 0.f);
  // 右测距
  init.figure7.fillFloat("dr", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::Black, 5, 1070,
                         200, 30, 0.f);
  PushBuffer(init);
}

void Init2() {
  rm::device::UIFigure7 init;
  // 左j3力矩
  init.figure1.fillFloat("ltq", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::Pink, 5, 750, 760,
                         30, 0.f);
  // 右j3力矩
  init.figure2.fillFloat("rtq", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::Pink, 5, 1070,
                         760, 30, 0.f);
  // 左夹爪开
  init.figure3.fillRound("lgo", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::Black, 12, 760,
                         540, 6);
  // 右夹爪开
  init.figure4.fillRound("rgo", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::Black, 12, 1160,
                         540, 6);

  PushBuffer(init);
}

void FastUpdate() {
  rm::device::UIFigure7 edit;

  if (chassis->fsm.get_state_id() == fsm_chassis::StateId::kNoForce) {
    edit.figure1.fillArc("coi", rm::device::UIFigure::Operation::Edit, 1, rm::device::UIFigure::Color::Pink, 0, 960,
                         540, 330, 30, 80, 80);
    edit.figure2.fillArc("ceb", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Black, 2, 960,
                         540, 0, 360, 80, 80);

  } else {
    // const float chassis_ori =
    //     gimbal->resources.yaw_calib.RealToLogical(gimbal->resources.yaw_motor.pos_rad()) / M_PIf * 180.f;
    edit.figure1.fillArc("ceb", rm::device::UIFigure::Operation::Edit, 0,
                         chassis->resources.controller.moving_gently() ? rm::device::UIFigure::Color::Yellow
                                                                       : rm::device::UIFigure::Color::White,
                         2, 960, 540, 0, 360, 80, 80);
    // edit.figure2.fillArc("coi", rm::device::UIFigure::Operation::Edit, 1, rm::device::UIFigure::Color::Pink, 10, 960,
    //                      540, rm::modules::Wrap(chassis_ori - 30.f, 0.f, 360.f),
    //                      rm::modules::Wrap(chassis_ori + 30.f, 0.f, 360.f), 80, 80);
  }

  if (gimbal->fsm.get_state_id() == fsm_gimbal::StateId::kNoForce) {
    edit.figure3.fillArc("lpb", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Black, 4, 960,
                         540, 45, 135, 390, 390);
    edit.figure4.fillRound("lps", rm::device::UIFigure::Operation::Edit, 1, rm::device::UIFigure::Color::Pink, 0, 0.f,
                           0.f, 4);
    edit.figure5.fillRound("ltg", rm::device::UIFigure::Operation::Edit, 1, rm::device::UIFigure::Color::Pink, 0, 0.f,
                           0.f, 12);
  } else {
    edit.figure3.fillArc("lpb", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::White, 4, 960,
                         540, 45, 135, 390, 390);
    const float lift_target = gimbal->resources.lift_tl.current_position();
    const float lift_pos = gimbal->resources.camera_lift_calib.RealToLogical(gimbal->resources.lift_motor.pos());
    edit.figure4.fillRound("lps", rm::device::UIFigure::Operation::Edit, 1, rm::device::UIFigure::Color::Pink, 8,
                           CalcLiftPos(lift_pos).x(), CalcLiftPos(lift_pos).y(), 4);
    edit.figure5.fillRound("ltg", rm::device::UIFigure::Operation::Edit, 1, rm::device::UIFigure::Color::Pink, 2,
                           CalcLiftPos(lift_target).x(), CalcLiftPos(lift_target).y(), 12);
  }
  // 左j3力矩
  edit.figure6.fillFloat("ltq", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Pink, 5, 750,
                         760, 30, left_arm->resources.actuator.GetEfforts()[2] * 1000.f);
  // 右j3力矩
  edit.figure7.fillFloat("rtq", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Pink, 5, 1070,
                         760, 30, right_arm->resources.actuator.GetEfforts()[2] * 1000.f);

  PushBuffer(edit);
}

void SlowUpdate() {
  auto &shared = SharedResources::GetInstance();

  rm::device::UIFigure7 edit;
  // 左夹爪开
  if (left_arm->fsm.get_state_id() == fsm_gimbal::StateId::kNoForce) {
    edit.figure1.fillRound("lgo", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Black, 12, 760,
                           540, 6);
  } else {
    if (left_arm->resources.gripper_controller.needs_calibration()) {
      edit.figure1.fillRound("lgo", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Yellow, 12,
                             760, 540, 6);
    } else if (left_arm->resources.gripper_controller.state() == GripperController<>::kClose) {
      edit.figure1.fillRound("lgo", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Pink, 12,
                             760, 540, 6);
    } else if (left_arm->resources.gripper_controller.state() == GripperController<>::kOpen) {
      edit.figure1.fillRound("lgo", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::White, 12,
                             760, 540, 6);
    }
  }

  // 右夹爪开
  if (right_arm->fsm.get_state_id() == fsm_gimbal::StateId::kNoForce) {
    edit.figure2.fillRound("rgo", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Black, 12,
                           1160, 540, 6);
  } else {
    if (right_arm->resources.gripper_controller.needs_calibration()) {
      edit.figure2.fillRound("rgo", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Yellow, 12,
                             1160, 540, 6);
    } else if (right_arm->resources.gripper_controller.state() == GripperController<>::kClose) {
      edit.figure2.fillRound("rgo", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Pink, 12,
                             1160, 540, 6);
    } else if (right_arm->resources.gripper_controller.state() == GripperController<>::kOpen) {
      edit.figure2.fillRound("rgo", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::White, 12,
                             1160, 540, 6);
    }
  }

  if (shared.jetson.online_status() == rm::device::Device::kOk) {
    // if (shared.jetson.data().range_l_valid) {
    //   edit.figure6.fillFloat("dl", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Pink, 5, 750,
    //                          200, 30, shared.jetson.data().range_l * 1000.f);
    // } else {
    //   edit.figure6.fillFloat("dl", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::White, 5, 750,
    //                          200, 30, shared.jetson.data().range_l * 1000.f);
    // }
    // if (shared.jetson.data().range_r_valid) {
    //   edit.figure7.fillFloat("dr", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Pink, 5, 1070,
    //                          200, 30, shared.jetson.data().range_r * 1000.f);
    // } else {
    //   edit.figure7.fillFloat("dr", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::White, 5,
    //                          1070, 200, 30, shared.jetson.data().range_r * 1000.f);
    // }
  } else {
    edit.figure6.fillFloat("dl", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Black, 5, 750,
                           200, 30, 0.f);
    edit.figure7.fillFloat("dr", rm::device::UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Black, 5, 1070,
                           200, 30, 0.f);
  }

  PushBuffer(edit);
}

rm::device::UITask init_ref_points_task{InitRefPoints};
rm::device::UITask init_ref_lines_task{InitRefLines};
rm::device::UITask del_ref_points_task{DelRefPoints};
rm::device::UITask del_ref_lines_task{DelRefLines};
rm::device::UITask hide_center_task{HideCenter};

rm::device::UITask init_1_task{Init1};
rm::device::UITask init_2_task{Init2};

rm::device::UITask edit_task{FastUpdate, 15};
rm::device::UITask slow_update_task{SlowUpdate, 10};

void Init() {
  auto &shared = SharedResources::GetInstance();
  shared.ui_ts.addTaskStatic(&init_1_task);
  shared.ui_ts.addTaskStatic(&init_2_task);
  shared.ui_ts.addTask(&edit_task);
  shared.ui_ts.addTask(&slow_update_task);

  // 定义键盘键位
  using RC = rm::device::VT03::KeyboardKey;
  shared.kh.OnKeyDown(RC::kZ,
                      [&](const rm::modules::KeyEvent &e) {  // Z键刷新UI
                        shared.ui_ts.addTaskStatic(&init_1_task);
                        shared.ui_ts.addTaskStatic(&init_2_task);
                      });
  shared.kh.OnKeyDown(RC::kX,
                      [&](const rm::modules::KeyEvent &e) {  // X键画参考
                        shared.ui_ts.addTaskStatic(&init_ref_points_task);
                        shared.ui_ts.addTaskStatic(&init_ref_lines_task);
                        shared.ui_ts.addTaskStatic(&hide_center_task);
                      });
  shared.kh.OnKeyDown(RC::kC,
                      [&](const rm::modules::KeyEvent &e) {  // C键隐藏参考
                        shared.ui_ts.addTaskStatic(&del_ref_points_task);
                        shared.ui_ts.addTaskStatic(&del_ref_lines_task);
                        shared.ui_ts.addTaskStatic(&init_1_task);
                        shared.ui_ts.addTaskStatic(&init_2_task);
                      });
}

};  // namespace ui