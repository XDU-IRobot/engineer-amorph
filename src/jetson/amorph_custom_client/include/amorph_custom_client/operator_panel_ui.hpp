/**
 * @file operator_panel_ui.hpp
 * @brief OperatorPanel 的纯 UI 布局层
 *
 * 使用 Qt Designer .ui 文件定义布局，编译期由 uic 生成 Ui::OperatorPanelUi。
 * 本类持有生成的 Ui 对象并暴露控件指针，供 OperatorPanel 连接信号/更新数据。
 *
 * .ui 文件路径: ui/operator_panel.ui（可用 designer 直接编辑）
 */

#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLabel>

#include <memory>

// 前向声明 uic 生成的类（实现在 .cpp 中包含 "ui_operator_panel.h"）
namespace Ui {
class OperatorPanelUi;
}

namespace amorph {

class OperatorPanelUi {
 public:
  OperatorPanelUi();
  ~OperatorPanelUi();

  /// 在 parent 上构建全部 UI 控件（委托给 uic 生成的 setupUi）
  void setupUi(QWidget* parent);

  // ── 控件访问器 ──
  QPushButton* refreshBtn() const { return refresh_btn_; }

  // 识别结果
  QLabel* detectionLabel() const { return detection_label_; }

  // 左臂
  QPushButton* leftUpBtn() const { return left_up_btn_; }
  QPushButton* leftSetBtn() const { return left_set_btn_; }
  QPushButton* leftDownBtn() const { return left_down_btn_; }
  QLabel* leftTargetLabel() const { return left_target_label_; }
  QPushButton* planLeftBtn() const { return plan_left_btn_; }

  // 右臂
  QPushButton* rightUpBtn() const { return right_up_btn_; }
  QPushButton* rightSetBtn() const { return right_set_btn_; }
  QPushButton* rightDownBtn() const { return right_down_btn_; }
  QLabel* rightTargetLabel() const { return right_target_label_; }
  QPushButton* planRightBtn() const { return plan_right_btn_; }

  // 状态栏
  QLabel* statusLabel() const { return status_label_; }

 private:
  std::unique_ptr<Ui::OperatorPanelUi> generated_ui_;

  QPushButton* refresh_btn_{nullptr};
  QLabel* detection_label_{nullptr};

  QPushButton* left_up_btn_{nullptr};
  QPushButton* left_set_btn_{nullptr};
  QPushButton* left_down_btn_{nullptr};
  QLabel* left_target_label_{nullptr};
  QPushButton* plan_left_btn_{nullptr};

  QPushButton* right_up_btn_{nullptr};
  QPushButton* right_set_btn_{nullptr};
  QPushButton* right_down_btn_{nullptr};
  QLabel* right_target_label_{nullptr};
  QPushButton* plan_right_btn_{nullptr};

  QLabel* status_label_{nullptr};
};

}  // namespace amorph
