/**
 * @file operator_panel_ui.cpp
 * @brief OperatorPanelUi 实现 —— 委托给 uic 生成的 Ui::OperatorPanelUi
 *
 * .ui 文件位于 ui/operator_panel.ui，可在 Qt Designer 中直接编辑。
 */

#include "amorph_rviz_plugins/operator_panel_ui.hpp"

// uic 生成的头文件（由 qt5_wrap_ui 在 build 目录中生成）
#include "ui_operator_panel.h"

namespace amorph {

OperatorPanelUi::OperatorPanelUi() = default;

OperatorPanelUi::~OperatorPanelUi() = default;

void OperatorPanelUi::setupUi(QWidget* parent) {
  generated_ui_ = std::make_unique<Ui::OperatorPanelUi>();
  generated_ui_->setupUi(parent);

  // 从生成的 Ui 对象中缓存控件指针
  refresh_btn_       = generated_ui_->refreshBtn;
  detection_label_   = generated_ui_->detectionLabel;

  left_up_btn_       = generated_ui_->leftUpBtn;
  left_set_btn_      = generated_ui_->leftSetBtn;
  left_down_btn_     = generated_ui_->leftDownBtn;
  left_target_label_ = generated_ui_->leftTargetLabel;
  plan_left_btn_     = generated_ui_->planLeftBtn;

  right_up_btn_       = generated_ui_->rightUpBtn;
  right_set_btn_      = generated_ui_->rightSetBtn;
  right_down_btn_     = generated_ui_->rightDownBtn;
  right_target_label_ = generated_ui_->rightTargetLabel;
  plan_right_btn_     = generated_ui_->planRightBtn;

  status_label_      = generated_ui_->statusLabel;
}

}  // namespace amorph
