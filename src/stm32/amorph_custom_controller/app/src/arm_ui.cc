#include "arm_ui.hpp"

#include <lvgl.h>

#include "arm_renderer.hpp"
#include "globals.hpp"

extern Globals *globals;

namespace arm_ui {

static constexpr uint32_t ARM_LINE_WIDTH = 4;

/* LVGL objects */
static lv_obj_t *arm_container; /* 机械臂容器 */
static lv_obj_t *line_base;     /* 立柱 */
static lv_obj_t *line_arm1;     /* 后臂 */
static lv_obj_t *line_arm2;     /* 前臂 */
static lv_obj_t *label_j0;
static lv_obj_t *label_j1;
static lv_obj_t *label_j2;
static lv_obj_t *label_loop_time; /* MainLoop 执行时间 */

/* Point arrays – must stay alive as long as lines exist */
static lv_point_precise_t pts_base[2];
static lv_point_precise_t pts_arm1[2];
static lv_point_precise_t pts_arm2[2];

static ArmRenderer renderer;

static void set_line_style(lv_obj_t *line, lv_color_t color) {
  lv_obj_set_style_line_width(line, ARM_LINE_WIDTH, 0);
  lv_obj_set_style_line_color(line, color, 0);
  lv_obj_set_style_line_rounded(line, true, 0);
}

/* Timer callback – runs at ~25 Hz to update the arm drawing */
static void update_cb(lv_timer_t *timer) {
  (void)timer;

  const auto joint_pos = globals->actuator.GetPositions();
  const float j0 = joint_pos[0]; /* yaw */
  const float j1 = joint_pos[1]; /* pitch1 */
  const float j2 = joint_pos[2]; /* pitch2 */

  /* Compute projected line segments */
  const auto segments = renderer.Draw(j0, j1, j2);

  /* Update point arrays */
  pts_base[0] = {(lv_value_precise_t)segments[0].first.x, (lv_value_precise_t)segments[0].first.y};
  pts_base[1] = {(lv_value_precise_t)segments[0].second.x, (lv_value_precise_t)segments[0].second.y};

  pts_arm1[0] = {(lv_value_precise_t)segments[1].first.x, (lv_value_precise_t)segments[1].first.y};
  pts_arm1[1] = {(lv_value_precise_t)segments[1].second.x, (lv_value_precise_t)segments[1].second.y};

  pts_arm2[0] = {(lv_value_precise_t)segments[2].first.x, (lv_value_precise_t)segments[2].first.y};
  pts_arm2[1] = {(lv_value_precise_t)segments[2].second.x, (lv_value_precise_t)segments[2].second.y};

  lv_line_set_points_mutable(line_base, pts_base, 2);
  lv_line_set_points_mutable(line_arm1, pts_arm1, 2);
  lv_line_set_points_mutable(line_arm2, pts_arm2, 2);

  /* Update joint angle labels (display in degrees) */
  lv_label_set_text_fmt(label_j0, "J0:%+.1f", (float)(j0 * 180.0f / M_PI));
  lv_label_set_text_fmt(label_j1, "J1:%+.1f", (float)(j1 * 180.0f / M_PI));
  lv_label_set_text_fmt(label_j2, "J2:%+.1f", (float)(j2 * 180.0f / M_PI));

  /* Update MainLoop execution time label */
  lv_label_set_text_fmt(label_loop_time, "LP:%luus", (unsigned long)g_mainloop_time_us);
}

void create() {
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_white(), 0);

  /* ── Left: arm container (200 x 240, full height) ─── */
  arm_container = lv_obj_create(scr);
  lv_obj_set_size(arm_container, 200, 240);
  lv_obj_set_pos(arm_container, 0, 0);
  lv_obj_set_style_bg_opa(arm_container, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_opa(arm_container, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(arm_container, 0, 0);
  lv_obj_set_scrollbar_mode(arm_container, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(arm_container, LV_OBJ_FLAG_SCROLLABLE);

  line_base = lv_line_create(arm_container);
  line_arm1 = lv_line_create(arm_container);
  line_arm2 = lv_line_create(arm_container);

  set_line_style(line_base, lv_color_hex(0x808080));
  set_line_style(line_arm1, lv_color_hex(0x00BFFF));
  set_line_style(line_arm2, lv_color_hex(0xFF6347));

  /* ── Right: joint angle labels (x=210, stacked vertically, centered) ─── */
  label_j0 = lv_label_create(scr);
  label_j1 = lv_label_create(scr);
  label_j2 = lv_label_create(scr);

  lv_obj_set_style_text_font(label_j0, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_font(label_j1, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_font(label_j2, &lv_font_montserrat_14, 0);

  lv_obj_set_style_text_color(label_j0, lv_color_hex(0x808080), 0);
  lv_obj_set_style_text_color(label_j1, lv_color_hex(0x00BFFF), 0);
  lv_obj_set_style_text_color(label_j2, lv_color_hex(0xFF6347), 0);

  lv_label_set_text(label_j0, "J0:   0.0");
  lv_label_set_text(label_j1, "J1:   0.0");
  lv_label_set_text(label_j2, "J2:   0.0");

  /* Right column, vertically centered */
  lv_obj_set_pos(label_j0, 210, 90);
  lv_obj_set_pos(label_j1, 210, 110);
  lv_obj_set_pos(label_j2, 210, 130);

  /* ── MainLoop 执行时间 label ─── */
  label_loop_time = lv_label_create(scr);
  lv_obj_set_style_text_font(label_loop_time, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(label_loop_time, lv_color_hex(0x333333), 0);
  lv_label_set_text(label_loop_time, "LP:---us");
  lv_obj_set_pos(label_loop_time, 210, 155);

  /* Initial draw */
  update_cb(nullptr);

  /* Create periodic update timer – 40ms ≈ 25 Hz */
  lv_timer_create(update_cb, 40, nullptr);
}

}  // namespace arm_ui
