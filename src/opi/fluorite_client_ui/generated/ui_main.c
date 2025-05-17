//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_main.h"

#define TOTAL_FIGURE 6
#define TOTAL_STRING 1

ui_interface_figure_t ui_main_now_figures[TOTAL_FIGURE];
uint8_t ui_main_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_main_now_strings[TOTAL_STRING];
uint8_t ui_main_dirty_string[TOTAL_STRING];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_main_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_main_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND()                                                                                                \
  ui_scan_and_send(ui_main_now_figures, ui_main_dirty_figure, ui_main_now_strings, ui_main_dirty_string, TOTAL_FIGURE, \
                   TOTAL_STRING)

void ui_init_main() {
  ui_main_dynamic_pitch2->figure_type = 0;
  ui_main_dynamic_pitch2->operate_type = 1;
  ui_main_dynamic_pitch2->layer = 0;
  ui_main_dynamic_pitch2->color = 3;
  ui_main_dynamic_pitch2->start_x = 1096;
  ui_main_dynamic_pitch2->start_y = 590;
  ui_main_dynamic_pitch2->width = 5;
  ui_main_dynamic_pitch2->end_x = 1230;
  ui_main_dynamic_pitch2->end_y = 736;

  ui_main_dynamic_right_storage->figure_type = 4;
  ui_main_dynamic_right_storage->operate_type = 1;
  ui_main_dynamic_right_storage->layer = 0;
  ui_main_dynamic_right_storage->color = 7;
  ui_main_dynamic_right_storage->start_x = 955;
  ui_main_dynamic_right_storage->start_y = 535;
  ui_main_dynamic_right_storage->width = 10;
  ui_main_dynamic_right_storage->start_angle = 0;
  ui_main_dynamic_right_storage->end_angle = 90;
  ui_main_dynamic_right_storage->rx = 76;
  ui_main_dynamic_right_storage->ry = 76;

  ui_main_dynamic_left_storage->figure_type = 4;
  ui_main_dynamic_left_storage->operate_type = 1;
  ui_main_dynamic_left_storage->layer = 0;
  ui_main_dynamic_left_storage->color = 7;
  ui_main_dynamic_left_storage->start_x = 955;
  ui_main_dynamic_left_storage->start_y = 535;
  ui_main_dynamic_left_storage->width = 10;
  ui_main_dynamic_left_storage->start_angle = 270;
  ui_main_dynamic_left_storage->end_angle = 0;
  ui_main_dynamic_left_storage->rx = 76;
  ui_main_dynamic_left_storage->ry = 76;

  ui_main_dynamic_pitch1->figure_type = 0;
  ui_main_dynamic_pitch1->operate_type = 1;
  ui_main_dynamic_pitch1->layer = 0;
  ui_main_dynamic_pitch1->color = 6;
  ui_main_dynamic_pitch1->start_x = 959;
  ui_main_dynamic_pitch1->start_y = 589;
  ui_main_dynamic_pitch1->width = 5;
  ui_main_dynamic_pitch1->end_x = 1102;
  ui_main_dynamic_pitch1->end_y = 589;

  ui_main_dynamic_arm_pump->figure_type = 4;
  ui_main_dynamic_arm_pump->operate_type = 1;
  ui_main_dynamic_arm_pump->layer = 0;
  ui_main_dynamic_arm_pump->color = 2;
  ui_main_dynamic_arm_pump->start_x = 955;
  ui_main_dynamic_arm_pump->start_y = 535;
  ui_main_dynamic_arm_pump->width = 10;
  ui_main_dynamic_arm_pump->start_angle = 90;
  ui_main_dynamic_arm_pump->end_angle = 270;
  ui_main_dynamic_arm_pump->rx = 76;
  ui_main_dynamic_arm_pump->ry = 76;

  ui_main_static_yaw->figure_type = 0;
  ui_main_static_yaw->operate_type = 1;
  ui_main_static_yaw->layer = 0;
  ui_main_static_yaw->color = 0;
  ui_main_static_yaw->start_x = 960;
  ui_main_static_yaw->start_y = 540;
  ui_main_static_yaw->width = 5;
  ui_main_static_yaw->end_x = 960;
  ui_main_static_yaw->end_y = 593;

  ui_main_dynamic_arm_status->figure_type = 7;
  ui_main_dynamic_arm_status->operate_type = 1;
  ui_main_dynamic_arm_status->layer = 0;
  ui_main_dynamic_arm_status->color = 2;
  ui_main_dynamic_arm_status->start_x = 845;
  ui_main_dynamic_arm_status->start_y = 432;
  ui_main_dynamic_arm_status->width = 4;
  ui_main_dynamic_arm_status->font_size = 40;
  ui_main_dynamic_arm_status->str_length = 6;
  strcpy(ui_main_dynamic_arm_status->string, "MANUAL");

  uint32_t idx = 0;
  for (int i = 0; i < TOTAL_FIGURE; i++) {
    ui_main_now_figures[i].figure_name[2] = idx & 0xFF;
    ui_main_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
    ui_main_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
    ui_main_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
    ui_main_last_figures[i] = ui_main_now_figures[i];
#endif
    ui_main_dirty_figure[i] = 1;
    idx++;
  }
  for (int i = 0; i < TOTAL_STRING; i++) {
    ui_main_now_strings[i].figure_name[2] = idx & 0xFF;
    ui_main_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
    ui_main_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
    ui_main_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
    ui_main_last_strings[i] = ui_main_now_strings[i];
#endif
    ui_main_dirty_string[i] = 1;
    idx++;
  }

  SCAN_AND_SEND();

  for (int i = 0; i < TOTAL_FIGURE; i++) {
    ui_main_now_figures[i].operate_type = 2;
  }
  for (int i = 0; i < TOTAL_STRING; i++) {
    ui_main_now_strings[i].operate_type = 2;
  }
}

void ui_update_main() {
#ifndef MANUAL_DIRTY
  for (int i = 0; i < TOTAL_FIGURE; i++) {
    if (memcmp(&ui_main_now_figures[i], &ui_main_last_figures[i], sizeof(ui_main_now_figures[i])) != 0) {
      ui_main_dirty_figure[i] = 1;
      ui_main_last_figures[i] = ui_main_now_figures[i];
    }
  }
  for (int i = 0; i < TOTAL_STRING; i++) {
    if (memcmp(&ui_main_now_strings[i], &ui_main_last_strings[i], sizeof(ui_main_now_strings[i])) != 0) {
      ui_main_dirty_string[i] = 1;
      ui_main_last_strings[i] = ui_main_now_strings[i];
    }
  }
#endif
  SCAN_AND_SEND();
}
