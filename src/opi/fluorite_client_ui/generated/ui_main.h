//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_main_H
#define UI_main_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_main_now_figures[6];
extern uint8_t ui_main_dirty_figure[6];
extern ui_interface_string_t ui_main_now_strings[1];
extern uint8_t ui_main_dirty_string[1];

#define ui_main_dynamic_pitch2 ((ui_interface_line_t*)&(ui_main_now_figures[0]))
#define ui_main_dynamic_right_storage ((ui_interface_arc_t*)&(ui_main_now_figures[1]))
#define ui_main_dynamic_left_storage ((ui_interface_arc_t*)&(ui_main_now_figures[2]))
#define ui_main_dynamic_pitch1 ((ui_interface_line_t*)&(ui_main_now_figures[3]))
#define ui_main_dynamic_arm_pump ((ui_interface_arc_t*)&(ui_main_now_figures[4]))
#define ui_main_static_yaw ((ui_interface_line_t*)&(ui_main_now_figures[5]))

#define ui_main_dynamic_arm_status (&(ui_main_now_strings[0]))

#ifdef MANUAL_DIRTY
#define ui_main_dynamic_pitch2_dirty (ui_main_dirty_figure[0])
#define ui_main_dynamic_right_storage_dirty (ui_main_dirty_figure[1])
#define ui_main_dynamic_left_storage_dirty (ui_main_dirty_figure[2])
#define ui_main_dynamic_pitch1_dirty (ui_main_dirty_figure[3])
#define ui_main_dynamic_arm_pump_dirty (ui_main_dirty_figure[4])
#define ui_main_static_yaw_dirty (ui_main_dirty_figure[5])

#define ui_main_dynamic_arm_status_dirty (ui_main_dirty_string[0])
#endif

void ui_init_main();
void ui_update_main();

#endif // UI_main_H
