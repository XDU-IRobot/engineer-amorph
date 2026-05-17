//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[3];
extern uint8_t ui_g_dirty_figure[3];

extern uint8_t ui_g_max_send_count[3];

#define ui_g_g_main_chassis_ori ((ui_interface_arc_t*)&(ui_g_now_figures[0]))
#define ui_g_g_main_cross_1 ((ui_interface_line_t*)&(ui_g_now_figures[1]))
#define ui_g_g_main_cross_2 ((ui_interface_line_t*)&(ui_g_now_figures[2]))


#define ui_g_g_main_chassis_ori_max_send_count (ui_g_max_send_count[0])
#define ui_g_g_main_cross_1_max_send_count (ui_g_max_send_count[1])
#define ui_g_g_main_cross_2_max_send_count (ui_g_max_send_count[2])


#ifdef MANUAL_DIRTY
#define ui_g_g_main_chassis_ori_dirty (ui_g_dirty_figure[0])
#define ui_g_g_main_cross_1_dirty (ui_g_dirty_figure[1])
#define ui_g_g_main_cross_2_dirty (ui_g_dirty_figure[2])

#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H
