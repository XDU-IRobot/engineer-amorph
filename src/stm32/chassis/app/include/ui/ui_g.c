//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 3
#define TOTAL_STRING 0

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];

uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    1,
    1,
    1,
};

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, NULL, NULL, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g() {
    ui_g_g_main_chassis_ori->figure_type = 4;
    ui_g_g_main_chassis_ori->operate_type = 1;
    ui_g_g_main_chassis_ori->layer = 0;
    ui_g_g_main_chassis_ori->color = 2;
    ui_g_g_main_chassis_ori->start_x = 957;
    ui_g_g_main_chassis_ori->start_y = 537;
    ui_g_g_main_chassis_ori->width = 7;
    ui_g_g_main_chassis_ori->start_angle = 330;
    ui_g_g_main_chassis_ori->end_angle = 30;
    ui_g_g_main_chassis_ori->rx = 80;
    ui_g_g_main_chassis_ori->ry = 80;

    ui_g_g_main_cross_1->figure_type = 0;
    ui_g_g_main_cross_1->operate_type = 1;
    ui_g_g_main_cross_1->layer = 0;
    ui_g_g_main_cross_1->color = 0;
    ui_g_g_main_cross_1->start_x = 905;
    ui_g_g_main_cross_1->start_y = 487;
    ui_g_g_main_cross_1->width = 5;
    ui_g_g_main_cross_1->end_x = 1014;
    ui_g_g_main_cross_1->end_y = 591;

    ui_g_g_main_cross_2->figure_type = 0;
    ui_g_g_main_cross_2->operate_type = 1;
    ui_g_g_main_cross_2->layer = 0;
    ui_g_g_main_cross_2->color = 0;
    ui_g_g_main_cross_2->start_x = 1012;
    ui_g_g_main_cross_2->start_y = 487;
    ui_g_g_main_cross_2->width = 5;
    ui_g_g_main_cross_2->end_x = 906;
    ui_g_g_main_cross_2->end_y = 588;

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = ui_g_max_send_count[i];
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
