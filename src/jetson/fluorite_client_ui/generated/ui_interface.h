//
// Created by bismarckkk on 2025/3/22.
// Dynamic Edition
//

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "ui_types.h"

extern int ui_self_id;

// void print_message(const uint8_t* message, int length);

typedef void (*MsgTxCallback)(const uint8_t *message, int length);
extern MsgTxCallback send_message_fn;

#define SEND_MESSAGE(message, length) send_message_fn(message, length)

void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);
void ui_proc_delete_frame(ui_delete_frame_t *msg);

void ui_delete_layer(const uint8_t delete_type, const uint8_t layer);

void ui_scan_and_send(const ui_interface_figure_t *ui_now_figures, uint8_t *ui_dirty_figure,
                      const ui_interface_string_t *ui_now_strings, uint8_t *ui_dirty_string, int total_figures,
                      int total_strings);

#endif  // UI_INTERFACE_H
