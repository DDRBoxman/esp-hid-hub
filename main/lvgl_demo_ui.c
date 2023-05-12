/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/widgets/extra/meter.html#simple-meter

#include "lvgl.h"

#include "esp_timer.h"

extern lv_font_t lato_60; 

static lv_obj_t *meter;
static lv_obj_t * btn;
static lv_meter_indicator_t *indic;
lv_obj_t * label1;
//static lv_disp_rot_t rotation = LV_DISP_ROT_NONE;

void set_value(int32_t v)
{
    lv_meter_set_indicator_end_value(meter, indic, v);
}

static void btn_cb(lv_event_t * e)
{
    lv_disp_t *disp = lv_event_get_user_data(e);
    /*rotation++;
    if (rotation > LV_DISP_ROT_270) {
        rotation = LV_DISP_ROT_NONE;
    }
    lv_disp_set_rotation(disp, rotation);*/
}

static void back_event_handler(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    lv_obj_t * menu = lv_event_get_user_data(e);

    if(lv_menu_back_btn_is_root(menu, obj)) {
        lv_obj_t * mbox1 = lv_msgbox_create(NULL, "Hello", "Root back btn click.", NULL, true);
        lv_obj_center(mbox1);
    }
}

void my_timer(lv_timer_t * timer)
{
  
    uint64_t time = esp_timer_get_time() / 1000000LL;

    char buffer [100];

    snprintf ( buffer, 100, "%lld", time );

    lv_label_set_text(label1, buffer);
}


void example_lvgl_demo_ui(lv_disp_t *disp, lv_indev_t* indev)
{

    lv_obj_t *scr = lv_disp_get_scr_act(disp);

   label1 = lv_label_create(scr);
    lv_label_set_text(label1, "12:34");
    //lv_obj_set_width(label1, 150);  /*Set smaller width to make the lines wrap*/
    //lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, 0);
    //voidlv_obj_align_mid(label1, )

    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_text_font(&style, &lato_60); // <--- you have to enable other font sizes in menuconfig
    lv_obj_add_style(label1, &style, 0);

    lv_timer_t * timer = lv_timer_create(my_timer, 500,  NULL);

    /*lv_group_t * group = lv_group_create();
    //lv_group_add_obj(g, menu);
    lv_group_set_default(group);
    lv_indev_set_group(indev, group);

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t * menu = lv_menu_create(scr);
    lv_menu_set_mode_root_back_btn(menu, LV_MENU_ROOT_BACK_BTN_ENABLED);
    lv_obj_add_event_cb(menu, back_event_handler, LV_EVENT_CLICKED, menu);
    lv_obj_set_size(menu, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_center(menu);

    lv_obj_t* back_button = lv_menu_get_main_header_back_btn(menu);

    lv_group_add_obj(group, back_button);
    lv_obj_clear_flag(back_button, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(back_button, LV_OBJ_FLAG_SCROLL_ON_FOCUS);

    lv_obj_t * cont;
    lv_obj_t * label;

    // Create a sub page
    lv_obj_t * sub_page = lv_menu_page_create(menu, NULL);

    cont = lv_menu_cont_create(sub_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Hello, I am hiding here");

    //Create a main page
    lv_obj_t * main_page = lv_menu_page_create(menu, NULL);

    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Item 1");

    lv_group_add_obj(group, cont);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(cont, LV_OBJ_FLAG_SCROLL_ON_FOCUS);

    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Item 2");

    lv_group_add_obj(group, cont);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(cont, LV_OBJ_FLAG_SCROLL_ON_FOCUS);

    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Item 3 (Click me!)");
    lv_menu_set_load_page_event(menu, cont, sub_page);

    lv_group_add_obj(group, cont);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(cont, LV_OBJ_FLAG_SCROLL_ON_FOCUS);

    lv_menu_set_page(menu, main_page);*/
}