// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Smartwatch

#include "ui.h"

void ui_weather_2_screen_init(void)
{
    ui_weather_2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_weather_2, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(ui_weather_2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_weather_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_bg5 = lv_img_create(ui_weather_2);
    lv_img_set_src(ui_bg5, &ui_img_bg3_png);
    lv_obj_set_width(ui_bg5, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_bg5, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_bg5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_bg5, LV_OBJ_FLAG_ADV_HITTEST);  /// Flags
    lv_obj_clear_flag(ui_bg5, LV_OBJ_FLAG_SCROLLABLE); /// Flags

    ui_weather_dots_group1 = lv_obj_create(ui_weather_2);
    lv_obj_set_width(ui_weather_dots_group1, 49);
    lv_obj_set_height(ui_weather_dots_group1, 18);
    lv_obj_set_x(ui_weather_dots_group1, 0);
    lv_obj_set_y(ui_weather_dots_group1, -14);
    lv_obj_set_align(ui_weather_dots_group1, LV_ALIGN_BOTTOM_MID);
    lv_obj_set_flex_flow(ui_weather_dots_group1, LV_FLEX_FLOW_ROW_REVERSE);
    lv_obj_set_flex_align(ui_weather_dots_group1, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_weather_dots_group1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(ui_weather_dots_group1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_weather_dots_group1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_weather_dots_group1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_weather_dots_group1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_dot3 = lv_obj_create(ui_weather_dots_group1);
    lv_obj_set_width(ui_dot3, 10);
    lv_obj_set_height(ui_dot3, 10);
    lv_obj_set_align(ui_dot3, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_dot3, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_radius(ui_dot3, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_dot4 = lv_obj_create(ui_weather_dots_group1);
    lv_obj_set_width(ui_dot4, 6);
    lv_obj_set_height(ui_dot4, 6);
    lv_obj_set_align(ui_dot4, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_dot4, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_radius(ui_dot4, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_dot4, lv_color_hex(0x676767), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_dot4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_content = lv_obj_create(ui_weather_2);
    lv_obj_set_width(ui_content, lv_pct(100));
    lv_obj_set_height(ui_content, lv_pct(100));
    lv_obj_set_align(ui_content, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_content, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scrollbar_mode(ui_content, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui_content, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_content, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_content, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_content, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_content, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_content, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_content, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_content, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_city_gruop_2 = ui_titlegroup_create(ui_content);
    lv_obj_set_x(ui_city_gruop_2, -42);
    lv_obj_set_y(ui_city_gruop_2, -303);
    lv_obj_set_align(ui_city_gruop_2, LV_ALIGN_BOTTOM_MID);

    lv_obj_set_x(ui_comp_get_child(ui_city_gruop_2, UI_COMP_TITLEGROUP_TITLE), 0);
    lv_obj_set_y(ui_comp_get_child(ui_city_gruop_2, UI_COMP_TITLEGROUP_TITLE), 0);
    lv_obj_set_align(ui_comp_get_child(ui_city_gruop_2, UI_COMP_TITLEGROUP_TITLE), LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_comp_get_child(ui_city_gruop_2, UI_COMP_TITLEGROUP_TITLE), "New York");
    lv_obj_set_style_text_font(ui_comp_get_child(ui_city_gruop_2, UI_COMP_TITLEGROUP_TITLE), &ui_font_Subtitle,
                               LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_label_set_text(ui_comp_get_child(ui_city_gruop_2, UI_COMP_TITLEGROUP_SUBTITLE), "03. 01. Monday");

    ui_todady_weather_content = lv_obj_create(ui_content);
    lv_obj_set_height(ui_todady_weather_content, 137);
    lv_obj_set_width(ui_todady_weather_content, lv_pct(100));
    lv_obj_set_x(ui_todady_weather_content, 0);
    lv_obj_set_y(ui_todady_weather_content, -12);
    lv_obj_set_align(ui_todady_weather_content, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_todady_weather_content, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_todady_weather_content, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_todady_weather_content, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_todady_weather_content, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_todady_weather_content, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_todady_weather_content, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_todady_weather_content, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_todady_weather_content, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_todady_weather_content, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_todady_weather_content, 30, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_todady_weather_content, lv_color_hex(0xFFFFFF), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_todady_weather_content, 30, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_todady_weather_content, 30, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_todady_weather_content, 30, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_todady_weather_content, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_todady_weather_content, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

    ui_today_weather_group = ui_todayweathergroup_create(ui_todady_weather_content);
    lv_obj_set_x(ui_today_weather_group, 0);
    lv_obj_set_y(ui_today_weather_group, 0);

    lv_img_set_src(ui_comp_get_child(ui_today_weather_group, UI_COMP_TODAYWEATHERGROUP_CLOUD_SUN),
                   &ui_img_weather_sun_cloud_png);

    lv_obj_set_style_text_color(ui_comp_get_child(ui_today_weather_group, UI_COMP_TODAYWEATHERGROUP_DEGREE_8),
                                lv_color_hex(0xED1E1E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_comp_get_child(ui_today_weather_group, UI_COMP_TODAYWEATHERGROUP_DEGREE_8), 255,
                              LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_today_weather_group1 = ui_todayweathergroup_create(ui_todady_weather_content);
    lv_obj_set_x(ui_today_weather_group1, 0);
    lv_obj_set_y(ui_today_weather_group1, 0);

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group1, UI_COMP_TODAYWEATHERGROUP_DEGREE_6), "26°");

    lv_img_set_src(ui_comp_get_child(ui_today_weather_group1, UI_COMP_TODAYWEATHERGROUP_CLOUD_SUN),
                   &ui_img_weather_cloud_png);

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group1, UI_COMP_TODAYWEATHERGROUP_DEGREE_8), "16:00");

    ui_today_weather_group2 = ui_todayweathergroup_create(ui_todady_weather_content);
    lv_obj_set_x(ui_today_weather_group2, 0);
    lv_obj_set_y(ui_today_weather_group2, 0);

    lv_img_set_src(ui_comp_get_child(ui_today_weather_group2, UI_COMP_TODAYWEATHERGROUP_CLOUD_SUN),
                   &ui_img_weather_sun_cloud_png);

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group2, UI_COMP_TODAYWEATHERGROUP_DEGREE_8), "17:00");

    ui_today_weather_group3 = ui_todayweathergroup_create(ui_todady_weather_content);
    lv_obj_set_x(ui_today_weather_group3, 0);
    lv_obj_set_y(ui_today_weather_group3, 0);

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group3, UI_COMP_TODAYWEATHERGROUP_DEGREE_6), "22°");

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group3, UI_COMP_TODAYWEATHERGROUP_DEGREE_8), "18:00");

    ui_today_weather_group4 = ui_todayweathergroup_create(ui_todady_weather_content);
    lv_obj_set_x(ui_today_weather_group4, 0);
    lv_obj_set_y(ui_today_weather_group4, 0);

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group4, UI_COMP_TODAYWEATHERGROUP_DEGREE_6), "21°");

    lv_img_set_src(ui_comp_get_child(ui_today_weather_group4, UI_COMP_TODAYWEATHERGROUP_CLOUD_SUN),
                   &ui_img_weather_cloud_fog_png);

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group4, UI_COMP_TODAYWEATHERGROUP_DEGREE_8), "19:00");

    ui_today_weather_group5 = ui_todayweathergroup_create(ui_todady_weather_content);
    lv_obj_set_x(ui_today_weather_group5, 0);
    lv_obj_set_y(ui_today_weather_group5, 0);

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group5, UI_COMP_TODAYWEATHERGROUP_DEGREE_6), "20°");

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group5, UI_COMP_TODAYWEATHERGROUP_DEGREE_8), "20:00");

    ui_today_weather_group6 = ui_todayweathergroup_create(ui_todady_weather_content);
    lv_obj_set_x(ui_today_weather_group6, 0);
    lv_obj_set_y(ui_today_weather_group6, 0);

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group6, UI_COMP_TODAYWEATHERGROUP_DEGREE_6), "20°");

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group6, UI_COMP_TODAYWEATHERGROUP_DEGREE_8), "21:00");

    ui_today_weather_group7 = ui_todayweathergroup_create(ui_todady_weather_content);
    lv_obj_set_x(ui_today_weather_group7, 0);
    lv_obj_set_y(ui_today_weather_group7, 0);

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group7, UI_COMP_TODAYWEATHERGROUP_DEGREE_6), "19°");

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group7, UI_COMP_TODAYWEATHERGROUP_DEGREE_8), "22:00");

    ui_today_weather_group8 = ui_todayweathergroup_create(ui_todady_weather_content);
    lv_obj_set_x(ui_today_weather_group8, 0);
    lv_obj_set_y(ui_today_weather_group8, 0);

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group8, UI_COMP_TODAYWEATHERGROUP_DEGREE_6), "17°");

    lv_label_set_text(ui_comp_get_child(ui_today_weather_group8, UI_COMP_TODAYWEATHERGROUP_DEGREE_8), "23:00");

    ui_days_forecast = lv_label_create(ui_content);
    lv_obj_set_width(ui_days_forecast, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_days_forecast, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_days_forecast, -87);
    lv_obj_set_y(ui_days_forecast, 62);
    lv_obj_set_align(ui_days_forecast, LV_ALIGN_CENTER);
    lv_label_set_text(ui_days_forecast, "10 days forecast");
    lv_obj_add_flag(ui_days_forecast, LV_OBJ_FLAG_IGNORE_LAYOUT); /// Flags
    lv_obj_set_style_text_font(ui_days_forecast, &ui_font_Title, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_week_weather_group = lv_obj_create(ui_content);
    lv_obj_set_width(ui_week_weather_group, 310);
    lv_obj_set_height(ui_week_weather_group, LV_SIZE_CONTENT); /// 205
    lv_obj_set_x(ui_week_weather_group, 0);
    lv_obj_set_y(ui_week_weather_group, 16);
    lv_obj_set_align(ui_week_weather_group, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_week_weather_group, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_week_weather_group, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_week_weather_group, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(ui_week_weather_group, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_week_weather_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_week_weather_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_week_weather_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_week_weather_group, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_week_weather_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_forecast_group = ui_forecastgroup_create(ui_week_weather_group);
    lv_obj_set_x(ui_forecast_group, 9);
    lv_obj_set_y(ui_forecast_group, 79);

    lv_label_set_text(ui_comp_get_child(ui_forecast_group, UI_COMP_FORECASTGROUP_DAY1), "Tue");

    lv_label_set_text(ui_comp_get_child(ui_forecast_group, UI_COMP_FORECASTGROUP_DEGREE_GROUP_DAYTIME), "24°");

    lv_label_set_text(ui_comp_get_child(ui_forecast_group, UI_COMP_FORECASTGROUP_DEGREE_GROUP_NIGHTTIME), "17°");

    lv_img_set_src(ui_comp_get_child(ui_forecast_group, UI_COMP_FORECASTGROUP_CLOUD_SUN), &ui_img_weather_cloud_fog_png);

    ui_forecast_group1 = ui_forecastgroup_create(ui_week_weather_group);
    lv_obj_set_x(ui_forecast_group1, 9);
    lv_obj_set_y(ui_forecast_group1, 79);

    lv_label_set_text(ui_comp_get_child(ui_forecast_group1, UI_COMP_FORECASTGROUP_DAY1), "Wed");

    lv_label_set_text(ui_comp_get_child(ui_forecast_group1, UI_COMP_FORECASTGROUP_DEGREE_GROUP_NIGHTTIME), "19°");

    ui_forecast_group2 = ui_forecastgroup_create(ui_week_weather_group);
    lv_obj_set_x(ui_forecast_group2, 9);
    lv_obj_set_y(ui_forecast_group2, 79);

    lv_label_set_text(ui_comp_get_child(ui_forecast_group2, UI_COMP_FORECASTGROUP_DAY1), "Thu");

    lv_label_set_text(ui_comp_get_child(ui_forecast_group2, UI_COMP_FORECASTGROUP_DEGREE_GROUP_DAYTIME), "26°");

    lv_label_set_text(ui_comp_get_child(ui_forecast_group2, UI_COMP_FORECASTGROUP_DEGREE_GROUP_NIGHTTIME), "19°");

    lv_img_set_src(ui_comp_get_child(ui_forecast_group2, UI_COMP_FORECASTGROUP_CLOUD_SUN), &ui_img_weather_cloud_png);

    ui_forecast_group3 = ui_forecastgroup_create(ui_week_weather_group);
    lv_obj_set_x(ui_forecast_group3, 9);
    lv_obj_set_y(ui_forecast_group3, 79);

    lv_label_set_text(ui_comp_get_child(ui_forecast_group3, UI_COMP_FORECASTGROUP_DAY1), "Fri");

    lv_label_set_text(ui_comp_get_child(ui_forecast_group3, UI_COMP_FORECASTGROUP_DEGREE_GROUP_NIGHTTIME), "16°");

    lv_img_set_src(ui_comp_get_child(ui_forecast_group3, UI_COMP_FORECASTGROUP_CLOUD_SUN), &ui_img_weather_cloud_fog_png);

    ui_forecast_group4 = ui_forecastgroup_create(ui_week_weather_group);
    lv_obj_set_x(ui_forecast_group4, 9);
    lv_obj_set_y(ui_forecast_group4, 79);

    lv_label_set_text(ui_comp_get_child(ui_forecast_group4, UI_COMP_FORECASTGROUP_DAY1), "Sat");

    lv_label_set_text(ui_comp_get_child(ui_forecast_group4, UI_COMP_FORECASTGROUP_DEGREE_GROUP_DAYTIME), "27°");

    lv_label_set_text(ui_comp_get_child(ui_forecast_group4, UI_COMP_FORECASTGROUP_DEGREE_GROUP_NIGHTTIME), "20°");

    ui_forecast_group5 = ui_forecastgroup_create(ui_week_weather_group);
    lv_obj_set_x(ui_forecast_group5, 9);
    lv_obj_set_y(ui_forecast_group5, 79);

    lv_label_set_text(ui_comp_get_child(ui_forecast_group5, UI_COMP_FORECASTGROUP_DAY1), "Sun");

    lv_label_set_text(ui_comp_get_child(ui_forecast_group5, UI_COMP_FORECASTGROUP_DEGREE_GROUP_DAYTIME), "29°");

    lv_label_set_text(ui_comp_get_child(ui_forecast_group5, UI_COMP_FORECASTGROUP_DEGREE_GROUP_NIGHTTIME), "21°");

    ui_space = lv_obj_create(ui_week_weather_group);
    lv_obj_set_height(ui_space, 50);
    lv_obj_set_width(ui_space, lv_pct(100));
    lv_obj_set_align(ui_space, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_space, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(ui_space, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_space, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_button_down4 = ui_buttondown_create(ui_weather_2);
    lv_obj_set_x(ui_button_down4, 146);
    lv_obj_set_y(ui_button_down4, 102);

    lv_img_set_src(ui_comp_get_child(ui_button_down4, UI_COMP_BUTTONDOWN_BUTTON_DOWN_ICON), &ui_img_house_png);

    lv_obj_add_event_cb(ui_button_down4, ui_event_button_down4_buttondown, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_weather_2, ui_event_weather_2, LV_EVENT_ALL, NULL);
}
