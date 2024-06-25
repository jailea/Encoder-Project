// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Smartwatch

#include "ui.h"

// COMPONENT dategroup

lv_obj_t *ui_dategroup_create(lv_obj_t *comp_parent)
{

    lv_obj_t *cui_dategroup;
    cui_dategroup = lv_obj_create(comp_parent);
    lv_obj_set_width(cui_dategroup, 100);
    lv_obj_set_height(cui_dategroup, 82);
    lv_obj_set_x(cui_dategroup, 58);
    lv_obj_set_y(cui_dategroup, 53);
    lv_obj_set_align(cui_dategroup, LV_ALIGN_TOP_MID);
    lv_obj_set_flex_flow(cui_dategroup, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cui_dategroup, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(cui_dategroup, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(cui_dategroup, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(cui_dategroup, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *cui_day;
    cui_day = lv_label_create(cui_dategroup);
    lv_obj_set_width(cui_day, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(cui_day, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(cui_day, LV_ALIGN_CENTER);
    lv_label_set_text(cui_day, "MON");
    lv_obj_set_style_text_color(cui_day, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(cui_day, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(cui_day, &ui_font_Subtitle, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *cui_month;
    cui_month = lv_label_create(cui_dategroup);
    lv_obj_set_width(cui_month, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(cui_month, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(cui_month, LV_ALIGN_CENTER);
    lv_label_set_text(cui_month, "18 FEB");
    lv_obj_set_style_text_color(cui_month, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(cui_month, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(cui_month, &ui_font_Title, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *cui_year;
    cui_year = lv_label_create(cui_dategroup);
    lv_obj_set_width(cui_year, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(cui_year, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(cui_year, LV_ALIGN_CENTER);
    lv_label_set_text(cui_year, "2022");
    lv_obj_set_style_text_color(cui_year, lv_color_hex(0x6B6B6B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(cui_year, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(cui_year, &ui_font_Title, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t **children = lv_mem_alloc(sizeof(lv_obj_t *) * _UI_COMP_DATEGROUP_NUM);
    children[UI_COMP_DATEGROUP_DATEGROUP] = cui_dategroup;
    children[UI_COMP_DATEGROUP_DAY] = cui_day;
    children[UI_COMP_DATEGROUP_MONTH] = cui_month;
    children[UI_COMP_DATEGROUP_YEAR] = cui_year;
    lv_obj_add_event_cb(cui_dategroup, get_component_child_event_cb, LV_EVENT_GET_COMP_CHILD, children);
    lv_obj_add_event_cb(cui_dategroup, del_component_child_event_cb, LV_EVENT_DELETE, children);
    ui_comp_dategroup_create_hook(cui_dategroup);
    return cui_dategroup;
}
