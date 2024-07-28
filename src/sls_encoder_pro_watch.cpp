#include <Arduino.h>
#include "sh8601.h"
#include "TouchDrvCHSC5816.hpp"
#include "lvgl.h"
#include "SPI.h"
#include "ui/ui.h"
#include "pins_config.h"

TouchDrvCHSC5816 touch;
TouchDrvInterface *pTouch;

void CHSC5816_Initialization(void)
{
    TouchDrvCHSC5816 *pd1 = static_cast<TouchDrvCHSC5816 *>(pTouch);

    touch.setPins(TOUCH_RST, TOUCH_INT);
    if (!touch.begin(Wire, CHSC5816_SLAVE_ADDRESS, IIC_SDA, IIC_SCL))
    {
        Serial.println("Failed to find CHSC5816 - check your wiring!");
        while (1)
        {
            delay(1000);
        }
    }
}

void lv_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    lcd_PushColors(area->x1, area->y1, w, h, (uint16_t *)&color_p->full);
    lv_disp_flush_ready(disp);
}

void my_rounder(lv_disp_drv_t *disp_drv, lv_area_t *area)
{

    area->x1 = area->x1 & 0xFFFE;       // round down the refresh area x-axis start point to next even number - required for this display
    area->x2 = (area->x2 & 0xFFFE) + 1; // round down the refresh area x-axis end point to next even number - required for this display

    area->y1 = area->y1 & 0xFFFE;       // round down the refresh area y-axis start point to next even number - required for this display
    area->y2 = (area->y2 & 0xFFFE) + 1; // round down the refresh area y-axis end point to next even number - required for this display
}

static void lv_indev_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    int16_t Touch_x[2], Touch_y[2];
    uint8_t touchpad = touch.getPoint(Touch_x, Touch_y);

    if (touchpad > 0)
    {
        data->state = LV_INDEV_STATE_PR;

        data->point.x = Touch_x[0];
        data->point.y = Touch_y[0];

        // Serial.printf("X: %d   Y: %d\n", Touch_x[0], Touch_y[0]);
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
}

// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <numbers>

// #define SHOW_HUD 0

// #if SHOW_HUD
// #define SHOW_BORDER 1

// #if SHOW_BORDER
// #define BORDER_VERTICES 32
// #endif

// #define SHOW_COUNT 0
// #define SHOW_VELOCITY 1
// #define SHOW_VECTOR 1
// #define SHOW_VOLTAGE 0

// #endif

// constexpr size_t initial_balls = 125;

// struct ball
// {
//     lv_obj_t *obj_handle;

// #if SHOW_VECTOR
//     lv_obj_t *vector_handle;
//     lv_point_precise_t points[2];

//     void update_vector()
//     {
//         const float length = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
//         const float normal_x = velocity.x / length;
//         const float normal_y = velocity.y / length;

//         points[0].x = 15;
//         points[0].y = 15;
//         points[1].x = 15 + normal_x * 15;
//         points[1].y = 15 + normal_y * 15;

//         lv_line_set_points(vector_handle, points, 2);
//     }
// #endif

//     bool pressing;

// #if SHOW_VELOCITY
//     float label_value;
// #endif

//     float radius;

//     struct
//     {
//         float x;
//         float y;
//     } position;

//     struct
//     {
//         float x;
//         float y;
//     } velocity;
// };

// class sandbox
// {
// public:
//     sandbox() : m_width(390),
//                 m_height(390),
//                 m_radius(std::min<uint16_t>(m_width, m_height) / 2),
//                 m_group(lv_group_create()),
//                 m_screen(lv_scr_act())
//     {
//         lv_indev_t *indev = nullptr;

//         while ((indev = lv_indev_get_next(indev)))
//             if (lv_indev_get_type(indev) == LV_INDEV_TYPE_KEYPAD)
//                 lv_indev_set_group(indev, m_group);

//         lv_group_add_obj(m_group, m_screen);

//         auto on_key = [](lv_event_t *e)
//         {
//             auto app = static_cast<sandbox *>(lv_event_get_user_data(e));
//             auto key = lv_event_get_key(e);

//             switch (key)
//             {
//             case LV_KEY_UP:
//                 app->add_ball();
//                 break;
//             case LV_KEY_DOWN:
//                 app->remove_ball();
//                 break;
//             case LV_KEY_ENTER:
//                 app->reset_balls();
//                 break;
//             default:
//                 break;
//             }
//         };

//         lv_obj_add_event_cb(m_screen, on_key, LV_EVENT_KEY, this);
//     }

//     ~sandbox()
//     {
//         while (m_balls.size())
//             remove_ball();

//         lv_group_del(m_group);
//     }

//     void on_create()
//     {
//         lv_obj_clear_flag(m_screen, LV_OBJ_FLAG_SCROLLABLE);
//         lv_obj_set_style_bg_color(m_screen, lv_color_black(), LV_STATE_DEFAULT);

// #if SHOW_VOLTAGE
//         m_battery_voltage = lv_label_create(lv_layer_top());

//         lv_obj_set_style_text_color(m_battery_voltage, lv_color_white(), LV_STATE_DEFAULT);
//         lv_obj_align(m_battery_voltage, LV_ALIGN_BOTTOM_LEFT, 4, -22);

//         m_voltage_level = 3700;

//         for (size_t i = 0; i < 10; i++)
//             m_voltage_level = 0.9f * m_voltage_level + 0.1f * lv_rand(4100, 4200);
// #endif

// #if SHOW_COUNT
//         m_ball_count = lv_label_create(lv_layer_top());

//         lv_obj_set_style_text_color(m_ball_count, lv_color_white(), LV_STATE_DEFAULT);
//         lv_obj_align(m_ball_count, LV_ALIGN_BOTTOM_LEFT, 4, -4);
// #endif
//         m_balls.reserve(initial_balls);

//         reset_balls();
// #if SHOW_BORDER
//         {
//             auto line = lv_line_create(lv_layer_top());

//             lv_obj_set_style_line_width(line, 1, LV_STATE_DEFAULT);
//             lv_obj_set_style_line_color(line, lv_palette_main(LV_PALETTE_RED), LV_STATE_DEFAULT);
//             lv_obj_center(line);

//             static lv_point_precise_t points[BORDER_VERTICES + 1];
//             constexpr uint32_t point_count = sizeof(points) / sizeof(points[0]) - 1;

//             for (size_t i = 0; i < point_count; i++)
//             {
//                 points[i] = {
//                     .x = static_cast<decltype(lv_point_precise_t::y)>(m_radius + m_radius * std::cos(2 * std::numbers::pi * i / point_count)),
//                     .y = static_cast<decltype(lv_point_precise_t::x)>(m_radius + m_radius * std::sin(2 * std::numbers::pi * i / point_count)),
//                 };
//             }

//             points[point_count] = points[0];

//             lv_line_set_points(line, points, point_count + 1);
//         }
// #endif
// #if SHOW_HUD
//         auto hud_update = [](lv_timer_t *timer)
//         {
//             static_cast<sandbox *>(timer->user_data)->update_hud();
//         };

//         lv_timer_create(hud_update, 200, this);
// #endif
//     }

//     void on_update(float timestep)
//     {
//         for (const auto ball : m_balls)
//         {
//             if (!ball->pressing)
//             {
//                 for (const auto b : m_balls)
//                 {
//                     if (b == ball)
//                         continue;

//                     handle_collision(*b, *ball);
//                 }

//                 handle_borders(*ball);

//                 ball->position.x += ball->velocity.x * timestep;
//                 ball->position.y += ball->velocity.y * timestep;
//             }

//             lv_obj_set_pos(ball->obj_handle, ball->position.x - 15, ball->position.y - 15);
//         }
//     }

//     void add_ball()
//     {
//         auto b = static_cast<ball *>(lv_malloc(sizeof(ball)));

//         b->obj_handle = lv_image_create(m_screen);

//         b->pressing = false;
//         b->radius = 15;
//         b->position.x = (m_width / 2);
//         b->position.y = (m_height / 2);
//         b->velocity.x = lv_rand(50, 150);
//         b->velocity.y = lv_rand(50, 150);

//         if (lv_rand(0, 1))
//             b->velocity.x = -b->velocity.x;

//         if (lv_rand(0, 1))
//             b->velocity.y = -b->velocity.y;

//         auto on_pressed = [](lv_event_t *event)
//         {
//             auto b = static_cast<ball *>(lv_event_get_user_data(event));

//             b->pressing = true;
//             b->radius = 15 * 1.5;

//             lv_img_set_zoom(b->obj_handle, lv_img_get_zoom(b->obj_handle) * 1.5);
//         };

//         lv_obj_add_event_cb(b->obj_handle, on_pressed, LV_EVENT_PRESSED, b);

//         auto on_pressing = [](lv_event_t *event)
//         {
//             auto b = static_cast<ball *>(lv_event_get_user_data(event));

//             lv_indev_t *indev = lv_indev_active();

//             if (!indev)
//                 return;

//             lv_point_t point;
//             lv_point_t vector;

//             lv_indev_get_point(indev, &point);
//             lv_indev_get_vect(indev, &vector);

//             b->position.x = point.x;
//             b->position.y = point.y;
//             b->velocity.x = vector.x * 5;
//             b->velocity.y = vector.y * 5;
//         };

//         lv_obj_add_event_cb(b->obj_handle, on_pressing, LV_EVENT_PRESSING, b);

//         auto on_released = [](lv_event_t *event)
//         {
//             auto b = static_cast<ball *>(lv_event_get_user_data(event));

//             b->pressing = false;
//             b->radius = 15;

//             lv_img_set_zoom(b->obj_handle, LV_ZOOM_NONE);
//         };

//         lv_obj_add_event_cb(b->obj_handle, on_released, LV_EVENT_RELEASED, b);

//         lv_obj_add_flag(b->obj_handle, LV_OBJ_FLAG_CLICKABLE);

//         lv_obj_set_pos(b->obj_handle, b->position.x - b->radius, b->position.y - b->radius);
//         lv_obj_set_style_radius(b->obj_handle, b->radius, LV_STATE_DEFAULT);
//         lv_obj_set_style_border_width(b->obj_handle, 0, LV_STATE_DEFAULT);

// #if SHOW_VELOCITY
//         auto *label = lv_label_create(b->obj_handle);

//         lv_obj_set_style_text_color(label, lv_color_white(), LV_STATE_DEFAULT);
//         lv_obj_center(label);
//         lv_obj_set_user_data(b->obj_handle, label);

//         b->label_value = 0;
// #endif
// #if SHOW_VECTOR
//         b->vector_handle = lv_line_create(b->obj_handle);

//         lv_obj_set_style_line_width(b->vector_handle, 3, LV_STATE_DEFAULT);
//         lv_obj_set_style_line_color(b->vector_handle, lv_palette_main(LV_PALETTE_GREEN), LV_STATE_DEFAULT);
// #endif

//         char path[] = "F:/balls/ball_0.png";

//         path[14] = '0' + lv_rand(0, 7);

//         lv_image_set_src(b->obj_handle, path);

//         m_balls.push_back(b);
//     }

//     void remove_ball()
//     {
//         if (!m_balls.size())
//             return;

//         auto b = m_balls.back();

//         m_balls.pop_back();

//         lv_obj_del(b->obj_handle);
//         lv_free(b);
//     }

//     void reset_balls()
//     {
//         if (m_timer)
//             return;

//         auto timer_cb = [](lv_timer_t *timer)
//         {
//             auto app = static_cast<sandbox *>(timer->user_data);

//             if (app->m_balls.size() == initial_balls)
//             {
//                 lv_timer_del(app->m_timer);

//                 app->m_timer = nullptr;

//                 return;
//             }

//             if (app->m_balls.size() < initial_balls)
//                 app->add_ball();
//             else
//                 app->remove_ball();
//         };

//         m_timer = lv_timer_create(timer_cb, 50, this);
//     }

// #if SHOW_HUD
//     void update_hud()
//     {
// #if SHOW_VELOCITY
//         for (const auto ball : m_balls)
//         {
//             auto label = lv_obj_get_user_data(ball->obj_handle);

//             ball->label_value = std::sqrt(ball->velocity.x * ball->velocity.x + ball->velocity.y * ball->velocity.y);

//             lv_label_set_text_fmt(static_cast<lv_obj_t *>(label), "%.1f", ball->label_value);
//         }
// #endif
// #if SHOW_VECTOR
//         for (const auto ball : m_balls)
//             ball->update_vector();
// #endif
// #if SHOW_VOLTAGE
//         m_voltage_level = 0.9f * m_voltage_level + 0.1f * lv_rand(4100, 4200);

//         lv_label_set_text_fmt(m_battery_voltage, "Battery: %umv", m_voltage_level);
// #endif
// #if SHOW_COUNT
//         lv_label_set_text_fmt(m_ball_count, "Balls: %zu", m_balls.size());
// #endif
//     }
// #endif

//     void handle_collision(ball &b1, ball &b2)
//     {
//         const float dx = b2.position.x - b1.position.x;
//         const float dy = b2.position.y - b1.position.y;
//         const float distance_squared = dx * dx + dy * dy;
//         const float radius_squared = (b1.radius + b2.radius) * (b1.radius + b2.radius);

//         if (distance_squared >= radius_squared)
//             return;

//         const float distance = std::sqrt(distance_squared);
//         const float penetration_depth = (b1.radius + b2.radius) - distance;
//         const float normal_x = dx / distance;
//         const float normal_y = dy / distance;
//         const float resolution_distance = penetration_depth / 2;

//         b1.position.x -= normal_x * resolution_distance;
//         b1.position.y -= normal_y * resolution_distance;
//         b2.position.x += normal_x * resolution_distance;
//         b2.position.y += normal_y * resolution_distance;

//         const float relative_vx = b2.velocity.x - b1.velocity.x;
//         const float relative_vy = b2.velocity.y - b1.velocity.y;
//         const float v_along_normal = relative_vx * normal_x + relative_vy * normal_y;

//         if (v_along_normal > 0)
//             return;

//         const float j = -(1 + 0.99f) * v_along_normal / 2;
//         const float impulse_x = j * normal_x;
//         const float impulse_y = j * normal_y;

//         b1.velocity.x -= impulse_x;
//         b1.velocity.y -= impulse_y;
//         b2.velocity.x += impulse_x;
//         b2.velocity.y += impulse_y;
//     }

//     void handle_borders(ball &ball)
//     {
//         const float dx = m_radius - ball.position.x;
//         const float dy = m_radius - ball.position.y;
//         const float radius = m_radius - ball.radius;
//         const float distance_squared = dx * dx + dy * dy;
//         const float dot_product = ball.velocity.x * dx + ball.velocity.y * dy;

//         if (distance_squared <= radius * radius || dot_product > 0)
//             return;

//         const float distance = std::sqrt(distance_squared);
//         const float normal_x = dx / distance;
//         const float normal_y = dy / distance;
//         float normal_dot_product = dot_product / distance;

//         ball.velocity.x += -2 * normal_dot_product * normal_x;
//         ball.velocity.y += -2 * normal_dot_product * normal_y;
//     }

//     const uint16_t m_width;
//     const uint16_t m_height;
//     const uint16_t m_radius;

//     lv_group_t *m_group;
//     lv_obj_t *m_screen;

// #if SHOW_VOLTAGE
//     lv_obj_t *m_battery_voltage = nullptr;

//     uint32_t m_voltage_level = 0;
// #endif
// #if SHOW_COUNT
//     lv_obj_t *m_ball_count = nullptr;
// #endif
//     lv_timer_t *m_timer = nullptr;

//     std::vector<ball *> m_balls;
// };

// void init()
// {
//     auto sb = new sandbox();

//     auto on_create = [](void *userdata)
//     {
//         static_cast<sandbox *>(userdata)->on_create();
//     };

//     lv_async_call(on_create, sb);

//     auto on_update = [](lv_timer_t *timer)
//     {
//         static const auto app = static_cast<sandbox *>(timer->user_data);
//         static uint32_t previous_timestamp = 0;

//         auto now = lv_tick_get();
//         auto timestep = now - previous_timestamp;
//         previous_timestamp = now;

//         if (timestep < 0.0f)
//             timestep = 0.0f;

//         app->on_update(timestep / 1000.0f);
//     };

//     lv_timer_create(on_update, 16, sb);
// }

void setup()
{

    pinMode(LCD_VCI_EN, OUTPUT);
    digitalWrite(LCD_VCI_EN, HIGH); // enable display hardware

    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t *buf;

    Serial.begin(115200);
    CHSC5816_Initialization();
    sh8601_init();
    // lcd_setRotation(2); // 0-3
    lcd_brightness(200); // 0-255
    lv_init();
    buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * LVGL_LCD_BUF_SIZE, MALLOC_CAP_INTERNAL);
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_LCD_BUF_SIZE);
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;

    // The rounder callback function is supposed to be required for partial update with this display according to the datasheet but I've found it unecessary so far...
    // Perhaps the datasheet is out of date. Anyway, for the time being I've commented out the function.
    // disp_drv.rounder_cb = my_rounder;

    disp_drv.flush_cb = lv_disp_flush;
    disp_drv.draw_buf = &draw_buf;

    // The full_refresh setting is useful for testing if you run into problems with display artifacts caused by partial region updates.
    // It forces LVGL to render a whole screen at a time but is therefore (possibly) much slower than only updating the 'dirty areas' which need redrawing.
    // disp_drv.full_refresh = 1;

    lv_disp_drv_register(&disp_drv);
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lv_indev_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();
    // init();
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(lv_timer_handler()));
}
