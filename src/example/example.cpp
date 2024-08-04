#include "core/application.h"

#include <vector>
#include <cmath>
#include <algorithm>

constexpr size_t initial_balls = 15;

struct ball
{
    lv_obj_t *obj_handle;

    bool pressing;
    float radius;

    struct
    {
        float x;
        float y;
    } position;

    struct
    {
        float x;
        float y;
    } velocity;
};

class example : public application
{
public:
    example() : m_width(390),
                m_height(390),
                m_radius(std::min<uint16_t>(m_width, m_height) / 2),
                m_screen(lv_scr_act())
    {
    }

    ~example()
    {
        while (m_balls.size())
            remove_ball();
    }

    void on_create() override
    {
        lv_obj_clear_flag(m_screen, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(m_screen, lv_color_black(), LV_STATE_DEFAULT);

        m_balls.reserve(initial_balls);

        reset_balls();
    }

    void on_update(float timestep) override
    {
        for (const auto ball : m_balls)
        {
            if (!ball->pressing)
            {
                for (const auto b : m_balls)
                {
                    if (b == ball)
                        continue;

                    handle_collision(*b, *ball);
                }

                handle_borders(*ball);

                ball->position.x += ball->velocity.x * timestep;
                ball->position.y += ball->velocity.y * timestep;
            }

            lv_obj_set_pos(ball->obj_handle, ball->position.x - 15, ball->position.y - 15);
        }
    }

    void add_ball()
    {
        auto b = static_cast<ball *>(lv_malloc(sizeof(ball)));

        b->obj_handle = lv_image_create(m_screen);

        b->pressing = false;
        b->radius = 15;
        b->position.x = (m_width / 2);
        b->position.y = (m_height / 2);
        b->velocity.x = lv_rand(50, 150);
        b->velocity.y = lv_rand(50, 150);

        if (lv_rand(0, 1))
            b->velocity.x = -b->velocity.x;

        if (lv_rand(0, 1))
            b->velocity.y = -b->velocity.y;

        auto on_pressed = [](lv_event_t *event)
        {
            auto b = static_cast<ball *>(lv_event_get_user_data(event));

            b->pressing = true;
            b->radius = 15 * 1.5;

            lv_img_set_zoom(b->obj_handle, lv_img_get_zoom(b->obj_handle) * 1.5);
        };

        lv_obj_add_event_cb(b->obj_handle, on_pressed, LV_EVENT_PRESSED, b);

        auto on_pressing = [](lv_event_t *event)
        {
            auto b = static_cast<ball *>(lv_event_get_user_data(event));

            lv_indev_t *indev = lv_indev_active();

            if (!indev)
                return;

            lv_point_t point;
            lv_point_t vector;

            lv_indev_get_point(indev, &point);
            lv_indev_get_vect(indev, &vector);

            b->position.x = point.x;
            b->position.y = point.y;
            b->velocity.x = vector.x * 5;
            b->velocity.y = vector.y * 5;
        };

        lv_obj_add_event_cb(b->obj_handle, on_pressing, LV_EVENT_PRESSING, b);

        auto on_released = [](lv_event_t *event)
        {
            auto b = static_cast<ball *>(lv_event_get_user_data(event));

            b->pressing = false;
            b->radius = 15;

            lv_img_set_zoom(b->obj_handle, LV_ZOOM_NONE);
        };

        lv_obj_add_event_cb(b->obj_handle, on_released, LV_EVENT_RELEASED, b);

        lv_obj_add_flag(b->obj_handle, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_set_pos(b->obj_handle, b->position.x - b->radius, b->position.y - b->radius);
        lv_obj_set_style_radius(b->obj_handle, b->radius, LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(b->obj_handle, 0, LV_STATE_DEFAULT);

        char path[] = "F:/balls/ball_0.png";

        path[14] = '0' + lv_rand(0, 7);

        lv_image_set_src(b->obj_handle, path);

        m_balls.push_back(b);
    }

    void remove_ball()
    {
        if (!m_balls.size())
            return;

        auto b = m_balls.back();

        m_balls.pop_back();

        lv_obj_del(b->obj_handle);
        lv_free(b);
    }

    void reset_balls()
    {
        if (m_timer)
            return;

        auto timer_cb = [](lv_timer_t *timer)
        {
            auto app = static_cast<example *>(timer->user_data);

            if (app->m_balls.size() == initial_balls)
            {
                lv_timer_del(app->m_timer);

                app->m_timer = nullptr;

                return;
            }

            if (app->m_balls.size() < initial_balls)
                app->add_ball();
            else
                app->remove_ball();
        };

        m_timer = lv_timer_create(timer_cb, 50, this);
    }

    void handle_collision(ball &b1, ball &b2)
    {
        const float dx = b2.position.x - b1.position.x;
        const float dy = b2.position.y - b1.position.y;
        const float distance_squared = dx * dx + dy * dy;
        const float radius_squared = (b1.radius + b2.radius) * (b1.radius + b2.radius);

        if (distance_squared >= radius_squared)
            return;

        const float distance = std::sqrt(distance_squared);
        const float penetration_depth = (b1.radius + b2.radius) - distance;
        const float normal_x = dx / distance;
        const float normal_y = dy / distance;
        const float resolution_distance = penetration_depth / 2;

        b1.position.x -= normal_x * resolution_distance;
        b1.position.y -= normal_y * resolution_distance;
        b2.position.x += normal_x * resolution_distance;
        b2.position.y += normal_y * resolution_distance;

        const float relative_vx = b2.velocity.x - b1.velocity.x;
        const float relative_vy = b2.velocity.y - b1.velocity.y;
        const float v_along_normal = relative_vx * normal_x + relative_vy * normal_y;

        if (v_along_normal > 0)
            return;

        const float j = -(1 + 0.99f) * v_along_normal / 2;
        const float impulse_x = j * normal_x;
        const float impulse_y = j * normal_y;

        b1.velocity.x -= impulse_x;
        b1.velocity.y -= impulse_y;
        b2.velocity.x += impulse_x;
        b2.velocity.y += impulse_y;
    }

    void handle_borders(ball &ball)
    {
        const float dx = m_radius - ball.position.x;
        const float dy = m_radius - ball.position.y;
        const float radius = m_radius - ball.radius;
        const float distance_squared = dx * dx + dy * dy;
        const float dot_product = ball.velocity.x * dx + ball.velocity.y * dy;

        if (distance_squared <= radius * radius || dot_product > 0)
            return;

        const float distance = std::sqrt(distance_squared);
        const float normal_x = dx / distance;
        const float normal_y = dy / distance;
        float normal_dot_product = dot_product / distance;

        ball.velocity.x += -2 * normal_dot_product * normal_x;
        ball.velocity.y += -2 * normal_dot_product * normal_y;
    }

    const uint16_t m_width;
    const uint16_t m_height;
    const uint16_t m_radius;

    lv_obj_t *m_screen;

    lv_timer_t *m_timer = nullptr;

    std::vector<ball *> m_balls;
};

application *create_application()
{
    return new example();
}
