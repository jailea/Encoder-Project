#include <Arduino.h>
#include "sh8601.h"
#include "TouchDrvCHSC5816.hpp"
#include "lvgl.h"
#include "SPI.h"
#include "pins_config.h"

static TouchDrvCHSC5816 touch;
static TouchDrvInterface *pTouch;

static void CHSC5816_Initialization(void)
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

static void lv_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pix_map)
{
    const int32_t w = (area->x2 - area->x1 + 1);
    const int32_t h = (area->y2 - area->y1 + 1);

    lv_draw_sw_rgb565_swap(pix_map, w * h);

    lcd_PushColors(area->x1, area->y1, w, h, (uint16_t *)pix_map);

    lv_display_flush_ready(disp);
}

static void lv_indev_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    int16_t Touch_x[2], Touch_y[2];
    uint8_t touchpad = touch.getPoint(Touch_x, Touch_y);

    if (touchpad > 0)
    {
        data->state = LV_INDEV_STATE_PR;

        data->point.x = Touch_x[0];
        data->point.y = Touch_y[0];
    }
    else

        data->state = LV_INDEV_STATE_REL;
}

void init()
{
    CHSC5816_Initialization();

    lv_init();

    auto tick_get_cb = []() -> uint32_t
    {
        return esp_timer_get_time() / 1000ULL;
    };

    lv_tick_set_cb(tick_get_cb);

    auto display = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);

    constexpr auto DRAW_BUFFER_SIZE = (LV_COLOR_DEPTH / 8U) * 390U * 16U;

    auto draw_buf_1 = heap_caps_malloc(DRAW_BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    auto draw_buf_2 = heap_caps_malloc(DRAW_BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    lv_display_set_buffers(display, draw_buf_1, draw_buf_2, DRAW_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, lv_disp_flush);

    auto indev = lv_indev_create();

    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lv_indev_read);
}
