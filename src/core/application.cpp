#include "application.h"

#include <assert.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include "hardware/storage.h"

#include "sh8601.h"
#include "Arduino.h"
#include "pins_config.h"

application *application::s_instance = nullptr;

extern "C" void init();

application::application()
{
    assert(s_instance == nullptr);

    s_instance = this;

    hardware::storage::mount(hardware::storage::type::internal, LV_FS_POSIX_PATH);

    init();

    auto on_create = [](void *userdata)
    {
        static_cast<application *>(userdata)->on_create();
    };

    lv_async_call(on_create, this);

    auto on_update = [](lv_timer_t *timer)
    {
        static const auto app = static_cast<application *>(timer->user_data);

        auto now = esp_timer_get_time();
        auto timestep = now - app->m_previous_timestamp;
        app->m_previous_timestamp = now;

        if (timestep < 0.0f)
            timestep = 0.0f;

        app->on_update(timestep / 1000000.0f);
    };

    lv_timer_create(on_update, 33, this);
}

application::~application()
{
    lv_deinit();

    hardware::storage::unmount(hardware::storage::type::internal);
}

application *create_application();

void setup()
{
    Serial.begin(115200);

    pinMode(LCD_VCI_EN, OUTPUT);
    digitalWrite(LCD_VCI_EN, HIGH);

    sh8601_init();

    lcd_brightness(200); // 0-255

    // wait for initialization to finish.
    // this is required when running on core 1.
    // vTaskDelay(pdMS_TO_TICKS(1000));

    create_application();
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(lv_timer_handler()));
}