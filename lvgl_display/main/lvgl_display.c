#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "lv_port.h"
#include "lv_demos.h"
#include "st7789_driver.h"
#include "ui_led.h"
#include "driver/gpio.h"
#include "ui_home.h"


void app_main(void)
{
    lv_port_init();
    st7789_lcd_backlight(1);
    //lv_demo_widgets();
    ui_home_create();
#if 0
    ui_led_create();

    gpio_config_t led_gpio = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ull << GPIO_NUM_27),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    gpio_config(&led_gpio);

    gpio_set_level(GPIO_NUM_27, 0);
#endif

    while (1)
    {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    

}
