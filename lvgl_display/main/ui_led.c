#include "lvgl.h"
#include "esp_log.h"
#include "driver/gpio.h"

static lv_obj_t *ui_led_button = NULL;

static lv_obj_t *ui_led_label = NULL;

void ui_event_led_button(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    static uint32_t led_state = 0;

    switch (code)
    {
    case LV_EVENT_CLICKED:
        led_state = led_state?0:1;
        gpio_set_level(GPIO_NUM_27, led_state);
        break;
    
    default:
        break;
    }
}
void ui_led_create(void)
{
    //设置背景颜色为黑色
    lv_obj_set_style_bg_color(lv_scr_act(),lv_color_black(),0);

    //配置按钮控件
    ui_led_button = lv_btn_create(lv_scr_act());
    lv_obj_align(ui_led_button,LV_ALIGN_CENTER,0,0);
    lv_obj_set_style_bg_color(ui_led_button,lv_palette_main(LV_PALETTE_ORANGE),0);
    lv_obj_set_size(ui_led_button,80,40);

    //配置标签控件
    ui_led_label = lv_label_create(ui_led_button);
    lv_obj_align(ui_led_label,LV_ALIGN_CENTER,0,0);
    lv_label_set_text(ui_led_label,"LED");
    //配置字体大小
    lv_obj_set_style_text_font(ui_led_label,&lv_font_montserrat_20,LV_STATE_DEFAULT);

    //添加按钮事件响应函数
    lv_obj_add_event_cb(ui_led_button, ui_event_led_button, LV_EVENT_CLICKED, NULL);
}