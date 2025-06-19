#include "lvgl.h"
#include "led_ws2812.h"
#include "dht11.h"
#include "esp_log.h"

//声明图片结构体
LV_IMG_DECLARE(temp_img);
LV_IMG_DECLARE(humidity_img);


#define WS2812_NUM 12

static lv_obj_t* ui_temp_image;
static lv_obj_t* ui_humi_image;

static lv_obj_t* ui_temp_label;
static lv_obj_t* ui_humi_label;

static lv_obj_t* ui_light_slider;

static lv_timer_t* ui_dht11_timer;

ws2812_strip_handle_t ws2812_handle;

/**
 * @brief 处理滑动条事件的回调函数
 * 
 * 本函数主要目的是响应滑动条事件，根据滑动条的位置来改变LED灯的颜色
 * 当滑动条的值改变时，会根据新的滑动条值计算RGB值，并更新所有LED灯的颜色
 * 
 * @param e 指向事件的指针，包含事件相关的信息
 */
void light_slider_event_cb(lv_event_t *e)
{
    // 获取事件类型
    lv_event_code_t code = lv_event_get_code(e);

    // 根据事件类型执行相应的操作
    switch (code)
    {
        // 当滑动条的值改变时触发的事件
        case LV_EVENT_VALUE_CHANGED:
        {
            // 获取滑动条对象
            lv_obj_t* slider_obj = lv_event_get_target(e);
            // 获取滑动条的当前值
            int32_t value = lv_slider_get_value(slider_obj);
            // 根据滑动条的值计算RGB值
            uint32_t rgb_value = 150*value/100;

            //ESP_LOGI("LIGHT","slider value change");
            
            // 遍历所有LED灯，更新颜色
            for(int led_index = 0;led_index < WS2812_NUM;led_index++)
            {
                ws2812_write(ws2812_handle,led_index,rgb_value,rgb_value,rgb_value);
            }
            break;
        }
        default:
            break;
    }
}

void ui_dht11_timer_cb(struct _lv_timer_t *)
{
    int temp;
    int humi;
    if(DHT11_StartGet(&temp,&humi))
    {
        char disp_buf[32];
        snprintf(disp_buf,sizeof(disp_buf),"%1.f",(float)temp/10.0);
        lv_label_set_text(ui_temp_label,disp_buf);

        snprintf(disp_buf,sizeof(disp_buf),"%d%%",humi);
        lv_label_set_text(ui_humi_label,disp_buf);
    }
}

void ui_home_create(void)
{
    lv_obj_set_style_bg_color(lv_scr_act(),lv_color_black(),0);

    //创建调光使用的进度条
    ui_light_slider = lv_slider_create(lv_scr_act());
    lv_obj_set_pos(ui_light_slider,60,200);
    lv_obj_set_size(ui_light_slider,150,15);
    lv_slider_set_range(ui_light_slider,0,100);
    lv_obj_add_event_cb(ui_light_slider,light_slider_event_cb,LV_EVENT_VALUE_CHANGED,NULL);

    //创建温度图片
    ui_temp_image = lv_img_create(lv_scr_act());
    lv_img_set_src(ui_temp_image,&temp_img);
    lv_obj_set_pos(ui_temp_image,40,40);
    //创建湿度图片
    ui_humi_image =lv_img_create(lv_scr_act());
    lv_img_set_src(ui_humi_image,&humidity_img);
    lv_obj_set_pos(ui_humi_image,40,120);

    //创建温度文本
    ui_temp_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(ui_temp_label,110,40);
    lv_obj_set_style_text_font(ui_temp_label,&lv_font_montserrat_38,0);
    //创建湿度文本
    ui_humi_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(ui_humi_label,110,120);
    lv_obj_set_style_text_font(ui_humi_label,&lv_font_montserrat_38,0);
    //创建定时器
    ui_dht11_timer = lv_timer_create(ui_dht11_timer_cb, 2000, NULL);

    //初始化ws2812硬件接口
    ws2812_init(GPIO_NUM_32,WS2812_NUM,&ws2812_handle);

    //初始化dht11硬件接口
    DHT11_Init(GPIO_NUM_25);
}