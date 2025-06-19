#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// 定义LED连接的GPIO引脚
#define LED_GPIO_NUM  GPIO_NUM_27

// 定义事件组的位标志
#define FULL_EV_BIT0  BIT0  // 占空比非零事件标志
#define EMPTY_EV_BIT  BIT1  // 占空比为零事件标志
static EventGroupHandle_t ledc_event_handle;  // 事件组句柄



// LEDC中断回调函数，运行在IRAM中以保证快速响应
bool IRAM_ATTR ledc_finish_cb(const ledc_cb_param_t *param, void *user_arg)
{
    BaseType_t taskWorken;
    if (param->duty)
    {
        // 如果占空比非零，设置FULL_EV_BIT0事件标志
        xEventGroupSetBitsFromISR(ledc_event_handle,FULL_EV_BIT0,&taskWorken);
    }
    else
    {
        // 如果占空比为零，设置EMPTY_EV_BIT事件标志
        xEventGroupSetBitsFromISR(ledc_event_handle,EMPTY_EV_BIT,&taskWorken);
    }
    return taskWorken;
}

// LED回调事件函数
void led_run_task(void* parm)
{
    EventBits_t event_bits;
    while(1)
    {
        event_bits = xEventGroupWaitBits(ledc_event_handle,FULL_EV_BIT0|EMPTY_EV_BIT,pdTRUE,pdFALSE,pdMS_TO_TICKS(4000));
        if (event_bits & FULL_EV_BIT0)
        {
           ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 2000);
           ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
        }
        else if (event_bits & EMPTY_EV_BIT)
        {
            ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1023, 2000);
            ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
        }
        // 回调
        ledc_cbs_t cbs = {
            .fade_cb = ledc_finish_cb,  // 设置渐变完成回调函数
        };
       ledc_cb_register(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,&cbs,NULL);
        
    }
}

void app_main(void)
{
    // 配置LED GPIO为输出模式
    // gpio_config_t led_cfg = {
    //     .pin_bit_mask = (1ULL<<LED_GPIO_NUM),
    //     .pull_up_en = GPIO_PULLUP_DISABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .mode = GPIO_MODE_OUTPUT,
    //     .intr_type = GPIO_INTR_DISABLE,
    // };
    // gpio_config(&led_cfg);

    // 1. LEDC定时器配置
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,  // 使用低速模式
        .timer_num = LEDC_TIMER_0,          // 使用定时器0
        .clk_cfg = LEDC_AUTO_CLK,           // 自动选择时钟源
        .freq_hz = 5000,                    // 设置频率为5kHz
        .duty_resolution = LEDC_TIMER_10_BIT // 占空比分辨率为10位(0-1023)
    };
    ledc_timer_config(&ledc_timer);

    // 2. LEDC通道配置
    ledc_channel_config_t ledc_channel = {
        .gpio_num = LED_GPIO_NUM,            // 关联到指定GPIO
        .speed_mode = LEDC_LOW_SPEED_MODE,   // 使用低速模式
        .channel = LEDC_CHANNEL_0,           // 使用通道0
        .intr_type = LEDC_INTR_DISABLE,      // 不使用中断
        .timer_sel = LEDC_TIMER_0,           // 关联到之前配置的定时器
        .duty = 0,                           // 初始占空比为0
    };
    ledc_channel_config(&ledc_channel);

    //开启硬件PWM
    ledc_fade_func_install(0);

    // 3. 配置渐变参数：从当前值渐变到1023(最大)，持续2000ms
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1023, 2000);

    // 4. 启动渐变效果
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);

    // 创建事件组用于中断与任务间通信
    ledc_event_handle  = xEventGroupCreate();

    // 5. 注册渐变完成回调
    ledc_cbs_t cbs = {
        .fade_cb = ledc_finish_cb,  // 设置渐变完成回调函数
    };
    ledc_cb_register(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,&cbs,NULL);

    // 创建并启动LED翻转任务，该任务会周期性地反转LED状态
    xTaskCreatePinnedToCore(led_run_task,"led",2048,NULL,5,NULL,0);
}