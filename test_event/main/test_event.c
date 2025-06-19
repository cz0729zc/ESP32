#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

static TaskHandle_t TaskA_Handle;
static TaskHandle_t TaskB_Handle;

void taskA(void *param)
{
    //定时发送一个任务通知值
    uint32_t value = 0;
    vTaskDelay(pdMS_TO_TICKS(200));
    while(1)
    {
        xTaskNotify(TaskB_Handle, value, eSetValueWithOverwrite);
        vTaskDelay(pdMS_TO_TICKS(1000));
        value++;
    }
}

void taskB(void *param)
{
    //接收任务通知值并打印
    uint32_t value = 0;
    while(1)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, &value, portMAX_DELAY);
        ESP_LOGI("Event", "value = %lu", value);
    }
}

void app_main(void)
{
    xTaskCreatePinnedToCore(taskA, "taskA", 2048, NULL, 5, &TaskA_Handle, 1);
    xTaskCreatePinnedToCore(taskB, "taskB", 2048, NULL, 5, &TaskB_Handle, 1);
}
