#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>
QueueHandle_t xQueue = NULL;

typedef struct 
{
    int value;
}queue_data;


void queue_taskReceive(void* param)
{
    queue_data data;
    while(1)
    {
        if(xQueueReceive(xQueue, &data, 100) == pdTRUE)
        {
            ESP_LOGI("QUEUE", "Received value: %d", data.value);
        }
    }  
}

void queue_taskSend(void* param)
{
    queue_data data;
    memset(&data,0,sizeof(queue_data));
    while(1)
    {   
        xQueueSend(xQueue, &data, 100);
        vTaskDelay(pdMS_TO_TICKS(1000));
        data.value++;
    }
}

void app_main(void)
{
    xQueue = xQueueCreate(10, sizeof(queue_data));
    xTaskCreatePinnedToCore(queue_taskSend, "queue_taskSend", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(queue_taskReceive, "queue_taskReceive", 2048, NULL, 5, NULL, 1);
}
