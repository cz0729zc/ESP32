#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "dht11.h"
#include "driver/gpio.h"


SemaphoreHandle_t dht11_mutex;

void taskA(void *param)
{
    int temp = 300,humidity = 60;
    while (1) {
        xSemaphoreTake(dht11_mutex, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500));
        // if(DHT11_StartGet(&temp, &humidity))
        // {
            ESP_LOGI("dht11","taskA-->temp:%d, humidity:%d%%", temp/10, humidity);
        // }
        xSemaphoreGive(dht11_mutex);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskB(void *param)
{
    int temp = 200,humidity = 50;
    while (1) {
        xSemaphoreTake(dht11_mutex, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500));
        // if(DHT11_StartGet(&temp, &humidity))
        // {
            ESP_LOGI("dht11","taskB-->temp:%d, humidity:%d%%", temp/10, humidity);
        // }
        xSemaphoreGive(dht11_mutex);
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}

void app_main(void)
{
    dht11_mutex = xSemaphoreCreateMutex();
    //DHT11_Init(GPIO_NUM_4);
    xTaskCreatePinnedToCore(taskA, "taskA", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(taskB, "taskB", 2048, NULL, 5, NULL, 1);
    
}
