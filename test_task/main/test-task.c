#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

void taskA(void* param)
{
    while(1)
    {
        printf("taskA\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    
    xTaskCreatePinnedToCore(taskA, "helloword", 2048, NULL, 5, NULL, 0);
}
