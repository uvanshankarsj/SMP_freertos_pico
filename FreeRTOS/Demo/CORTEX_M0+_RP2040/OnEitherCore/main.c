#include <stdio.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"

#include "task.h"

#include "semphr.h"

 

const int taskDelay = 100;
const int taskSize = 128;
SemaphoreHandle_t mutex;

void vSafePrint(char *out) {

    xSemaphoreTake(mutex, portMAX_DELAY);

    puts(out);

    xSemaphoreGive(mutex);

} 

void vTaskSMP(void *pvParameters) {

    TaskHandle_t handle = xTaskGetCurrentTaskHandle();

    UBaseType_t mask = vTaskCoreAffinityGet(handle);
    printf("%d",get_core_num());
    char userInput;
     while(1){
        //Get User Input
        printf("Command (1 = on or 0 = off):\n");
        userInput = getchar();

        if(userInput == '1'){
            // Turn On LED
            gpio_put(25, 1); // Set pin 25 to high
            printf("LED switched on!\n");
        }
        else if(userInput == '0'){
            // Turn Off LED
            gpio_put(25, 0); // Set pin 25 to high.
            printf("LED switched off!\n");
        }
        else{
            printf("Invalid Input!\n");
        }
    }
}

void vTaskSMP2(void *pvParameters) {

    TaskHandle_t handle = xTaskGetCurrentTaskHandle();

    UBaseType_t mask = vTaskCoreAffinityGet(handle);
    printf("%d",get_core_num());
    gpio_put(25, 1);
    for(int i=0;i<100000;i++){}
    gpio_put(25, 0);
    
}

 

void main() {

    stdio_init_all();   

    mutex = xSemaphoreCreateMutex();

    TaskHandle_t handleA;

    TaskHandle_t handleB;

    xTaskCreate(vTaskSMP, "A", taskSize, NULL, 1, &handleA);

    xTaskCreate(vTaskSMP2, "B", taskSize, NULL, 1, &handleB);

    //xTaskCreate(vTaskSMP, "C", taskSize, NULL, 1, NULL);

    //xTaskCreate(vTaskSMP, "D", taskSize, NULL, 1, NULL);

    vTaskCoreAffinitySet(handleA, (1 << 0));

    vTaskCoreAffinitySet(handleB, (1 << 1));

    vTaskStartScheduler();

}


