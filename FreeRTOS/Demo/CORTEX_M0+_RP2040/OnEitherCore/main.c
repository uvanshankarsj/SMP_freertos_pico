#include <stdio.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"

#include "task.h"

#include "semphr.h"
#include "haw/MPU6050.h"

int motorDirection = 0;
double setpoint = 0;
double Kp = 10.532;
double Ki = 18.338;
double Kd = 1.077;
double error, lastError = 0;
double P, I, D;
double output;
int accelXOffset = 0;
int accelYOffset = 0;
int accelZOffset = 0;
int gyroXOffset = 0;
int gyroYOffset = 0;
int gyroZOffset = 0;
double errorTolerance = 1.0;

const int taskDelay = 100;
const int taskSize = 128;
SemaphoreHandle_t mutex;

void vSafePrint(char *out) {

    xSemaphoreTake(mutex, portMAX_DELAY);

    printf("%s",out);

    xSemaphoreGive(mutex);

} 

void vTaskSMP(void *pvParameters) {

    TaskHandle_t handle = xTaskGetCurrentTaskHandle();

    UBaseType_t mask = vTaskCoreAffinityGet(handle);
    // printf("%d",get_core_num());
    char userInput;
     while(1){
        //Get User Input
        vSafePrint("Command (1 = on or 0 = off):\n");
        userInput = getchar();

        if(userInput == '1'){
            // Turn On LED
            gpio_put(25, 1); // Set pin 25 to high
            vSafePrint("LED switched on!\n");
        }
        else if(userInput == '0'){
            // Turn Off LED
            gpio_put(25, 0); // Set pin 25 to high.
            vSafePrint("LED switched off!\n");
        }
        else{
            vSafePrint("Invalid Input!\n");
        }
    }
}

void vTaskSMP2(void *pvParameters) {

    TaskHandle_t handle = xTaskGetCurrentTaskHandle();

    UBaseType_t mask = vTaskCoreAffinityGet(handle);
    
     if (mpu6050_begin(&mpu6050))
    {
        gpio_put(25,0);
        // // Set scale of gyroscope
        // mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
        // // Set range of accelerometer
        // mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);

        // // Enable temperature, gyroscope and accelerometer readings
        // mpu6050_set_temperature_measuring(&mpu6050, true);
        // mpu6050_set_gyroscope_measuring(&mpu6050, true);
        // mpu6050_set_accelerometer_measuring(&mpu6050, true);

        // // Enable free fall, motion and zero motion interrupt flags
        // mpu6050_set_int_free_fall(&mpu6050, false);
        // mpu6050_set_int_motion(&mpu6050, false);
        // mpu6050_set_int_zero_motion(&mpu6050, false);

        // // Set motion detection threshold and duration
        // mpu6050_set_motion_detection_threshold(&mpu6050, 2);
        // mpu6050_set_motion_detection_duration(&mpu6050, 5);

        // // Set zero motion detection threshold and duration
        // mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);
        // mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);
    }
    else
    {
        gpio_put(25,0);
        while (1)
        {
            // Endless loop
            printf("Error! MPU6050 could not be initialized. Make sure you've entered the correct address. And double check your connections.");
            sleep_ms(500);
        }
    }
    // printf("%d",get_core_num());
    for(int j=0;j<10000000000;j++){}
    gpio_put(25, 0);
}

 

void main() {

    stdio_init_all();
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(25, GPIO_OUT);
    mpu6050_t mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_VCC);
    gpio_put(25,1);
    // printf("%d",mpu6050_begin(&mpu6050));
    for (int i=0;i<1000000000;i++){
        };


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


