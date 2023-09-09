#include <stdio.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"

#include "task.h"

#include "semphr.h"
#include "MPU6050.h"


// #define D0 2
// #define D1 3
// #define D2 4
// #define D3 5

// int motorDirection = 0;
// double setpoint = 0;
// double Kp = 10.532;
// double Ki = 18.338;
// double Kd = 1.077;
// double error, lastError = 0;
// double P, I, D;
// double output;
// int accelXOffset = 0;
// int accelYOffset = 0;
// int accelZOffset = 0;
// int gyroXOffset = 0;
// int gyroYOffset = 0;
// int gyroZOffset = 0;
// double errorTolerance = 1.0;
// int pan_pos = 90;
// int tilt_pos = 90;
// int self_count = 0;

const int taskDelay = 100;
const int taskSize = 128;
SemaphoreHandle_t mutex;

void vSafePrint(char out[]) {

    xSemaphoreTake(mutex, portMAX_DELAY);
    puts(out);
    xSemaphoreGive(mutex);

} 

void vTaskSMP(void *pvParameters) {

    TaskHandle_t handle = xTaskGetCurrentTaskHandle();

    UBaseType_t mask = vTaskCoreAffinityGet(handle);
    // char coreno[25];
    // vSafePrint(itoa(coreno,get_core_num(),10));
    char command;
     while(1){
    //     //Get User Input
        vSafePrint("Command (1 = on or 0 = off):\n");
        command = getchar();
        if (command=='1')
    {
      vSafePrint("motor_state:");
    //   printf("w");
    //   analogWrite(D0, 50);
    //   digitalWrite(D1, LOW);
    //   analogWrite(D2, 50);
    //   digitalWrite(D3, LOW);   
    }
    if (command=='2')
    {
      vSafePrint("motor_state:");
    //   printf("s");
    //   digitalWrite(D0, LOW);
    //   analogWrite(D1,50);
    //   digitalWrite(D2, LOW);
    //   analogWrite(D3,50); 
    }
    if (command=='3')
    {
      vSafePrint("motor_state:");
    //   printf("a");
    //   analogWrite(D0,50);
    //   digitalWrite(D1,LOW);
    //   analogWrite(D2,0);
    //   digitalWrite(D3,LOW);
    }
    if (command=='4')
    {
      vSafePrint("motor_state:");
    //   printf("d");
    //   analogWrite(D0,0);
    //   digitalWrite(D1,LOW);
    //   analogWrite(D2,50);
    //   digitalWrite(D3,LOW);
    }
    if (command=='a')
    {
      vSafePrint("stand_state:");
    //   printf("on");
    //   gripper.write(90);
    }
    if (command=='b')
    {
      vSafePrint("b");
    //   analogWrite(D0,0);
    //   digitalWrite(D1,LOW);
    //   analogWrite(D2,0);
    //   digitalWrite(D3,LOW);
    }

    if (command=='6')
    {
      vSafePrint("gripper_state:");
    //   printf("cw");    
    //   gripper.write(45);
    }
    else if(command=='7')
    {
      vSafePrint("gripper_state:");
    //   printf("ccw");
    //   gripper.write(135);
    }
    if (command=='8'){
    //   pan_pos=pan_pos+1;
    //   if(pan_pos>180){
    //     pan_pos=180;
      // }
      vSafePrint("pan_pos:");
    //   printf(pan_pos);
    //   pan.write(pan_pos);
    }
    // else if (command=='9')
    // {
    //   pan_pos=pan_pos-1;
    //   if(pan_pos<0)
    //   {
    //     pan_pos=0;
    //   }
    //   printf("pan_pos:");
    //   printf(pan_pos);
    //   pan.write(pan_pos);
    // }
    // if (command=='0')
    // {
    //   tilt_pos = tilt_pos+1;
    //   if(pan_pos>180){
    //     tilt_pos=180;
    //   }
    //   printf("tilt_pos:");
    //   printf(tilt_pos);
    //   tilt.write(tilt_pos);
    // }
    // if (command=='5')
    // {
    //   tilt_pos = tilt_pos-1;
    //   if(tilt_pos<0)
    //   {
    //     tilt_pos=0;
    //   }
    //   printf("tilt_pos:");
    //   printf(tilt_pos);
    //   tilt.write(tilt_pos);
    // }
    // tilt.write(tilt_pos);
    // if (command=='c')
    // {
    //   self_count+=1; 
    // }
    // if (command=='d')
    // {
    //   self_count=0;
    // }
    // if(self_count>0)
    // {
    //   //printf("Self balancing enabled");
    //   if (motorDirection == 1) {
    //     analogWrite(D0, 60);
    //     analogWrite(D1, 0);
    //     analogWrite(D2, 60);
    //     analogWrite(D3, 0);
    //   } else if (motorDirection == -1) {
    //     analogWrite(D0, 0);
    //     analogWrite(D1, 60);
    //     analogWrite(D2, 0);
    //     analogWrite(D3, 60);
    //   }
    //   else{
    //     analogWrite(D0, 0);
    //     analogWrite(D1, 0);
    //     analogWrite(D2, 0);
    //     analogWrite(D3, 0);
    //     }
    // }
    // if(self_count==0){
    //   //printf("Self balancing disabled");
    //   analogWrite(D0, 0);
    //   analogWrite(D1, 0);
    //   analogWrite(D2, 0);
    //   analogWrite(D3, 0);
    // }
    // }
     }
}

void vTaskSMP2(void *pvParameters) {

    TaskHandle_t handle = xTaskGetCurrentTaskHandle();

    UBaseType_t mask = vTaskCoreAffinityGet(handle);
    gpio_put(25,1);
    stdio_init_all();
    // Setup I2C properly
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    // Don't forget the pull ups! | Or use external ones
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    mpu6050_t mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_VCC);
    if (mpu6050_begin(&mpu6050))
    {
        // Set scale of gyroscope
        mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
        // Set range of accelerometer
        mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);
        // Enable temperature, gyroscope and accelerometer readings
        mpu6050_set_temperature_measuring(&mpu6050, true);
        mpu6050_set_gyroscope_measuring(&mpu6050, true);
        mpu6050_set_accelerometer_measuring(&mpu6050, true);

        // Enable free fall, motion and zero motion interrupt flags
        mpu6050_set_int_free_fall(&mpu6050, false);
        mpu6050_set_int_motion(&mpu6050, false);
        mpu6050_set_int_zero_motion(&mpu6050, false);

        // Set motion detection threshold and duration
        mpu6050_set_motion_detection_threshold(&mpu6050, 2);
        mpu6050_set_motion_detection_duration(&mpu6050, 5);

        // Set zero motion detection threshold and duration
        mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);
        mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);
    }
    else
    {
        // gpio_put(25,0);
        while (1)
        {}
        //     vSafePrint("Error! MPU6050 could not be initialized. Make sure you've entered the correct address. And double check your connections.");
        //     for (int i=0;i<10000000;i++);
        // }
    }
    // printf("%d",get_core_num());
// int accelX = a.acceleration.x - accelXOffset;
// int accelY = a.acceleration.y - accelYOffset;
// int accelZ = a.acceleration.z - accelZOffset;
// int gyroX = g.gyro.x - gyroXOffset;
// int gyroY = g.gyro.y - gyroYOffset;
// int gyroZ = g.gyro.z - gyroZOffset;

// double angle = atan2(accelY, accelZ) * 180 / PI;
// double gyroAngle = angle + (gyroX / 131.0) * 0.98;
// error = setpoint - gyroAngle;
//   P = Kp * error;
//   I += Ki * error;
//   D = Kd * (error - lastError);
//   output = P + I + D;
//   lastError = error;
  
//   standservo1.write(0);
//   standservo2.write(0);
//   pan.write(80);
  
//   // Check if the error is within the tolerance range
//   if (fabs(error) < errorTolerance) {
//     output = 0.0;  // Set the output to zero
//     I = 0.0;      // Reset the integral term
//   }

//   if (output > 255.00) {
//     output = 255.00;
//   } else if (output < -255.00) {
//     output = -255.00;
//   }
//   int motorSpeed = abs(output);  // Map the absolute output to the motor speed range
//   if (output < 0.00) {
//     motorDirection = -1;
//   } else if(output==0){
//     motorDirection = 0;
//   }
//   else{
//     motorDirection = 1;
//   }
//    if(self_count>0)
//     {
//       //printf("Self balancing enabled");
//       if (motorDirection == 1) {
//         analogWrite(D0, 60);
//         analogWrite(D1, 0);
//         analogWrite(D2, 60);
//         analogWrite(D3, 0);
//       } else if (motorDirection == -1) {
//         analogWrite(D0, 0);
//         analogWrite(D1, 60);
//         analogWrite(D2, 0);
//         analogWrite(D3, 60);
//       }
//       else{
//         analogWrite(D0, 0);
//         analogWrite(D1, 0);
//         analogWrite(D2, 0);
//         analogWrite(D3, 0);
//         }
//     }
//     if(self_count==0){
//       //printf("Self balancing disabled");
//       analogWrite(D0, 0);
//       analogWrite(D1, 0);
//       analogWrite(D2, 0);
//       analogWrite(D3, 0);
//     }
  }

 

void main() {
    stdio_init_all();
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(25, GPIO_OUT);
    mpu6050_t mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_VCC);
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


