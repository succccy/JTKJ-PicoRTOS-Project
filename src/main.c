// Group members: Eetu Matikkala, Miika Pernu, Akseli Östberg

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1
#define DEFAULT_I2C_SDA_PIN  12
#define DEFAULT_I2C_SCL_PIN  13

#define DATASIZE 5
#define GYROTHRESHOLD 0.5

enum state {WAITING=1, DATA_READY, SEND_DATA};
enum state programState = WAITING;

float normal_IMUGyro[3];
float saved_IMUGyro[DATASIZE][3];
int saveData_it = 0;
char morseMessage[50];
int msg_index = 0;

static void btn_fxn(uint gpio, uint32_t eventMask) {
    toggle_red_led();
    if (gpio == BUTTON1) {
        programState = DATA_READY;
    }

    else if (gpio == BUTTON2) {
        programState = SEND_DATA;
    }
}

// Collect data when programState == WAITING
static void sensor_task(void *arg){
    (void)arg;
    for(;;){
        if(programState == WAITING) {
            float ax, ay, az, gx, gy, gz, t;
            if(ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                // Assign all values from the sensor to an array
                float all_IMUData[7] = {ax, ay, az, gx, gy, gz, t};
                // Filter and normalize gyroscope absolute values into a new array
                for(int i = 3; i < 6; i++) {
                    normal_IMUGyro[i-3] = (fabs(all_IMUData[i])) / (ICM42670_GYRO_FSR_DEFAULT);
                }
                // Save the last n = DATASIZE gyro values into a matrix, reset saveData_it iterator to 0 when filled
                if(saveData_it < DATASIZE){
                    for(int i = 0; i < 3; i++) {
                        saved_IMUGyro[saveData_it][i] = normal_IMUGyro[i];
                    }
                    saveData_it++;
                }
                else {
                    saveData_it = 0;
                }
            }
        }

        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Print data when programState == DATA_READY
static void morse_task(void *arg){
    (void)arg;
    while(1){
        if(programState == DATA_READY) {
            float maxVal = 0;
            int maxVal_axis = 0;
            printf("\n__[ Gx    Gy    Gz ]__"); // Debug print

            // Loop through the rows of the saved gyro values matrix and check which axis has the largest value
            for(int i = 0; i < DATASIZE; i++) {
                printf("\n__[%.2f, %.2f, %.2f]__", saved_IMUGyro[i][0], saved_IMUGyro[i][1], saved_IMUGyro[i][2]); // Debug print
                for(int j = 0; j < 3; j++) {
                    if(saved_IMUGyro[i][j] > GYROTHRESHOLD && saved_IMUGyro[i][j] > maxVal) {
                        maxVal = saved_IMUGyro[i][j];
                        maxVal_axis = j;
                    }
                }
            }

            // Check that GYROTHRESHOLD has been exceeded, label the axis and add the morse symbol to the message
            if(maxVal > GYROTHRESHOLD) {
                char *axis;
                char *morSymbol;
                switch(maxVal_axis) {
                    // X axis - dot
                    case 0:
                        axis = "Gx";
                        morSymbol = "[.]";
                        morseMessage[msg_index] = '.';
                        msg_index++;
                        break;
                    // Y axis - dash
                    case 1:
                        axis = "Gy";
                        morSymbol = "[-]";
                        morseMessage[msg_index] = '-';
                        msg_index++;
                        break;
                    // Z axis - space
                    case 2:
                        axis = "Gz";
                        morSymbol = "[SPACE]";
                        morseMessage[msg_index] = ' ';
                        msg_index++;
                        break;
                }
                printf("\n__LARGEST SAVED GYRO DATA OF AXIS (%s) -- %s__", axis, morSymbol); // Debug print
            }
            programState = WAITING;
        }
        
        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// Send the morse message when programState == SEND_DATA
static void send_task(void *arg){
    (void) arg;
    while(1){
        if(programState == SEND_DATA){
            // Add a newline to mark the end of the message and print the message to the serial client
            morseMessage[msg_index] = '\n';
            printf(morseMessage);
            // Clear the message and reset the message index to 0
            for(int i = 0; i < msg_index+1; i++) {
                morseMessage[i] = '\0';
            }
            msg_index = 0;
            programState = WAITING;
        }

        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

int main() {

    stdio_init_all();

    // Uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    } 
    
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

    // Initialize and set interrupt handlers for the two buttons
    init_button1();
    init_button2();
    init_red_led();
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_FALL, true, btn_fxn);

    // Initializing the IMU with its default values
    init_ICM42670();
    ICM42670_start_with_default_values();

    
    TaskHandle_t hSensorTask, hMorseTask, hSendTask = NULL;

    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(sensor_task, // (en) Task function
                "sensor",                        // (en) Name of the task 
                DEFAULT_STACK_SIZE,              // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                            // (en) Arguments of the task 
                2,                               // (en) Priority of this task
                &hSensorTask);                   // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }
    result = xTaskCreate(morse_task,  // (en) Task function
                "morse",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hMorseTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }
    result = xTaskCreate(send_task,   // (en) Task function
                "send",               // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hSendTask);          // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();
    
    // Never reach this line.
    return 0;
}

