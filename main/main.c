/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "joystick_buttons.h"

#include "esp_system.h"
#include "driver/uart.h"
#include "string.h"

#include "driver/i2c.h"
#include "i2c-lcd.h"

#include <ds18x20.h>
#include <esp_err.h>

#include <inttypes.h>
#include "nvs_flash.h"
#include "nvs.h"

#define REMOTE_CONTR "REMOTE_CONTROL"


#define TXD1_PIN (GPIO_NUM_6)
#define RXD1_PIN (GPIO_NUM_7)
#define TXD2_PIN (GPIO_NUM_15)
#define RXD2_PIN (GPIO_NUM_16)

#define EN_PIN1 (GPIO_NUM_36)
#define EN_PIN2 (GPIO_NUM_37)
// #define GPIO_OUTPUT_PIN_SEL  (1ULL<<EN_PIN1)

#define RIGHT 1
#define LEFT  -1

#define CAM_RIGHT 1
#define CAM_LEFT  2
#define FORWORD_RIGHT  5
#define FORWORD_LEFT   6
#define BACKWORD_RIGHT  9
#define BACKWORD_LEFT   10
#define FORWORD    4
#define BACKWORD    8
#define SPEED_UP    16
#define SPEED_DOWN  32
#define CAM_SHIFT_RIGHT 260 
#define CAM_SHIFT_LEFT  264

#define FALT1       64
#define FALT2       128
#define FALT3       72
#define FALT4       136
#define FALT5       68
#define FALT6       132


static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 8;
char buffer[17];

// static const gpio_num_t SENSOR_GPIO = GPIO_NUM_17;

// Use address of your own sensor here!
// You can find out the address of your sensor by running ds18x20_multi example
// static const onewire_addr_t SENSOR_ADDR1 = 0x213de10457a2fa28;
// static const onewire_addr_t SENSOR_ADDR2 = 0xdb3de10457f15628 ;

// for second
// static const onewire_addr_t SENSOR_ADDR1 = 0xcb1529961d64ff28;
// static const onewire_addr_t SENSOR_ADDR2 = 0x0d3de10457703828 ;

volatile bool flagMotorRUN = 0,errorFlag = 0,flagCamRUN = 0,serialFlag = 0, camShiftFlag = 0;
volatile int cmdFlag = 0;

static const unsigned char CRCHi[] = {
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40
};

static const unsigned char CRCLo[] = {
 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
  0x41, 0x81, 0x80, 0x40
};
uint16_t CRC16_calc( unsigned char * frame, uint16_t len );
int32_t CalcAccelRate(int32_t speed);


#define MAX_SPEED 500
#define MIN_SPEED 1

int32_t winchSPEED = 10;
int32_t pulseRate = 800;
int32_t acceSPEED = 0;
int32_t deceSPEED = 0;
int32_t camSPEED = 0;
int32_t camAcceSPEED = 0;
int32_t camDeceSPEED = 0;
int8_t stoptime = 40;
int timerCam = 0;


int32_t incSPEED = 5;
int32_t camRatio = 800;
int8_t camDirection = 5,camMOV=1;
nvs_handle_t my_handle;
float camRatiof =1;


#include <stdio.h>

#define ARRAYSIZE 10
uint32_t numbers[ARRAYSIZE] = {0};
uint32_t *ptr = numbers; 
int arrayindex = 0; 
uint32_t movingAverage= 0;
int pointerMul = sizeof(numbers)/(ARRAYSIZE* sizeof(int));

void saveWinchSPEED(void);
void saveStopTime(void);
void saveCamRatio(void);
void savePulseRate(void);
void savePulseRate(void);

void processCommand(char dataArray[]) {

    if (('c' == dataArray[0]) && ('f' == dataArray[1]) && ('g' == dataArray[2]) && (' ' == dataArray[3])) {
        if (('s' == dataArray[4]) && ('p' == dataArray[5]) && ('d' == dataArray[6]) && (' ' == dataArray[7])) {
            uint32_t speedinput = atoi(&dataArray[8]); 
            printf("Setting speed to %ld\n", speedinput);
            winchSPEED= speedinput;
            saveWinchSPEED();
        }
        else if (('t' == dataArray[4]) && ('i' == dataArray[5]) && ('m' == dataArray[6]) && ('e' == dataArray[7]) && (' ' == dataArray[8])) {
            uint32_t timeValue = atoi(&dataArray[9]);
            printf("Setting time value to %ld\n", timeValue);
            stoptime =timeValue;
            saveStopTime();
        
        }
        else if (('c' == dataArray[4]) && ('a' == dataArray[5]) && ('m' == dataArray[6]) && ('r' == dataArray[7]) && (' ' == dataArray[8])) {
            int32_t inputCamRatio = atoi(&dataArray[9]);
            printf("Setting CamRatio %ld\n", inputCamRatio);
            camRatio =inputCamRatio;
            saveCamRatio();
            
        }
        else if (('p' == dataArray[4]) && ('u' == dataArray[5]) && ('l' == dataArray[6]) && ('r' == dataArray[7]) && (' ' == dataArray[8])) {
            int32_t inptPulseRate = atoi(&dataArray[9]);
            printf("Setting CamRatio %ld\n", inptPulseRate);
            pulseRate =inptPulseRate;
            savePulseRate();

        }
        else {
            printf("Invalid command\n");
        }
    }
    else if (('a' == dataArray[0]) && ('c' == dataArray[1]) && ('t' == dataArray[2]) && (' ' == dataArray[3])) {
        if (('f' == dataArray[4]) && ('w' == dataArray[5]) && ('d' == dataArray[6])) {
            cmdFlag = 1;
            serialFlag =1;
            printf("Activated Forward");
        }
        else if (('b' == dataArray[4]) && ('w' == dataArray[5]) && ('d' == dataArray[6])) {
            cmdFlag = 2;
            serialFlag =1;
            printf("Activated Backward");

        
        }
        else if (('s' == dataArray[4]) && ('t' == dataArray[5]) && ('o' == dataArray[6]) && ('p' == dataArray[7])) {
            cmdFlag = 3;
            serialFlag =1;
        }
        else {
            printf("Invalid command\n");
    }
    }
    else {
        printf("Invalid command\n");
    }
}


uint32_t calculateMovingAverage(uint32_t *arr, int size) {
    uint32_t sum = 0;
    // printf("array contents\r\n");
    for(int i = 0; i < size; i++) {
        // printf("%ld\r\n", *(arr + i));
        sum += *(arr + i);
    }
    
    return sum / size;
}


static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_2,
        .scl_io_num = GPIO_NUM_1,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}
float num = 12.34;

void initUART(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
}
void initUART1(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 38400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
void initUART2(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 38400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int sendData1(const char* logName, unsigned const char* data,const int len)
{
    // const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int sendData2(const char* logName, unsigned const char* data,const int len)
{
    // const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void startMOTOR()
{
    gpio_set_level(EN_PIN1, 1);

    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x1c;
    data[4] = 0x00;
    data[5] = 0x02;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData1("TX_DATA",data,8);
    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    free(data);
}    



void stopMOTOR()
{
    gpio_set_level(EN_PIN1, 0);
    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x1c;
    data[4] = 0x00;
    data[5] = 0x03;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData1("TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);
    free(data);
}



void setSPEED(int32_t speed)
{
    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x18;
    data[4] =((int32_t)speed >> 8) & 0xFF;
    data[5] =((int32_t)speed >> 0) & 0xFF;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData1("TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);

    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x19;
    data[4] =((int32_t)speed >> 24) & 0xFF;
    data[5] =((int32_t)speed >> 16) & 0xFF;

    dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData1("TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);

    free(data);
}

void setACCE(int32_t speed)
{
    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x14;
    data[4] =((int32_t)speed >> 8) & 0xFF;
    data[5] =((int32_t)speed >> 0) & 0xFF;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData1("TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);

    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x15;
    data[4] =((int32_t)speed >> 24) & 0xFF;
    data[5] =((int32_t)speed >> 16) & 0xFF;

    dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData1("TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);

    free(data);
}

void setDECE(int32_t speed)
{
    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x16;
    data[4] =((int32_t)speed >> 8) & 0xFF;
    data[5] =((int32_t)speed >> 0) & 0xFF;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData1("TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);

    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x17;
    data[4] =((int32_t)speed >> 24) & 0xFF;
    data[5] =((int32_t)speed >> 16) & 0xFF;

    dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData1("TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);

    free(data);
}

void startCamMOTOR()
{
    gpio_set_level(EN_PIN2, 1);

    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x1c;
    data[4] = 0x00;
    data[5] = 0x02;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData2("CAM_TX_DATA",data,8);
    // ESP_LOG_BUFFER_HEXDUMP("CAM_TX_DATA", data, 8, ESP_LOG_INFO);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    free(data);
}    



void stopCamMOTOR()
{
    gpio_set_level(EN_PIN2, 0);
    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x1c;
    data[4] = 0x00;
    data[5] = 0x03;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData2("CAM_TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);
    free(data);
}

void immediateStopCamMOTOR()
{
    gpio_set_level(EN_PIN2, 0);
    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x1c;
    data[4] = 0x00;
    data[5] = 0x04;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData2("CAM_TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);
    free(data);
}

void setCamSPEED(int32_t speed)
{
    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x18;
    data[4] =((int32_t)speed >> 8) & 0xFF;
    data[5] =((int32_t)speed >> 0) & 0xFF;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData2("CAM_TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("CAM_TX_DATA", data, 8, ESP_LOG_INFO);

    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x19;
    data[4] =((int32_t)speed >> 24) & 0xFF;
    data[5] =((int32_t)speed >> 16) & 0xFF;

    dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData2("CAM_TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("CAM_TX_DATA", data, 8, ESP_LOG_INFO);

    free(data);
}

void setCamACCE(int32_t speed)
{
    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x14;
    data[4] =((int32_t)speed >> 8) & 0xFF;
    data[5] =((int32_t)speed >> 0) & 0xFF;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData2("CAM_TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("CAM_TX_DATA", data, 8, ESP_LOG_INFO);

    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x15;
    data[4] =((int32_t)speed >> 24) & 0xFF;
    data[5] =((int32_t)speed >> 16) & 0xFF;

    dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData2("CAM_TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("CAM_TX_DATA", data, 8, ESP_LOG_INFO);

    free(data);
}

void setCamDECE(int32_t speed)
{
    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x16;
    data[4] =((int32_t)speed >> 8) & 0xFF;
    data[5] =((int32_t)speed >> 0) & 0xFF;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData2("CAM_TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("CAM_TX_DATA", data, 8, ESP_LOG_INFO);

    data[0] = 0x01;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x17;
    data[4] =((int32_t)speed >> 24) & 0xFF;
    data[5] =((int32_t)speed >> 16) & 0xFF;

    dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData2("CAM_TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOG_BUFFER_HEXDUMP("CAM_TX_DATA", data, 8, ESP_LOG_INFO);

    free(data);
}

void readSpeed(){
    uint8_t* data = (uint8_t*) malloc(TX_BUF_SIZE);
    data[0] = 0x01;
    data[1] = 0x04;
    data[2] = 0x00;
    data[3] = 0x01;
    data[4] = 0x00;
    data[5] = 0x01;
    int16_t dataCRC= CRC16_calc(data,6);
    data[6] =((uint16_t)dataCRC >> 0) & 0xFF;
    data[7] =((uint16_t)dataCRC >> 8) & 0xFF;
    sendData1("TX_DATA",data,8);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // ESP_LOGI("SPEED", "Sending speed data req");
    // ESP_LOG_BUFFER_HEXDUMP("TX_DATA", data, 8, ESP_LOG_INFO);

    free(data);
}


static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "CONFIGURE";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            char dataArray [20];
            memcpy(dataArray,data, rxBytes);

            processCommand(dataArray);


        }
    }
    free(data);
}
static void rx_task1(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            // ESP_LOGI("Motor-1", "Read %d bytes: '%s'", rxBytes, data);
            // ESP_LOG_BUFFER_HEXDUMP("Motor-1", data, rxBytes, ESP_LOG_INFO);
            // decodeSpeedInfo(data, rxBytes);

             uint8_t dataArray [7];
            memcpy(dataArray,data, rxBytes);

            if ((1== dataArray[0])&& (4== dataArray[1])&& (2== dataArray[2])&& (7==rxBytes ))
            {
                uint32_t speed = ((dataArray[3]  << 8) ) | ((dataArray[4]) );
                                
                // printf( "speed = %ld", speed);
                *(ptr + arrayindex) = speed; 
                arrayindex = (arrayindex + pointerMul*1) % ARRAYSIZE; 
                movingAverage = calculateMovingAverage(ptr, ARRAYSIZE);
                // printf( "avg speed = %ld", movingAverage);
            }
        }
    }
    free(data);
}
static void rx_task2(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI("Motor-2", "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP("Motor-2", data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

void saveCamDirection()
{
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

    err = nvs_set_i8(my_handle, "camDirection", camDirection);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(my_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}

void saveWinchSPEED()
{
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

    err = nvs_set_i32(my_handle, "winchSPEED",winchSPEED);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");


    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(my_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}

void savePulseRate()
{
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

    err = nvs_set_i32(my_handle, "pulseRate",pulseRate);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");


    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(my_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}
// void saveCamRatio()
// {
//     esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
//     if (err != ESP_OK) {
//         printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
//     } else {
//         printf("Done\n");

//     err = nvs_set_i32(my_handle, "camRatio",camRatio );
//     printf((err != ESP_OK) ? "Failed!\n" : "Done\n");


//     printf("Committing updates in NVS ... ");
//     err = nvs_commit(my_handle);
//     printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

//     // Close
// vTaskDelay(100 / portTICK_PERIOD_MS);
//     err = nvs_get_i32(my_handle, "camRatio", &camRatio);
//         switch (err) {
//             case ESP_OK:
//                 ESP_LOGI(REMOTE_CONTR,"Done\n");
//                 ESP_LOGI(REMOTE_CONTR,"camRatio = %ld\n", camRatio);
//                 break;
//             case ESP_ERR_NVS_NOT_FOUND:
//                 ESP_LOGE(REMOTE_CONTR,"The value camRatio is not initialized yet!\n");
//                 break;
//             default :
//                 ESP_LOGE(REMOTE_CONTR,"Error (%s) reading!\n", esp_err_to_name(err));
//         }


//     nvs_close(my_handle);
//     vTaskDelay(100 / portTICK_PERIOD_MS);

//     }
// }

void saveCamRatio()
{
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

    err = nvs_set_i32(my_handle, "camRatio",camRatio);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");


    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(my_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}

void saveStopTime()
{
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

    err = nvs_set_i8(my_handle, "stoptime",stoptime );
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");


    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(my_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    
    }
}


static void remote_read_task(void *pvParameter)
{

    uint16_t buttons = 0;
  
    uint16_t last_buttons = 0;
    
    joystick_buttons_event_t ev;
    QueueHandle_t joystick_buttons_events = joystick_buttons_init();

    while(true) {
        ESP_LOGD(REMOTE_CONTR, "wait for buttons_events");
       
        if (xQueueReceive(joystick_buttons_events, &ev, 50/portTICK_PERIOD_MS)|| (1  == serialFlag )) {
            // ESP_LOGI(REMOTE_CONTR, "buttons_events %d", ev.state);
            
        
            if (1 == cmdFlag){
                ev.state = ev.state | FORWORD; 
                buttons = ev.state;
            }
            else if(2 == cmdFlag){
                ev.state = ev.state | BACKWORD;
                buttons = ev.state;
            }
            else if(3 == cmdFlag){
                ev.state = 0;
                cmdFlag =0;
                buttons = ev.state;
            }
            else
            {
                buttons = ev.state;
            }
        }
        // printf("status= %d",ev.state);

        // only transmit if something changed
        if ((last_buttons != buttons)|| (1  == serialFlag )){

            switch (ev.state)
            {
                case CAM_RIGHT: ESP_LOGI(REMOTE_CONTR, "CAM_RIGHT CAM PRESSED");
                        if ( 1==flagCamRUN )
                        {
                            // stopCamMOTOR();
                            camRatiof = (float)camRatio/1000;
                            camSPEED= (int)(((winchSPEED*pulseRate)/60)*camRatiof);
                            camAcceSPEED = CalcAccelRate(camSPEED);
                            camDeceSPEED = CalcAccelRate(camSPEED);
                            // printf("cam acce = %ld", camAcceSPEED);
                            // printf("cam decce = %ld", camDeceSPEED);
                            setCamACCE(camAcceSPEED);
                            setCamDECE(camDeceSPEED);
                            camMOV= RIGHT;
                            setCamSPEED(camMOV * camSPEED);
                            startCamMOTOR();
                            ESP_LOGI(REMOTE_CONTR, "MOVING LEFT");
                            if (camDirection== FORWORD_LEFT)
                            {
                                ESP_LOGI(REMOTE_CONTR, " Previous cam direction is = FORWORD_RIGHT");  
                                camDirection= FORWORD_RIGHT;
                                ESP_LOGI(REMOTE_CONTR, " setting FORWORD_RIGHT");  
                                saveCamDirection();
                            }
                            else if (camDirection== BACKWORD_LEFT )
                            {
                                ESP_LOGI(REMOTE_CONTR, " Previous cam direction is BACKWORD_RIGHT");  
                                camDirection= BACKWORD_RIGHT;
                                ESP_LOGI(REMOTE_CONTR, " setting BACKWORD_RIGHt");  
                                saveCamDirection();
                            }  
                            // vTaskDelay(pdMS_TO_TICKS(1000)); 
                            stopCamMOTOR();
                            }
                        break;
                case CAM_LEFT: ESP_LOGI(REMOTE_CONTR, "CAM_LEFT CAM PRESSED");
                        if ( 1==flagCamRUN )
                        {
                            // stopCamMOTOR();
                            camRatiof = (float)camRatio/1000;
                            camSPEED= (int)(((winchSPEED*pulseRate)/60)*camRatiof);
                            camAcceSPEED = CalcAccelRate(camSPEED);
                            camDeceSPEED = CalcAccelRate(camSPEED);
                            // printf("cam acce = %ld", camAcceSPEED);
                            // printf("cam decce = %ld", camDeceSPEED);
                            setCamACCE(camAcceSPEED);
                            setCamDECE(camDeceSPEED);
                            camMOV= LEFT;
                            setCamSPEED(camMOV * camSPEED);
                            startCamMOTOR();
                            ESP_LOGI(REMOTE_CONTR, "MOVING RIGHT");
                            if (camDirection== FORWORD_RIGHT)
                            {
                                ESP_LOGI(REMOTE_CONTR, " Previous cam direction is = FORWORD_RIGHT");  
                                camDirection= FORWORD_LEFT;
                                ESP_LOGI(REMOTE_CONTR, " setting FORWORD_LEFT");  
                                saveCamDirection();
                            }
                            else if (camDirection== BACKWORD_RIGHT )
                            {
                                ESP_LOGI(REMOTE_CONTR, " Previous cam direction is BACKWORD_RIGHT");  
                                camDirection= BACKWORD_LEFT;
                                ESP_LOGI(REMOTE_CONTR, " setting BACKWORD_LEFT");  
                                saveCamDirection();
                            }  
                            // vTaskDelay(pdMS_TO_TICKS(1000)); 
                            stopCamMOTOR();
                        }
                        break;

                
                case FORWORD_RIGHT: ESP_LOGI(REMOTE_CONTR, "RIGHT CAM PRESSED");
                        // if ( 1==flagMotorRUN )
                        {
                            stopCamMOTOR();
                            camDirection =(int8_t)ev.state;
                            camRatiof = (float)camRatio/1000;
                            camSPEED= (int)(((winchSPEED*pulseRate)/60)*camRatiof);
                            camAcceSPEED = CalcAccelRate(camSPEED);
                            camDeceSPEED = CalcAccelRate(camSPEED);
                            // printf("cam acce = %ld", camAcceSPEED);
                            // printf("cam decce = %ld", camDeceSPEED);
                            setCamACCE(camAcceSPEED);
                            setCamDECE(camDeceSPEED);
                            camMOV= RIGHT;
                            setCamSPEED(camMOV * camSPEED);
                            startCamMOTOR();
                            ESP_LOGI(REMOTE_CONTR, "MOVING LEFT %d", camDirection);
                            saveCamDirection();


                            if ( 0==flagMotorRUN )
                            {
                                acceSPEED = CalcAccelRate((int)(winchSPEED*pulseRate)/60);
                                deceSPEED = CalcAccelRate((int)(winchSPEED*pulseRate)/60);
                                setACCE( acceSPEED );
                                setDECE( acceSPEED );
                                setSPEED(((int)(winchSPEED*pulseRate)/60));
                                printf("speed = %d", ((int)(winchSPEED*pulseRate)/60));
                                startMOTOR();
                                lcd_clear();
                                sprintf(buffer, "FORWORD= %ld ", winchSPEED);
                                flagMotorRUN =1;
                            }


                            }
                        break;
                case FORWORD_LEFT: ESP_LOGI(REMOTE_CONTR, "LEFT CAM PRESSED");
                        // if ( 1==flagMotorRUN )
                        {
                            stopCamMOTOR();
                            camDirection =(int8_t)ev.state;
                            camRatiof = (float)camRatio/1000;
                            camSPEED= (int)(((winchSPEED*pulseRate)/60)*camRatiof);
                            camAcceSPEED = CalcAccelRate(camSPEED);
                            camDeceSPEED = CalcAccelRate(camSPEED);
                            // printf("cam acce = %ld", camAcceSPEED);
                            // printf("cam decce = %ld", camDeceSPEED);
                            setCamACCE(camAcceSPEED);
                            setCamDECE(camDeceSPEED);
                            camMOV= LEFT;
                            setCamSPEED(camMOV * camSPEED);
                            startCamMOTOR();
                            ESP_LOGI(REMOTE_CONTR, "MOVING RIGHT %d", camDirection);
                            saveCamDirection();

                            if ( 0==flagMotorRUN )
                            {
                                acceSPEED = CalcAccelRate((int)(winchSPEED*pulseRate)/60);
                                deceSPEED = CalcAccelRate((int)(winchSPEED*pulseRate)/60);
                                setACCE( acceSPEED );
                                setDECE( acceSPEED );
                                setSPEED(((int)(winchSPEED*pulseRate)/60));
                                printf("speed = %d", ((int)(winchSPEED*pulseRate)/60));
                                startMOTOR();
                                lcd_clear();
                                sprintf(buffer, "FORWORD= %ld ", winchSPEED);
                                flagMotorRUN =1;
                            }

                        }
                        break;
                case BACKWORD_RIGHT: ESP_LOGI(REMOTE_CONTR, "RIGHT CAM PRESSED");
                        // if ( 1==flagMotorRUN )
                        {
                            stopCamMOTOR();
                            camDirection =(int8_t)ev.state;
                            camRatiof = (float)camRatio/1000;
                            camSPEED= (int)(((winchSPEED*pulseRate)/60)*camRatiof);
                            camAcceSPEED = CalcAccelRate(camSPEED);
                            camDeceSPEED = CalcAccelRate(camSPEED);
                            // printf("cam acce = %ld", camAcceSPEED);
                            // printf("cam decce = %ld", camDeceSPEED);                            
                            setCamACCE(camAcceSPEED);
                            setCamDECE(camDeceSPEED);
                            camMOV= RIGHT;
                            setCamSPEED(camMOV * camSPEED);
                            startCamMOTOR();
                            ESP_LOGI(REMOTE_CONTR, "MOVING LEFT %d", camDirection);
                            saveCamDirection();

                            if ( 0==flagMotorRUN )
                            {
                            acceSPEED = CalcAccelRate((int)(winchSPEED*pulseRate)/60);
                            deceSPEED = CalcAccelRate((int)(winchSPEED*pulseRate)/60);
                            setACCE( acceSPEED );
                            setDECE( acceSPEED );
                            setSPEED(-1*((int)(winchSPEED*pulseRate)/60));
                            printf("speed = %d", ((int)(winchSPEED*pulseRate)/60));
                            startMOTOR();
                            lcd_clear();
                            sprintf(buffer, "FORWORD= %ld ", winchSPEED);
                            flagMotorRUN =1;
                            }

                        }
                        break;
                case BACKWORD_LEFT: ESP_LOGI(REMOTE_CONTR, "LEFT CAM PRESSED");
                        // if ( 1==flagMotorRUN )
                        {
                            stopCamMOTOR();
                            camDirection =(int8_t)ev.state;
                            camRatiof = (float)camRatio/1000;
                            camSPEED= (int)(((winchSPEED*pulseRate)/60)*camRatiof);
                            camAcceSPEED = CalcAccelRate(camSPEED);
                            camDeceSPEED = CalcAccelRate(camSPEED);
                            // printf("cam acce = %ld", camAcceSPEED);
                            // printf("cam decce = %ld", camDeceSPEED);
                            setCamACCE(camAcceSPEED);
                            setCamDECE(camDeceSPEED);
                            camMOV= LEFT;
                            setCamSPEED(camMOV * camSPEED);
                            startCamMOTOR();
                            ESP_LOGI(REMOTE_CONTR, "MOVING RIGHT %d", camDirection);
                            saveCamDirection();


                            if ( 0==flagMotorRUN )
                            {
                            acceSPEED = CalcAccelRate((int)(winchSPEED*pulseRate)/60);
                            deceSPEED = CalcAccelRate((int)(winchSPEED*pulseRate)/60);
                            setACCE( acceSPEED );
                            setDECE( acceSPEED );
                            setSPEED(-1*((int)(winchSPEED*pulseRate)/60));
                            printf("speed = %d", ((int)(winchSPEED*pulseRate)/60));
                            startMOTOR();
                            lcd_clear();
                            sprintf(buffer, "FORWORD= %ld ", winchSPEED);
                            flagMotorRUN =1;
                            }

                        }
                        break;
                case FORWORD: 
                        if(0==flagMotorRUN)
                        {
                            flagMotorRUN =1;
                            flagCamRUN = 1;
                            acceSPEED = CalcAccelRate((int)(winchSPEED*pulseRate)/60);
                            deceSPEED = CalcAccelRate((int)(winchSPEED*pulseRate)/60);
                            // printf("acce = %ld", acceSPEED);
                            // printf("decce = %ld", deceSPEED);
                            // setACCE(acceRate);
                            // setDECE(acceRate);
                            setACCE( acceSPEED );
                            setDECE( acceSPEED );
                            setSPEED(((int)(winchSPEED*pulseRate)/60));
                            printf("speed = %d", ((int)(winchSPEED*pulseRate)/60));

                            camRatiof = (float)camRatio/1000;
                            camSPEED= (int)(((winchSPEED*pulseRate)/60)*camRatiof);
                            // printf("camSpeed= %d\r\n", (int)(((winchSPEED*pulseRate)/60)*camRatiof));
                            // printf("camratio= %f\r\n", camRatiof);

                            startMOTOR();
                            lcd_clear();
                            sprintf(buffer, "FORWORD= %ld ", winchSPEED);
                            lcd_put_cur(0, 0);
                            lcd_send_string(buffer);
                            camAcceSPEED = CalcAccelRate(camSPEED);
                            camDeceSPEED = CalcAccelRate(camSPEED);      
                            if ((camDirection== FORWORD_RIGHT) || (camDirection== BACKWORD_LEFT) )
                            {
                                camMOV= RIGHT;
                                ESP_LOGI(REMOTE_CONTR, " Previous cam direction is = %d FORWORD RIGHT/ BACKWORD_LEFT",camDirection);  
                                camDirection= FORWORD_RIGHT;
                                ESP_LOGI(REMOTE_CONTR, " setting FORWORD_RIGHT =%d",camDirection);  
                                saveCamDirection();
                            }
                            else if ((camDirection== FORWORD_LEFT) || (camDirection== BACKWORD_RIGHT) )
                            {
                                camMOV= LEFT;
                                ESP_LOGI(REMOTE_CONTR, " Previous cam direction is = %dFORWORD LEFT/ BACKWORD_RIGHT",camDirection);  
                                camDirection= FORWORD_LEFT;
                                ESP_LOGI(REMOTE_CONTR, " setting FORWORD_LEFT =%d",camDirection);  
                                saveCamDirection();
                            }  
                            // else
                            // {
                            //     camMOV=LEFT;
                            //     camDirection =FORWORD_RIGHT;
                            //     ESP_LOGI(REMOTE_CONTR, " setting FORWORD_RIGHT");                                  
                            //     saveCamDirection(camDirection);
                            // }                
                            setCamACCE(camAcceSPEED);
                            setCamDECE(camDeceSPEED);
                            printf("cam acce = %ld", camAcceSPEED);
                            // printf("cam decce = %ld", camDeceSPEED);
                            setCamSPEED(camMOV * camSPEED);
                            startCamMOTOR();
                            ESP_LOGI(REMOTE_CONTR, "FORWORD");                           

                        }

                        break;                                                
                case BACKWORD: 
                        if(0==flagMotorRUN)
                        {
                            flagMotorRUN =1;
                            flagCamRUN = 1;
                            acceSPEED = CalcAccelRate(((int)(winchSPEED*pulseRate)/60));
                            deceSPEED = CalcAccelRate(((int)(winchSPEED*pulseRate)/60));
                            // printf("acce = %ld", acceSPEED);
                            // printf("decce = %ld", deceSPEED);
                            // setACCE(acceRate);
                            // setDECE(acceRate);
                            setACCE( acceSPEED );
                            setDECE( deceSPEED );
                            setSPEED(-1*((int)(winchSPEED*pulseRate)/60));
                            printf("speed = %d", ((int)(winchSPEED*pulseRate)/60));
                            camRatiof = (float)camRatio/1000;
                            camSPEED= (int)(((int)(winchSPEED*pulseRate)/60)*camRatiof);
                            startMOTOR();
                            lcd_clear();
                            sprintf(buffer, "BACKWORD= %ld", winchSPEED);
                            lcd_put_cur(0, 0);
                            lcd_send_string(buffer);
                            camAcceSPEED = CalcAccelRate(camSPEED);
                            camDeceSPEED = CalcAccelRate(camSPEED);
                            if ((camDirection== FORWORD_RIGHT) || (camDirection== BACKWORD_LEFT) )
                            {
                                camMOV=LEFT;
                                ESP_LOGI(REMOTE_CONTR, " Previous cam direction is = %d FORWORD RIGHT/ BACKWORD_LEFTT",camDirection);  
                                camDirection = BACKWORD_LEFT;
                                ESP_LOGI(REMOTE_CONTR, " setting BACKWORD_LEFT=%d",camDirection);  

                                saveCamDirection();

                            }
                            else if ((camDirection== FORWORD_LEFT) || (camDirection== BACKWORD_RIGHT) )
                            {
                                camMOV= RIGHT;
                                ESP_LOGI(REMOTE_CONTR, " Previous cam direction is = %d FORWORD LEFT/ BACKWORD_RIGHT",camDirection); 
                                camDirection = BACKWORD_RIGHT;
                                ESP_LOGI(REMOTE_CONTR, " setting BACKWORD_RIGHT=%d",camDirection);  
                                
                                saveCamDirection();
                            }  
                            // else
                            // {
                            //     camMOV=RIGHT;
                            //     camDirection = BACKWORD_LEFT;
                            //     ESP_LOGI(REMOTE_CONTR, " setting BACKWORD_LEFT");  
                            //     saveCamDirection(camDirection);
                            // }  
                            setCamACCE(camAcceSPEED);
                            setCamDECE(camDeceSPEED);
                            printf("cam acce = %ld", camAcceSPEED);
                            // printf("cam decce = %ld", camDeceSPEED);
                            setCamSPEED(camMOV * camSPEED);
                            startCamMOTOR();
                            ESP_LOGI(REMOTE_CONTR, "BACKWORD");

                        }

                        break;
                case SPEED_UP: ESP_LOGI(REMOTE_CONTR, "SPEED UP");
                        if(0==flagMotorRUN)
                        {
                            if (MAX_SPEED >= (winchSPEED+ incSPEED))
                            {
                            winchSPEED+=incSPEED;
                            }
                            ESP_LOGI(REMOTE_CONTR, "SPEED = %ldRPM", winchSPEED);
                            lcd_clear();
                            sprintf(buffer, "SPEED= %ldRPM", winchSPEED);
                            lcd_put_cur(0, 0);
                            lcd_send_string(buffer);                            

                        }
                        break;
                case SPEED_DOWN: ESP_LOGI(REMOTE_CONTR, "SPEED DOWN");
                        if(0==flagMotorRUN)
                        {
                            if (MIN_SPEED <= (winchSPEED- incSPEED))
                            {
                                winchSPEED-=incSPEED;

                            }
                            ESP_LOGI(REMOTE_CONTR, "SPEED = %ldRPM", winchSPEED);
                            lcd_clear();
                            sprintf(buffer, "SPEED= %ldRPM", winchSPEED);
                            lcd_put_cur(0, 0);
                            lcd_send_string(buffer);                               
                        }                
                        break;
                
                case CAM_SHIFT_RIGHT: ESP_LOGI(REMOTE_CONTR, "CAM_SHIFT_RIGHT PRESSED");
                        if (0 == camShiftFlag)
                        {
                            // stopCamMOTOR();
                            camRatiof = (float)camRatio/1000;
                            camSPEED= (int)(((winchSPEED*pulseRate)/60)*camRatiof);
                            camAcceSPEED = camSPEED;
                            camDeceSPEED = camSPEED;
                            // printf("cam acce = %ld", camAcceSPEED);
                            // printf("cam decce = %ld", camDeceSPEED);
                            setCamACCE(camAcceSPEED);
                            setCamDECE(camDeceSPEED);
                            camMOV= RIGHT;
                            setCamSPEED(camMOV * camSPEED);
                            startCamMOTOR();
                            ESP_LOGI(REMOTE_CONTR, "MOVING RIGHT");
                            camShiftFlag = 1;
                            }
                        break;
                 case CAM_SHIFT_LEFT: ESP_LOGI(REMOTE_CONTR, "CAM_SHIFT_LEFT PRESSED");
                        if (0 == camShiftFlag)
                        {
                            // stopCamMOTOR();
                            camRatiof = (float)camRatio/1000;
                            camSPEED= (int)(((winchSPEED*pulseRate)/60)*camRatiof);
                            camAcceSPEED = camSPEED;
                            camDeceSPEED = camSPEED;
                            // printf("cam acce = %ld", camAcceSPEED);
                            // printf("cam decce = %ld", camDeceSPEED);
                            setCamACCE(camAcceSPEED);
                            setCamDECE(camDeceSPEED);
                            camMOV= LEFT;
                            setCamSPEED(camMOV * camSPEED);
                            startCamMOTOR();
                            ESP_LOGI(REMOTE_CONTR, "MOVING LEFT");
                            camShiftFlag = 1;
                            }
                        break;

                default: ESP_LOGI(REMOTE_CONTR, "BUTTON RELEASED");
                        
                        if((ev.state==FALT1)||(ev.state==FALT2)||(ev.state==FALT3)||(ev.state==FALT4)||(ev.state==FALT5)||(ev.state==FALT6))
                        {
                                                   
                            if ( 1==flagMotorRUN )
                            {   
                                stopMOTOR();
                                immediateStopCamMOTOR();
                                flagMotorRUN =0;
                                ESP_LOGI(REMOTE_CONTR, "MOTOR STOP");
                                cmdFlag=0;
                            }
                            ESP_LOGI(REMOTE_CONTR, "SOMETHING WRONG");
                            lcd_clear();
                            sprintf(buffer, "SOMETHING WRONG");
                            lcd_put_cur(0, 0);
                            lcd_send_string(buffer);
                            errorFlag =1;
                        }
                        else
                        { 
                            if ( 1==flagMotorRUN )
                            {   
                                stopMOTOR();
                                stopCamMOTOR();
                                flagMotorRUN =0;
                                // flagCamRUN =0;
                                timerCam = stoptime;
                                printf("starting timerCam");
                                ESP_LOGI(REMOTE_CONTR, "MOTOR STOP");
                                lcd_clear();
                                sprintf(buffer, "MOTOR STOPS");
                                lcd_put_cur(0, 0);
                                lcd_send_string(buffer);
                                // cmdFlag=0;
                            }
                            else if (( 1==flagCamRUN )&& (0==timerCam))
                            {
                                stopCamMOTOR();
                                flagCamRUN=0;
                            }
                            else if (1 == camShiftFlag){
                                immediateStopCamMOTOR();
                                camShiftFlag = 0;
                            }
                            errorFlag =0;
                            cmdFlag=0;

                        }
                        break;  

            }
        }
        serialFlag =0;
        // used to detect state changes which trigger sending a packet
        last_buttons = buttons;
    }

}


void app_main(void)
{
    
    // esp_err_t res;
    float temperature1=0, temperature2=0;

    // Initialize NVS==============================================
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(REMOTE_CONTR,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(REMOTE_CONTR,"Done\n");

        // Read
        ESP_LOGI(REMOTE_CONTR,"Reading camDirection from NVS ... ");
        err = nvs_get_i8(my_handle, "camDirection", &camDirection);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(REMOTE_CONTR,"Done\n");
                ESP_LOGI(REMOTE_CONTR,"Cam Direction = %d\n", camDirection);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGE(REMOTE_CONTR,"The value is not initialized yet!\n");
                break;
            default :
                ESP_LOGE(REMOTE_CONTR,"Error (%s) reading!\n", esp_err_to_name(err));
        }
         vTaskDelay(100 / portTICK_PERIOD_MS);

        err = nvs_get_i8(my_handle, "stoptime", &stoptime);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(REMOTE_CONTR,"Done\n");
                ESP_LOGI(REMOTE_CONTR,"stop time = %d\n", stoptime);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGE(REMOTE_CONTR,"The value stoptime is not initialized yet!\n");
                break;
            default :
                ESP_LOGE(REMOTE_CONTR,"Error (%s) readingstoptime !\n", esp_err_to_name(err));
        }
         vTaskDelay(100 / portTICK_PERIOD_MS);

        err = nvs_get_i32(my_handle, "winchSPEED", &winchSPEED);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(REMOTE_CONTR,"Done\n");
                ESP_LOGI(REMOTE_CONTR,"winch SPEED = %ld\n", winchSPEED);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGE(REMOTE_CONTR,"The value winchSPEED is not initialized yet!\n");
                break;
            default :
                ESP_LOGE(REMOTE_CONTR,"Error (%s) reading!\n", esp_err_to_name(err));
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);

        err = nvs_get_i32(my_handle, "camRatio", &camRatio);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(REMOTE_CONTR,"Done\n");
                ESP_LOGI(REMOTE_CONTR,"camRatio = %ld\n", camRatio);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGE(REMOTE_CONTR,"The value camRatio is not initialized yet!\n");
                break;
            default :
                ESP_LOGE(REMOTE_CONTR,"Error (%s) reading!\n", esp_err_to_name(err));
            }

        vTaskDelay(100 / portTICK_PERIOD_MS);


        err = nvs_get_i32(my_handle, "pulseRate", &pulseRate);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(REMOTE_CONTR,"Done\n");
                ESP_LOGI(REMOTE_CONTR,"pulseRate = %ld\n", pulseRate);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGE(REMOTE_CONTR,"The value pulseRate is not initialized yet!\n");
                break;
            default :
                ESP_LOGE(REMOTE_CONTR,"Error (%s) reading!\n", esp_err_to_name(err));
            }

        vTaskDelay(100 / portTICK_PERIOD_MS);
            // Close
        nvs_close(my_handle);
    }

    //=================================================

    initUART();
    initUART1();
    initUART2();
    ESP_LOGI(REMOTE_CONTR, "I2C initialized successfully");
    
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 3, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(rx_task1, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
    // xTaskCreate(tx_task1, "uart_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(rx_task2, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
    // xTaskCreate(tx_task2, "uart_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
    ESP_LOGI(REMOTE_CONTR, "Initialization DONE");

    xTaskCreate(remote_read_task, "remote_read_task",  1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
    


    ESP_ERROR_CHECK(i2c_master_init());
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    lcd_init();
    sprintf(buffer, "EYEROV AUTOWINCH");
    lcd_clear();
    lcd_put_cur(0, 0);
    lcd_send_string(buffer);
    
    while (1) {
        
        // //====================================================================
        // res = ds18x20_measure_and_read(SENSOR_GPIO, SENSOR_ADDR1, &temperature1);
        // if (res != ESP_OK)
        //     ESP_LOGE(REMOTE_CONTR, "Could not read from sensor %08" PRIx32 "%08" PRIx32 ": %d (%s)",
        //             (uint32_t)(SENSOR_ADDR1 >> 32), (uint32_t)SENSOR_ADDR1, res, esp_err_to_name(res));
        // else
        //     // ESP_LOGI(REMOTE_CONTR, "Sensor 1 %08" PRIx32 "%08" PRIx32 ": %.2f°C",
        //     //         (uint32_t)(SENSOR_ADDR1 >> 32), (uint32_t)SENSOR_ADDR1, temperature1);
        
        
        // res = ds18x20_measure_and_read(SENSOR_GPIO, SENSOR_ADDR2, &temperature2);
        // if (res != ESP_OK)
        //     ESP_LOGE(REMOTE_CONTR, "Could not read from sensor %08" PRIx32 "%08" PRIx32 ": %d (%s)",
        //             (uint32_t)(SENSOR_ADDR2 >> 32), (uint32_t)SENSOR_ADDR2, res, esp_err_to_name(res));
        // else
            // ESP_LOGI(REMOTE_CONTR, "Sensor 2 %08" PRIx32 "%08" PRIx32 ": %.2f°C",
            //         (uint32_t)(SENSOR_ADDR2 >> 32), (uint32_t)SENSOR_ADDR2, temperature2);
        //====================================================================            

        vTaskDelay(pdMS_TO_TICKS(1000));   
        if((0==flagMotorRUN) && (errorFlag==0))
        {        
            // lcd_clear();
            sprintf(buffer, "EYEROV AUTOWINCH");
            lcd_put_cur(0, 0);
            lcd_send_string(buffer);    
        } 
        else if (1==flagMotorRUN)                    
        {
            lcd_clear();
            sprintf(buffer, "SPEED= %ld", movingAverage);
            lcd_put_cur(0, 0);
            lcd_send_string(buffer);
        }

        sprintf(buffer, "T1=%d\'C T2=%d\'C", (int)temperature1,(int) temperature2);
        lcd_put_cur(1, 0);
        lcd_send_string(buffer);
    
    
    
        if (0 < timerCam)
        {
            printf("timerCam= %d\r\n", timerCam);
            timerCam--;
            if ((0 ==timerCam) && (0== flagMotorRUN))
            {
                flagCamRUN =0;
                printf("timerCam stops\r\n");
            }

        }
        if (1 == flagMotorRUN){
        readSpeed();
        }
    }
}



uint16_t CRC16_calc( unsigned char * frame, uint16_t len )
{
 unsigned char ucCRCHi = 0xFF;
 unsigned char ucCRCLo = 0xFF;
 int16_t iIndex;
 while( len-- )
 {
 iIndex = ucCRCLo ^ *( frame++ );
 ucCRCLo = ( unsigned char )( ucCRCHi ^ CRCHi[iIndex] );
 ucCRCHi = CRCLo[iIndex];
 }
 return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}



int32_t CalcAccelRate(int32_t speed){
    int32_t AccelRate=0;

    AccelRate = (speed/stoptime);
    return AccelRate;  
}