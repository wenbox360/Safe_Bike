#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "stm32l4xx_hal.h"

// A1 LIDAR Protocol Constants
#define SL_LIDAR_CMD_SYNC_BYTE              0xA5
#define SL_LIDAR_CMD_GET_DEVICE_INFO        0x50
#define SL_LIDAR_CMD_EXPRESS_SCAN           0x82
#define SL_LIDAR_CMD_SET_MOTOR_PWM          0xF0
#define SL_LIDAR_CMD_STOP                   0x25

#define MIN_SCAN_QUALITY 10

#define BOUND1LOW 150
#define BOUND1HIGH 210
#define BOUND2LOW 150
#define BOUND2HIGH 210

//float angle_diff(float angle1, float angle2);


void send_stop_command(UART_HandleTypeDef* huart_addr);

void send_scan_command(UART_HandleTypeDef *huart_addr);

void send_express_scan_command(UART_HandleTypeDef* huart_addr);


//void decode_express_capsule(uint8_t* capsule_data);
bool decode_normal_scan(uint8_t* raw);
// void decode_normal_scan(uint8_t* raw);
