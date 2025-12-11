/*
 * lidar.c
 *
 *  Created on: Nov 20, 2025
 *      Author: darrendong
 */

#include "lidar.h"
#include "lcd.h"
#include <stdio.h>

extern bool zone_mode;
extern bool filter_mode;

int new_scan_flag = false;
uint16_t* f = pingframe;


const uint8_t express_scan[9] = {
            0xA5, 0x82,
            0x05,                    // Payload length
            0x00,                    // working_mode (0 for legacy express)
            0x00, 0x00,             // working_flags
            0x00, 0x00,             // param
            0x22                     // Checksum
        };

const uint8_t stop[2] = {
            0xA5, 0x25
        };

const uint8_t scan[2] = {
            0xA5, 0x20
        };

void send_stop_command(UART_HandleTypeDef* huart_addr) {
    HAL_UART_Transmit(huart_addr, stop, sizeof(stop), HAL_MAX_DELAY);
}

void send_express_scan_command(UART_HandleTypeDef* huart_addr) {
    HAL_UART_Transmit(huart_addr, express_scan, sizeof(express_scan), HAL_MAX_DELAY);
}

void send_scan_command(UART_HandleTypeDef* huart_addr){
    HAL_UART_Transmit(huart_addr, scan, sizeof(scan), HAL_MAX_DELAY);
}

bool decode_normal_scan(uint8_t* capsule_data) {
    // Validate quality
    uint8_t quality = capsule_data[0] >> 2;
    if (quality > 15) {
        return false; // Invalid packet
    }

    // Validate s and ~s bits
    bool new_scan = (capsule_data[0] & 0b1) != 0;
    bool inversed_new_scan = ((capsule_data[0] >> 1) & 0b1) != 0;
    if (new_scan == inversed_new_scan) {
        return false; // Invalid packet
    }

    // Validate c bit
    uint8_t check_bit = capsule_data[1] & 0b1;
    if (check_bit != 1) {
        return false; // Invalid packet
    }

    if (new_scan){
		new_scan_flag = true;
	}

    // If all validations pass, proceed with decoding
    uint16_t angle_raw = ((capsule_data[1] >> 1) + (capsule_data[2] << 7));

    float angle = angle_raw / 64.0f;

    if ((angle > BOUND1LOW && angle < BOUND1HIGH) || (angle > BOUND2LOW && angle < BOUND2HIGH)) {
    	return true;
	}

    uint16_t distance_raw = capsule_data[3] + (capsule_data[4] << 8);
    float distance_mm = distance_raw / 4.0f;

    if (quality >= MIN_SCAN_QUALITY) {
		UpdateZone(angle, distance_mm);
		if (!zone_mode) {
			if(filter_mode){
				coneZone(angle, distance_mm);
			}
			else{	
				DrawPoint(angle, distance_mm);
			}
		}
    }

    return true; // Valid packet
}
