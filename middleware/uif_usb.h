#ifndef _UIF_USB_H_
#define _UIF_USB_H_

#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "cdcd_serial_driver.h"
#include "usbd.h"
#include "usbd_hal.h"

#include "uif_list.h"

#define I2S_PINGPONG_BUF_SIZE_MS           4  // 4ms per ssc pingpong buffer
#define I2S_PINGPONG_SIZE_6K               6144 //(48*I2S_PINGPONG_BUF_SIZE_MS*8*4)

extern List ep2_ssc0_rec;
extern List ep4_cmd_rec;
extern List ep6_ssc1_rec;
extern List ep8_spi_rec;
extern List ep9_log_rec;
extern List ep11_i2s0_rec;

extern List ep1_ssc0_play;
extern List ep3_cmd_play;
extern List ep5_ssc1_play;
extern List ep7_spi_play;
extern List ep10_i2s0_play;



extern uint8_t usb_ep1_ssc0_play_data[128*I2S_PINGPONG_SIZE_6K];
extern uint8_t usb_ep3_cmd_play_data[128*I2S_PINGPONG_SIZE_6K];
extern uint8_t usb_ep5_ssc1_play_data[128*I2S_PINGPONG_SIZE_6K];
extern uint8_t usb_ep7_spi_play_data[128*I2S_PINGPONG_SIZE_6K];
extern uint8_t usb_ep10_i2s0_play_data[128*I2S_PINGPONG_SIZE_6K];

extern uint8_t usb_ep2_ssc0_record_data[128*I2S_PINGPONG_SIZE_6K];
extern uint8_t usb_ep4_cmd_record_data[128*I2S_PINGPONG_SIZE_6K];
extern uint8_t usb_ep6_ssc1_record_data[128*I2S_PINGPONG_SIZE_6K];
extern uint8_t usb_ep8_spi_record_data[128*I2S_PINGPONG_SIZE_6K];
extern uint8_t usb_ep9_log_record_data[128*I2S_PINGPONG_SIZE_6K];
extern uint8_t usb_ep11_i2s0_record_data[128*I2S_PINGPONG_SIZE_6K];

void usb_ep1_ssc0_play_cb(void *read, uint8_t status,
		uint32_t received, uint32_t remaining);
void usb_ep3_cmd_play_cb(void *read, uint8_t status,
		uint32_t received, uint32_t remaining);
void usb_ep5_ssc1_play_cb(void *read, uint8_t status,
		uint32_t received, uint32_t remaining);
void usb_ep7_spi_play_cb(void *read, uint8_t status,
		uint32_t received, uint32_t remaining);
void usb_ep10_i2s0_play_cb(void *read, uint8_t status,
		uint32_t received, uint32_t remaining);

void usb_ep2_ssc0_record_cb(void *unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining);
void usb_ep4_cmd_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining);
void usb_ep6_ssc1_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining);
void usb_ep8_spi_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining);

void usb_ep9_log_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining);
void usb_ep11_i2s0_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining);

#endif

