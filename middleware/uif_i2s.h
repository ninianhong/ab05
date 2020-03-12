#ifndef _UIF_I2S_H_
#define _UIF_I2S_H_

#include "ssc.h"

#include "uif_usb.h"

#define SAMPLE_RATE             (48000)
#define SLOT_BY_FRAME           (8)
#define BITS_BY_SLOT            (16)


int _ssc0_rx_ep2_rec_cb(void* arg, void* arg2);
int _ssc1_rx_ep6_rec_cb(void* arg, void* arg2);
int _i2s0_rx_ep11_rec_cb(void* arg, void* arg2);
int _i2s1_rx_ep13_rec_cb(void* arg, void* arg2);
int _ssc0_tx_ep1_play_cb(void* arg, void* arg2);
int _ssc1_tx_ep5_play_cb(void* arg, void* arg2);
int _i2s0_tx_ep10_play_cb(void* arg, void* arg2);
int _i2s1_tx_ep12_play_cb(void* arg, void* arg2);
void starting_ssc0_play( void );
void starting_ssc1_play( void );
void starting_ssc0_record( void );
void starting_ssc1_record( void );

#endif
