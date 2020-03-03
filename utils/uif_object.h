/*
*********************************************************************************************************
*
*                                          APP PACKAGE
*
*                                         Atmel  AT91SAMA5D3
*                                             on the
*                                      Audio Bridge 04 Board (AB04 V1.0)
*
* Filename      : object.h
* Version       : V0.0.1
* Programmer(s) : Leo
*********************************************************************************************************
* Note(s)       :abstract data struct for communicating port
*********************************************************************************************************
*/

#ifndef __OBJECT_H__
#define __OBJECT_H__


#define PINGPONG

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//#include "board.h"
#include "kfifo.h"

//define stream direct
typedef enum _port_direct {
        IN = 0,
        OUT,
        BI,
        INVALIDDIR,
} DIRECT;

//define application layer direct
typedef enum _audio_direct {
        REC = 0,
        PLAY,
        INVALIDAUDIODIR,
} AUDIODIRECT;

//trace port status;
typedef enum _status {
        FREE = 0,
        CONFIGURED,
        START,
        BUFFERED,
        RUNNING,
        BUFFEREMPTY,
        BUFFERFULL,
        STOP
} STATUS;

//define port that all of uif board used;
typedef enum _uif_port {
        UIDSSC0 = 1,
        UIFSSC1 = 2,
        SPI = 4,
        GPIO = 8, //treat it as a port now,if has better idea,change it;
        I2C = 0x10,
        INVALIDPORT
} UIFPORT;

typedef enum _uif_port_mask {
        SSC0_IN = 1,
        SSC0_OUT = 2,
        SSC1_IN = 4,
        SSC1_OUT = 8,
        SPI0_IN = 16,
        SPI0_OUT = 32,
        GPIO_IN = 64,
        INVALIDMASK
} UIFPORTMASK;

typedef struct _abstractDevice
{
        uint32_t identify;       //peripheral id that defined by datasheet
        uint32_t instanceHandle; //store port handle
        uint32_t rxDMAChannel;   //dma rx channel with this port
        uint32_t txDMAChannel;   //dma tx channel with this port
        uint32_t direct;         //data stream direct
        uint32_t sampleRate;     //for extend in the furture
} Device;
#pragma pack(4)
typedef struct _DataSource
{
        //public interface to reveal ucosII task,that I want to hide peripheral info in this struct;
        void (*init_source)(void *pParameter, void *dParameter);     //initialize the hardware according result that parse protocol
        uint8_t (*set_peripheral)(void *pInstance, void *parameter); //set register;
        void (*peripheral_start)(void *pInstance);                   //starting peripheral
        void (*peripheral_stop)(void *pInstance);                    //stoping peripheral
        uint8_t (*buffer_read)(void *pInstance,
                               const uint8_t *buf,
                               uint32_t len);
        uint8_t (*buffer_write)(void *pInstance,
                                const uint8_t *buf,
                                uint32_t len);

        //private info about peripheral used;
        Device dev;
        void *peripheralParameter; //peripheral register parameter;
        void *privateData;         //user config parameter;
        uint32_t warmWaterLevel;   //corresponding to i2s_play_buffer_size,maybe
                                   //should move it to another struct?
                                   //these points followed pointed public buffers
        kfifo_t *pRingBulkOut;     //the out ring buffer that this port relevant
        kfifo_t *pRingBulkIn;      //the in  ring buffer that this port relevant
        uint8_t *pBufferIn;        //the pointer pointed PingPong buffer in
        uint8_t *pBufferOut;       //the pointer pointed PingPong buffer out
        uint8_t status[BI];        //state machine of port
        uint16_t isSync;           //keep alive
        uint16_t isDoublePort;     //
        uint16_t slotNum;          //

#ifdef PINGPONG
        uint8_t tx_index; //indicate current buffer is Ping or Pong;
        uint8_t rx_index;
        //uint32_t tx_index_cnt_0;
        //uint32_t tx_index_cnt_1;
        //uint32_t rx_index_cnt_0;
        //uint32_t rx_index_cnt_1;
        
        uint32_t txSize; //indicate actul data size of 2ms
        uint32_t rxSize;
#endif
        uint32_t baud_rate;
        void (*handler)(void);
} DataSource;
#pragma pack()
#endif
