#ifndef _CONSTANT_H_
#define _CONSTANT_H_

#define I2S_PLAY_PRE_BUF_NUM               8 // 10 x I2S_PINGPONG_OUT_SIZE_3K,  4ms * 10 = 40ms preplay buffer, 20? 2018-05-19 
#define I2S_PINGPONG_BUF_SIZE_MS           4  // 4ms per ssc pingpong buffer
#define App_Task_SSC_Dma_timeout_DELAY 1 // ms

#define I2S_PINGPONG_IN_SIZE_3K            (48*I2S_PINGPONG_BUF_SIZE_MS*8*4)   
#define I2S_PINGPONG_OUT_SIZE_3K           (48*I2S_PINGPONG_BUF_SIZE_MS*8*4) 


#define USB_DATAEP_SIZE_64B                (    64*1    )            // force use 64Bytes
#define USB_LOGEP_SIZE_256B                (    256   )            // force use 256Bytes
#define USB_CMDEP_SIZE_64B                 1024//( 64 )

#define USB_RINGOUT_SIZE_16K               (16384*8) //USB audio data, size MUST be 2^n .2^14=16384
#define USB_RINGIN_SIZE_16K                (16384*8) //USB audio data, size MUST be 2^n .2^14=16384
#define USB_CMD_RINGOUT_SIZE_16K           ( 16384  )                  //USB cmd data, size MUST be 2^n .
#define USB_CMD_RINGIN_SIZE_16k            ( 16384  )                  //USB cmd data, size MUST be 2^n .
#define USART_BUFFER_SIZE_1K               ( 1024  )                //buffer size of usart

#define SPI_RINGOUT_SIZE_50K               ( 3072 * 2 )              //3072B/2ms * 16 > ( 3072B * 5 )
#define SPI_RINGIN_SIZE_50K                ( 3072 * 2 )              //3072B/2ms * 16 > ( 3072B * 5 )

#define FIRST_PADDING_SIZE                 ( 128 )

#endif
