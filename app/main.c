/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2016, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */
/** \cond usb_cdc_serial
 * \page usb_cdc_serial USB CDC Serial Converter Example
 *
 * \section Purpose
 *
 * The USB CDC Serial Project will help you to get familiar with the
 * USB Device Port(UDP) and USART interface on SAMA5D4x microcontrollers. Also
 * it can help you to be familiar with the USB Framework that is used for
 * rapid development of USB-compliant class drivers such as USB Communication
 * Device class (CDC).
 *
 * \section Requirements
 *
 * This package can be used with SAMA5D4EK and SAMA5D4-XULT board.
 *
 * \section Description
 *
 * This demo simulates a USB to RS-232 Serial Port Converter.
 *
 * When the board running this program connected to a host (PC for example), with
 * USB cable, the board appears as a serial COM port for the host, after driver
 * installation with the offered 6119.inf. Then the host can send or receive
 * data through the port with host software. The data stream from the host is
 * then sent to the board, and forward to USART port of AT91SAM chips. The USART
 * port of the board is monitored by the timer and the incoming data will be sent
 * to the host.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6421.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>, depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *     \code
 *     -- USB Device CDC Serial Project xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the host
 *    reports a new USB device attachment (if it's the first time you connect
 *    an %audio speaker demo board to your host). You can use the inf file
 *    libraries\\usb\\device\\cdc-serial\\drv\\6119.inf to install the serial
 *    port. Then new "AT91 USB to Serial Converter (COMx)" appears in the
 *    hardware device list.
 * -# You can run hyperterminal to send data to the port. And it can be seen
 *    at the other hyperterminal connected to the USART port of the boad.
 *
 * \section References
 * - usb_cdc_serial/main.c
 * - usart: USART interface driver
  * - usb: USB Framework, USB CDC driver and UDP interface driver
 *    - \ref usbd_framework
 *       - \ref usbd_api
 *    - \ref usbd_cdc
 *       - \ref usbd_cdc_serial_drv
 *       - \ref usbd_cdc_host_drv
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_cdc_serial example.
 *
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "chip.h"

#include "trace.h"
#include "compiler.h"
#include "errno.h"

#if 0
#include "mm/cache.h"
#include "serial/console.h"

#include "irq/irq.h"
#include "gpio/pio.h"
#include "peripherals/pit.h"
#include "peripherals/pmc.h"
#include "peripherals/tc.h"
#include "serial/usartd.h"
#include "serial/usart.h"
#else
#include "cache.h"
#include "console.h"

#include "irq.h"
#include "pio.h"
#include "pit.h"
#include "pmc.h"
#include "tc.h"
#include "usartd.h"
#include "usart.h"
#endif

#include "chip_pins.h"
#include "chip_common.h"
#include "bus.h"
#include "uif_list.h"

#include "cdcd_serial_driver.h"
#include "usbd.h"
#include "usbd_hal.h"

#include "main_usb_common.h"
#include "tcd.h"

#include "ssc.h"
#include "codec.h"
   
#include "uif_i2s.h"  
#include "constant.h"

#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/*----------------------------------------------------------------------------
 *      Definitions
 *----------------------------------------------------------------------------*/

/** Size in bytes of the packet used for reading data from the USB & USART */
#define DATAPACKETSIZE (10)

/** Size in bytes of the buffer used for reading data from the USB & USART */
#define DATABUFFERSIZE (DATAPACKETSIZE+2)

/** Basic asynchronous mode, i.e. 8 bits no parity.*/
#define USART_MODE_ASYNCHRONOUS        (US_MR_CHMODE_NORMAL | US_MR_CHRL_8_BIT | US_MR_PAR_NO)

/** test buffer size */
#define TEST_BUFFER_SIZE    (2*1024)

/** write loop count */
#define TEST_COUNT          (1)

/** define the peripherals and pins used for USART */
#if defined(CONFIG_BOARD_SAMA5D2_PTC_EK)
#define USART_ADDR FLEXUSART4
#define USART_PINS PINS_FLEXCOM4_USART_IOS3

#elif defined(CONFIG_BOARD_SAMA5D2_XPLAINED)
#define USART_ADDR FLEXUSART3
#define USART_PINS PINS_FLEXCOM3_USART_IOS3

#elif defined(CONFIG_BOARD_SAMA5D27_SOM1_EK)
#define USART_ADDR FLEXUSART3
#define USART_PINS PINS_FLEXCOM3_USART_IOS2

#elif defined(CONFIG_BOARD_SAMA5D4_XPLAINED)
#define USART_ADDR USART4
#define USART_PINS PINS_USART4

#elif defined(CONFIG_BOARD_SAMA5D4_EK)
#define USART_ADDR USART4
#define USART_PINS PINS_USART4

#elif defined(CONFIG_BOARD_SAMA5D3_XPLAINED)
#define USART_ADDR USART3
#define USART_PINS PINS_USART3

#elif defined(CONFIG_BOARD_SAMA5D3_EK)
#define USART_ADDR USART1
#define USART_PINS PINS_USART1

#elif defined(CONFIG_BOARD_SAM9G15_EK)
#define USART_ADDR USART0
#define USART_PINS PINS_USART0

#elif defined(CONFIG_BOARD_SAM9G25_EK)
#define USART_ADDR USART0
#define USART_PINS PINS_USART0

#elif defined(CONFIG_BOARD_SAM9G35_EK)
#define USART_ADDR USART0
#define USART_PINS PINS_USART0

#elif defined(CONFIG_BOARD_SAM9X25_EK)
#define USART_ADDR USART0
#define USART_PINS PINS_USART0

#elif defined(CONFIG_BOARD_SAM9X35_EK)
#define USART_ADDR USART0
#define USART_PINS PINS_USART0

#elif defined(CONFIG_BOARD_SAM9X60_EK)
#define USART_ADDR FLEXUSART2
#define USART_PINS PINS_FLEXCOM2_USART_IOS1

#elif defined(CONFIG_BOARD_SAME70_XPLAINED)
#define USART_ADDR USART2
#define USART_PINS PINS_USART2

#elif defined(CONFIG_BOARD_SAMV71_XPLAINED)
#define USART_ADDR USART2
#define USART_PINS PINS_USART2

#else
#error Unsupported SoC!
#endif
   
/*----------------------------------------------------------------------------
 *      port list define
 *----------------------------------------------------------------------------*/
DataSource source_ssc0;
DataSource source_ssc1;
DataSource source_spi0;
DataSource source_spi1;
DataSource source_twi0;
DataSource source_twi1;
DataSource source_twi2;
DataSource source_usart0;
DataSource source_usart1;
DataSource source_gpio;   
/*----------------------------------------------------------------------------
 *      buffer define
 *----------------------------------------------------------------------------*/   
#pragma pack(4)
//Buffer Level 1:  USB data stream buffer : 64 B
uint8_t usbCacheBulkOut0[USB_DATAEP_SIZE_64B*256] ;
uint8_t usbCacheBulkIn0[USB_DATAEP_SIZE_64B*256] ;
//uint8_t usbCacheBulkIn0_1[USB_DATAEP_SIZE_64B * 16 * 3 ] ;
uint8_t usbCacheBulkOut1[USB_DATAEP_SIZE_64B*256] ;
uint8_t usbCacheBulkIn1[USB_DATAEP_SIZE_64B*256] ;
//uint8_t usbCacheBulkIn1_1[USB_DATAEP_SIZE_64B * 16 * 3 ] ;
uint8_t usbCacheBulkOut2[USB_DATAEP_SIZE_64B * 16 * 3 ] ;
uint8_t usbCacheBulkIn2[USB_DATAEP_SIZE_64B * 16 * 3 ] ;
uint8_t usbCacheBulkIn3[USB_LOGEP_SIZE_256B] ;
//Buffer Level 1:  USB Cmd data stream buffer : 64 B
uint8_t usbCmdCacheBulkOut[ USB_CMDEP_SIZE_64B ] ;             //64B
uint8_t usbCmdCacheBulkIn[ USB_CMDEP_SIZE_64B ]  ;             //64B

////Buffer Level 2:  Ring Data Buffer for usb: 16384 B
uint8_t usbRingBufferBulkOut0[ USB_RINGOUT_SIZE_16K ] ;        //16384B
uint8_t usbRingBufferBulkIn0[ USB_RINGIN_SIZE_16K ] ;          //16384B
uint8_t usbRingBufferBulkOut1[ USB_RINGOUT_SIZE_16K ] ;        //16384B
uint8_t usbRingBufferBulkIn1[ USB_RINGIN_SIZE_16K ] ;          //16384B
uint8_t usbRingBufferBulkOut2[ USB_RINGOUT_SIZE_16K ] ;        //16384B
uint8_t usbRingBufferBulkIn2[ USB_RINGIN_SIZE_16K ] ;          //16384B
uint8_t usbRingBufferBulkIn3[ USB_RINGIN_SIZE_16K ] ;          //16384B
//Buffer Level 2:  To PC Ring CMD Buffer : 1024 B
uint8_t usbCmdRingBulkOut[ USB_CMD_RINGOUT_SIZE_16K ] ;         //1024B
uint8_t usbCmdRingBulkIn[ USB_CMD_RINGIN_SIZE_16k ]  ;          //1024B
//Buffer Level 2:  To RULER Ring CMD Buffer : 1024 B
uint8_t rulerCmdRingBulkOut[ USART_BUFFER_SIZE_1K ] ;          
uint8_t rulerCmdRingBulkIn[ USART_BUFFER_SIZE_1K ]  ;          

//Buffer Level 3:  Ring  Data Buffer for audio port include ssc and spi: 16384 B
uint8_t ssc0_RingBulkOut[ USB_RINGOUT_SIZE_16K ] ;             //16384B
uint8_t ssc0_RingBulkIn[ USB_RINGIN_SIZE_16K ] ;               //16384B
uint8_t ssc1_RingBulkOut[ USB_RINGOUT_SIZE_16K ] ;             //16384B
uint8_t ssc1_RingBulkIn[ USB_RINGIN_SIZE_16K ] ;               //16384B

uint8_t spi0_RingBulkOut[ SPI_RINGOUT_SIZE_50K ];
uint8_t spi0_RingBulkIn[ SPI_RINGIN_SIZE_50K];
uint8_t spi1_RingBulkOut[ SPI_RINGOUT_SIZE_50K ];
uint8_t spi1_RingBulkIn[ SPI_RINGIN_SIZE_50K ];

//Buffer Level 4:  PingPong buffer for audio data : MAX 48*2*8*2*2 = 3072 B
//these buffer is private
uint8_t ssc0_PingPongOut[2][ I2S_PINGPONG_OUT_SIZE_3K ];         // Play
uint8_t ssc0_PingPongIn[2][ I2S_PINGPONG_IN_SIZE_3K ] ;          // Record
uint8_t ssc1_PingPongOut[2][ I2S_PINGPONG_OUT_SIZE_3K ];         // Play
uint8_t ssc1_PingPongIn[2][ I2S_PINGPONG_IN_SIZE_3K ] ;          // Record

uint8_t spi0_2MSOut[2][ I2S_PINGPONG_OUT_SIZE_3K ];
uint8_t spi0_2MSIn[2][ I2S_PINGPONG_IN_SIZE_3K ];
uint8_t spi1_2MSOut[2][ I2S_PINGPONG_OUT_SIZE_3K ];
uint8_t spi1_2MSIn[2][ I2S_PINGPONG_IN_SIZE_3K ];

// gpio has no private ring buffer, it share with ssc0;
uint8_t gpio_PingPong_bufferOut[2][I2S_PINGPONG_OUT_SIZE_3K];
uint8_t gpio_PingPong_bufferIn[2][I2S_PINGPONG_IN_SIZE_3K];
#pragma pack()  

//Ring for ssc
kfifo_t  ssc0_bulkout_fifo;
kfifo_t  ssc0_bulkin_fifo;
kfifo_t  ssc1_bulkout_fifo;
kfifo_t  ssc1_bulkin_fifo;

//Ring for USB data endpoint
kfifo_t  ep0BulkOut_fifo;
kfifo_t  ep0BulkIn_fifo;
kfifo_t  ep1BulkOut_fifo;
kfifo_t  ep1BulkIn_fifo;
kfifo_t  ep2BulkOut_fifo;
kfifo_t  ep2BulkIn_fifo;

//Ring for USB cmd endpoint
kfifo_t  cmdEpBulkOut_fifo;
kfifo_t  cmdEpBulkIn_fifo;

//Ring for Ruler cmd endpoint
kfifo_t  cmd_ruler_rece_fifo;
kfifo_t  cmd_ruler_send_fifo;


//Ring for spi
kfifo_t  spi0_bulkOut_fifo;
kfifo_t  spi0_bulkIn_fifo;
kfifo_t  spi1_bulkOut_fifo;
kfifo_t  spi1_bulkIn_fifo;

/*----------------------------------------------------------------------------
 *      External variables
 *----------------------------------------------------------------------------*/

extern const USBDDriverDescriptors cdcd_serial_driver_descriptors;

/*----------------------------------------------------------------------------
 *      Internal variables
----------------------------------------------------------------------------*/

static const struct _pin usart_pins[] = USART_PINS;

/** Buffer for storing incoming USB data. */
CACHE_ALIGNED static uint8_t usb_buffer[DATABUFFERSIZE];

/** Serial Port ON/OFF */
static uint8_t is_cdc_serial_on = 0;

/** CDC Echo back ON/OFF */
#ifdef ORIGIN_CODE
static uint8_t is_cdc_echo_on = 0;
#endif

/** USB Tx flag */
static volatile uint8_t tx_done_flag = 0;
/** Test buffer */
CACHE_ALIGNED static uint8_t test_buffer[TEST_BUFFER_SIZE];

static struct _usart_desc usart_desc = {
	.addr           = USART_ADDR,
	.baudrate       = 115200,
	.mode           = US_MR_CHMODE_NORMAL | US_MR_PAR_NO | US_MR_CHRL_8_BIT,
	.transfer_mode = USARTD_MODE_DMA,
};

static volatile bool usart_rx_flag = false;
static volatile uint8_t char_recv;

List *audioQueue;
static volatile int cnt = 0;
/*----------------------------------------------------------------------------
 *         Internal Prototypes
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *  Interrupt handlers
 *----------------------------------------------------------------------------*/

/**
 * USART interrupt handler
 */
static void usart_irq_handler(uint32_t source, void* user_arg)
{
	assert(source == get_usart_id_from_addr(usart_desc.addr));

	Usart* p_us = usart_desc.addr;

	/* If USB device is not configured, do nothing */
	if (!is_cdc_serial_on) {
		usart_disable_it(p_us, 0xFFFFFFFF);
		return;
	}
	char_recv = usart_get_char(p_us);
	usart_rx_flag = true;
	p_us->US_CR = US_CR_RSTSTA;
}

/*-----------------------------------------------------------------------------
 *         Callback re-implementation
 *-----------------------------------------------------------------------------*/

/**
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void usbd_driver_callbacks_configuration_changed(unsigned char cfgnum)
{
	cdcd_serial_driver_configuration_changed_handler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void usbd_callbacks_request_received(const USBGenericRequest *request)
{
	cdcd_serial_driver_request_handler(request);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/

/**
 * Callback invoked when data has been received on the USB.
 */
#if 0
void _spi_transfer();
static void _usb_data_received(void *read, uint8_t status,
		uint32_t received, uint32_t remaining)
{
	/* Check that data has been received successfully */
	if (status == USBD_STATUS_SUCCESS) {
                printf("-- data received at %d --\n\r",__LINE__);
		*(uint8_t *)read = 1;
		/* Send back CDC data */
		//if (is_cdc_echo_on){

			//cdcd_serial_driver_write(usb_buffer, received, 0, 0);           //0x82-0x01
                        //cdcd_serial_driver_WriteAudio_1(usb_buffer, received, 0, 0);    //0x86-0x05
                        //cdcd_serial_driver_WriteSPI(usb_buffer, received, 0, 0);        //0x88-0x07
                        //cdcd_serial_driver_WriteCmd(usb_buffer, received, 0, 0);        //0x84-0x03
                        cdcd_serial_driver_WriteLog(usb_buffer, received, 0, 0);          //0x89
                        //cdcd_serial_driver_WriteLog(NULL, 0, NULL, NULL);
		//}
		/* Send data through USART */
		//if (is_cdc_serial_on) {

		//	_usart_dma_tx( usb_buffer, received );
		//}
                //    _spi_transfer();
		/* Check if bytes have been discarded */
		if ((received == DATAPACKETSIZE) && (remaining > 0)) {

			trace_warning(
				"_usb_data_received: %u bytes discarded\n\r",
					(unsigned int)remaining);
                        printf("_usb_data_received£ºdata received %d --\n\r",received);
		}
	} else {

		trace_warning( "_usb_data_received: Transfer error\n\r");
	}
}
#endif


/**
 * console help dump
 */
#if 0
static void _debug_help(void)
{
	printf("-- ESC to Enable/Disable ECHO on cdc serial --\n\r");
	printf("-- Press 't' to test trasfer --\n\r");
}
#endif

/**
 * Configure USART to work @ 115200
 */
static void _configure_usart(void)
{
	Usart* usart = usart_desc.addr;
	uint32_t id = get_usart_id_from_addr(usart);
	/* Driver initialize */
	usartd_configure(0, &usart_desc);
	pio_configure(usart_pins, ARRAY_SIZE(usart_pins));
	usart_enable_it(usart, US_IER_RXRDY);
	irq_add_handler(id, usart_irq_handler, NULL);
	irq_enable(id);
}

//==============================================================================
/** define timer/counter */
#define EXAMPLE_TC TC0
/** define channel for timer/counter */
#define EXAMPLE_TC_CHANNEL_COUNTER 1
#define COUNTER_FREQ    1000

static uint32_t _tick = 0;
static bool is_Tc_started = false;
volatile static int initial_dealy = 0;
/** define Timer Counter descriptor for counter/timer */
static struct _tcd_desc tc_counter = {
	.addr = EXAMPLE_TC,
	.channel = EXAMPLE_TC_CHANNEL_COUNTER,
};

static  uint8_t usb_serial_read1 = 0;
static  uint8_t usb_serial_send_ssc1 = 1;
static int _tc_counter_callback(void* arg, void* arg2)
{
	const int div = 8;

        _tick++;

	if (_tick % div == 0)
        {
		//printf("time: %us\r\n", (unsigned)_tick / div);
                usb_serial_read1 = 1;
                usb_serial_send_ssc1 = 1;
        }
        
        //tcd_stop(&tc_counter);
	return 0;
}


static void _tc_counter_initialize(uint32_t freq)
{
	uint32_t frequency;
	struct _callback _cb;

	printf("* Configure TC: channel %d: counter mode\r\n", tc_counter.channel);

	frequency = tcd_configure_counter(&tc_counter, freq, freq);

	printf("  - Required frequency = %uHz\r\n", (unsigned)freq);
	printf("  - Configured frequency = %uHz\r\n", (unsigned)frequency);
	callback_set(&_cb, _tc_counter_callback, NULL);
	tcd_start(&tc_counter, &_cb);
}

//==============================================================================
/*----------------------------------------------------------------------------
 *          initialize spi slave
 *----------------------------------------------------------------------------*/
/** define the address of SPI slave */
#define SPI_SLAVE_ADDR SPI1
/** define the pins of SPI slave */
#define SPI_SLAVE_PINS PINS_SPI1_NPCS0_IOS3
#define SPI_MASTER_BUS BUS(BUS_TYPE_SPI, 1)
#define SPI_MASTER_CS 0
#define SPI_MASTER_BITRATE 100
#define SPI_MASTER_PINS PINS_SPI1_NPCS0_IOS3

#define DMA_TRANS_SIZE 32
/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
#pragma pack(1)
/** data buffer for SPI master's receive */
CACHE_ALIGNED static uint8_t spi_buffer_master_tx[DMA_TRANS_SIZE];
#pragma pack()

#pragma pack(1)
/** data buffer for SPI master's receive */
CACHE_ALIGNED static uint8_t spi_buffer_master_tx_protocol[4]={0x01,0x0,0x0,0xff};
#pragma pack()

/** data buffer for SPI slave's transfer */
CACHE_ALIGNED static uint8_t spi_buffer_slave_rx[DMA_TRANS_SIZE];

/** Pio pins for SPI slave */
static const struct _pin pins_spi_slave[] = SPI_SLAVE_PINS;
	const struct _bus_iface iface_bus1 = {
		.type = BUS_TYPE_SPI,
		.spi = {
			.hw = BOARD_SPI_BUS1,
		},
		.transfer_mode = BUS_TRANSFER_MODE_ASYNC,//BOARD_SPI_BUS1_MODE,
	};

/** descriptor for SPI master */
static const struct _bus_dev_cfg spi_master_dev = {
	.bus = SPI_MASTER_BUS,
	.spi_dev = {
		.chip_select = SPI_MASTER_CS,
		.bitrate = SPI_MASTER_BITRATE,
		.delay = {
			.bs = 0,
			.bct = 0,
		},
		.spi_mode = SPID_MODE_1,
	},
};

static struct _spi_desc spi_slave_dev = {
	.addr = SPI_SLAVE_ADDR,
	.chip_select = 0,
	.transfer_mode = BUS_TRANSFER_MODE_ASYNC,//BUS_TRANSFER_MODE_DMA,
};

//==============================================================================
/*----------------------------------------------------------------------------
 *          initialize ssc 
 *----------------------------------------------------------------------------*/
/*
#define SAMPLE_RATE             (48000)
#define SLOT_BY_FRAME           (8)
#define BITS_BY_SLOT            (16)

#define BUFFERS (32)
#define BUFFER_SIZE ROUND_UP_MULT(192, L1_CACHE_BYTES)
#define BUFFER_THRESHOLD (8)

// Audio record buffer 
CACHE_ALIGNED_DDR static uint16_t _sound_buffer[BUFFERS][BUFFER_SIZE];
CACHE_ALIGNED_DDR static uint16_t _sound_buffer1[BUFFERS][BUFFER_SIZE];

static struct _audio_ctx {
	uint32_t threshold;
	struct {
		uint16_t rx;
		uint16_t tx;
		uint32_t count;
	} circ;
	uint8_t volume;
	bool playing;
	bool recording;
} _audio_ctx = {
	.threshold = BUFFER_THRESHOLD,
	.circ = {
		.rx = 0,
		.tx = 0,
		.count = 0,
	},
	.volume = 30,
	.recording = false,
	.playing = false,
};
*/
// SSC instance
extern struct _ssc_desc ssc_dev_desc[2];
static bool is_ssc_started = false;

void ssc0_init(void)
{
    //initialize ssc0 object and it's operation
    memset((void *)&source_ssc0, 0, sizeof(DataSource));
    memset((void *)ssc0_PingPongOut, 0, sizeof(ssc0_PingPongOut));
    memset((void *)ssc0_PingPongIn, 0, sizeof(ssc0_PingPongIn));

    source_ssc0.dev.direct = (uint8_t)BI;
    source_ssc0.dev.identify = ID_SSC0;
    source_ssc0.dev.instanceHandle = (uint32_t)SSC0;
    source_ssc0.status[IN] = (uint8_t)FREE;
    source_ssc0.status[OUT] = (uint8_t)FREE;
    source_ssc0.tx_index = 0;
    source_ssc0.rx_index = 0;
    //    source_ssc0.peripheralParameter = ( void * )&Audio_Configure_Instance[0];
    source_ssc0.warmWaterLevel = 0; //  I2S_PLAY_PRE_BUF_NUM  * source_ssc0.txSize ;

    source_ssc0.txSize = 0;         //(( SAMPLE_RATE_DEFAULT / 1000 ) * ( SAMPLE_LENGTH_DEFAULT / 8 ) * SLOT_NUM_DEFAULT * I2S_PINGPONG_BUF_SIZE_MS );
    source_ssc0.rxSize = 0;         //(( SAMPLE_RATE_DEFAULT / 1000 ) * ( SAMPLE_LENGTH_DEFAULT / 8 ) * SLOT_NUM_DEFAULT * I2S_PINGPONG_BUF_SIZE_MS );

    source_ssc0.init_source = NULL;
    source_ssc0.buffer_write = NULL;
    source_ssc0.buffer_read = NULL;
    source_ssc0.peripheral_stop = NULL;

    source_ssc0.pRingBulkOut = &ssc0_bulkout_fifo;
    source_ssc0.pRingBulkIn = &ssc0_bulkin_fifo;
    source_ssc0.pBufferOut = (uint8_t *)ssc0_PingPongOut;
    source_ssc0.pBufferIn = (uint8_t *)ssc0_PingPongIn;

    if (NULL != source_ssc0.init_source)
        source_ssc0.init_source(&source_ssc0, NULL);
}

void ssc1_init(void)
{
    //initialize ssc1 object and it's operation
    memset((void *)&source_ssc1, 0, sizeof(DataSource));
    memset((void *)ssc1_PingPongOut, 0, sizeof(ssc1_PingPongOut));
    memset((void *)ssc1_PingPongIn, 0, sizeof(ssc1_PingPongIn));
    source_ssc1.dev.direct = (uint8_t)BI;
    source_ssc1.dev.identify = ID_SSC1;
    source_ssc1.dev.instanceHandle = (uint32_t)SSC1;
    source_ssc1.status[IN] = (uint8_t)FREE;
    source_ssc1.status[OUT] = (uint8_t)FREE;
    source_ssc1.tx_index = 0;
    source_ssc1.rx_index = 0;
    //    source_ssc1.peripheralParameter = ( void * )&Audio_Configure_Instance[1];
    source_ssc1.warmWaterLevel = 0; //(( SAMPLE_RATE_DEFAULT / 1000 ) * ( SAMPLE_LENGTH_DEFAULT / 8 ) * SLOT_NUM_DEFAULT * 2 ) * 4;

    source_ssc1.txSize = 0;         //(( SAMPLE_RATE_DEFAULT/ 1000 ) * ( SAMPLE_LENGTH_DEFAULT / 8 ) * SLOT_NUM_DEFAULT * 2 );
    source_ssc1.rxSize = 0;         //(( SAMPLE_RATE_DEFAULT/ 1000 ) * ( SAMPLE_LENGTH_DEFAULT / 8 )* SLOT_NUM_DEFAULT * 2 );

    source_ssc1.init_source = NULL;
    source_ssc1.buffer_write = NULL;
    source_ssc1.buffer_read = NULL;
    source_ssc1.peripheral_stop = NULL;

    source_ssc1.pRingBulkOut = &ssc1_bulkout_fifo;
    source_ssc1.pRingBulkIn = &ssc1_bulkin_fifo;
    source_ssc1.pBufferOut = (uint8_t *)ssc1_PingPongOut;
    source_ssc1.pBufferIn = (uint8_t *)ssc1_PingPongIn;

    if (NULL != source_ssc1.init_source)
        source_ssc1.init_source(&source_ssc1, NULL);
}


//==============================================================================

void _spi_transfer()
{
	int i;
	struct _buffer master_buf = {
		.data = spi_buffer_master_tx,
		.size = DMA_TRANS_SIZE,
		.attr = BUS_BUF_ATTR_TX | BUS_SPI_BUF_ATTR_RELEASE_CS,
	};
        
//        printf("%s-%d:running...\r\n",__FILE__,__LINE__);
	for (i = 0; i < DMA_TRANS_SIZE; i++)
        {
//            if (( i >= 10 ) && ( i <= 21  ))
		spi_buffer_master_tx[i] = 0x0;
//            else
//                spi_buffer_master_tx[i] = 0; 
        }
        spi_buffer_master_tx[31] = 0x05;
        spi_buffer_master_tx[30] = 0x10;        
//        spi_buffer_master_tx[29] = 0;
        spi_buffer_master_tx[28] = 4;
        spi_buffer_master_tx[27] = 6;
        spi_buffer_master_tx[26] = 3;
        spi_buffer_master_tx[25] = 1;
        spi_buffer_master_tx[24] = 5; 
        spi_buffer_master_tx[23] = 12; 
        spi_buffer_master_tx[22] = 4; 
	//memset(spi_buffer_slave_rx, 0, DMA_TRANS_SIZE);

	bus_start_transaction(spi_master_dev.bus);
	bus_transfer(spi_master_dev.bus, spi_master_dev.spi_dev.chip_select, &master_buf, 1, NULL);
        bus_wait_transfer(spi_master_dev.bus);   
	bus_stop_transaction(spi_master_dev.bus);
}

//==============================================================================
unsigned char aic3204_init_default(void)
{
    unsigned char err;
    CODEC_SETS codec_set;

    codec_set.id = 0;                                 //CODEC 0
    codec_set.sr = 48000;                             //SAMPLE_RATE_DEFAULT;
    codec_set.sample_len = 16;                        //SAMPLE_LENGTH_DEFAULT;
    codec_set.format = 2;                             //I2S-TDM
    codec_set.slot_num = 8;                           //SLOT_NUM_DEFAULT;
    codec_set.m_s_sel = 0;                            //master
    codec_set.bclk_polarity = 0;
    codec_set.flag = 0;
    codec_set.delay = 0;

    I2C_Switcher(I2C_SWITCH_CODEC0);                 //I2C bus switcher

    err = Init_CODEC(codec_set);                     //CODEC0 connetced to TWI2
    if (err != NO_ERR)
    {
        return err;
    }

    codec_set.id = 1;                               //CODEC 1
    codec_set.sr = 48000;                           //SAMPLE_RATE_DEFAULT;
    codec_set.sample_len = 16;                      //SAMPLE_LENGTH_DEFAULT;
    codec_set.format = 2;                           //I2S-TDM
    codec_set.slot_num = 8;                         //SLOT_NUM_DEFAULT;
    codec_set.m_s_sel = 0;                          //slave
    codec_set.bclk_polarity = 0;
    codec_set.flag = 0;
    codec_set.delay = 0;

    I2C_Switcher(I2C_SWITCH_CODEC1);                //I2C bus switcher
    err = Init_CODEC(codec_set);                    //CODEC1 connetced to TWI2

    return err;
}
//==============================================================================



/*----------------------------------------------------------------------------
 *          Main
 *----------------------------------------------------------------------------*/

/**
 * \brief usb_cdc_serial Application entry point.
 *
 * Initializes drivers and start the USB <-> Serial bridge.
 */
int main(void)
{
        uint8_t *memory_alloc = NULL;
        static int alloc_cnt ;
	uint8_t is_usb_connected = 0;
	uint8_t usb_serial_read_ssc0 = 1;
        uint8_t usb_serial_read_ssc1 = 1;
        uint8_t usb_serial_read_spi = 1;
        uint8_t usb_serial_read_cmd = 1;
        //uint8_t usb_serial_read_i2s0 = 1;
        //uint8_t usb_serial_read_i2s1 = 1;
        


	// Output example information 
	console_example_info("USB Device Test Suite for AB05");

	// Initialize all USB power (off) 
	usb_power_configure();

	// Configure USART 
	_configure_usart();

	// CDC serial driver initialization 
	cdcd_serial_driver_initialize(&cdcd_serial_driver_descriptors);

	// Help informaiton 
	//_debug_help();

	// connect if needed 
	usb_vbus_configure();
        
        //config spi
        pio_configure(pins_spi_slave, ARRAY_SIZE(pins_spi_slave));
        bus_configure(BUS(BUS_TYPE_SPI, 1), &iface_bus1);
        bus_configure_slave(spi_master_dev.bus, &spi_master_dev);
        
        //config ssc
	ssc_configure(&ssc_dev_desc[1]);
#ifdef INTERRUPT_SSC
        ssc_enable_interrupts(&ssc_dev_desc[1], (1<<0));
#endif
	ssc_disable_receiver(&ssc_dev_desc[1]);
	ssc_disable_transmitter(&ssc_dev_desc[1]);
        
        //config codec
        init_codec_rst_pin();
        aic3204_init_default();

#if 1        
        list_init(audioQueue, NULL);
        list_init(&ep2_ssc0_rec, NULL);
        list_init(&ep4_cmd_rec, NULL);
        list_init(&ep6_ssc1_rec, NULL);
        list_init(&ep8_spi_rec, NULL);
        list_init(&ep9_log_rec, NULL);
        list_init(&ep11_i2s0_rec, NULL);

        list_init(&ep1_ssc0_play, NULL);
        list_init(&ep3_cmd_play, NULL);
        list_init(&ep5_ssc1_play, NULL);
        list_init(&ep7_spi_play, NULL);
        list_init(&ep10_i2s0_play, NULL);
#endif

	// Driver loop 
	while (1) {                
		// Device is not configured 
		if (usbd_get_state() < USBD_STATE_CONFIGURED) {

			if (is_usb_connected) {
				is_usb_connected = 0;
				is_cdc_serial_on  = 0;
			}

		} else if (is_usb_connected == 0) {
				is_usb_connected = 1;

		}
                
                if( 1 == is_usb_connected )
                {
                    if( false == is_Tc_started )
                    {
                        if(initial_dealy++ > 500){
                            _tc_counter_initialize(COUNTER_FREQ);
                            is_Tc_started = true;
                        }
                    }
                    
                    if(( false == is_ssc_started ) && ( initial_dealy ++ > 150000 ))
                    {
                       is_ssc_started = true;
                       ssc_enable_receiver(&ssc_dev_desc[1]);
                       starting_ssc1_record();
                       ssc_enable_transmitter(&ssc_dev_desc[1]);
                       starting_ssc1_play();
                    }
                }
                else
                {
                       
                }
                //memory_alloc = (uint8_t *)malloc(6144);
                //if( memory_alloc != NULL )
                //  alloc_cnt++;
                //else
                //  printf("alloc memory failed,here success %d times.\r\n",alloc_cnt);
                
                if(usb_serial_read1 == 1) 
                {
                    usb_serial_read1 = 0;
		    /* Start receiving data on the USB */
#if 1
                    //if( usb_serial_send_ssc1 == 1 )
                    {
                        usb_serial_send_ssc1 = 0;
                        cdcd_serial_driver_WriteAudio_1(usb_ep6_ssc1_record_data, 
                                                    sizeof(usb_ep6_ssc1_record_data),
                                                    usb_ep6_ssc1_record_cb, &usb_serial_send_ssc1);    //0x86
                    }
                    //if( 1 == usb_serial_read_ssc0 )
                    //{
                    //    cdcd_serial_driver_readAudio_0(usb_ep1_ssc0_play_data, sizeof(usb_ep1_ssc0_play_data),        //0x82-0x01
                    //                          usb_ep1_ssc0_play_cb, &usb_serial_read_ssc0);
                    //}
                    //if( 1 == usb_serial_read_ssc1 )
                    //{
                    //    cdcd_serial_driver_readAudio_1(usb_ep5_ssc1_play_data, sizeof(usb_ep5_ssc1_play_data),        //0x86-0x05
                    //                          usb_ep5_ssc1_play_cb, &usb_serial_read_ssc1);
                    //}
                    //if( 1 == usb_serial_read_spi )
                    //{
                    //    cdcd_serial_driver_readSPI(usb_ep7_spi_play_data, sizeof(usb_ep7_spi_play_data),              //0x88-0x07
                    //                          usb_ep7_spi_play_cb, &usb_serial_read_spi);
                    //}
                    //if( 1 == usb_serial_read_cmd )
                    //{
                    //    usb_serial_read_cmd = 0;
                    //    cdcd_serial_driver_readCmd(usb_ep3_cmd_play_data, sizeof(usb_ep3_cmd_play_data),              //0x84-0x03
                    //                            usb_ep3_cmd_play_cb, &usb_serial_read_cmd);
                    //}
                    _spi_transfer();
#else                    
		    //cdcd_serial_driver_write((char*)"Alive\n\r", 8,
		    //				NULL, NULL);
                    //cdcd_serial_driver_WriteAudio_1((char*)"Alive\n\r", 8,NULL, NULL);    //0x86
                    //cdcd_serial_driver_WriteSPI((char*)"HiSPI\n\r", 8,NULL, NULL);        //0x88
                    //cdcd_serial_driver_WriteLog((char*)"HiLOG\n\r", 8,NULL, NULL);        //0x89
                    //cdcd_serial_driver_WriteCmd((char*)"HiCMD\n\r", 8,NULL, NULL);      //0x84
                    //cdcd_serial_driver_WriteLog(NULL, 0, NULL, NULL);
//                  _spi_transfer();

#ifdef INTERRUPT_SSC
                    ssc_enable_transmitter(&ssc_dev_desc);
                    ssc_write(&ssc_dev_desc, 0x01);
                    while(!(ssc_dev_desc.addr->SSC_SR & SSC_SR_TXRDY) == SSC_SR_TXRDY);
#endif              
                    if( ( cnt % 100 ) == 0 )
                      printf( "queue size = %d\r\n",audioQueue->size );
#endif                                          
                }  
	}//while(1)
}
/** \endcond */
