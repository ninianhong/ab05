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
#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "chip.h"

#include "trace.h"
#include "compiler.h"

#include "cache.h"
#include "console.h"

#include "irq.h"
#include "pio.h"
#include "pit.h"
#include "pmc.h"
#include "tc.h"
#include "usartd.h"
#include "usart.h"

//usb
#include "cdcd_serial_driver.h"
#include "usbd.h"
#include "usbd_hal.h"
#include "main_usb_common.h"
#include "tcd.h"

//codec
#include "ssc.h"
#include "codec.h"



/*----------------------------------------------------------------------------
 *      Definitions
 *----------------------------------------------------------------------------*/

/** Size in bytes of the packet used for reading data from the USB & USART */
#define DATAPACKETSIZE (128)

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
static uint8_t is_cdc_echo_on = 0;

/** USB Tx flag */
static volatile uint8_t tx_done_flag = 1;
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

static int _usart_finish_tx_transfer_callback(void* arg, void* arg2)
{
	usartd_finish_tx_transfer(0);
	return 0;
}

/**
 *  \brief Send single buffer data through DMA
 */
static void _usart_dma_tx(const uint8_t* buffer, uint32_t len )
{

	struct _buffer tx = {
		.data = (unsigned char*)buffer,
		.size = len,
		.attr = USARTD_BUF_ATTR_WRITE,
	};
	struct _callback _cb = {
		.method = _usart_finish_tx_transfer_callback,
		.arg = 0,
	};
	usartd_transfer(0, &tx, &_cb);

}

/**
 * Callback invoked when data has been received on the USB.
 */
static void _usb_data_received(void *read, uint8_t status,
		uint32_t received, uint32_t remaining)
{
	/* Check that data has been received successfully */
	if (status == USBD_STATUS_SUCCESS) 
        {
                printf("-- data received at %d --\n\r",__LINE__);
		*(uint8_t *)read = 1;

		/* Check if bytes have been discarded 
		if ((received == DATAPACKETSIZE) && (remaining > 0)) 
                {

			trace_warning(
				"_usb_data_received: %u bytes discarded\n\r",
					(unsigned int)remaining);
		}*/
          } 
          else 
          {
		trace_warning( "_usb_data_received: Transfer error\n\r");
	  }
}

/**
 * Callback invoked when data has been received on the USB.
 */
static void _usb_data_transfer(void *read, uint8_t status,
		uint32_t received, uint32_t remaining)
{
	/* Check that data has been received successfully */
	if (status == USBD_STATUS_SUCCESS) 
        {
                printf("-- data transfered at %d --\n\r",__LINE__);
		*(uint8_t *)read = 1;
          } 
          else 
          {
		trace_warning( "_usb_data_received: Transfer error\n\r");
	  }
}



/**
 * console help dump
 */
static void _debug_help(void)
{
	printf("-- ESC to Enable/Disable ECHO on cdc serial --\n\r");
	printf("-- Press 't' to test trasfer --\n\r");
}


/**
 * Callback invoked when data has been sent.
 */
#define PLAIN_1  1


static void _usb_data_sent(void *arg, uint8_t status, uint32_t transferred, uint32_t remaining)
{
	tx_done_flag = 1;  
}


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

/**
 * Callback invoked when data has been received on the USB.
 */
static void _usb_data_received1(void *read, uint8_t status,
		uint32_t received, uint32_t remaining)
{
	/* Check that data has been received successfully */
	if (status == USBD_STATUS_SUCCESS) {
                printf("data received --\n\r");
		*(uint8_t *)read = 1;

		/* Check if bytes have been discarded */
		if ((received == DATAPACKETSIZE) && (remaining > 0)) {

			trace_warning(
				"_usb_data_received: %u bytes discarded\n\r",
					(unsigned int)remaining);
		}
                } else {

		trace_warning( "_usb_data_received: Transfer error\n\r");
              }
}

static  uint8_t usb_serial_read1 = 0;
static  uint8_t app_start = 0;
static int _tc_counter_callback(void* arg, void* arg2)
{
	const int div = 4;

        _tick++;
        
        if( _tick > 30 * 1000 ) app_start = 1;

	if (_tick % div == 0)
		//printf("time: %us\r\n", (unsigned)_tick / div);
                usb_serial_read1 = 1;
        
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

unsigned char aic3204_init_default(void)
{
    unsigned char err;
    CODEC_SETS codec_set;

    codec_set.id = 0; //CODEC 0
    codec_set.sr = 48000;//SAMPLE_RATE_DEFAULT;
    codec_set.sample_len = 16;//SAMPLE_LENGTH_DEFAULT;
    codec_set.format = 2; //I2S-TDM
    codec_set.slot_num = 8;//SLOT_NUM_DEFAULT;
    codec_set.m_s_sel = 0; //master
    codec_set.bclk_polarity = 0;
    codec_set.flag = 0;
    codec_set.delay = 0;

    I2C_Switcher(I2C_SWITCH_CODEC0); //I2C bus switcher
//    while( 1 ){
    err = Init_CODEC(codec_set);     //CODEC0 connetced to TWI2
    if (err != 0/*NO_ERR*/)
    {
        return err;
    }
//    msleep(5);
//    }

    codec_set.id = 1; //CODEC 1
    codec_set.sr = 48000;//SAMPLE_RATE_DEFAULT;
    codec_set.sample_len = 16;//SAMPLE_LENGTH_DEFAULT;
    codec_set.format = 2; //I2S-TDM
    codec_set.slot_num = 8;//SLOT_NUM_DEFAULT;
    codec_set.m_s_sel = 0; //slave
    codec_set.bclk_polarity = 0;
    codec_set.flag = 0;
    codec_set.delay = 0;

    I2C_Switcher(I2C_SWITCH_CODEC1); //I2C bus switcher
    err = Init_CODEC(codec_set);     //CODEC1 connetced to TWI2

    return err;
}

//==============================================================================
#define SAMPLE_RATE             (48000)
#define SLOT_BY_FRAME           (8)
#define BITS_BY_SLOT            (16)

#define BUFFERS (32)
#define BUFFER_SIZE (ROUND_UP_MULT(192, L1_CACHE_BYTES)* 32)
#define BUFFER_THRESHOLD (8)

// Audio record buffer 
CACHE_ALIGNED_DDR static uint16_t _sound_buffer[BUFFERS][DATAPACKETSIZE];
CACHE_ALIGNED_DDR static uint16_t _sound_buffer1[BUFFERS][DATAPACKETSIZE];

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
}; 
struct _audio_ctx _audio_ctx_rec[4] = {
  {
	.threshold = BUFFER_THRESHOLD,
	.circ = {
		.rx = 0,
		.tx = 0,
		.count = 0,
	},
	.volume = 30,
	.recording = false,
	.playing = false,
  },
    {
	.threshold = BUFFER_THRESHOLD,
	.circ = {
		.rx = 0,
		.tx = 0,
		.count = 0,
	},
	.volume = 30,
	.recording = false,
	.playing = false,
  },
    {
	.threshold = BUFFER_THRESHOLD,
	.circ = {
		.rx = 0,
		.tx = 0,
		.count = 0,
	},
	.volume = 30,
	.recording = false,
	.playing = false,
  },
    {
	.threshold = BUFFER_THRESHOLD,
	.circ = {
		.rx = 0,
		.tx = 0,
		.count = 0,
	},
	.volume = 30,
	.recording = false,
	.playing = false,
  }
};

// SSC instance
static struct _ssc_desc ssc_dev_desc = {
	.addr = SSC1,
	.bit_rate = 0,
	.sample_rate = SAMPLE_RATE,
	.slot_num = SLOT_BY_FRAME,
	.slot_length = BITS_BY_SLOT,
	/* Select RK pin as transmit and receive clock */
	.rx_cfg_cks_rk = true,
	.tx_cfg_cks_tk = false,
	.tx_start_selection = SSC_TCMR_START_TF_EDGE,
	.rx_start_selection = SSC_RCMR_START_RF_EDGE,
};


static void _usb_ep2_ssc0_rec(void *arg, uint8_t status, uint32_t transferred, uint32_t remaining)
{
        //printf("%s-%d-- data transfered --\n\r",__FUNCTION__,__LINE__);
#ifdef PLAIN_1
        _audio_ctx_rec[0].circ.tx = (_audio_ctx_rec[0].circ.tx + 1) % BUFFERS;
        _audio_ctx_rec[0].circ.count--; 
        if( _audio_ctx_rec[0].circ.count > 0/*_audio_ctx_rec[0].threshold*/ )
        {
            cdcd_serial_driver_write((unsigned char*)&_sound_buffer[_audio_ctx_rec[0].circ.tx], 
                                                DATAPACKETSIZE,		    				
                                                _usb_ep2_ssc0_rec, NULL);
        }
        else
            tx_done_flag = 1;
#else
	tx_done_flag = 1;
#endif

}

static int content = 0x01;
static int _ssc_rx_transfer_callback(void* arg, void* arg2)
{
	struct _ssc_desc* desc = (struct _ssc_desc*)arg;
	struct _callback _cb;

	/* New buffer received */
        memset( _sound_buffer[_audio_ctx_rec[0].circ.rx],(content++)/128,DATAPACKETSIZE);
	_audio_ctx_rec[0].circ.rx = (_audio_ctx_rec[0].circ.rx + 1) % BUFFERS;
	_audio_ctx_rec[0].circ.count++;

        //if( _audio_ctx_rec[0].circ.count > 20 )
        {
          printf("%s-%d:_audio_ctx_rec[0].circ.count=%d\r\n",__FUNCTION__,
                                                            __LINE__,
                                                            _audio_ctx_rec[0].circ.count); 
          //assert( 0 );
        }

	//if (!_audio_ctx_rec[0].playing && (_audio_ctx_rec[0].circ.count > _audio_ctx_rec[0].threshold)) {
	//	_audio_ctx_rec[0].playing = true;
	//	ssc_enable_transmitter(&ssc_dev_desc);
	//	_ssc_tx_transfer_callback(desc, NULL);
	//}
#if 0        
        if((_audio_ctx_rec[0].circ.count > _audio_ctx_rec[0].threshold) && ( usb_serial_read ))
        {
          usb_serial_read = 0;
          cdcd_serial_driver_WriteAudio_1(_sound_buffer[_audio_ctx_rec[0].circ.tx],  //0x86-0x05
                                                        DATAPACKETSIZE,
                                            _usb_rx_transfer_callback, &usb_serial_read);
                        //while( !usb_serial_read );
          _audio_ctx_rec[0].circ.tx = (_audio_ctx_rec[0].circ.tx + 1) % BUFFERS;
          _audio_ctx_rec[0].circ.count--;
        }
#endif
        
	struct _buffer _rx = {
		.data = (unsigned char*)&_sound_buffer[_audio_ctx_rec[0].circ.rx],
		.size = DATAPACKETSIZE,
		.attr = SSC_BUF_ATTR_READ,
	};

        //printf("%s-%d:tranfers size:%d...\r\n",__FUNCTION__,__LINE__,DATAPACKETSIZE);
	callback_set(&_cb, _ssc_rx_transfer_callback, desc);
	ssc_transfer(desc, &_rx, &_cb);

	return 0;
}


void starting_record( void )
{
		//_audio_ctx_rec[0].recording = true;
		ssc_enable_receiver(&ssc_dev_desc);

		{ /* Start recording */
			struct _callback _cb;
			callback_set(&_cb, _ssc_rx_transfer_callback, &ssc_dev_desc);                        
			struct _buffer _rx = {
				.data = (unsigned char*)&_sound_buffer[_audio_ctx_rec[0].circ.rx],
				.size = DATAPACKETSIZE,
				.attr = SSC_BUF_ATTR_READ,
			};

			ssc_transfer(&ssc_dev_desc, &_rx, &_cb);
		}

		printf("SSC start to record sound\r\n");  
}



static bool is_ssc_started = false;
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
	uint8_t is_usb_connected = 0;
	uint8_t usb_serial_read = 1;
        uint8_t err = 0;

	/* Output example information */
	console_example_info("USB Device CDC Serial Example");

	/* Initialize all USB power (off) */
	usb_power_configure();

	/* Configure USART */
	_configure_usart();

	/* CDC serial driver initialization */
	cdcd_serial_driver_initialize(&cdcd_serial_driver_descriptors);

	/* Help informaiton */
	_debug_help();

	/* connect if needed */
	usb_vbus_configure();
        
        
        ssc_configure(&ssc_dev_desc);
#ifdef INTERRUPT_SSC
        ssc_enable_interrupts(&ssc_dev_desc, (1<<0));
#endif
	ssc_disable_receiver(&ssc_dev_desc);
	ssc_disable_transmitter(&ssc_dev_desc);
        
        //initialize codec
        init_codec_rst_pin();
        aic3204_init_default();
        
	/* Driver loop */
	while (1) {

		/* Device is not configured */
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
                    
                    if (( false == is_ssc_started ) && ( 1 == app_start ))
                    {
                       is_ssc_started = true;
                       //ssc_enable_receiver(&ssc_dev_desc);
                       starting_record();
                       //ssc_enable_transmitter(&ssc_dev_desc);
                       //starting_play();
                    }
                }
                else
                {
                  
                }
                if((usb_serial_read1 == 1) &&( app_start == 1 )) 
                {
                    usb_serial_read1 = 0;
                    
		    /* Start receiving data on the USB */
		    //cdcd_serial_driver_read(usb_buffer, DATAPACKETSIZE,
                    //                        _usb_data_received1, &usb_serial_read1);
		    //cdcd_serial_driver_read(usb_buffer, DATAPACKETSIZE,
                    //                        _usb_data_received, &usb_serial_read); 
		    //cdcd_serial_driver_write((char*)"Alive\n\r", 8,
		    //				NULL, NULL);
#ifdef PLAIN_1                    
                    if( 1 == tx_done_flag )
                    {
                        tx_done_flag = 0;
                        err = cdcd_serial_driver_WriteAudio_0((unsigned char*)&_sound_buffer[_audio_ctx_rec[0].circ.tx], 
                                                                        DATAPACKETSIZE,		    				
                                                                        _usb_ep2_ssc0_rec, 
                                                                        NULL);
                        if( 0 != err )
                          tx_done_flag = 1;
                        else
                        {
                              _audio_ctx_rec[0].circ.tx = (_audio_ctx_rec[0].circ.tx + 1) % BUFFERS;
                              _audio_ctx_rec[0].circ.count--;                          
                        }
                        
                    }

#else
                    {
                        tx_done_flag = 0;
                        //memset( (unsigned char*)&_sound_buffer[_audio_ctx_rec[0].circ.tx], 0x32, DATAPACKETSIZE );

                        cdcd_serial_driver_write((unsigned char*)&_sound_buffer[_audio_ctx_rec[0].circ.tx]/*usb_buffer*/, 
                                                 DATAPACKETSIZE,		    				
                                                _usb_data_sent, NULL);
                        while( !tx_done_flag );
                    }
#endif                        
                }           
	}//while(1)
}
/** \endcond */
