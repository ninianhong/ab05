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

#include "bus.h"
#include "uif_list.h"

#include "cdcd_serial_driver.h"
#include "usbd.h"
#include "usbd_hal.h"

#include "main_usb_common.h"
#include "tcd.h"

#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

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
#ifdef ORIGIN_CODE
static int _usart_finish_tx_transfer_callback(void* arg, void* arg2)
{
	usartd_finish_tx_transfer(0);
	return 0;
}
#endif

/**
 *  \brief Send single buffer data through DMA
 */
#ifdef ORIGIN_CODE
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
#endif

/**
 * Callback invoked when data has been received on the USB.
 */
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
                        cdcd_serial_driver_WriteLog(NULL, 0, NULL, NULL);
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
                        printf("_usb_data_received��data received %d --\n\r",received);
		}
	} else {

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
#ifdef ORIGIN_CODE
static void _usb_data_sent(void *arg, uint8_t status, uint32_t transferred, uint32_t remaining)
{
	tx_done_flag = 1;
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

/**
 * Test USB CDC Serial
 */
#ifdef ORIGIN_CODE
static void _send_text(void)
{
	uint32_t i, test_cnt;

	if (!is_cdc_serial_on) {

		printf("\n\r!! Host serial program not ready!\n\r");
		return;
	}
	printf("\n\r- USB CDC Serial writing ...\n\r");

	/* Test data initialize */
	for (i = 0; i < TEST_BUFFER_SIZE; i ++) test_buffer[i] = (i % 10) + '0';

	printf("- Send 0,1,2 ... to host:\n\r");
	for (test_cnt = 0; test_cnt < TEST_COUNT; test_cnt ++) {

		tx_done_flag = 0;
		cdcd_serial_driver_write(test_buffer, TEST_BUFFER_SIZE,
				_usb_data_sent, NULL);
		while(!tx_done_flag);
	}

	/* Finish sending */
	cdcd_serial_driver_write(NULL, 0, NULL, NULL);
	_usart_dma_tx(test_buffer, TEST_BUFFER_SIZE);
}
#endif

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
#ifdef ORIGIN_CODE
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
#endif

static  uint8_t usb_serial_read1 = 0;
static int _tc_counter_callback(void* arg, void* arg2)
{
	const int div = 4;

        _tick++;

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

#if 0
#pragma pack(1)
/** data buffer for SPI master's receive */
CACHE_ALIGNED static uint8_t spi_buffer_master_tx[DMA_TRANS_SIZE];
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
		.transfer_mode = BOARD_SPI_BUS1_MODE,
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
#endif

void _spi_transfer()
{
	int err;
	int i;
	struct _buffer master_buf = {
		.data = spi_buffer_master_tx,
		.size = DMA_TRANS_SIZE,
		.attr = BUS_BUF_ATTR_TX | BUS_SPI_BUF_ATTR_RELEASE_CS,
	};
        
        printf("%s-%d:running...\r\n",__FILE__,__LINE__);
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

	err = bus_start_transaction(spi_master_dev.bus);
	err = bus_transfer(spi_master_dev.bus, spi_master_dev.spi_dev.chip_select, &master_buf, 1, NULL);
        err = bus_wait_transfer(spi_master_dev.bus);   
	err = bus_stop_transaction(spi_master_dev.bus);
}

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

	// Output example information 
	console_example_info("USB Device Test Suite for AB05");

	// Initialize all USB power (off) 
	usb_power_configure();

	// Configure USART 
	_configure_usart();

	// CDC serial driver initialization 
	cdcd_serial_driver_initialize(&cdcd_serial_driver_descriptors);

	// Help informaiton 
	_debug_help();

	// connect if needed 
	usb_vbus_configure();
        
        //config spi
        pio_configure(pins_spi_slave, ARRAY_SIZE(pins_spi_slave));
        bus_configure(BUS(BUS_TYPE_SPI, 1), &iface_bus1);
        bus_configure_slave(spi_master_dev.bus, &spi_master_dev);
        
        //config ssc
	ssc_configure(&ssc_dev_desc);
	ssc_disable_receiver(&ssc_dev_desc);
	ssc_disable_transmitter(&ssc_dev_desc);

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
                }
                else
                {
                  
                }
                
                if(usb_serial_read1 == 1) 
                {
                    usb_serial_read1 = 0;
		    /* Start receiving data on the USB */
		    //cdcd_serial_driver_read(usb_buffer, DATAPACKETSIZE,
                    //                        _usb_data_received1, &usb_serial_read1);
#if 0
		    //cdcd_serial_driver_read(usb_buffer, DATAPACKETSIZE,
                    //                        _usb_data_received, &usb_serial_read); 
                    //cdcd_serial_driver_readAudio_0(usb_buffer, DATAPACKETSIZE,        //0x82-0x01
                    //                        _usb_data_received, &usb_serial_read);
                    //cdcd_serial_driver_readAudio_1(usb_buffer, DATAPACKETSIZE,        //0x86-0x05
                    //                        _usb_data_received, &usb_serial_read);
                    //cdcd_serial_driver_readSPI(usb_buffer, DATAPACKETSIZE,            //0x88-0x07
                    //                        _usb_data_received, &usb_serial_read); 
                    cdcd_serial_driver_readCmd(usb_buffer, DATAPACKETSIZE,            //0x84-0x03
                                            _usb_data_received, &usb_serial_read);

#else                    
		    //cdcd_serial_driver_write((char*)"Alive\n\r", 8,
		    //				NULL, NULL);
                    //cdcd_serial_driver_WriteAudio_1((char*)"Alive\n\r", 8,NULL, NULL);    //0x86
                    //cdcd_serial_driver_WriteSPI((char*)"HiSPI\n\r", 8,NULL, NULL);        //0x88
                    cdcd_serial_driver_WriteLog((char*)"HiLOG\n\r", 8,NULL, NULL);        //0x89
                    //cdcd_serial_driver_WriteCmd((char*)"HiCMD\n\r", 8,NULL, NULL);      //0x84
                    //cdcd_serial_driver_WriteLog(NULL, 0, NULL, NULL);
                    _spi_transfer();
                    
#endif                                          
                }


                
#ifdef ORIGIN_CODE                
		// Serial port ON/OFF 
		if (cdcd_serial_driver_get_control_line_state()
					& CDCControlLineState_DTR) {
			if (!is_cdc_serial_on) {
				is_cdc_serial_on = 1;
				}
			if(usb_serial_read == 1) {
				usb_serial_read = 0;
				/* Start receiving data on the USB */
				cdcd_serial_driver_read(usb_buffer, DATAPACKETSIZE,
							_usb_data_received, &usb_serial_read);
			}
			if(usart_rx_flag == true) {
				usart_rx_flag = false;
				cdcd_serial_driver_write((void *)&char_recv, 1, 0, 0);
				if(is_cdc_echo_on) {
					_usart_dma_tx((uint8_t*)&char_recv, 1);
				}
			}
		} else if (is_cdc_serial_on) {
			is_cdc_serial_on = 0;
		}

		if (console_is_rx_ready()) {
			uint8_t key = console_get_char();
			// ESC: CDC Echo ON/OFF 
			if (key == 27) {

				printf("** CDC Echo %s\n\r",
						is_cdc_echo_on ? "OFF" : "ON");
				is_cdc_echo_on = !is_cdc_echo_on;

			} else if (key == 't') {
				// 't': Test CDC writing  
				_send_text();

			} else {
				printf("Alive\n\r");
				cdcd_serial_driver_write((char*)"Alive\n\r", 8,
						NULL, NULL);
				_usart_dma_tx((uint8_t*)"Alive\n\r", 8);
				_debug_help();
			}
		}//if (console_is_rx_ready())
#else    
                //uint8_t key = console_get_char();
                //printf("input char = %d\n",key);
                //if ('n'==key) {
                  /*
			if(usb_serial_read == 1) {
				usb_serial_read = 0;
				// Start receiving data on the USB 
				cdcd_serial_driver_read(usb_buffer, DATAPACKETSIZE,
							_usb_data_received, &usb_serial_read);
			}*/
                //  	cdcd_serial_driver_write((char*)"aaaaa\n\r", 8,
		//				NULL, NULL);
                //}
#endif     
	}//while(1)
}
/** \endcond */
