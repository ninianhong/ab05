#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "trace.h"
#include "compiler.h"
#include "errno.h"

#include "uif_i2s.h"
#include "circbuf.h"

// SSC instance
struct _ssc_desc ssc_dev_desc[2] = {
    {
	    .addr = SSC0,
	    .bit_rate = 0,
	    .sample_rate = SAMPLE_RATE,
	    .slot_num = SLOT_BY_FRAME,
	    .slot_length = BITS_BY_SLOT,
	    /* Select RK pin as transmit and receive clock */
	    .rx_cfg_cks_rk = true,
	    .tx_cfg_cks_tk = false,
	    .tx_start_selection = SSC_TCMR_START_TF_EDGE,
	    .rx_start_selection = SSC_RCMR_START_RF_EDGE,
    },
    {
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
    },    
};


//usb->ssc
int _ssc0_rx_ep2_rec_cb(void* arg, void* arg2)
{
    struct _ssc_desc* desc = (struct _ssc_desc*)arg;
    struct _callback _cb;

    uint8_t *data = NULL;

       
    //step1:process the newest received data,here this port is a productor;
    data = malloc(sizeof(usb_ep2_ssc0_record_data));
    if( NULL != data )
    {
        memcpy( data,usb_ep2_ssc0_record_data,sizeof(data) );        
        list_ins_next( &ep2_ssc0_rec,(&ep2_ssc0_rec)->tail,data);
    }
    ///TODO:the flow controller logic here;
    //step2:start the next transfer
	struct _buffer _rx = 
	{
		.data = (unsigned char*)&usb_ep2_ssc0_record_data[0],
		.size = sizeof(usb_ep2_ssc0_record_data),
		.attr = SSC_BUF_ATTR_READ,
	};

	callback_set(&_cb, _ssc0_rx_ep2_rec_cb, desc);
	ssc_transfer(desc, &_rx, &_cb);


	return 0;

}

uint8_t *data = NULL;
int _ssc1_rx_ep6_rec_cb(void* arg, void* arg2)
{
    extern uint8_t usb_serial_send_ssc1;
    extern CircularBuffer ep6_ssc1_rec_cb;
    struct _ssc_desc* desc = (struct _ssc_desc*)arg;
    struct _callback _cb;


    data = cb_writePacket(&ep6_ssc1_rec_cb);
    if( NULL != data )
    {
         memcpy( data,usb_ep6_ssc1_record_data,sizeof(data) ); 
         memset( data, 0x56, sizeof(usb_ep6_ssc1_record_data));
         cb_doneWritePacket(&ep6_ssc1_rec_cb);
         usb_serial_send_ssc1 = 1;

    }
    else
    {
         //return 0;
    }

    /*
    struct _buffer _rx = 
    {
	.data = (unsigned char*)&usb_ep6_ssc1_record_data[0],
	.size = sizeof(usb_ep6_ssc1_record_data)>>7,
	.attr = SSC_BUF_ATTR_READ,
    };

    callback_set(&_cb, _ssc1_rx_ep6_rec_cb, desc);
    ssc_transfer(desc, &_rx, &_cb);
    */
    return 0;
}

int _i2s0_rx_ep11_rec_cb(void* arg, void* arg2)
{
	return 0;
}

int _i2s1_rx_ep13_rec_cb(void* arg, void* arg2)
{
	return 0;
}

//ssc->usb
int _ssc0_tx_ep1_play_cb(void* arg, void* arg2)
{
    return 0;
}

int _ssc1_tx_ep5_play_cb(void* arg, void* arg2)
{
    struct _ssc_desc* desc = (struct _ssc_desc*)arg;
    struct _callback _cb;

    //step1:use local point to point usb cache array
    uint8_t *data = usb_ep5_ssc1_play_data;

    //step2:release list node data to usb cache array
    if( ep5_ssc1_play.size > 0)
    {       
        list_rem_next( &ep5_ssc1_play,NULL,(void **)&data);
    }
    else
    {
      ///TODO:here some error process ,but NOW return directly;
      return 0;
    }

	//step3:start next dma
    struct _buffer _tx = 
    {
	.data = data,
	.size = sizeof(usb_ep5_ssc1_play_data),
	.attr = SSC_BUF_ATTR_WRITE,
    };

    callback_set(&_cb, _ssc1_tx_ep5_play_cb, desc);
    ssc_transfer(desc, &_tx, &_cb);

    return 0;
}

int _i2s0_tx_ep10_play_cb(void* arg, void* arg2)
{
    return 0;
}

int _i2s1_tx_ep12_play_cb(void* arg, void* arg2)
{
    return 0;
}

//==============================================================================

//==============================================================================
void starting_ssc0_play( void )
{
		{ /* Start playing */
			struct _callback _cb;
            callback_set(&_cb, _ssc0_tx_ep1_play_cb, &ssc_dev_desc[0]);
			struct _buffer _tx = {
                                .data = (unsigned char*)&usb_ep1_ssc0_play_data[0],
				.size = sizeof(usb_ep1_ssc0_play_data),
                                .attr = SSC_BUF_ATTR_WRITE,
			};

			ssc_transfer(&ssc_dev_desc[0], &_tx, &_cb);
		}

		printf("SSC start to play sound\r\n");  
}

void starting_ssc1_play( void )
{
		{ /* Start recording */
			struct _callback _cb;
                        callback_set(&_cb, _ssc1_tx_ep5_play_cb, &ssc_dev_desc[1]);
			struct _buffer _tx = {
                          .data = (unsigned char*)&usb_ep5_ssc1_play_data[0],
                          .size = sizeof(usb_ep5_ssc1_play_data),
                          .attr = SSC_BUF_ATTR_WRITE,
                        };

                        ssc_transfer(&ssc_dev_desc[1], &_tx, &_cb);
		}

		printf("SSC start to play sound\r\n");  
}


void starting_ssc0_record( void )
{
		ssc_enable_receiver(&ssc_dev_desc[0]);

		{ /* Start recording */
			struct _callback _cb;
			callback_set(&_cb, _ssc0_rx_ep2_rec_cb, &ssc_dev_desc[0]);                        
			struct _buffer _rx = {
                        .data = (unsigned char*)&usb_ep2_ssc0_record_data[0],
				.size = sizeof(usb_ep2_ssc0_record_data)/128,
				.attr = SSC_BUF_ATTR_READ,
			};

			ssc_transfer(&ssc_dev_desc[0], &_rx, &_cb);
		}

		printf("SSC start to record sound\r\n");  
}

void starting_ssc1_record( void )
{
		ssc_enable_receiver(&ssc_dev_desc[1]);

		{ /* Start recording */
			struct _callback _cb;
			callback_set(&_cb, _ssc1_rx_ep6_rec_cb, &ssc_dev_desc[1]);                        
			struct _buffer _rx = {
                        .data = (unsigned char*)&usb_ep6_ssc1_record_data[0],
				.size = sizeof(usb_ep6_ssc1_record_data)/128,
				.attr = SSC_BUF_ATTR_READ,
			};

			ssc_transfer(&ssc_dev_desc[1], &_rx, &_cb);
		}

		printf("SSC start to record sound\r\n");  
}


