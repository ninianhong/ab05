#include "compiler.h"
#include "cache.h"

#include "constant.h"
#include "uif_object.h"
#include "kfifo.h"
#include "uif_usb.h"
#include "uif_i2s.h"
#include "circbuf.h"
#include "usbd.h"

List ep2_ssc0_rec;
List ep4_cmd_rec;
List ep6_ssc1_rec;
List ep8_spi_rec;
List ep9_log_rec;
List ep11_i2s0_rec;

List ep1_ssc0_play;
List ep3_cmd_play;
List ep5_ssc1_play;
List ep7_spi_play;
List ep10_i2s0_play;

CACHE_ALIGNED_DDR uint8_t usb_cache_data0[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data1[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data2[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data3[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data4[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data5[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data6[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data7[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data8[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data9[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data10[256][I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_cache_data11[256][I2S_PINGPONG_SIZE_6K];

CACHE_ALIGNED_DDR uint8_t usb_ep1_ssc0_play_data[128*I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_ep3_cmd_play_data[128*I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_ep5_ssc1_play_data[128*I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_ep7_spi_play_data[128*I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_ep10_i2s0_play_data[128*I2S_PINGPONG_SIZE_6K];

CACHE_ALIGNED_DDR uint8_t usb_ep2_ssc0_record_data[128*I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_ep4_cmd_record_data[128*I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_ep6_ssc1_record_data[128*I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_ep8_spi_record_data[128*I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_ep9_log_record_data[128*I2S_PINGPONG_SIZE_6K];
CACHE_ALIGNED_DDR uint8_t usb_ep11_i2s0_record_data[128*I2S_PINGPONG_SIZE_6K];

bool volatile restart_audio_0_bulk_out  = false ; 
bool volatile restart_audio_0_bulk_in   = false ; 
bool volatile restart_audio_1_bulk_out  = false ; 
bool volatile restart_audio_1_bulk_in   = false ; 
bool volatile restart_audio_2_bulk_out  = false ; 
bool volatile restart_audio_2_bulk_in   = false ; 
bool volatile restart_log_bulk_in       = true ;
bool volatile restart_cmd_bulk_out      = false ; 
bool volatile restart_cmd_bulk_in       = false ; 
bool volatile audio_run_control         = false ; 
bool volatile padding_audio_0_bulk_out  = false ;
bool volatile padding_audio_1_bulk_out  = false ;
bool volatile padding_audio_2_bulk_out  = false ;
bool volatile flag_SSC0_start_DmaTx  = false ;
bool volatile flag_SSC1_start_DmaTx  = false ;

extern DataSource source_ssc0;
extern DataSource source_ssc1;

extern uint8_t usbCacheBulkOut0[USB_DATAEP_SIZE_64B*256] ;
extern uint8_t usbCacheBulkIn0[USB_DATAEP_SIZE_64B*256] ;
extern uint8_t usbCacheBulkOut1[USB_DATAEP_SIZE_64B*256] ;
extern uint8_t usbCacheBulkIn1[USB_DATAEP_SIZE_64B*256] ;

//Ring for ssc
extern kfifo_t  ssc0_bulkout_fifo;
extern kfifo_t  ssc0_bulkin_fifo;
extern kfifo_t  ssc1_bulkout_fifo;
extern kfifo_t  ssc1_bulkin_fifo;
extern CircularBuffer ep6_ssc1_rec_cb;

//Ring for USB data endpoint
extern kfifo_t  ep0BulkOut_fifo;
extern kfifo_t  ep0BulkIn_fifo;
extern kfifo_t  ep1BulkOut_fifo;
extern kfifo_t  ep1BulkIn_fifo;
extern kfifo_t  ep2BulkOut_fifo;
extern kfifo_t  ep2BulkIn_fifo;

//Ring for USB cmd endpoint
extern kfifo_t  cmdEpBulkOut_fifo;
extern kfifo_t  cmdEpBulkIn_fifo;

//Ring for Ruler cmd endpoint
extern kfifo_t  cmd_ruler_rece_fifo;
extern kfifo_t  cmd_ruler_send_fifo;


//Ring for spi
extern kfifo_t  spi0_bulkOut_fifo;
extern kfifo_t  spi0_bulkIn_fifo;
extern kfifo_t  spi1_bulkOut_fifo;
extern kfifo_t  spi1_bulkIn_fifo;

bool First_Pack_Check_BO3(uint8_t *pData, unsigned int size, uint32_t *pos)
{
    extern uint8_t volatile global_audio_padding_byte;
    uint32_t i;
    static uint32_t cnt;
    bool ret = false;

    for (i = 0; i < size; i++)
    {
        if (global_audio_padding_byte == *pData)
        {
            cnt++;
        }
        else
        {
            if (cnt >= FIRST_PADDING_SIZE)
              {
              cnt = 0;
              *pos = i;
              ret = true;
              break;
              }
            else
              {
              cnt = 0;
              }
        }
        pData++;
    }
    return ret;
}
//usb->ssc0

void UsbAudio0DataReceived(void * unused,
                           uint8_t status,
                           uint32_t received,
                           uint32_t remaining)
{
    remaining = remaining;
    *(( uint8_t * )unused ) = 0;
    uint32_t pos = 0;

    if (status == USBD_STATUS_SUCCESS)
    {
        if (!padding_audio_0_bulk_out)
        {
            padding_audio_0_bulk_out = First_Pack_Check_BO3(usbCacheBulkOut0, received, &pos);
            if (padding_audio_0_bulk_out && !restart_audio_0_bulk_out)
            {
                kfifo_put(&ssc0_bulkout_fifo, usbCacheBulkOut0 + pos, received - pos);
            }
        }
        else
        {
            if( kfifo_get_free_space(&ssc0_bulkout_fifo) >= received && !restart_audio_0_bulk_out)
              {
                kfifo_put(&ssc0_bulkout_fifo, usbCacheBulkOut0, received);
              }
            else
                assert( 0 );           //It will never be carried out here
        }

        restart_audio_0_bulk_out = true;
    }
    else
    {

    }
}

void UsbAudio0DataTransmit(void *unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining)
{
    remaining = remaining;
    *( ( uint8_t *)unused ) = 1;
    if (status == USBD_STATUS_SUCCESS)
    {
        uint32_t counter = kfifo_get_data_size(&ep0BulkIn_fifo);
        if (counter >= source_ssc0.rxSize && !restart_audio_0_bulk_in)
        {
            // ep0 ring --> usb cache
            kfifo_get(&ep0BulkIn_fifo,
                      usbCacheBulkIn0,
                      source_ssc0.rxSize);
            // send ep0 data ---> pc
            cdcd_serial_driver_WriteAudio_0(usbCacheBulkIn0,
                                          source_ssc0.rxSize, //64B size for low delay
                                          UsbAudio0DataTransmit,
                                          0);
        }
        else
        {
            restart_audio_0_bulk_in = true;
        }
    }
    else
    {
        cdcd_serial_driver_WriteAudio_0(usbCacheBulkIn0,
                                     source_ssc0.rxSize,
                                      UsbAudio0DataTransmit,
                                      0);
    }
}

// I2S1 Play
void UsbAudio1DataReceived(uint32_t unused,
                           uint8_t status,
                           uint32_t received,
                           uint32_t remaining)
{
    remaining = remaining;
    uint32_t pos = 0;

    if (status == USBD_STATUS_SUCCESS)
    {
        if (!padding_audio_1_bulk_out)
        {
            padding_audio_1_bulk_out = First_Pack_Check_BO3(usbCacheBulkOut1, received, &pos);
            if (padding_audio_1_bulk_out && !restart_audio_1_bulk_out)
            {
                kfifo_put(&ssc1_bulkout_fifo, usbCacheBulkOut1 + pos, received - pos);
            }
        }
        else
        {
            if( kfifo_get_free_space(&ssc1_bulkout_fifo) >= received && !restart_audio_1_bulk_out)
              kfifo_put(&ssc1_bulkout_fifo, usbCacheBulkOut1, received);
            else
                assert( 0 );           //It will never be carried out here
        }
        restart_audio_1_bulk_out = true;        
    }
    else
    {
    }
}

// I2S1 Record
void UsbAudio1DataTransmit(void  *unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining)
{
    remaining = remaining;
    if (status == USBD_STATUS_SUCCESS)
    {
        uint32_t counter = kfifo_get_data_size(&ep1BulkIn_fifo);
        if (counter >= source_ssc1.rxSize && !restart_audio_1_bulk_in)
        {
            // ep0 ring --> usb cache
            
            kfifo_get(&ep1BulkIn_fifo,
                      usbCacheBulkIn1,
                      source_ssc1.rxSize);
            // send ep0 data ---> pc
            cdcd_serial_driver_WriteAudio_1(usbCacheBulkIn1,
                                          source_ssc1.rxSize, //64B size for low delay
                                          UsbAudio1DataTransmit,
                                          0);
        }
        else
        {
            restart_audio_1_bulk_in = true;
        }
    }
    else
    {
        cdcd_serial_driver_WriteAudio_1(usbCacheBulkIn1,
                                      source_ssc1.rxSize,
                                      UsbAudio1DataTransmit,
                                      0);
    }
}


void usb_ep1_ssc0_play_cb(void *read, uint8_t status,
		uint32_t received, uint32_t remaining)
{
	uint8_t *data = NULL;
	///TODO:check padding
	//step1:check padding
	//step2:add the newest data to list tail
	//
	data = malloc(sizeof(usb_ep1_ssc0_play_data));
	if( NULL != data )
	{
		memcpy(data,usb_ep1_ssc0_play_data,sizeof(data));
		list_ins_next( &ep1_ssc0_play,(&ep1_ssc0_play)->tail,data);
	}

}

void usb_ep3_cmd_play_cb(void *read, uint8_t status,
		uint32_t received, uint32_t remaining)
{
	uint8_t *data = NULL;
	///TODO:check padding
	//step1:check padding
	//step2:add the newest data to list tail
	//
	data = malloc(sizeof(usb_ep3_cmd_play_data));
	if( NULL != data )
	{
		memcpy(data,usb_ep3_cmd_play_data,sizeof(data));
		list_ins_next( &ep3_cmd_play,(&ep3_cmd_play)->tail,data);
	}


}

void usb_ep5_ssc1_play_cb(void *read, uint8_t status,
		uint32_t received, uint32_t remaining)
{
	uint8_t *data = NULL;
	///TODO:check padding
	//step1:check padding
	//step2:add the newest data to list tail
	//
	data = malloc(sizeof(usb_ep5_ssc1_play_data));
	if( NULL != data )
	{
		memcpy(data,usb_ep5_ssc1_play_data,sizeof(data));
		list_ins_next( &ep5_ssc1_play,(&ep5_ssc1_play)->tail,data);
	}


}

void usb_ep7_spi_play_cb(void *read, uint8_t status,
		uint32_t received, uint32_t remaining)
{
	uint8_t *data = NULL;
	///TODO:check padding
	//step1:check padding
	//step2:add the newest data to list tail
	//
	data = malloc(sizeof(usb_ep7_spi_play_data));
	if( NULL != data )
	{
		memcpy(data,usb_ep7_spi_play_data,sizeof(data));
		list_ins_next( &ep7_spi_play,(&ep7_spi_play)->tail,data);
	}


}

void usb_ep10_i2s0_play_cb(void *read, uint8_t status,
		uint32_t received, uint32_t remaining)
{
	uint8_t *data = NULL;
	///TODO:check padding
	//step1:check padding
	//step2:add the newest data to list tail
	//
	if (status == USBD_STATUS_SUCCESS) 
	{
		data = malloc(sizeof(usb_ep10_i2s0_play_data));
		if( NULL != data )
		{
			memcpy(data,usb_ep10_i2s0_play_data,sizeof(data));
			list_ins_next( &ep10_i2s0_play,(&ep10_i2s0_play)->tail,data);
		}
	}
	else
	{
	
	}
}

//ssc->usb
void usb_ep2_ssc0_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining)
{
    uint8_t *data = usb_ep2_ssc0_record_data;
	if (status == USBD_STATUS_SUCCESS)
		{	
			///TODO:here do nothing,move this action to usb transfer;
			if( ep2_ssc0_rec.size > 0 )
				list_rem_next(&ep2_ssc0_rec, NULL, (void **)&data);
			
		}
	else
		{
			//   //if transfer failed,restart
		}


}

void usb_ep4_cmd_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining)
{


}

extern struct _ssc_desc ssc_dev_desc[2];
void usb_ep6_ssc1_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining)
{
    uint8_t *data = NULL;
    if (status == USBD_STATUS_SUCCESS)
    {
          //*(uint8_t *)unused = 1;
          //if usb transfer successfully,remove the first node,because the data
          //has send;
          //data = cb_readPacket(&ep6_ssc1_rec_cb);
          if( NULL != data )
          {  
              //list_rem_next( &ep6_ssc1_rec,NULL,( void ** )&data );
          }
            
    }
    else
    {
      
    }
    /*
    if( false == is_ssc_started )
    {
        is_ssc_started = true;
        ssc_enable_receiver( ( struct _ssc_desc*)&ssc_dev_desc[1]);
        starting_ssc1_record();
        ssc_enable_transmitter( (struct _ssc_desc*)&ssc_dev_desc[1]);
        starting_ssc1_play();
    }
    */
}

void usb_ep8_spi_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining)
{


}


void usb_ep9_log_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining)
{


}

void usb_ep11_i2s0_record_cb(void* unused,
                           uint8_t status,
                           uint32_t transmit,
                           uint32_t remaining)
{


}

