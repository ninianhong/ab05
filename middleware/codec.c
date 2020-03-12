/*
*********************************************************************************************************
*                                           UIF BOARD DRIVER PACKAGE
*
*                            (c) Copyright 2013 - 2016; Fortemedia Inc.; Nanjing, China
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                          CODEC TLV320AIC3204 Setup
*
*                                          Atmel ATSAMA5D3X
*                                               on the
*                                      Audio Bridge 04 Board (AB04 V1.0)
*
* Filename      : codec.c
* Version       : V2.0.0
* Programmer(s) : PQ
* Porting       : Leo
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/
#include <stdlib.h>
#include "errno.h"
#include "chip.h"
#include "pio.h"
#include "codec.h"
#include "chip_pins.h"
#include "bus.h"
#include "board.h"
#include "timer.h"


#define PA_GAIN0    30
#define PA_GAIN1    31

//extern SpiDevice fm1388;

static const DataSource *pSource;
CODEC_SETS Codec_Set_Saved[2]; //for 2 CODEC

typedef uint8_t CPU_INT08U;

#define SET_VOLUME_MUTE                   1000

#define  DEF_BIT_CLR(val, mask)                        ((val) = ((val) & ~(mask)))
#define  DEF_BIT_CLR_08(val, mask)                     DEF_BIT_CLR(val, mask)

#define  DEF_BIT_SET(val, mask)                        ((val) = ((val) | (mask)))
#define  DEF_BIT_SET_08(val, mask)                     DEF_BIT_SET(val, mask)

#define  DEF_BIT_MASK(bit_mask, bit_shift)             ((bit_mask) << (bit_shift))
#define  DEF_BIT_MASK_08(bit_mask, bit_shift)         ((CPU_INT08U)((CPU_INT08U)(bit_mask) << (bit_shift)))

//#define BOARD_CODEC0_RESET { PIO_GROUP_C, PIO_PC9, PIO_OUTPUT_1 }
//#define BOARD_CODEC1_RESET { PIO_GROUP_C, PIO_PC8, PIO_OUTPUT_1 }
//#define BOARD_CODEC2_RESET { PIO_GROUP_C, PIO_PC7, PIO_OUTPUT_1 }
//#define BOARD_CODEC3_RESET { PIO_GROUP_C, PIO_PC6, PIO_OUTPUT_1 }

//static struct _pin reset_pio_output[] = { BOARD_CODEC0_RESET, BOARD_CODEC1_RESET, \
                                          BOARD_CODEC2_RESET, BOARD_CODEC3_RESET };

//LED play&Record
static struct _pin codec_reset_pin[] = {
                                          { PIO_GROUP_C, PIO_PC9, PIO_OUTPUT_1, PIO_DEFAULT },
                                          { PIO_GROUP_C, PIO_PC8, PIO_OUTPUT_1, PIO_DEFAULT },
                                          { PIO_GROUP_C, PIO_PC7, PIO_OUTPUT_1, PIO_DEFAULT },
                                          { PIO_GROUP_C, PIO_PC6, PIO_OUTPUT_1, PIO_DEFAULT }
                                        };
void init_codec_rst_pin()
{

  pio_configure(codec_reset_pin, sizeof(codec_reset_pin)/sizeof(struct _pin));  
  
}

void Pin_Reset_Codec(unsigned char id)
{
    pio_clear(&codec_reset_pin[id]);
    //OSTimeDly(1); //reset must active >10ns from Spec
    msleep(1);
    //UIF_Misc_On(reset_pin[id]);
    pio_set(&codec_reset_pin[id]);
    //OSTimeDly(1);
    msleep(1);
}

static void PA_shutdown(bool en)
{
    //if (en)
        //UIF_Misc_On(PA_SHUTDOWN);
    //else
        //UIF_Misc_Off(PA_SHUTDOWN);
}

static void PA_set_gain0(bool val)
{
    //if (val)
        //UIF_Misc_Off(PA_GAIN0);
    //else
        //UIF_Misc_On(PA_GAIN0);
}

static void PA_set_gain1(bool val)
{
    //if (val)
    //    UIF_Misc_Off(PA_GAIN1);
    //else
    //    UIF_Misc_On(PA_GAIN1);
}

static int32_t pa_x10_cur = SET_VOLUME_MUTE;
/*
*********************************************************************************************************
*                                       PA_set_gain()
*
* Description : Set PA's gain
*
* Argument(s) : pa_x10 :  80(8db),
                          120(12db),
                          175(17.5db),
                          235(23.5db),
                          1000(shutdown).
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void PA_set_gain(int32_t pa_x10)
{
    if (pa_x10 != pa_x10_cur)
    {
        switch (pa_x10)
        {
        case 80: // 8dB-00
            if (SET_VOLUME_MUTE == pa_x10_cur)
            {
                PA_shutdown(false);
            }
            PA_set_gain1(false);
            PA_set_gain0(false);
            break;

        case 120: // 12dB-01
            if (SET_VOLUME_MUTE == pa_x10_cur)
            {
                PA_shutdown(false);
            }
            PA_set_gain1(false);
            PA_set_gain0(true);
            break;

        case 175: // 17.5dB-10
            if (SET_VOLUME_MUTE == pa_x10_cur)
            {
                PA_shutdown(false);
            }
            PA_set_gain1(true);
            PA_set_gain0(false);
            break;

        case 235: // 23.5dB-11
            if (SET_VOLUME_MUTE == pa_x10_cur)
            {
                PA_shutdown(false);
            }
            PA_set_gain1(true);
            PA_set_gain0(true);
            break;

        case SET_VOLUME_MUTE: // shutdown
            PA_shutdown(true);
            break;

        default: // gain=8dB-00,shutdown
            PA_shutdown(true);
            PA_set_gain1(false);
            PA_set_gain0(false);
            pa_x10_cur = SET_VOLUME_MUTE;
            break;
        }
        pa_x10_cur = pa_x10;
    }
}

uint8_t Codec_Read(const DataSource *pSource, uint8_t dev_addr, uint8_t reg, uint8_t *pVal)
{
    TWI_CFG *twi_option = (TWI_CFG *)pSource->privateData;

    uint8_t state = 0;

    twi_option->address = dev_addr >> 1;
    twi_option->iaddress = reg;
    twi_option->isize = 1;

    //    state =  TWID_Read( dev_addr>>1, reg, 1, pVal++,1, NULL) ;
    state = pSource->buffer_read((void *)pSource, pVal++, 1);
    return state;
}

#if 0
uint8_t Codec_Write(const DataSource *pSource, uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint8_t buf[] = {data};
    uint8_t state;

    TWI_CFG *twi_option = (TWI_CFG *)pSource->privateData;

    twi_option->address = dev_addr >> 1;
    twi_option->iaddress = reg;
    twi_option->isize = 1;

    //    state =  TWID_Write( dev_addr>>1, reg, 1 , buf, sizeof(buf), NULL);
    //    state = twi1_write( ( void * )pSource , buf, sizeof( buf ) );
    state = pSource->buffer_write((void *)pSource, buf, sizeof(buf));
    return state;
}
#else
uint8_t Codec_Write(const DataSource *pSource, uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    pSource = pSource;
    int err;
    uint8_t _data[2] = { reg , data };

    //TWI_CFG *twi_option = (TWI_CFG *)pSource->privateData;

    //twi_option->address = dev_addr >> 1;
    //twi_option->iaddress = reg;
    //twi_option->isize = 1;

    //    state =  TWID_Write( dev_addr>>1, reg, 1 , buf, sizeof(buf), NULL);
    //    state = twi1_write( ( void * )pSource , buf, sizeof( buf ) );
    //state = pSource->buffer_write((void *)pSource, buf, sizeof(buf));
	struct _buffer buf[1] = {
		{
			.data = _data,
			.size = 2,
			.attr = BUS_I2C_BUF_ATTR_START | BUS_BUF_ATTR_TX | BUS_I2C_BUF_ATTR_STOP,
		}
	};
        
            //state = twi2_write((void *)pSource, buf, 1);
    bus_start_transaction(pca9546.bus);
    err = bus_transfer(pca9546.bus, dev_addr>>1, buf, 1, NULL);
    bus_stop_transaction(pca9546.bus);

    if (err < 0)
	return err;//false;
    return 0;//true;
    //return state;
}

/*
static int _act8865_write_reg(struct _act8865* act8865, uint8_t iaddr, uint8_t value)
{
	int err;
	uint8_t _data[2] = { iaddr , value };
	struct _buffer buf[1] = {
		{
			.data = _data,
			.size = 2,
			.attr = BUS_I2C_BUF_ATTR_START | BUS_BUF_ATTR_TX | BUS_I2C_BUF_ATTR_STOP,
		}
	};

	bus_start_transaction(act8865->bus);
	err = bus_transfer(act8865->bus, act8865->addr, buf, 1, NULL);
	bus_stop_transaction(act8865->bus);

	return err;
}
*/
#endif

uint8_t Codec_Read_SPI(uint8_t dev_addr, uint8_t reg, uint8_t *pVal)
{
    uint8_t state = 0;
    //state =  TWID_Read( dev_addr>>1, reg, 1, pVal++,1, NULL) ;
    return (state);
}

uint8_t Codec_Write_SPI(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    //uint8_t buf[] = {dev_addr << 1, reg, data};
    //uint8_t state;

    //state = SPI_WriteBuffer_API(&fm1388, buf, 3);

    return 0;//state;
}

/*
*********************************************************************************************************
*                                    I2C_Switcher()
*
* Description :  set PCA9546 channel
*
* Argument(s) :  i2c_channel  : channel index:  For AB04_2nd
*		            0 - FM36 , 1 - CODEC0, 2 - CODEC1       
*                
*
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static unsigned char I2C_Switcher_Index_Save = 0;
#if 0
uint8_t I2C_Switcher(uint8_t i2c_channel)
{
    const uint8_t dev_addr = PCA9546A_ADDRESS;

    uint8_t buf[] = {0x01 << i2c_channel};
    uint8_t state;
    DataSource *pSource = &source_twi2;

    TWI_CFG *twi_option = (TWI_CFG *)pSource->privateData;

    twi_option->address = dev_addr >> 1;
    twi_option->iaddress = 0;
    twi_option->isize = 0;

    if (i2c_channel > 3)
    {
        return 1; //err
    }
    if (I2C_Switcher_Index_Save == i2c_channel)
    { //no need re-set
        return 0;
    }
    I2C_Switcher_Index_Save = i2c_channel;

    state = twi2_write((void *)pSource, buf, 1);
    
    return state;
}
#else
uint8_t I2C_Switcher(uint8_t i2c_channel)
{
    //const uint8_t dev_addr = PCA9546A_ADDRESS;

    uint8_t channel[] = {0x01 << i2c_channel};
    int err;
    
    //DataSource *pSource = &source_twi2;

    //TWI_CFG *twi_option = (TWI_CFG *)pSource->privateData;

    //twi_option->address = dev_addr >> 1;
    //twi_option->iaddress = 0;
    //twi_option->isize = 0;
    
    struct _buffer buf[1] = {
		//{
		//	.data = &iaddr,
		//	.size = 1,
		//	.attr = BUS_I2C_BUF_ATTR_START | BUS_BUF_ATTR_TX | BUS_I2C_BUF_ATTR_STOP,
		//},
		{
			.data = channel,
			.size = sizeof(channel),
			.attr = BUS_I2C_BUF_ATTR_START | BUS_BUF_ATTR_TX | BUS_I2C_BUF_ATTR_STOP,
		},
	};

    if (i2c_channel > 3)
    {
        return 1; //err
    }
    if (I2C_Switcher_Index_Save == i2c_channel)
    { //no need re-set
        return 0;
    }
    I2C_Switcher_Index_Save = i2c_channel;

    //state = twi2_write((void *)pSource, buf, 1);
    bus_start_transaction(pca9546.bus);
    err = bus_transfer(pca9546.bus, pca9546.addr, buf, 1, NULL);
    bus_stop_transaction(pca9546.bus);

    if (err < 0)
	return err;
    return 0;
}
#endif

uint8_t Set_Codec(const DataSource *pSource, uint8_t codec_control_type, uint8_t size_para, uint8_t *pdata)
{
    uint8_t i, state = 0;
    for (i = 0; i < size_para; i++)
    {
        if (codec_control_type == 0)
        {
            state = Codec_Write(pSource, CODEC_ADDRESS, *(pdata + i * 2), *(pdata + i * 2 + 1));
        }
        else
        {
            state = Codec_Write_SPI(AD1938_ADDRESS, *(pdata + i * 2), *(pdata + i * 2 + 1));
        }
        if (state != 0)
            break;
    }
    return state;
}

uint8_t Get_Codec(const DataSource *pSource, uint8_t codec_control_type, uint8_t reg, uint8_t *pdata)
{
    uint8_t state = 0;

    if (codec_control_type == 0)
    {
        state = Codec_Read(pSource, CODEC_ADDRESS, reg, pdata);
    }
    else
    {
        state = Codec_Read_SPI(CODEC_ADDRESS, reg, pdata);
    }

    return state;
}

uint8_t I2CWrite_Codec(const DataSource *pSource, uint8_t reg, uint8_t data)
{
    uint8_t err;
    err = Codec_Write(pSource, AD1937_ADDRESS, reg, data);
    return err;
}

uint8_t I2CWrite_Codec_AIC3204(const DataSource *pSource, uint8_t reg, uint8_t data)
{
    uint8_t err;
    err = Codec_Write(pSource, CODEC_ADDRESS, reg, data);
    return err;
}

uint8_t I2CRead_Codec_AIC3204(const DataSource *pSource, uint8_t reg, uint8_t *pdata)
{
    uint8_t err;
    err = Codec_Read(pSource, CODEC_ADDRESS, reg, pdata);
    return err;
}

/******************************      AD1937        ************************************/
// OSC=12.288Mhz
/*
void ALL_POWER_ON(void)
{
    I2CWrite_Codec(DAC_Control0,0x00);  //
    I2CWrite_Codec(ADC_Control0,0x00);  //
}


void ALL_POWER_OFF(void)
{
    I2CWrite_Codec(DAC_Control0,0x01);  //
    I2CWrite_Codec(ADC_Control0,0x01);  //
}


uint8_t Codec_DAC_Attenuation( uint8_t DAC_NAME, uint32_t x10gain )
{

    uint8_t err;
    float temp ;

    if( x10gain == 1000 ){
        return 0 ;
    }
    if (x10gain > 945) {//0.375*252=94.5dB
        err = CODEC_SETVOL_RANGE_ERR;
        APP_TRACE_INFO(("ERR: CODEC Gain Over Range!\r\n"));
        return err;
    }
    temp = x10gain/(0.375*10);
    err = I2CWrite_Codec(DAC_NAME,(uint8_t)temp);
    if( OS_ERR_NONE != err ) {
        err = CODEC_WR_REG_ERR;
        return err ;
    }
    APP_TRACE_DBG(("\r\nSet CODEC REG[%d]=0x%0X",DAC_NAME,(uint8_t)temp));

    return 0;

}

//uint8_t Codec_DAC_Attenuation( uint8_t DAC_NAME, float gain )
//{
//
//    uint8_t err;
//    float temp ;
//
//    if ( gain > 0 ) {
//        err = 1;
//        return err;
//    }
//
//    if(gain< -95.25 ) {
//        temp = 255 ;
//
//    } else {
//        temp = -gain/0.375;
//
//    }
//
//    err = I2CWrite_Codec(DAC_NAME,(uint8_t)temp);
//
//    return err;
//
//}

unsigned short SR_Support[] =         {
                                            //8000,
                                            16000,
                                            24000,
                                            32000,
                                            //44100,
                                            48000
                                            //96000
                                      };

uint8_t Check_SR_Support( uint32_t sample_rate )
{
   uint32_t i;
   for( i = 0; i<(sizeof(SR_Support)/2); i++ ) {
       if( SR_Support[i] == sample_rate ) {
           return OS_ERR_NONE ; //find the SR
       }
   }
   return CODEC_SR_NOT_SUPPORT_ERR;  //SR not support

}


// Main clock source = 12.288 MHz.
uint8_t Codec_SetFCLK( uint32_t fclk )
{

    uint8_t err;
    uint8_t pll_reg;

    switch( fclk )   {

        case 8000:  // Need  12.288 / 2 MHz Osc
            pll_reg = 0x9e;
            return CODEC_SETFCLK_RANGE_ERR;
            break;
        case 16000:
            pll_reg = 0x9e;
            break;
        case 24000:
            pll_reg = 0x9c;
            break;
        case 32000:
            pll_reg = 0x9a;
            break;
        case 44100: // Need 11.2896 MHz Osc
            pll_reg = 0x98;
            return CODEC_SETFCLK_RANGE_ERR;
            break;
        case 48000:
            pll_reg = 0x98;
            break;
        case 96000: // Need 12.288 * 2 MHz Osc
            pll_reg = 0x98;
            return CODEC_SETFCLK_RANGE_ERR;
            break;
        case 0 : //power down clock
            pll_reg = 0x01;
            break;
        default:
            return CODEC_SETFCLK_RANGE_ERR;
            break;

    }

    err = I2CWrite_Codec( PLL_Control0, pll_reg );
    if( OS_ERR_NONE != err ) {
        err = CODEC_WR_REG_ERR;
        return err ;
    }

    return err;

}


uint8_t Codec_SetMode( uint8_t mode )
{
    uint8_t err;
    uint8_t dac_reg0,dac_reg1;
    uint8_t adc_reg1,adc_reg2;

    switch( mode )  {

        case I2S_MODE:
                 dac_reg0 = 0x00;      //I2S
                 dac_reg1 = 0x70;      //2 channels
                 adc_reg1 = 0x03;      //I2S
                 adc_reg2 = 0x01;      //2 channels
                 break;
        case TDM_MODE:
                 dac_reg0 = 0x40;      //TDM
                 dac_reg1 = 0x74;      // 8 channels
                 adc_reg1 = 0x23;      //TDM
                 adc_reg2 = 0x21;      // 8 channels
                 break;
        case TDM16_MODE:
                 dac_reg0 = 0x40;      //TDM
                 dac_reg1 = 0x76;      // 16 channels
                 adc_reg1 = 0x23;      //TDM
                 adc_reg2 = 0x31;      // 16 channels
                 break;
        default:
                return CODEC_SETMODE_RANGE_ERR;
                break;
    }

    err = I2CWrite_Codec( DAC_Control0, dac_reg0 );
    if( OS_ERR_NONE != err ) {
        err = CODEC_WR_REG_ERR;
        return err ;
    }
    err = I2CWrite_Codec( DAC_Control1, dac_reg1 );
    if( OS_ERR_NONE != err ) {
        err = CODEC_WR_REG_ERR;
        return err ;
    }

    err = I2CWrite_Codec( ADC_Control1, adc_reg1 );
    if( OS_ERR_NONE != err ) {
        err = CODEC_WR_REG_ERR;
        return err ;
    }
    err = I2CWrite_Codec( ADC_Control2, adc_reg2 );
    if( OS_ERR_NONE != err ) {
        err = CODEC_WR_REG_ERR;
        return err ;
    }

    return err;

}


uint8_t Init_CODEC( uint32_t sample_rate )
{
    uint8_t err;
    uint8_t i;
    static uint32_t sr_saved;

    uint8_t reg_para[][2] = {

        { PLL_Control0,0x98 },
        { PLL_Control1,0x00 },  //
        { DAC_Control2,0x18 },  // 16bit
        { DAC_Mute    ,0xF0 },  // DAC3.4 mute.
        { DAC1L_Volume,0x00 },  // no attenuation on SPK
        { DAC1R_Volume,0x00 },  // no attenuation on SPK
        { DAC2L_Volume,0x00 },  // no attenuation on Lout
        { DAC2R_Volume,0x00 },  // no attenuation on Lout
        { ADC_Control0,0x30 }  //Enable ADC
 //    { DAC3L_Volume,0x00 },  // not used on AB03
 //    { DAC3R_Volume,0x00 },  // not used on AB03

    };

    if( sample_rate == sr_saved ) {
        return 0;
    } else {
        sr_saved = sample_rate ;
    }

    for( i = 0; i< sizeof(reg_para)>>1; i++ ) {
      err = I2CWrite_Codec(reg_para[i][0], reg_para[i][1]);
      if( OS_ERR_NONE != err ) {
          err = CODEC_WR_REG_ERR;
          return err ;
      }

    }

    err = Codec_SetFCLK( sample_rate );
    if( OS_ERR_NONE != err ) {
        err = CODEC_WR_REG_ERR;
        return err ;
    }

    err = Codec_SetMode( TDM_MODE );
    if( OS_ERR_NONE != err ) {
        err = CODEC_WR_REG_ERR;
        return err ;
    }

    if( !( Get_Switches() & 0x01 ) ) {
        err = CODEC_LOUT_Small_Gain_En( true ); //attenuation enable
    }

    return err;

}



*/

uint8_t CODEC_LOUT_Small_Gain_En(bool small_gain)
{
    /*
    uint8_t err;
    uint8_t reg;
    if( small_gain ) {
        reg = 64;    // 64*0.375=24dB attenuation on Lout
        APP_TRACE_INFO(("Lout Gain 24dB attenuation: Enabled \r\n"));

    } else {
        reg = 0;    // 0dB attenuation on Lout
        APP_TRACE_INFO(("Lout Gain 24dB attenuation: Disabled \r\n"));

    }
    err = I2CWrite_Codec( DAC2L_Volume, reg );
    if( OS_ERR_NONE != err ) {
        err = CODEC_WR_REG_ERR;
        return err ;
    }

    err = I2CWrite_Codec( DAC2R_Volume, reg );
    if( OS_ERR_NONE != err ) {
        err = CODEC_WR_REG_ERR;
        return err ;
    }

    return err;
    */
    return 236u;        //CODEC_FUNC_NOT_SUPPORT;
}

uint8_t encode(int8_t value)
{
    int8_t temp;
    if (value >= 0)
    {
        return value;
    }
    else
    {
        temp = ~(abs(value) - 1);
        return temp;
    }
}

//hsw170515
/*
*********************************************************************************************************
*                                       codec_set_gains()
*
* Description : Set codec's hp,lo,lin gain at the same time
*
* Argument(s) : pSource : pointer to SET_VOLUME structure data
                hp_x10 :    -60~290, step 10.(1000-mute) (The actual value: -6~29dB.step 1dB)
                lout_x10 :  -60~290, step 10.(1000-mute) (The actual value: -6~29dB.step 1dB)
                lin_x10 :   0~475, step 5.(1000-mute)    (The actual value: 0~47.5dB.step 0.5dB)
* Return(s)   : NO_ERR :   execute successfully
*               others :   =error code .
*
* Note(s)     : None.
*********************************************************************************************************
*/
uint8_t codec_set_gains(const DataSource *pSource, int32_t hp_x10, int32_t lout_x10, int32_t lin_x10)
{
    uint8_t err = 0;
    int8_t hp = 0;
    int8_t lout = 0;
    int8_t lin = 0;

    uint8_t hp_us = 0;
    uint8_t lout_us = 0;
    uint8_t lin_us = 0;
    //-----------------------------------lin_x10
    if (SET_VOLUME_MUTE == lin_x10)
    {
        lin = 0;
        lin_us = (uint8_t)lin;
        MICPGA_GAIN_ENABLE_OFF(lin_us);
    }
    else
    {
        lin = lin_x10 / 5;
        if (lin < 0)
        {
            lin = 0;
        }
        if (lin > 95)
        {
            lin = 95;
        }
        lin_us = (uint8_t)lin;
        MICPGA_GAIN_ENABLE_ON(lin_us);
    }

    //----------------------------------hp_x10
    if (SET_VOLUME_MUTE == hp_x10)
    {
        hp = 0;
        hp_us = (uint8_t)hp;
        HP_LOUT_DRV_GAIN_SET_RESERVED_SET_0(hp_us);
        HP_LOUT_DRV_GAIN_SET_MUTE_ON(hp_us);
    }
    else
    {
        hp = hp_x10 / 10;
        if (hp < -6)
        {
            hp = -6;
        }
        if (hp > 29)
        {
            hp = 29;
        }
        hp_us = (uint8_t)hp;
        HP_LOUT_DRV_GAIN_SET_RESERVED_SET_0(hp_us);
        HP_LOUT_DRV_GAIN_SET_MUTE_OFF(hp_us);
    }

    //----------------------------------lout_x10
    if (SET_VOLUME_MUTE == lout_x10)
    {
        lout = 0;
        lout_us = (uint8_t)lout;
        HP_LOUT_DRV_GAIN_SET_RESERVED_SET_0(lout_us);
        HP_LOUT_DRV_GAIN_SET_MUTE_ON(lout_us);
    }
    else
    {
        lout = lout_x10 / 10;
        if (lout < -6)
        {
            lout = -6;
        }
        if (lout > 29)
        {
            lout = 29;
        }
        lout_us = (uint8_t)lout;
        HP_LOUT_DRV_GAIN_SET_RESERVED_SET_0(lout_us);
        HP_LOUT_DRV_GAIN_SET_MUTE_OFF(lout_us);
    }

    err = I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1
    if (OS_ERR_NONE != err)
    {
        err = 230u;                //CODEC_WR_REG_ERR;
        return err;
    }

    I2CWrite_Codec_AIC3204(pSource, 59, lin_us);
    I2CWrite_Codec_AIC3204(pSource, 60, lin_us);

    I2CWrite_Codec_AIC3204(pSource, 16, hp_us);
    I2CWrite_Codec_AIC3204(pSource, 17, hp_us);

    I2CWrite_Codec_AIC3204(pSource, 18, lout_us);
    I2CWrite_Codec_AIC3204(pSource, 19, lout_us);

    return err;
}

static int32_t hp1_x10_cur = 0;
//static uint8_t hp1_gain_cur =  0;

static int32_t lout1_x10_cur = 0;
static uint8_t lout1_gain_cur = 0;

static int32_t lin1_x10_cur = 0;
//static uint8_t lin1_gain_cur =  0;

/*
*********************************************************************************************************
*                                       codec1_set_volume()
*
* Description : Set codec1's volume
*
* Argument(s) : pSource : pointer to SET_VOLUME structure data
                hp_x10 :    0
                lout_x10 :  -695~530@step 5 (The actual value: -69.5~53dB@step 0.5dB), 1000-mute
                lin_x10 :   -120~675@step 5 (The actual value: -12~67.5dB@step 0.5dB), 1000-mute
* Return(s)   : NO_ERR :   execute successfully
*               others :   =error code .
*
* Note(s)     : hp_x10 == 0.
*********************************************************************************************************
*/
uint8_t codec1_set_volume(const DataSource *pSource, int32_t hp_x10, int32_t lout_x10, int32_t lin_x10, uint8_t force_set)
{
    uint8_t err = NO_ERR;
    int32_t temp = 0;
    bool flag = false;
    uint8_t lin_gain = 0, ADC_gain = 0;

    if (lin_x10 != lin1_x10_cur || force_set == 1)
    {
        if (SET_VOLUME_MUTE == lin_x10)
        {
            err = I2CWrite_Codec_AIC3204(pSource, 0, 0); // switch to Page0
            if (OS_ERR_NONE != err)
            {
                err = 230u;             //CODEC_WR_REG_ERR;
                return err;
            }
            I2CWrite_Codec_AIC3204(pSource, ADC_fine_gain_reg_P0, 0x88); // ADC mute
        }
        else
        {
            if (lin_x10 < 0)
            {
                lin_gain = 0;
                ADC_gain = (uint8_t)(lin_x10 / 5);
            }
            else
            {
                for (int8_t j = 0; j < 20 + 1; j++)
                { // ADC_gain
                    for (int8_t i = 0; i < 95 + 1; i++)
                    { // lin_gain
                        temp = i * 5 + j * 10;
                        if (temp == lin_x10)
                        {
                            lin_gain = (uint8_t)i;
                            ADC_gain = (uint8_t)(j << 1);
                            flag = true;
                        }
                        if (flag)
                            break;
                    }
                    if (flag)
                        break;
                }
                flag = false;
            }
            err = I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1
            if (OS_ERR_NONE != err)
            {
                err = 230u;             //CODEC_WR_REG_ERR;
                return err;
            }
            MICPGA_GAIN_ENABLE_ON(lin_gain);
            I2CWrite_Codec_AIC3204(pSource, LIL_gain_reg_P1, lin_gain);
            I2CWrite_Codec_AIC3204(pSource, LIR_gain_reg_P1, lin_gain);
            I2CWrite_Codec_AIC3204(pSource, 0, 0); //switch to Page0
            DEF_BIT_CLR_08(ADC_gain, DEF_BIT_MASK_08(1u, 7));
            if (SET_VOLUME_MUTE == lin1_x10_cur)
                I2CWrite_Codec_AIC3204(pSource, ADC_fine_gain_reg_P0, 0x00); // ADC unmute
            I2CWrite_Codec_AIC3204(pSource, ADC_l_gain_reg_P0, ADC_gain);
            I2CWrite_Codec_AIC3204(pSource, ADC_r_gain_reg_P0, ADC_gain);

            //lin1_gain_cur = lin_gain;
        }
        lin1_x10_cur = lin_x10;
    }
    //----------------------------------------------------hp,lout
    bool flag1 = false;
    bool flag2 = false;
    int32_t temp1 = 0;
    int32_t temp2 = 0;
    uint8_t DAC_gain = 0, hp_gain = 0, lout_gain = 0;

    if (lout_x10 != lout1_x10_cur || force_set == 1)
    {
        if (SET_VOLUME_MUTE == lout_x10)
        {
            err = I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1
            if (OS_ERR_NONE != err)
            {
                err = 230u;                 //CODEC_WR_REG_ERR;
                return err;
            }
            HP_LOUT_DRV_GAIN_SET_RESERVED_SET_0(lout1_gain_cur);
            HP_LOUT_DRV_GAIN_SET_MUTE_ON(lout1_gain_cur);
            lout_gain = lout1_gain_cur;
            hp_gain = lout_gain;

            I2CWrite_Codec_AIC3204(pSource, LOL_gain_reg_P1, lout_gain);
            I2CWrite_Codec_AIC3204(pSource, LOR_gain_reg_P1, lout_gain);
            I2CWrite_Codec_AIC3204(pSource, HPL_gain_reg_P1, hp_gain);
            I2CWrite_Codec_AIC3204(pSource, HPR_gain_reg_P1, hp_gain);
        }
        else
        {
            for (int8_t k = 0; k < 48 + 1; k++)
            { // DAC_gain
                for (int8_t m = -6; m < 29 + 1; m++)
                {
                    temp1 = k * 5 + m * 10;
                    if (temp1 == lout_x10)
                    {
                        DAC_gain = (uint8_t)k;
                        lout_gain = (uint8_t)m;
                        hp_gain = lout_gain;
                        flag1 = true;
                    }
                    if (flag1)
                        break;
                }
                if (flag1)
                    break;
                flag1 = false;
            }

            if (!flag1)
            {
                for (int8_t k = 0; k > -127 - 1; k--)
                { // DAC_gain
                    for (int8_t m = -6; m < 29 + 1; m++)
                    {
                        temp2 = k * 5 + m * 10;
                        if (temp2 == lout_x10)
                        {
                            DAC_gain = (uint8_t)k;
                            lout_gain = (uint8_t)m;
                            hp_gain = lout_gain;
                            flag2 = true;
                        }
                        if (flag2)
                            break;
                    }
                    if (flag2)
                        break;
                    flag2 = false;
                }
            }

            flag1 = false;
            flag2 = false;
            I2CWrite_Codec_AIC3204(pSource, 0, 0); //switch to Page0
            I2CWrite_Codec_AIC3204(pSource, DAC_l_gain_reg_P0, DAC_gain);
            I2CWrite_Codec_AIC3204(pSource, DAC_r_gain_reg_P0, DAC_gain);
            I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1

            HP_LOUT_DRV_GAIN_SET_RESERVED_SET_0(lout_gain);
            HP_LOUT_DRV_GAIN_SET_MUTE_OFF(lout_gain);
            hp_gain = lout_gain;

            I2CWrite_Codec_AIC3204(pSource, LOL_gain_reg_P1, lout_gain);
            I2CWrite_Codec_AIC3204(pSource, LOR_gain_reg_P1, lout_gain);
            I2CWrite_Codec_AIC3204(pSource, HPL_gain_reg_P1, hp_gain);
            I2CWrite_Codec_AIC3204(pSource, HPR_gain_reg_P1, hp_gain);

            lout1_gain_cur = lout_gain;
            //hp1_gain_cur = hp_gain;
        }
        lout1_x10_cur = lout_x10;
    }
    return err;
}

/*
*********************************************************************************************************
*                                       codec_IN1_to_MICPGA_route()
*
* Description : Set codec's IN1 route or not routed to MICPGA
*
* Argument(s) : pSource : pointer to SET_VOLUME structure data
                status :  true-routed, false-not routed.
* Return(s)   : NO_ERR :   execute successfully
*               others :   =error code .
*
* Note(s)     : nothing
*********************************************************************************************************
*/
uint8_t codec_IN1_to_MICPGA_route(const DataSource *pSource, bool status)
{
    uint8_t err = NO_ERR;
    err = I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1
    if (OS_ERR_NONE != err)
    {
        err = 230u;             //CODEC_WR_REG_ERR;
        return err;
    }
    if (status)
    {
        I2CWrite_Codec_AIC3204(pSource, P1_R52, 0x40); //IN1L is routed to Left MICPGA
        I2CWrite_Codec_AIC3204(pSource, P1_R55, 0x40); //IN1R is routed to Right MICPGA
    }
    else
    {
        I2CWrite_Codec_AIC3204(pSource, P1_R52, 0x00); //IN1L is not routed to Left MICPGA
        I2CWrite_Codec_AIC3204(pSource, P1_R55, 0x00); //IN1R is not routed to Right MICPGA
    }
    return err;
}

/*
*********************************************************************************************************
*                                       codec_DAC_to_HP_route()
*
* Description : Set DAC reconstruction filter's terminal routed or not routed to HP
*
* Argument(s) : pSource : pointer to SET_VOLUME structure data
                status :  true-routed, false-not routed.
* Return(s)   : NO_ERR :   execute successfully
*               others :   =error code .
*
* Note(s)     : nothing
*********************************************************************************************************
*/
uint8_t codec_DAC_to_HP_route(const DataSource *pSource, bool status)
{
    uint8_t err = NO_ERR;
    err = I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1
    if (OS_ERR_NONE != err)
    {
        err = 230u;             //CODEC_WR_REG_ERR;
        return err;
    }
    if (status)
    {
        I2CWrite_Codec_AIC3204(pSource, P1_R12, 0x08); //routed
        I2CWrite_Codec_AIC3204(pSource, P1_R13, 0x08); //routed
    }
    else
    {
        I2CWrite_Codec_AIC3204(pSource, P1_R12, 0x00); //not routed
        I2CWrite_Codec_AIC3204(pSource, P1_R13, 0x00); //not routed
    }
    return err;
}

/*
*********************************************************************************************************
*                                       codec_DAC_to_LO_route()
*
* Description : Set DAC reconstruction filter's terminal routed or not routed to LO
*
* Argument(s) : pSource : pointer to SET_VOLUME structure data
                status :  true-routed, false-not routed.
* Return(s)   : NO_ERR :   execute successfully
*               others :   =error code .
*
* Note(s)     : nothing
*********************************************************************************************************
*/
uint8_t codec_DAC_to_LO_route(const DataSource *pSource, bool status)
{
    uint8_t err = NO_ERR;
    err = I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1
    if (OS_ERR_NONE != err)
    {
        err = 230u;       //CODEC_WR_REG_ERR;
        return err;
    }
    if (status)
    {
        I2CWrite_Codec_AIC3204(pSource, P1_R14, 0x08); //routed
        I2CWrite_Codec_AIC3204(pSource, P1_R15, 0x08); //routed
    }
    else
    {
        I2CWrite_Codec_AIC3204(pSource, P1_R14, 0x00); //not routed
        I2CWrite_Codec_AIC3204(pSource, P1_R15, 0x00); //not routed
    }
    return err;
}

static int32_t hp2_x10_cur = 0;
//static uint8_t hp2_gain_cur =  0;

static int32_t lout2_x10_cur = 0;
static uint8_t lout2_gain_cur = 0;

static int32_t lin2_x10_cur = 0;
//static uint8_t lin2_gain_cur =  0;

/*
*********************************************************************************************************
*                                       codec2_set_volume()
*
* Description : Set codec2's volume
*
* Argument(s) : pSource : pointer to SET_VOLUME structure data
                hp_x10 :    0
                lout_x10 :  -695~530@step 5 (The actual value: -69.5~53dB@step 0.5dB), 1000-mute
                lin_x10 :   -120~675@step 5 (The actual value: -12~67.5dB@step 0.5dB), 1000-mute
* Return(s)   : NO_ERR :   execute successfully
*               others :   =error code .
*
* Note(s)     : hp_x10 == 0.
*********************************************************************************************************
*/
uint8_t codec2_set_volume(const DataSource *pSource, int32_t hp_x10, int32_t lout_x10, int32_t lin_x10, uint8_t force_set)
{
    uint8_t err = NO_ERR;
    int32_t temp = 0;
    bool flag = false;
    uint8_t lin_gain = 0, ADC_gain = 0;

    if (lin_x10 != lin2_x10_cur || force_set == 1)
    {
        if (SET_VOLUME_MUTE == lin_x10)
        {
            err = I2CWrite_Codec_AIC3204(pSource, 0, 0); // switch to Page0
            if (OS_ERR_NONE != err)
            {
                err = 230u;           //CODEC_WR_REG_ERR;
                return err;
            }
            I2CWrite_Codec_AIC3204(pSource, ADC_fine_gain_reg_P0, 0x88); // ADC mute
        }
        else
        {
            if (lin_x10 < 0)
            {
                lin_gain = 0;
                ADC_gain = (uint8_t)(lin_x10 / 5);
            }
            else
            {
                for (int8_t j = 0; j < 20 + 1; j++)
                { // ADC_gain
                    for (int8_t i = 0; i < 95 + 1; i++)
                    { // lin_gain
                        temp = i * 5 + j * 10;
                        if (temp == lin_x10)
                        {
                            lin_gain = (uint8_t)i;
                            ADC_gain = (uint8_t)(j << 1);
                            flag = true;
                        }
                        if (flag)
                            break;
                    }
                    if (flag)
                        break;
                }
                flag = false;
            }
            err = I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1
            if (OS_ERR_NONE != err)
            {
                err = 230u;            //CODEC_WR_REG_ERR;
                return err;
            }
            MICPGA_GAIN_ENABLE_ON(lin_gain);
            I2CWrite_Codec_AIC3204(pSource, LIL_gain_reg_P1, lin_gain);
            I2CWrite_Codec_AIC3204(pSource, LIR_gain_reg_P1, lin_gain);
            I2CWrite_Codec_AIC3204(pSource, 0, 0); //switch to Page0
            DEF_BIT_CLR_08(ADC_gain, DEF_BIT_MASK_08(1u, 7));
            if (SET_VOLUME_MUTE == lin2_x10_cur)
                I2CWrite_Codec_AIC3204(pSource, ADC_fine_gain_reg_P0, 0x00); // ADC unmute
            I2CWrite_Codec_AIC3204(pSource, ADC_l_gain_reg_P0, ADC_gain);
            I2CWrite_Codec_AIC3204(pSource, ADC_r_gain_reg_P0, ADC_gain);

            //lin2_gain_cur = lin_gain;
        }
        lin2_x10_cur = lin_x10;
    }
    //----------------------------------------------------hp,lout
    bool flag1 = false;
    bool flag2 = false;
    int32_t temp1 = 0;
    int32_t temp2 = 0;
    uint8_t DAC_gain = 0, hp_gain = 0, lout_gain = 0;

    if (lout_x10 != lout2_x10_cur || force_set == 1)
    {
        if (SET_VOLUME_MUTE == lout_x10)
        {
            err = I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1
            if (OS_ERR_NONE != err)
            {
                err = 230u;          //CODEC_WR_REG_ERR;
                return err;
            }
            HP_LOUT_DRV_GAIN_SET_RESERVED_SET_0(lout2_gain_cur);
            HP_LOUT_DRV_GAIN_SET_MUTE_ON(lout2_gain_cur);
            lout_gain = lout2_gain_cur;
            hp_gain = lout_gain;

            I2CWrite_Codec_AIC3204(pSource, LOL_gain_reg_P1, lout_gain);
            I2CWrite_Codec_AIC3204(pSource, LOR_gain_reg_P1, lout_gain);
            I2CWrite_Codec_AIC3204(pSource, HPL_gain_reg_P1, hp_gain);
            I2CWrite_Codec_AIC3204(pSource, HPR_gain_reg_P1, hp_gain);
        }
        else
        {
            for (int8_t k = 0; k < 48 + 1; k++)
            { // DAC_gain
                for (int8_t m = -6; m < 29 + 1; m++)
                {
                    temp1 = k * 5 + m * 10;
                    if (temp1 == lout_x10)
                    {
                        DAC_gain = (uint8_t)k;
                        lout_gain = (uint8_t)m;
                        hp_gain = lout_gain;
                        flag1 = true;
                    }
                    if (flag1)
                        break;
                }
                if (flag1)
                    break;
                flag1 = false;
            }

            if (!flag1)
            {
                for (int8_t k = 0; k > -127 - 1; k--)
                { // DAC_gain
                    for (int8_t m = -6; m < 29 + 1; m++)
                    {
                        temp2 = k * 5 + m * 10;
                        if (temp2 == lout_x10)
                        {
                            DAC_gain = (uint8_t)k;
                            lout_gain = (uint8_t)m;
                            hp_gain = lout_gain;
                            flag2 = true;
                        }
                        if (flag2)
                            break;
                    }
                    if (flag2)
                        break;
                    flag2 = false;
                }
            }

            flag1 = false;
            flag2 = false;
            I2CWrite_Codec_AIC3204(pSource, 0, 0); //switch to Page0
            I2CWrite_Codec_AIC3204(pSource, DAC_l_gain_reg_P0, DAC_gain);
            I2CWrite_Codec_AIC3204(pSource, DAC_r_gain_reg_P0, DAC_gain);
            I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1

            HP_LOUT_DRV_GAIN_SET_RESERVED_SET_0(lout_gain);
            HP_LOUT_DRV_GAIN_SET_MUTE_OFF(lout_gain);
            hp_gain = lout_gain;

            I2CWrite_Codec_AIC3204(pSource, LOL_gain_reg_P1, lout_gain);
            I2CWrite_Codec_AIC3204(pSource, LOR_gain_reg_P1, lout_gain);
            I2CWrite_Codec_AIC3204(pSource, HPL_gain_reg_P1, hp_gain);
            I2CWrite_Codec_AIC3204(pSource, HPR_gain_reg_P1, hp_gain);

            lout2_gain_cur = lout_gain;
            //hp2_gain_cur = hp_gain;
        }
        lout2_x10_cur = lout_x10;
    }
    return err;
}

//init codec volume with saved volume in Codec_Init()
unsigned char codec_set_volume_init(unsigned char codec_id)
{
    unsigned char err = 0;

    //APP_TRACE_INFO_T(("Set Volume Init..."));
    DataSource *pSource = NULL;//&source_twi2;

    if (codec_id == 0)
    {
        I2C_Switcher(I2C_SWITCH_CODEC0);
        err = codec1_set_volume(pSource, hp1_x10_cur, lout1_x10_cur, lin1_x10_cur, 1);
        if (OS_ERR_NONE != err)
        {
            //APP_TRACE_INFO_T(("<ERROR>: Set CODEC0 gain failed(%d).", err));
            return err;
        }
    }
    else
    {
        I2C_Switcher(I2C_SWITCH_CODEC1);
        err = codec2_set_volume(pSource, hp2_x10_cur, lout2_x10_cur, lin2_x10_cur, 1);
        if (OS_ERR_NONE != err)
        {
            //APP_TRACE_INFO_T(("<ERROR>: Set CODEC1 gain failed(%d).", err));
            return err;
        }
    }

    return err;
}

uint8_t CODEC_Set_Volume(const DataSource *pSource, int vol_spk_tmp, int vol_lout_tmp, int vol_lin_tmp)
{
    uint8_t err;
    float temp = 0;
    float vol_spk = 0;
    float vol_lout = 0;
    float vol_lin = 0;
    uint8_t flag = 0;
    uint8_t Mic_PGA = 0, ADC_GAIN = 0;

    vol_spk = (vol_spk_tmp - vol_spk_tmp % 5) / 10;
    vol_lout = (vol_lout_tmp - vol_lout_tmp % 5) / 10;
    vol_lin = (vol_lin_tmp - vol_lin_tmp % 5) / 10;

    if (vol_lin < -12)
    {
        vol_lin = -12;
    }
    if (vol_lin > 67.5)
    {
        vol_lin = 67.5;
    }
    if (vol_spk < -69.5)
    {
        vol_spk = -69.5;
    }
    if (vol_spk > 53)
    {
        vol_spk = 53;
    }
    if (vol_lout < -69.5)
    {
        vol_lout = -69.5;
    }
    if (vol_lout > 53)
    {
        vol_lout = 53;
    }

    for (uint8_t i = 0; i < 95 + 1; i++)
    {
        for (int8_t j = -24; j < 40 + 1; j++)
        {
            temp = i * 0.5 + j * 0.5;
            if (temp == vol_lin)
            {
                Mic_PGA = encode(i);
                ADC_GAIN = encode(j); //now not support negative
                flag = 1;
            }
            if (flag == 1)
                break;
        }
        if (flag == 1)
            break;
    }
    flag = 0;
    err = I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1
    if (OS_ERR_NONE != err)
    {
        err = 230u;      //CODEC_WR_REG_ERR;
        return err;
    }
    I2CWrite_Codec_AIC3204(pSource, 59, Mic_PGA);
    I2CWrite_Codec_AIC3204(pSource, 60, Mic_PGA);
    I2CWrite_Codec_AIC3204(pSource, 0, 0); //switch to Page0
    I2CWrite_Codec_AIC3204(pSource, 83, ADC_GAIN);
    I2CWrite_Codec_AIC3204(pSource, 84, ADC_GAIN);

    signed char DAC_GAIN = 0, HPL_GAIN = 0, LOL_GAIN = 0;
    unsigned char flag1 = 0, flag2 = 0;
    for (signed char k = 0; k < 48 + 1; k++)
    {
        for (signed char m = -6; m < 29 + 1; m++)
        {
            temp = k * 0.5 + m;
            if (temp == vol_lout && flag1 == 0)
            {
                DAC_GAIN = encode(k);
                LOL_GAIN = encode(m);
                flag1 = 1;
            }
            if (temp == vol_spk && flag2 == 0)
            {
                DAC_GAIN = encode(k);
                HPL_GAIN = encode(m);
                flag2 = 1;
            }
            if (flag1 == 1 && flag2 == 1)
                break;
        }
        if (flag1 == 1 && flag2 == 1)
            break;
        flag1 = 0;
        flag2 = 0;
    }
    if (flag1 == 0 || flag2 == 0)
    {
        for (signed char k = 0; k > -127 - 1; k--)
        {
            for (signed char m = -6; m < 29 + 1; m++)
            {
                temp = k * 0.5 + m;
                if (temp == vol_lout && flag1 == 0)
                {
                    DAC_GAIN = encode(k);
                    LOL_GAIN = encode(m);
                    flag1 = 1;
                }
                if (temp == vol_spk && flag2 == 0)
                {
                    DAC_GAIN = encode(k);
                    HPL_GAIN = encode(m);
                    flag2 = 1;
                }
                if (flag1 == 1 && flag2 == 1)
                    break;
            }
            if (flag1 == 1 && flag2 == 1)
                break;
            flag1 = 0;
            flag2 = 0;
        }
    }

    flag1 = 0;
    flag2 = 0;
    I2CWrite_Codec_AIC3204(pSource, 0, 0); //switch to Page0
    I2CWrite_Codec_AIC3204(pSource, 65, DAC_GAIN);
    I2CWrite_Codec_AIC3204(pSource, 66, DAC_GAIN);
    I2CWrite_Codec_AIC3204(pSource, 0, 1); //switch to Page1
    I2CWrite_Codec_AIC3204(pSource, 16, HPL_GAIN);
    I2CWrite_Codec_AIC3204(pSource, 17, HPL_GAIN);
    I2CWrite_Codec_AIC3204(pSource, 18, LOL_GAIN);
    I2CWrite_Codec_AIC3204(pSource, 19, LOL_GAIN);

    //    if( vol_spk == SET_VOLUME_MUTE ) {
    //        temp += (3<<0);
    //    }
    //    if( vol_lin == SET_VOLUME_MUTE ) {
    //        temp += (3<<2);
    //    }
    //    err = I2CWrite_Codec_AIC3204(pSource,DAC_Mute,temp);

    return err;
}

uint8_t Set_AIC3204_DSP_Offset(uint8_t slot_index)
{

    uint8_t err;

    if (slot_index > 6)
    { //slot_index is for line in channels
        return 0x99;
    }
    err = I2CWrite_Codec_AIC3204(pSource, 28, slot_index << 5);

    return err;
}

unsigned int CODEC_SUPPORT_SR[] = {

    96000, 48000, 44100, 32000, 24000, 22050, 16000, 8000

};

uint8_t Check_SR_Support(uint32_t sample_rate)
{
    uint32_t i;
    for (i = 0; i<sizeof(CODEC_SUPPORT_SR)>> 2; i++)
    {
        if (CODEC_SUPPORT_SR[i] == sample_rate)
        {
            return 0; //find the SR
        }
    }
    return 234u;              //CODEC_SR_NOT_SUPPORT_ERR; //SR not support
}

/*
//CODEC PLL setting based on 24.576MHz MCLK
unsigned short CODEC_PLL_PARA_TABLE[][14][7] = {

    { //mode 0
          //I2S format
          //BCLK = 16 * 2 * FCLK = 32 * FCLK
	  //parameter for MCLK = 24.576MHz
          {48000, 44100, 32000, 24000, 22050, 16000, 8000 }, //SR
	  {0x82,  0x88,  0x83,  0x82,  0x88,  0x83,  0x83 }, //  REG_NDAC    =
	  {0x82,  0x84,  0x82,  0x84,  0x84,  0x84,  0x81 }, //  REG_MDAC    = --R12
	  {0x80,  0x40,  0x80,  0x80,  0x80,  0x80,  0x00 }, //  REG_DOSR    = --R13-14
	  {0x84,  0x82,  0x84,  0x84,  0x84,  0x84,  0xA0 }, //  REG_BCLK_DIV=  --R30
	  {0x82,  0x88,  0x83,  0x82,  0x88,  0x83,  0x86 }, //  REG_NADC    =
	  {0x84,  0x82,  0x84,  0x84,  0x82,  0x84,  0x84 }, //  REG_MADC    =
	  {0x00,  0x80,  0x40,  0x80,  0x00,  0x80,  0x80 }, //REG_AOSR    =  --R20
	  //{3.072M, 2.8224M, 2.048M, 3.072M, 2.8224M, 2.048M, 1.024M},//     --PDMCLK   :
	  {0x00,  0x03,  0x00,  0x00,  0x03,  0x00,  0x00 }, //CLK_MUX     =  --Select CODEC_CLKIN
	  {0,     1,     0,     0,     1,    0,     0}, //PLL_EN      =
	  {0, 	  1,     0,     0,     1,    0,     0}, //PLL_R       =
	  {0,     2,     0,     0,     2,    0,     0}, //PLL_P       =
	  {0,     7,     0,     0,     7,    0,     0}, //PLL_J       =
	  {0,     3500,  0,     0,     3500, 0,     0}  //PLL_D       =


    },

    { //mode 1
          //TDM16 format
          //BCLK = 16 * 8 * FCLK = 128 * FCLK
	  //parameter for MCLK = 24.576MHz
          {48000, 44100, 32000, 24000, 22050, 16000, 8000 }, //SR
	  {0x82,  0x88,  0x83,  0x82,  0x88,  0x83,  0x83 }, //  REG_NDAC    =
	  {0x82,  0x84,  0x82,  0x84,  0x84,  0x84,  0x81 }, //  REG_MDAC    = --R12
	  {0x80,  0x40,  0x80,  0x80,  0x80,  0x80,  0x00 }, //  REG_DOSR    = --R13-14
	  {0x82 , 0x82,  0x82,  0x84,  0x84,  0x84,  0x88 }, //  REG_BCLK_DIV= --R30
	  {0x82,  0x88,  0x83,  0x82,  0x88,  0x83,  0x86 }, //  REG_NADC    =
	  {0x84,  0x82,  0x84 , 0x84,  0x82,  0x84,  0x84 }, //  REG_MADC    =
	  {0x00,  0x80,  0x40,  0x80,  0x00,  0x80,  0x80 }, //  REG_AOSR    =  --R20
	  //{3.072M, 2.8224M, 2.048M, 3.072M, 2.8224M, 2.048M, 1.024M},//     --PDMCLK   :
	  {0x00,  0x03,  0x00,  0x00,  0x03,  0x00,  0x00 }, //  CLK_MUX     =  --Select CODEC_CLKIN
	  {0,     1,     0,     0,     1,    0,     0}, //PLL_EN      =
	  {0, 	  1,     0,     0,     1,    0,     0}, //PLL_R       =
	  {0,     2,     0,     0,     2,    0,     0}, //PLL_P       =
	  {0,     7,     0,     0,     7,    0,     0}, //PLL_J       =
	  {0,     3500,  0,     0,     3500, 0,     0}  //PLL_D       =


    },

    { //mode 2
          //TDM32 format
          //BCLK = 32 * 8 * FCLK = 256 * FCLK
	  //parameter for MCLK = 24.576MHz
          {48000, 44100, 32000, 24000, 22050, 16000, 8000 }, //SR
	  {0x82,  0x88,  0x83,  0x82,  0x88,  0x83,  0x83 }, //  REG_NDAC    =
	  {0x82,  0x84,  0x82,  0x84,  0x84,  0x84,  0x81 }, //  REG_MDAC    = --R12
	  {0x80,  0x40,  0x80,  0x80,  0x80,  0x80,  0x00 }, //  REG_DOSR    = --R13-14
	  {0x81,  0x81,  0x81,  0x82,  0x82,  0x82,  0x84 }, //  REG_BCLK_DIV=  --R30
	  {0x82,  0x88,  0x83,  0x82,  0x88,  0x83,  0x86 }, //  REG_NADC    =
	  {0x84,  0x82,  0x84,  0x84,  0x82,  0x84,  0x84 }, //  REG_MADC    =
	  {0x00,  0x80,  0x40,  0x80,  0x00,  0x80,  0x80 }, //REG_AOSR    =  --R20
	  //{3.072M, 2.8224M, 2.048M, 3.072M, 2.8224M, 2.048M, 1.024M},//     --PDMCLK   :
	  {0x00,  0x03,  0x00,  0x00,  0x03,  0x00,  0x00 }, //CLK_MUX     =  --Select CODEC_CLKIN
	  {0,     1,     0,     0,     1,    0,     0}, //PLL_EN      =
	  {0, 	  1,     0,     0,     1,    0,     0}, //PLL_R       =
	  {0,     2,     0,     0,     2,    0,     0}, //PLL_P       =
	  {0,     7,     0,     0,     7,    0,     0}, //PLL_J       =
	  {0,     3500,  0,     0,     3500, 0,     0}  //PLL_D       =
    },

    { //mode 3
          //I2S32 format
          //BCLK = 32 * 2 * FCLK = 64 * FCLK
	  //parameter for MCLK = 24.576MHz
          {48000, 44100, 32000, 24000, 22050, 16000, 8000 }, //SR
	  {0x82,  0x88,  0x83,  0x82,  0x88,  0x83,  0x83 }, //  REG_NDAC    =
	  {0x82,  0x84,  0x82,  0x84,  0x84,  0x84,  0x81 }, //  REG_MDAC    = --R12
	  {0x80,  0x40,  0x80,  0x80,  0x80,  0x80,  0x00 }, //  REG_DOSR    = --R13-14
	  {0x84,  0x88,  0x88,  0x90,  0x90,  0x90,  0xA0 }, //  REG_BCLK_DIV=  --R30
	  {0x82,  0x88,  0x83,  0x82,  0x88,  0x83,  0x86 }, //  REG_NADC    =
	  {0x84,  0x82,  0x84,  0x84,  0x82,  0x84,  0x84 }, //  REG_MADC    =
	  {0x00,  0x80,  0x40,  0x80,  0x00,  0x80,  0x80 }, //REG_AOSR    =  --R20
	  //{3.072M, 2.8224M, 2.048M, 3.072M, 2.8224M, 2.048M, 1.024M},//     --PDMCLK   :
	  {0x00,  0x03,  0x00,  0x00,  0x03,  0x00,  0x00 }, //CLK_MUX     =  --Select CODEC_CLKIN R4
	  {0,     1,     0,     0,     1,    0,     0}, //PLL_EN      =
	  {0, 	  1,     0,     0,     1,    0,     0}, //PLL_R       =
	  {0,     2,     0,     0,     2,    0,     0}, //PLL_P       =
	  {0,     7,     0,     0,     7,    0,     0}, //PLL_J       =
	  {0,     3500,  0,     0,     3500, 0,     0}  //PLL_D       =
    },

    {//mode 4
          //TDM 4slot 32bit
         //BCLK = 32 * 4 * FCLK = 128 * FCLK
	 //parameter for MCLK = 24.576MHz
          {48000, 44100, 32000, 24000, 22050, 16000, 8000 }, //SR
	  {0x82,  0x88,  0x83,  0x82,  0x88,  0x83,  0x83 }, //  REG_NDAC    =
	  {0x82,  0x84,  0x82,  0x84,  0x84,  0x84,  0x81 }, //  REG_MDAC    = --R12
	  {0x80,  0x40,  0x80,  0x80,  0x80,  0x80,  0x00 }, //  REG_DOSR    = --R13-14
	  {0x82,  0x82,  0x82,  0x84,  0x84,  0x84,  0x88 }, //  REG_BCLK_DIV=  --R30
	  {0x82,  0x88,  0x83,  0x82,  0x88,  0x83,  0x86 }, //  REG_NADC    =
	  {0x84,  0x82,  0x84,  0x84,  0x82,  0x84,  0x84 }, //  REG_MADC    =
	  {0x00,  0x80,  0x40,  0x80,  0x00,  0x80,  0x80 }, //  REG_AOSR    =  --R20
	  //{3.072M, 2.8224M, 2.048M, 3.072M, 2.8224M, 2.048M, 1.024M},//     --PDMCLK   :
	  {0x00,  0x03,  0x00,  0x00,  0x03,  0x00,  0x00 }, //CLK_MUX     =  --Select CODEC_CLKIN
	  {0,     1,     0,     0,     1,    0,     0}, //PLL_EN      =
	  {0, 	  1,     0,     0,     1,    0,     0}, //PLL_R       =
	  {0,     2,     0,     0,     2,    0,     0}, //PLL_P       =
	  {0,     7,     0,     0,     7,    0,     0}, //PLL_J       =
	  {0,     3500,  0,     0,     3500, 0,     0}  //PLL_D       =

    }

};


uint8_t audio_interface[] = {
          0x0c,   //I2S mode,16bit,master
          0x00,   //I2S mode,16bit,Slave
          0x4c,   //DSP mode,16bit,master
          0x40,   //DSP mode,16bit,Slave
          0xcc,   //LJF mode,16bit,master
          0X8c,   //RJF mode,16bit,master

};

unsigned short BCLK_SOURCE[] = { //reg29 0:DAC_CLK  1: DAC_MOD_CLK , D3 = 1 for bclk invert
          0x9, //I2S 16bit format
          0x8, //TDM 16bit format
          0x8, //TDM 32bit format
          0x8, //I2S 32bit format
          0x8  //TDM 32bit 4slot format
};
*/

uint8_t config_aic3204[][2] = {

    0, 0x00, //page0

    //Software reset codec
    1, 0X01,

    //		      //SET PLL == MCLK*R*J.D/P   mclk == 12.288MHz;
    //		      4,CLK_MUX[SR_Index],
    //		      5,PLL_EN[SR_Index]*128 +PLL_P[SR_Index]*16 + PLL_R[SR_Index],
    //		      6,PLL_J[SR_Index],
    //		      7,math.floor( PLL_D[SR_Index]/256),
    //		      8,PLL_D[SR_Index]%256,
    //
    //		      //Set DAC_fs == PLL/NDAC*MDAC*DOSR
    //		      11,REG_NDAC[SR_Index],  //NDAC=3
    //		      12,REG_MDAC[SR_Index],  //MDAC=4
    //		      //DOSR=128
    //		      13,math.floor(REG_DOSR[SR_Index]/256),
    //		      14,REG_DOSR[SR_Index]%256,
    //
    //		      //Set ADC_fs == PLL/NADC*MADC*AOSR   SET PDMCLK=ADC_MOD_CLK = 2.048MHz
    //		      18,REG_NADC[SR_Index],  //NADC=3
    //		      19,REG_MADC[SR_Index],  //MADC=4
    //		      //AOSR=128
    //		      20,REG_AOSR[SR_Index],
    //		      //if master mode,reg20,reg30 is needed.
    //
    //		      //BDIV_CLKIN Multiplexer Control
    //		      29,BCLK_SOURCE,           // ADC2DAC_ROUTED is not rounted  ;
    ////		      29,0X10+BCLK_SOURCE,    // ADC2DAC_ROUTED
    //		      30,REG_BCLK_DIV[SR_Index], //0X84,  //bclk=bdiv_clkin/4

    //SET interface mode(I2S,PCM,Left,right)
    27, 0X0c, //I2S mode,16bit,master
    //27,0X00,   //I2S mode,16bit,Slave
    //27,0X4c,   //DSP mode,16bit,master
    //27,0Xcc,   //LJF mode,16bit,master
    //27,0X8c,   //RJF mode,16bit,master
    //		      27, HS,
    //		      //Data offset
    28, 0X00, //data offset == 0'bclk for I2S Mode, there have a cycle delay in I2S mode itself
              //		      //28,0X01,    //data offset == 1'bclk for DSP Mode
    32, 0X00,
    33, 0X4d,
    34, 0X20,
    53, 0X02, //Dout is pin5
    54, 0X02, //pin4 is i2s data input

    //-set DAC channels
    63, 0xd4,                      //DAC Channel Setup :  0xD4: L2L, R2R; 0xE8: L2R, R2L
    64, 0X00,                      //
    65, (0X100 + 2 * (0)) % 0x100, //DAC Volume L set 0 dB  : [-63.5,+24] @ 0.5dB  //20
    66, (0X100 + 2 * (0)) % 0x100, //DAC Volume R set 0 dB  : [-63.5,+24] @ 0.5dB  //20
    //-set dmic data pin setting
    55, 0X0e, // Set MISO as PDM CLK ouput pin
    56, 0X02, // SCLK pin is enabled

    81, 0xD0, // enable ADC and set SCLK as PDM DATA input pin//////-
    //Dmic clock output(=adc_mod_clk), PDM CLK = ADC_MOD_CLK
    82, 0X00,    //ADC Fine gain adjust, 0dB, unmute
    83, 2 * (0), //ADC Volume L set 0dB  : [-12,+20] @ 0.5dB   D?óú0dBμ?éè??2?°′?a??à′￡?±è???é·3
    84, 2 * (0), //ADC Volume R set 0dB  : [-12,+20] @ 0.5dB   D?óú0dBμ?éè??2?°′?a??à′￡?±è???é·3

    0, 0X01, //page1//////////////////////////
             //
             //		      //-set power
    1, 0x08, //disconnect AVDD and DVDD
    2, 0X01, //enable Master Analog Power Control
    3, 0X00, //Set the DAC L PTM mode to PTM_P3/4 //L-class-AB -hsw
    4, 0X00, //Set the DAC R PTM mode to PTM_P3/4 //R-class-AB
    9, 0XFF, //All HPOUT,LOUT and Mixer Amplifier are Power up
    //9, 0x3C,
    10, 0X00, //Set the Input Common Mode to 0.9V and Output Common Modefor Headphone to Input Common Mode
    20, 0Xa4, //headphone driver startup // 100ms,5.0,25k

    //-set route settings
    //CODEC LO to FL124 LIN, single ended
    12, 0X08, //HPL route on -hrd1708
    13, 0X08, //HPR route on
    14, 0X08, //LOL route on
    15, 0X08, //LOR route on

    //Analog input mixer settings -hrd1708
    52, 0X40, // IN1L to L_MICPGA
    54, 0X40, // CM1L to L_MICPGA
    55, 0X40, // IN1R to R_MICPGA
    57, 0X40, // CM1R to R_MICPGA

    //init HP gains
    16, (0X00 + (0)), //HPL 0 db gain :  [-6,+29] @ 1dB,init 0db,unmute -hsw170623
    17, (0X00 + (0)), //HPR 0 db gain :  [-6,+29] @ 1dB,init 0db,unmute -hsw170623

    //init LO gains
    18, (0X00 + (0)), //LOL 0 db gain :  [-6,+29] @ 1dB,init 0db,unmute -hsw170623
    19, (0X00 + (0)), //LOR 0 db gain :  [-6,+29] @ 1dB,init 0db,unmute -hsw170623

    //init MIC PGA Gain
    59, 0X00, //L_MICPGA 0db gain
    60, 0X00, //R_MICPGA 0db gain

};

uint8_t Generate_FCLK(uint32_t codec_clkin, uint32_t sr, uint8_t NDAC, uint8_t MDAC, uint32_t DOSR)
{
    uint32_t temp;

    temp = codec_clkin * 1000 / NDAC / MDAC / DOSR;
    if (temp == sr)
    {
        return 1;
    }
    return 0;
}

uint8_t Generate_BCLK(uint32_t codec_clkin, uint32_t BCLK, uint8_t NDAC, uint8_t BCLK_N_divider)
{
    uint32_t temp;

    temp = codec_clkin * 1000 / NDAC / BCLK_N_divider;
    if (temp == BCLK)
    {
        return 1;
    }
    return 0;
}

uint8_t Set_Codec_PLL(const DataSource *pSource, uint32_t sr, uint8_t sample_length, uint8_t slot_num, uint8_t bclk_polarity)
{
    uint8_t err, i;
    uint8_t flag = 0;
    uint8_t NDAC, MDAC, NADC, MADC;
    uint32_t DOSR;
    uint8_t DOSR_H, DOSR_L;
    uint8_t BCLK_N_divider;
    uint32_t BCLK;

    uint32_t codec_clkin = 73728;
    uint8_t codec_para[6][2];

    uint8_t codec_para1[][2] = {
        0x04, 0x03, // PLL is codec_clkin
        0x05, 0xC3, // PLL power up , P=4, R=1
        0x06, 0x04, // J=4
        0x07, 0x00, // D=0
        0x08, 0x00, // D=0
        0x1D, 0x00, //BCLK source=DAC_CLK
    };              //make sure codec_clkin=73.728M  length=16bit 32bit slot_num=2,4,6,8
    uint8_t codec_para2[][2] = {
        0x04, 0x03, // PLL is codec_clkin
        0x05, 0xC1, // PLL power up , P=4, R=1
        0x06, 0x03, // J=3
        0x07, 0x00, // D=0
        0x08, 0x00, // D=0
        0x1D, 0x00, //BCLK source=DAC_CLK
    };              //make sure codec_clkin=18.432M  length=24bit slot_num=2,4,6,8
    uint8_t codec_para3[][2] = {
        0x04, 0x03, // PLL is codec_clkin
        0x05, 0xC2, // PLL power up , P=4, R=2
        0x06, 0x0D, // J=13
        0x07, 0x04, // D=1250
        0x08, 0xE2, // D=1250
        0x1D, 0x00, //BCLK source=DAC_CLK
    };
    //make sure codec_clkin=161.28M  length=16,24,32bit slot_num=1,3,5,7
    uint8_t codec_para4[][2] = {
        0x04, 0x03, // PLL is codec_clkin
        0x05, 0x85, // PLL power up , P=8, R=5
        0x06, 0x08, // J=26
        0x07, 0x0A, // D=2688
        0x08, 0x80, // D=2688
        0x1D, 0x00, //BCLK source=DAC_CLK
    };              //make sure codec_clkin=127.008M  length=24bit slot_num=1,2,4,6,8
    uint8_t codec_para5[][2] = {
        0x04, 0x03, // PLL is codec_clkin
        0x05, 0x81, // PLL power up , P=8, R=1
        0x06, 0x30, // J=48
        0x07, 0x09, // D=2344
        0x08, 0x28, // D=2344
        0x1D, 0x00, //BCLK source=DAC_CLK
    };              //make sure codec_clkin=148.176M  length=16,32bit slot_num=3,5,7
    uint8_t codec_para6[][2] = {
        0x04, 0x03, // PLL is codec_clkin
        0x05, 0x81, // PLL power up , P=8, R=1
        0x06, 0x24, // J=36
        0x07, 0x06, // D=1758
        0x08, 0xDE, // D=1758
        0x1D, 0x00, //BCLK source=DAC_CLK
    };              //make sure codec_clkin=111.132M  length=24bit slot_num=3,5,7
    uint8_t codec_para7[][2] = {
        0x04, 0x03, // PLL is codec_clkin
        0x05, 0x81, // PLL power up , P=8, R=1
        0x06, 0x37, // J=55
        0x07, 0x04, // D=1250
        0x08, 0xE2, // D=1250
        0x1D, 0x00, //BCLK source=DAC_CLK
    };              //make sure codec_clkin=169.334M  length=16,32bit slot_num=1,2,4,6,8

    if ((sample_length == 16 || sample_length == 32) && (slot_num == 1 || slot_num == 2 || slot_num == 4 || slot_num == 6 || slot_num == 8))
    {
        for (uint8_t i = 0; i<sizeof(codec_para1)>> 1; i++)
        {
            codec_para[i][0] = codec_para1[i][0];
            codec_para[i][1] = codec_para1[i][1];
        }
        codec_clkin = 73728;
    }
    else if ((sample_length == 16 || sample_length == 32) && (slot_num == 3 || slot_num == 5 || slot_num == 7))
    {
        for (uint8_t i = 0; i<sizeof(codec_para3)>> 1; i++)
        {
            codec_para[i][0] = codec_para3[i][0];
            codec_para[i][1] = codec_para3[i][1];
            codec_clkin = 161280;
        }
    }
    if ((sample_length == 24) && (slot_num == 1 || slot_num == 2 || slot_num == 4 || slot_num == 6 || slot_num == 8))
    {
        for (uint8_t i = 0; i<sizeof(codec_para2)>> 1; i++)
        {
            codec_para[i][0] = codec_para2[i][0];
            codec_para[i][1] = codec_para2[i][1];
        }
        codec_clkin = 18432;
    }
    else if ((sample_length == 24) && (slot_num == 3 || slot_num == 5 || slot_num == 7))
    {
        for (uint8_t i = 0; i<sizeof(codec_para3)>> 1; i++)
        {
            codec_para[i][0] = codec_para3[i][0];
            codec_para[i][1] = codec_para3[i][1];
        }
        codec_clkin = 161280;
    }

    if ((sr == 22050 || sr == 44100) && sample_length == 24 && (slot_num == 1 || slot_num == 2 || slot_num == 4 || slot_num == 6 || slot_num == 8))
    {
        for (uint8_t i = 0; i<sizeof(codec_para4)>> 1; i++)
        {
            codec_para[i][0] = codec_para4[i][0];
            codec_para[i][1] = codec_para4[i][1];
        }
        codec_clkin = 127008;
    }
    else if ((sr == 22050 || sr == 44100) && sample_length == 32 && (slot_num == 3 || slot_num == 5 || slot_num == 7))
    {
        for (uint8_t i = 0; i<sizeof(codec_para5)>> 1; i++)
        {
            codec_para[i][0] = codec_para5[i][0];
            codec_para[i][1] = codec_para5[i][1];
        }
        codec_clkin = 148176;
    }
    else if ((sr == 22050 || sr == 44100) && sample_length == 24 && (slot_num == 3 || slot_num == 5 || slot_num == 7))
    {
        for (uint8_t i = 0; i<sizeof(codec_para6)>> 1; i++)
        {
            codec_para[i][0] = codec_para6[i][0];
            codec_para[i][1] = codec_para6[i][1];
        }
        codec_clkin = 111132;
    }
    else if ((sr == 22050 || sr == 44100) && sample_length == 32 && (slot_num == 1 || slot_num == 2 || slot_num == 4 || slot_num == 6 || slot_num == 8))
    {
        for (uint8_t i = 0; i<sizeof(codec_para7)>> 1; i++)
        {
            codec_para[i][0] = codec_para7[i][0];
            codec_para[i][1] = codec_para7[i][1];
        }
        codec_clkin = 169344;
    }

    if (48000 == sr)
    {
        BCLK = sr * sample_length * slot_num;
        NDAC = 1;
        MDAC = 12;
        DOSR = 128;
        //BCLK_N_divider=12;
        for (BCLK_N_divider = 1; BCLK_N_divider < 129; BCLK_N_divider++)
        {
            if (Generate_BCLK(codec_clkin, BCLK, NDAC, BCLK_N_divider) == 1)
                break;
        }
    }
    else
    {
        BCLK = sr * sample_length * slot_num;
        for (NDAC = 1; NDAC < 129; NDAC++)
        {
            for (MDAC = 1; MDAC < 129; MDAC++)
            {
                for (DOSR = 1; DOSR < 1025; DOSR++)
                {
                    if (Generate_FCLK(codec_clkin, sr, NDAC, MDAC, DOSR) == 1)
                    {
                        for (BCLK_N_divider = 1; BCLK_N_divider < 129; BCLK_N_divider++)
                        {
                            if (Generate_BCLK(codec_clkin, BCLK, NDAC, BCLK_N_divider) == 1)
                            {
                                flag = 1;
                            }
                            if (flag == 1)
                                break;
                        }
                    }
                    if (flag == 1)
                        break;
                }
                if (flag == 1)
                    break;
            }
            if (flag == 1)
                break;
        }
        flag = 0;
    }

    if (DOSR == 1024)
    {
        DOSR_H = 0;
        DOSR_L = 0;
    }
    else
    {
        DOSR_H = DOSR / 256;
        DOSR_L = DOSR % 256;
    }
    NADC = NDAC;
    MADC = MDAC;

    for (i = 0; i<sizeof(codec_para)>> 1; i++)
    {
        if ((bclk_polarity == 1) && (codec_para[i][0] == 0x1D))
        {
            codec_para[i][1] |= 0x08; //blck poarity = 1, bclk inverted
        }
        err = I2CWrite_Codec_AIC3204(pSource, codec_para[i][0], codec_para[i][1]);
        if (OS_ERR_NONE != err)
        {
            return err;
        }
    }

    uint8_t reg_data[][2] = {
        0x0B, 0x80 | NDAC,
        0x0C, 0x80 | MDAC,
        0x0D, DOSR_H,
        0x0E, DOSR_L,
        0x12, 0x80 | NADC,
        0x13, 0x80 | MADC,
        0x1E, 0x80 | BCLK_N_divider};

    for (i = 0; i<sizeof(reg_data)>> 1; i++)
    {
        err = I2CWrite_Codec_AIC3204(pSource, reg_data[i][0], reg_data[i][1]);
        if (OS_ERR_NONE != err)
        {
            return err;
        }
    }

    return err;
}

CODEC_SETS Codec_Set_Saved[2]; //for 2 CODEC

uint8_t Init_CODEC(CODEC_SETS codec_set)

{
    uint8_t err;
    uint8_t i, if_set;
    DataSource *pSource = NULL;//&source_twi2;

    if ((Codec_Set_Saved[codec_set.id].sr == codec_set.sr) &&
        (Codec_Set_Saved[codec_set.id].sample_len == codec_set.sample_len) &&
        (Codec_Set_Saved[codec_set.id].format == codec_set.format) &&
        (Codec_Set_Saved[codec_set.id].slot_num == codec_set.slot_num) &&
        (Codec_Set_Saved[codec_set.id].bclk_polarity == codec_set.bclk_polarity) &&
        (Codec_Set_Saved[codec_set.id].delay == codec_set.delay) &&
        (Codec_Set_Saved[codec_set.id].m_s_sel == codec_set.m_s_sel))
    {
        //APP_TRACE_INFO_T(("Init CODEC[%d]: <Skip> Parameter not changed.", codec_set.id));
        //return 0;
    }
    else
    {
        Codec_Set_Saved[codec_set.id] = codec_set;
    }

    //APP_TRACE_INFO_T(("Init CODEC[%d]: [%5d SR][%d CH][%d-Bit][%s].", codec_set.id, codec_set.sr, codec_set.slot_num, codec_set.sample_len, codec_set.m_s_sel == 0 ? "master" : "slave"));

    Pin_Reset_Codec(codec_set.id);

    err = Check_SR_Support(codec_set.sr);
    if (OS_ERR_NONE != err)
    {
        return err;
    }

    if (codec_set.m_s_sel == 0)
    {
        if_set = 0x0C; //master
    }
    else
    {
        if_set = 0; //slave
    }

    if (codec_set.sample_len == 16)
    {
        if_set += 0x00;
        //    } else if(codec_set.sample_len == 20) //Not yet support 20/24bit on Audio MCU side
        //        if_set += 0x10;
        //    } else if(codec_set.sample_len == 24)
        //        if_set += 0x20;
    }
    else if (codec_set.sample_len == 32)
    {
        if_set += 0x30;
    }
    else
    {
        return 237u;  //CODEC_BIT_LEN_NOT_SUPPORT_ERR;
    }

    if (codec_set.format == 1 || codec_set.format == 2)
    { //PDM or I2S/TDM-I2S
        if_set += 0x00;
    }
    else if (codec_set.format == 3)
    { //PCM DSP
        if_set += 0x40;
        codec_set.bclk_polarity ^= 1; //for PCM : TLV320AIC3204 polarity definition is different from FM1388
    }
    else
    {
        return 238u;    //CODEC_FORMAT_NOT_SUPPORT_ERR;
    }
    //    if (codec_set.format == 1)
    //    {
    //        codec_set.slot_num = 8;
    //    }

    //    if( codec_set.sample_len == 16 ) {
    //        if( codec_set.slot_num == 2 ) {
    //            mode = 0;
    //        } else if( codec_set.slot_num == 8 ) {
    //            mode = 1;
    //        } else {
    //            return CODEC_CH_NUM_NOT_SUPPORT_ERR;
    //        }
    //
    //    } else if( codec_set.sample_len == 32 ) {
    //        if( codec_set.slot_num == 8 ) {
    //            mode = 2;
    //        } else if( codec_set.slot_num == 2 ) {
    //            mode = 3;
    //        } else if( codec_set.slot_num == 4 ) {
    //            mode = 4;
    //        } else {
    //            return CODEC_CH_NUM_NOT_SUPPORT_ERR;
    //        }
    //
    //    } else {
    //        return CODEC_BIT_LEN_NOT_SUPPORT_ERR;
    //
    //    }

    for (i = 0; i<sizeof(config_aic3204)>> 1; i++)
    {
        err = I2CWrite_Codec_AIC3204(pSource, config_aic3204[i][0], config_aic3204[i][1]);
        if (OS_ERR_NONE != err)
        {
            return err;
        }
    }

    err = I2CWrite_Codec_AIC3204(pSource, 0, 0); //switch to Page0
    if (OS_ERR_NONE != err)
    {
        return err;
    }

    err = I2CWrite_Codec_AIC3204(pSource, 27, if_set); //set format
    if (OS_ERR_NONE != err)
    {
        return err;
    }

    err = Set_Codec_PLL(pSource, codec_set.sr, codec_set.sample_len, codec_set.slot_num, codec_set.bclk_polarity);
    if (OS_ERR_NONE != err)
    {
        return err;
    }

    err = codec_set_volume_init(codec_set.id); //init volume with last setting saved
    return err;
}

unsigned char Live_Set_Codec_Master_Slave(const DataSource *pSource, unsigned char m_s_sel)
{
    unsigned char err;
    unsigned char if_set, codec_id;

    codec_id = 0;
    //if (pSource == &source_twi2)
    //{
    //    codec_id = 0; //codec0 for i2s0
    //}
    //else
    //{
    //    codec_id = 1;
    //}

    if (Codec_Set_Saved[codec_id].m_s_sel == 1)
    { //no need set master/slave
        return 0;
    }

    err = I2CRead_Codec_AIC3204(pSource, 27, &if_set);
    if (OS_ERR_NONE != err)
    {
        return err;
    }
    if (m_s_sel == 0)
    {
        if_set |= 0x0C; //master
    }
    else
    {
        if_set &= 0xF0; //slave
    }
    err = I2CWrite_Codec_AIC3204(pSource, 27, if_set); //set format
    if (OS_ERR_NONE != err)
    {
        return err;
    }

    return OS_ERR_NONE;
}
