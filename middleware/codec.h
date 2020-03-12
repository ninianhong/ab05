/*
*********************************************************************************************************
*
*                                          APP PACKAGE
*
*                                         Atmel  AT91SAMA5D3
*                                             on the
*                                      Audio Bridge 04 Board (AB04 V1.0) 2.0
*
* Filename      : uif_i2s.c
* Version       : V0.0.1
* Programmer(s) : PQ
* Modifer       : Leo
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#ifndef __CODEC_H__
#define __CODEC_H__

#include "uif_object.h"
#include <stdbool.h>
#include "chip_pins.h"

#include "board.h"

#define PCA9546A_ADDRESS  0xE0
#define PCA9540B_ADDRESS  0xE0
#define PCA9548A_ADDRESS  0xEE

#define CODEC_ADDRESS     0x30
#define AD1938_ADDRESS    0x04
#define AD1937_ADDRESS    0x08

#define I2C_SWITCH_FM36     0
#define I2C_SWITCH_CODEC0   1
#define I2C_SWITCH_CODEC1   2

/****************************************/
#define   ALC5610      10
#define   ALC5620      20
#define   PLL_Control0 0x0
#define   PLL_Control1 0x1
#define   DAC_Control0 0x2
#define   DAC_Control1 0x3
#define   DAC_Control2 0x4
#define   DAC_Mute     0x5
#define   DAC1L_Volume 0x6
#define   DAC1R_Volume 0x7
#define   DAC2L_Volume 0x8
#define   DAC2R_Volume 0x9
#define   DAC3L_Volume 0xa
#define   DAC3R_Volume 0xb
#define   DAC4L_Volume 0xc
#define   DAC4R_Volume 0xd
#define   ADC_Control0 0xE
#define   ADC_Control1 0xF
#define   ADC_Control2 0x10
#define   I2S_MODE     1
#define   TDM_MODE     2
#define   TDM16_MODE   3

/* ================== CODEC definition ============================ */
//#define BOARD_CODEC0_RESET_PINS BOARD_CODEC0_RESET_IOS0 
//#define BOARD_CODEC1_RESET_PINS BOARD_CODEC1_RESET_IOS0 
//#define BOARD_CODEC2_RESET_PINS BOARD_CODEC2_RESET_IOS0 
//#define BOARD_CODEC3_RESET_PINS BOARD_CODEC3_RESET_IOS0 

#define   MICPGA_GAIN_ENABLE_MASK   DEF_BIT_MASK_08(1u,7)
#define   MICPGA_GAIN_ENABLE_ON(val)   DEF_BIT_CLR_08(val,MICPGA_GAIN_ENABLE_MASK)
#define   MICPGA_GAIN_ENABLE_OFF(val)   DEF_BIT_SET_08(val,MICPGA_GAIN_ENABLE_MASK)

#define   HP_LOUT_DRV_GAIN_SET_RESERVED_MASK   DEF_BIT_MASK_08(1u,7)
#define   HP_LOUT_DRV_GAIN_SET_RESERVED_SET_0(val)   DEF_BIT_CLR_08(val,HP_LOUT_DRV_GAIN_SET_RESERVED_MASK)

#define   HP_LOUT_DRV_GAIN_SET_MUTE_MASK    DEF_BIT_MASK_08(1u,6)
#define   HP_LOUT_DRV_GAIN_SET_MUTE_ON(val)    DEF_BIT_SET_08(val,HP_LOUT_DRV_GAIN_SET_MUTE_MASK)
#define   HP_LOUT_DRV_GAIN_SET_MUTE_OFF(val)    DEF_BIT_CLR_08(val,HP_LOUT_DRV_GAIN_SET_MUTE_MASK)

#define   hp_x10_MIN    (-695)
#define   hp_x10_MAX    530
#define   lout_x10_MIN    (-695)
#define   lout_x10_MAX    530
#define   lin_MIN    (-120)
#define   lin_MAX    675

#define   DAC_l_gain_reg_P0    65
#define   DAC_r_gain_reg_P0    66

#define   ADC_fine_gain_reg_P0    82
#define   ADC_l_gain_reg_P0    83
#define   ADC_r_gain_reg_P0    84

#define   P1_R12    12 //Page 1 / Register 12: HPL Routing Selection Register - 0x01 / 0x0C (P1_R12)
#define   P1_R13    13 //Page 1 / Register 13: HPR Routing Selection Register - 0x01 / 0x0D (P1_R13)

#define   P1_R14    14 //Page 1 / Register 14: LOL Routing Selection Register - 0x01 / 0x0E (P1_R14)
#define   P1_R15    15 //Page 1 / Register 15: LOR Routing Selection Register - 0x01 / 0x0F (P1_R15)

#define   HPL_gain_reg_P1    16
#define   HPR_gain_reg_P1    17
#define   LOL_gain_reg_P1    18
#define   LOR_gain_reg_P1    19
#define   P1_R52    52 //Page 1 / Register 52: Left MICPGA Positive Terminal Input Routing Configuration
#define   P1_R55    55 //Page 1 / Register 55: Right MICPGA Positive Terminal Input Routing Configuration
#define   LIL_gain_reg_P1    59
#define   LIR_gain_reg_P1    60

typedef struct twi_parameter
{
  uint32_t address;
  uint32_t iaddress;
  uint8_t  isize;
  uint8_t  revers;
}TWI_CFG;

struct _pca9546 
{
  uint8_t bus;
  uint8_t addr;
  TWI_CFG cfg;
};
static struct _pca9546 pca9546 = {
	.bus = BOARD_PCA9546_TWI_BUS,
	.addr = BOARD_PCA9546_TWI_ADDR>>1,
};


typedef struct {
    uint32_t sr; // 8000 ~ 96000
    uint8_t  sample_len ; //16 or 32 only
    uint8_t  format; // 0 : i2s, 1 : pcm
    uint8_t  slot_num ; //2, 4, 8
    uint8_t  m_s_sel; //0 : master, 1 : slave;
    uint8_t  flag;  // flag if received audio_cfg command
    uint8_t  bclk_polarity; //1: frame start rising edge match bclk,  0: frame start rising edge inverted bclk
    uint8_t  delay;
    uint8_t  id;//CODEC ID : 0 or 1
    uint8_t  reserved[2];
}CODEC_SETS ;

/****************************************/
extern CODEC_SETS Codec_Set_Saved[];   //for 2 CODEC

static void PA_set_gain0( bool val );
static void PA_set_gain1( bool val );
static void PA_shutdown( bool en );

void init_codec_rst_pin();

uint8_t Set_Codec(const DataSource *pSource,uint8_t codec_control_type, uint8_t size_para, uint8_t *pdata);
uint8_t Get_Codec(const DataSource *pSource,uint8_t codec_control_type, uint8_t reg, uint8_t *pdata);

uint8_t I2C_Switcher( uint8_t i2c_channel );

uint8_t Init_CODEC( CODEC_SETS codec_set ) ;

uint8_t Set_AIC3204_DSP_Offset( uint8_t slot_index ) ;
uint8_t Init_CODEC_AIC3204( uint32_t sample_rate ) ;

uint8_t CODEC_LOUT_Small_Gain_En( bool small_gain );
uint8_t CODEC_Set_Volume( const DataSource *pSource, int vol_spk_tmp, int vol_lout_tmp, int vol_lin_tmp );
uint8_t Check_SR_Support( uint32_t sample_rate );
void Pin_Reset_Codec( unsigned char id );

void PA_set_gain( int32_t pa_x10 );

uint8_t Live_Set_Codec_Master_Slave( const DataSource *pSource, unsigned char m_s_sel);
uint8_t codec_set_gains( const DataSource *pSource, int32_t hp_x10, int32_t lout_x10, int32_t lin_x10 );
uint8_t codec1_set_volume( const DataSource *pSource, int32_t hp_x10, int32_t lout_x10, int32_t lin_x10, uint8_t force_set  );
uint8_t codec2_set_volume( const DataSource *pSource, int32_t hp_x10, int32_t lout_x10, int32_t lin_x10, uint8_t force_set  );
uint8_t codec_set_volume_init(  unsigned char codec_id );
uint8_t codec_IN1_to_MICPGA_route( const DataSource *pSource, bool status);
uint8_t codec_DAC_to_HP_route( const DataSource *pSource, bool status);
uint8_t codec_DAC_to_LO_route( const DataSource *pSource, bool status);

#endif
