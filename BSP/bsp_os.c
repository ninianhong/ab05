/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can find our product's user manual, API reference, release notes and
*               more information at https://doc.micrium.com.
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*                                            ATMEL ATSAMA5
*
* Filename      : bsp_os.c
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDES
*********************************************************************************************************
*/

#include  <os.h>
#include  <bsp_os.h>
#include  <bsp_int.h>
#include  <peripherals/pmc.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  BSP_SAMA5_PIT_REG         ((SAMA5_REG_PIT_PTR)(0xF8048030u))


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/

                                                                /* ---------- ADVANCED INTERRUPT CONTROLLER ----------- */
typedef  struct  sama5_reg_pit {
    CPU_REG32  PIT_MR;                                          /* Mode Register.                                       */
    CPU_REG32  PIT_SR;                                          /* Status Register.                                     */
    CPU_REG32  PIT_PIVR;                                        /* Periodic Interval Value Register.                    */
    CPU_REG32  PIT_PIIR;                                        /* Periodic Interval Image Register.                    */
} SAMA5_REG_PIT, *SAMA5_REG_PIT_PTR;


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  BSP_OSTickISRHandler (void);


/*
*********************************************************************************************************
*                                    INITIALIZE OS TICK INTERRUPT
*
* Description : Initialize the tick interrupt.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none
*********************************************************************************************************
*/

void  BSP_OSTickInit (void)
{
    CPU_INT32U  val;
    CPU_INT32U  per_freq;


    BSP_IntVectSet ( 3u,
                     0u,
                    &BSP_OSTickISRHandler);

    BSP_IntSrcEn(3u);

    CPU_MB();

    per_freq = pmc_get_peripheral_clock(ID_PIT) / 16u;

#if (OS_VERSION >= 30000u)
    val = (per_freq / OSCfg_TickRate_Hz) + 1u;
#else
    val = (per_freq / OS_TICKS_PER_SEC) + 1u;
#endif

    BSP_SAMA5_PIT_REG->PIT_MR = DEF_BIT_24 | DEF_BIT_25 | val;
}


/*
*********************************************************************************************************
*                                       BSP_OSTickISRHandler()
*
* Description : Interrupt handler for the tick timer
*
* Argument(s) : cpu_id     Source core id
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  BSP_OSTickISRHandler (void)
{
    volatile  CPU_INT32U  val;


    val = BSP_SAMA5_PIT_REG->PIT_PIVR;
    OSTimeTick();
}


/*
 *********************************************************************************************************
 *                                          OS_CPU_ExceptHndlr()
 *
 * Description : Handle any exceptions.
 *
 * Argument(s) : except_id     ARM exception type:
 *
 *                                  OS_CPU_ARM_EXCEPT_RESET             0x00
 *                                  OS_CPU_ARM_EXCEPT_UNDEF_INSTR       0x01
 *                                  OS_CPU_ARM_EXCEPT_SWI               0x02
 *                                  OS_CPU_ARM_EXCEPT_PREFETCH_ABORT    0x03
 *                                  OS_CPU_ARM_EXCEPT_DATA_ABORT        0x04
 *                                  OS_CPU_ARM_EXCEPT_ADDR_ABORT        0x05
 *                                  OS_CPU_ARM_EXCEPT_IRQ               0x06
 *                                  OS_CPU_ARM_EXCEPT_FIQ               0x07
 *
 * Return(s)   : none.
 *
 * Caller(s)   : OS_CPU_ARM_EXCEPT_HANDLER(), which is declared in os_cpu_a.s.
 *********************************************************************************************************
 */

void  OS_CPU_ExceptHndlr  (CPU_INT32U  except_id)
{
    switch (except_id) {
        case OS_CPU_ARM_EXCEPT_FIQ:
             BSP_IntHandler();
             break;


        case OS_CPU_ARM_EXCEPT_IRQ:
             BSP_IntHandler();
             break;


        case OS_CPU_ARM_EXCEPT_RESET:
            /* $$$$ Insert code to handle a Reset exception               */

        case OS_CPU_ARM_EXCEPT_UNDEF_INSTR:
            /* $$$$ Insert code to handle a Undefine Instruction exception */

        case OS_CPU_ARM_EXCEPT_SWI:
            /* $$$$ Insert code to handle a Software exception             */

        case OS_CPU_ARM_EXCEPT_PREFETCH_ABORT:
            /* $$$$ Insert code to handle a Prefetch Abort exception       */

        case OS_CPU_ARM_EXCEPT_DATA_ABORT:
            /* $$$$ Insert code to handle a Data Abort exception           */

        case OS_CPU_ARM_EXCEPT_ADDR_ABORT:
            /* $$$$ Insert code to handle a Address Abort exception        */
        default:
             while (DEF_TRUE) { /* Infinite loop on other exceptions. (see note #1)          */
                //CPU_WaitForEvent();
             }
    }
}
