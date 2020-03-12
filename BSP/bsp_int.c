
/*
*********************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*
*                          (c) Copyright 2003-2016; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               This BSP is provided in source form to registered licensees ONLY.  It is
*               illegal to distribute this source code to any third party unless you receive
*               written permission by an authorized Micrium representative.  Knowledge of
*               the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*                             ATMEL ATSAMA5 ADVANCED INTERRUPT CONTROLLER
*
* Filename      : bsp_int.c
* Version       : V1.00
* Programmer(s) : JBL
*
* Note(s) : This is a minimal example of a board support package for the Atmel advanced interrupt
*           controller. Most advanced functions aren't supported.
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  <cpu.h>
#include  <lib_def.h>
#include  <bsp_int.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  BSP_SAMA5_AIC_REG         ((SAMA5_REG_AIC_PTR)(0xFC020000u))


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
                                                                /* ---------- ADVANCED INTERRUPT CONTROLLER ----------- */
typedef  struct  sama5_reg_aic {
    CPU_REG32  AIC_SSR;                                         /* Source Select Register.                              */
    CPU_REG32  AIC_SMR;                                         /* Source Mode Register.                                */
    CPU_REG32  AIC_SVR;                                         /* Source Vector Register.                              */
    CPU_REG32  RESERVED1[1];
    CPU_REG32  AIC_IVR;                                         /* Interrupt Vector Register.                           */
    CPU_REG32  AIC_FVR;                                         /* FIQ Interrupt Vector Register.                       */
    CPU_REG32  AIC_ISR;                                         /* Interrupt Status Register.                           */
    CPU_REG32  RESERVED2[1];
    CPU_REG32  AIC_IPR[4];                                      /* Interrupt Pending Register 0-3.                      */
    CPU_REG32  AIC_IMR;                                         /* Interrupt Mask Register.                             */
    CPU_REG32  AIC_CISR;                                        /* Core Interrupt Status Register.                      */
    CPU_REG32  AIC_EOICR;                                       /* End of Interrupt Command Register.                   */
    CPU_REG32  AIC_SPU;                                         /* Spurious Interrupt Vector Register.                  */
    CPU_REG32  AIC_IECR;                                        /* Interrupt Enable Command Register.                   */
    CPU_REG32  AIC_IDCR;                                        /* Interrupt Disable Command Register.                  */
    CPU_REG32  AIC_ICCR;                                        /* Interrupt Clear Command Register.                    */
    CPU_REG32  AIC_ISCR;                                        /* Interrupt Set Command Register.                      */
    CPU_REG32  RESERVED3[7];
    CPU_REG32  AIC_DCR;                                         /* Debug Control Register.                              */
    CPU_REG32  RESERVED4[29];
    CPU_REG32  AIC_WPMR;                                        /* Write Protect Mode Register.                         */
    CPU_REG32  AIC_WPSR;                                        /* Write Protect Status Register.                       */
} SAMA5_REG_AIC, *SAMA5_REG_AIC_PTR;


/*
*********************************************************************************************************
*********************************************************************************************************
**                                         GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          BSP_IntVectSet()
*
* Description : Configure an interrupt vector.
*
* Argument(s) : int_id              Interrupt ID.
*
*               int_prio            Interrupt priority.
*
*               int_fnct            ISR function pointer.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*
*********************************************************************************************************
*/

CPU_BOOLEAN  BSP_IntVectSet (CPU_INT32U        int_id,
                             CPU_INT32U        int_prio,
                             BSP_INT_FNCT_PTR  int_fnct)
{
    CPU_SR_ALLOC();


    CPU_CRITICAL_ENTER();
    BSP_SAMA5_AIC_REG->AIC_SSR = int_id;

    BSP_SAMA5_AIC_REG->AIC_IDCR = DEF_BIT_00;

    BSP_SAMA5_AIC_REG->AIC_SMR = int_prio & 0x7u;

    BSP_SAMA5_AIC_REG->AIC_SVR = (CPU_INT32U)int_fnct;
    CPU_CRITICAL_EXIT();

    return (DEF_YES);
}


/*
*********************************************************************************************************
*                                           BSP_IntSrcEn()
*
* Description : Enable an interrupt source.
*
* Argument(s) : int_id              Interrupt ID.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*
*********************************************************************************************************
*/

void  BSP_IntSrcEn  (CPU_INT32U  int_id)
{
    CPU_SR_ALLOC();


    CPU_CRITICAL_ENTER();
    BSP_SAMA5_AIC_REG->AIC_SSR = int_id;
    BSP_SAMA5_AIC_REG->AIC_IECR = DEF_BIT_00;
    CPU_CRITICAL_EXIT();

}


/*
*********************************************************************************************************
*                                           BSP_IntSrcDis()
*
* Description : Disable an interrupt source.
*
* Argument(s) : int_id              Interrupt ID.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*
*********************************************************************************************************
*/

void  BSP_IntSrcDis  (CPU_INT32U  int_id)
{
    CPU_SR_ALLOC();


    CPU_CRITICAL_ENTER();
    BSP_SAMA5_AIC_REG->AIC_SSR = int_id;
    BSP_SAMA5_AIC_REG->AIC_IDCR = DEF_BIT_00;
    CPU_CRITICAL_EXIT();

}


/*
*********************************************************************************************************
*                                           BSP_IntHandler()
*
* Description : Global interrupt handler.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : none.
*
* Note(s)     : none.
*
*********************************************************************************************************
*/

void  BSP_IntHandler (void)
{
    BSP_INT_FNCT_PTR int_fnct;


    int_fnct = (BSP_INT_FNCT_PTR)BSP_SAMA5_AIC_REG->AIC_IVR;

    if(int_fnct != DEF_NULL) {
        int_fnct();
    }

    CPU_MB();
    BSP_SAMA5_AIC_REG->AIC_EOICR = DEF_BIT_00;
}
