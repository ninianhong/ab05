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
* Filename      : bsp_int.h
* Version       : V1.00
* Programmer(s) : JBL
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               MODULE
*********************************************************************************************************
*/

#ifndef  BSP_INT_PRESENT
#define  BSP_INT_PRESENT


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

typedef  void  (*BSP_INT_FNCT_PTR)(void);


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void         BSP_IntSrcEn    (CPU_INT32U        int_id);

void         BSP_IntSrcDis   (CPU_INT32U        int_id);

CPU_BOOLEAN  BSP_IntVectSet  (CPU_INT32U        int_id,
                              CPU_INT32U        int_prio,
                              BSP_INT_FNCT_PTR  int_fnct);

void         BSP_IntHandler  (void);

#endif
