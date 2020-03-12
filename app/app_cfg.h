/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*                          (c) Copyright 2009-2016; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only.
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
*                                      APPLICATION CONFIGURATION
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : JM
*********************************************************************************************************
*/

#ifndef  APP_CFG_MODULE_PRESENT
#define  APP_CFG_MODULE_PRESENT


/*
*********************************************************************************************************
*                                                INCLUDE
*********************************************************************************************************
*/

#include  <stdio.h>


/*
*********************************************************************************************************
*                                       MODULE ENABLE / DISABLE
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            START TASK
*********************************************************************************************************
*/

#define  APP_CFG_TASK_START_PRIO                        2u
#define  APP_CFG_TASK_START_STK_SIZE                  128u


/*
*********************************************************************************************************
*                                     TRACE / DEBUG CONFIGURATION
*********************************************************************************************************
*/

#define  TRACE_LEVEL_OFF                               0
#define  TRACE_LEVEL_INFO                              1
#define  TRACE_LEVEL_DBG                               2
#define  TRACE_LEVEL_LOG                               3
                                                                            /* Choose the level of debug messages                   */
#define  BSP_CFG_TRACE_LEVEL                       TRACE_LEVEL_OFF
#define  APP_CFG_TRACE_LEVEL                       TRACE_LEVEL_OFF


#define  APP_CFG_TRACE                              printf


#define  APP_TRACE_INFO(x)               ((APP_CFG_TRACE_LEVEL >= TRACE_LEVEL_INFO)  ? (void)(APP_CFG_TRACE x) : (void)0)
#define  APP_TRACE_DBG(x)                ((APP_CFG_TRACE_LEVEL >= TRACE_LEVEL_DBG)   ? (void)(APP_CFG_TRACE x) : (void)0)

#endif
