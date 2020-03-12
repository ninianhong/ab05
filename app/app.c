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
*                                          APPLICATION CODE
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : JM
*********************************************************************************************************
* Note(s)       : none.
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include  <app_cfg.h>
#include  <lib_mem.h>
#include  <bsp_os.h>
#include  <os.h>

#include  "board.h"
#include  "chip.h"
#include  "trace.h"
#include  "compiler.h"
#include  "timer.h"

#include  "peripherals/aic.h"
#include  "peripherals/pio.h"
#include  "peripherals/pmc.h"
#include  "peripherals/tc.h"
#include  "peripherals/twid.h"
#include  "peripherals/wdt.h"

#include  "misc/led.h"


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static  OS_TCB   AppTaskStartTCB;
static  CPU_STK  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart    (void  *p_arg);
static  void  AppTaskCreate   (void);
static  void  AppEventCreate  (void);


/*
*********************************************************************************************************
*                                               console_handler()
*
* Description : Entry point for C code.
*
* Arguments   : none.
*
* Returns     : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/
#define CMD_BUFFER_SIZE   (1024)
#define READ_BUFFER_SIZE  (4 * 1024)
static volatile bool cmd_complete = false;
static volatile uint32_t cmd_length = 0;
static uint8_t cmd_buffer[CMD_BUFFER_SIZE];
static void console_handler(uint8_t key)
{
	/* already processing a command: ignore input */
	if (cmd_complete)
	       return;

	switch (key) {
	case '\r':
	case '\n':
		console_echo(key);
		cmd_buffer[cmd_length] = '\0';
		cmd_complete = true;
		break;
	case 0x7F:
	case '\b':
		if (cmd_length > 0) {
			console_echo(key);
			cmd_length--;
			cmd_buffer[cmd_length] = '\0';
		}
		break;
	default:
		if (cmd_length < (ARRAY_SIZE(cmd_buffer) - 1)) {
			console_echo(key);
			cmd_buffer[cmd_length] = key;
			cmd_length++;
		}
		break;
	}

}

/*
*********************************************************************************************************
*                                               main()
*
* Description : Entry point for C code.
*
* Arguments   : none.
*
* Returns     : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

int main()
{
    OS_ERR  os_err;


    wdt_disable();                                              /* Disable watchdog                                     */

    board_cfg_pmic();

    CPU_Init();

    Mem_Init();                                                 /* Initialize memory module                             */

    
    console_set_rx_handler(console_handler);
    console_enable_rx_interrupt();
    
    printf("<?xml version=\"1.0\" encoding=\"UTF-8\">\r\n\r\n");
    //	printf("<testsuite>\r\n");
    //	printf("  <testcase classname=\"pio.setup\" name=\"Setup\">\r\n");
    //	printf("    <system-out>\r\n");
    
    
    OSInit(&os_err);                                            /* Initialize uC/OS-III                                 */

    if(os_err != OS_ERR_NONE) {
        APP_TRACE_INFO(("Error initialising OS. OsInit() returned with error %u\r\n", os_err));
        while (1);
    }

    OSTaskCreate(&AppTaskStartTCB,                              /* Create the start task                                */
                 "App Task Start",
                  AppTaskStart,
                  DEF_NULL,
                  APP_CFG_TASK_START_PRIO,
                 &AppTaskStartStk[0],
                  APP_CFG_TASK_START_STK_SIZE / 10u,
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    if(os_err != OS_ERR_NONE) {
        APP_TRACE_INFO(("Error creating task. OSTaskCreate() returned with error %u\r\n", os_err));
        while (1);
    }

    OSStart(&os_err);                                           /* Start multitasking                                   */
    if(os_err != OS_ERR_NONE) {
        APP_TRACE_INFO(("Error starting. OSStart() returned with error %u\r\n", os_err));
        while (1);
    }

    return 0;
}


/*
*********************************************************************************************************
*                                          AppTaskStart()
*
* Description : The startup task.
*
* Argument(s) : p_arg       Argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Note(s)     : (1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                   used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskStart (void  *p_arg)
{
    OS_ERR  os_err;
    int k = 0;

    (void)&p_arg;


    BSP_OSTickInit();                                           /* Initialize the tick interrupt                        */

                                                                /* Initialize LEDs                                      */
    led_configure(0);
    led_configure(1);
    led_configure(2);

    APP_TRACE_INFO(("Creating Application Events...\n\r"));
    AppEventCreate();                                           /* Create Application Objects                           */

    APP_TRACE_INFO(("Creating Application Tasks...\n\r"));
    AppTaskCreate();                                            /* Create application tasks                             */

    //pio_set(&pio_output);
    //msleep(10);
    //pio_clear(&pio_output);
    while (DEF_TRUE) {                                          /* Task body, always as an infinite loop                */
        /*
        led_toggle(0);
        APP_TRACE_DBG(("Task is aliving %u\r\n"));
        OSTimeDlyHMSM( 0u, 0u, 0u, 250u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
        led_toggle(1);
        OSTimeDlyHMSM( 0u, 0u, 0u, 250u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
        led_toggle(2);
        OSTimeDlyHMSM( 0u, 0u, 0u, 250u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
       */
#define   POLLING_DELAY 0    
#if POLLING_DELAY      
       for( int k=1000000; k>=0;k--);
#else
       OSTimeDlyHMSM( 0u, 0u, 0u, 10000u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
#endif
       
       led_set(0);
       
#if POLLING_DELAY
       for( k=0; k<=1000000;k++);
#else
       OSTimeDlyHMSM( 0u, 0u, 0u, 250u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
#endif
       
       led_clear(0);
       
#if POLLING_DELAY
       k=0;
       for( k=0; k<=1000000;k++);
#else
       OSTimeDlyHMSM( 0u, 0u, 0u, 250u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
#endif       
    }
}


/*
*********************************************************************************************************
*                                      AppObjCreate()
*
* Description : Create the application Objects
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : AppTasStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppEventCreate (void)
{

}


/*
*********************************************************************************************************
*                                      AppTaskCreate()
*
* Description : Create the application tasks.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : AppTasStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskCreate (void)
{

}
