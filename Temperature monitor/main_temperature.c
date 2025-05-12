/*
 * FreeRTOS V202212.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/******************************************************************************
 * NOTE: Windows will not be running the FreeRTOS demo threads continuously, so
 * do not expect to get real time behaviour from the FreeRTOS Windows port, or
 * this demo application.  Also, the timing information in the FreeRTOS+Trace
 * logs have no meaningful units.  See the documentation page for the Windows
 * port for further information:
 * https://www.FreeRTOS.org/FreeRTOS-Windows-Simulator-Emulator-for-Visual-Studio-and-Eclipse-MingW.html
 *
 * NOTE 2:  This project provides two demo applications.  A simple blinky style
 * project, and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting in main.c is used to select
 * between the two.  See the notes on using mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
 * in main.c.  This file implements the simply blinky version.  Console output
 * is used in place of the normal LED toggling.
 *
 * NOTE 3:  This file only contains the source code that is specific to the
 * basic demo.  Generic functions, such FreeRTOS hook functions, are defined
 * in main.c.
 ******************************************************************************
 *
 * main_blinky() creates one queue, one software timer, and two tasks.  It then
 * starts the scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  It uses vTaskDelayUntil() to create a periodic task that sends
 * the value 100 to the queue every 200 milliseconds (please read the notes
 * above regarding the accuracy of timing under Windows).
 *
 * The Queue Send Software Timer:
 * The timer is a one-shot timer that is reset by a key press.  The timer's
 * period is set to two seconds - if the timer expires then its callback
 * function writes the value 200 to the queue.  The callback function is
 * implemented by prvQueueSendTimerCallback() within this file.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() waits for data to arrive on the queue.
 * When data is received, the task checks the value of the data, then outputs a
 * message to indicate if the data came from the queue send task or the queue
 * send software timer.
 *
 * Expected Behaviour:
 * - The queue send task writes to the queue every 200ms, so every 200ms the
 *   queue receive task will output a message indicating that data was received
 *   on the queue from the queue send task.
 * - The queue send software timer has a period of two seconds, and is reset
 *   each time a key is pressed.  So if two seconds expire without a key being
 *   pressed then the queue receive task will output a message indicating that
 *   data was received on the queue from the queue send software timer.
 *
 * NOTE:  Console input and output relies on Windows system calls, which can
 * interfere with the execution of the FreeRTOS Windows port.  This demo only
 * uses Windows system call occasionally.  Heavier use of Windows system calls
 * can crash the port.
 */

/* Standard includes. */
#include <stdio.h>
#include <conio.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY    ( tskIDLE_PRIORITY + 3 )
#define mainQUEUE_TEMP_TASK_PRIORITY       ( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_STATUS_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The times are converted from
 * milliseconds to ticks using the pdMS_TO_TICKS() macro. */
#define TASK_LED_STATUS_MS   pdMS_TO_TICKS( 250UL ) //2hz frequency
#define TEMP_CHECK_MS        pdMS_TO_TICKS( 1000UL )

//NEWCODE ====================================================================================
 /* Example register addresses (replace with real ones) */
uint32_t I2C_CONTROL_REG = 0;
uint32_t I2C_DATA_REG = 0;
uint32_t I2C_STATUS_REG = 0;

/* Helper macros for the bits in the I2C control/status registers (placeholders). */
#define I2C_CONTROL_EXECUTE           (1U << 0)  /* Bit 0 */
#define I2C_CONTROL_CLEAR_WR_FIFO     (1U << 1)  /* Bit 1 */
#define I2C_CONTROL_DATA_LOAD         (1U << 2)  /* Bit 2 */
#define I2C_CONTROL_READ_MODE           (1U << 31) /* Bit 31: 0=Write, 1=Read */

#define I2C_STATUS_READY              (1U << 0)  /* Bit 0 = 1 => Ready */
#define I2C_STATUS_NACK               (1U << 1)  /* Bit 1 = 1 => NACK */
#define I2C_STATUS_WRITE_FIFO_OVERFLOW (1U << 2) 
#define I2C_STATUS_READ_FIFO_OVERFLOW (1U << 3) 

/* 7-bit I2C addresses shifted into bits [25:19]. */
#define PCAL6408_ADDRESS   (0x20) // 0x20 as addr is tied to GND, typical when address pin is grounded 
#define TEMP_ADDRESS       (0x48) // 0x48 as tmp101 documentation 

#define STATUS_LED_PIN_MASK   0x01  /* Example: PCAL6408 bit 0 for the LED. */
                                   /* Adjust if the LED is on a different pin. */
#define LED_STATUS         0x01   // Identifier for status LED (e.g. PCAL6408 P0)
#define LED_TEMP_ALERT     0x02   // Identifier for temperature alert LED (e.g. PCAL6408 P5)

#define TEMP_LOW_THRESHOLD  19.0f
#define TEMP_HIGH_THRESHOLD 21.0f

//====================================================================================

/* The number of items the queue can hold at once. */
#define mainQUEUE_LENGTH                   ( 2 )

/* The values sent to the queue receive task from the queue send task and the
 * queue send software timer respectively. */
#define VALUE_SENT_FROM_STATUS_TASK           ( 100UL )
#define VALUE_SENT_FROM_TMP_TASK              ( 200UL )

uint8_t pcal_led_state = 0x21; //as it is npn transistor setting outport to high makes the led go ON 
                                //so by setting bits 0 and 5 to 1 Initially while the rest are 0, means the led are OFF
typedef enum {
    LED_OFF = 0,
    LED_ON = 1
} LedState;

LedState Status = 0;
LedState Temp = 0;

static float i2cReadTemperatureSensor(void)
{
    static float temp = 20.0f;
    temp += 0.5f;
    if (temp > 22.0f)
    {
        temp = 18.0f;
    }
    return temp;
}

LedState i2cTMP101(void)
{
    //Wait until controller is ready (Bit0 in status =1)
    /*Control Register info
     *    - Bits [25:19] = 7-bit device address
     *    - Bit 31 = 0 => Write/read
     *    - Bits [15:10] = amount to send 
     */
    
    //STEP 1 first, complete a write to indicate we want to access temperature register 
    //while loop in order to redo in case of nack
    int maxnack = 10;
    while (maxnack > 0) {
        
        //1.1 set initial values for control reg
        uint32_t ctrlVal = 0;
        // Bit31 = 0 => write, so do NOT set that bit. 
        ctrlVal |= ((uint32_t)TEMP_ADDRESS << 19); 
        ctrlVal |= (1 << 10);  /* Data size = 1 byte. */
    
        I2C_CONTROL_REG = ctrlVal; 

        //1.2 set data register values 
        I2C_DATA_REG = 0x00; //zero as we want to pointer register value to indicate temperature register 

        //1.3 load data from register into write FIFO
        I2C_CONTROL_REG |= I2C_CONTROL_DATA_LOAD;

        //simulate signal of loaded buffer 
        I2C_STATUS_REG |= I2C_STATUS_READY;

        //1.5 Wait for values to loaded
        while ((I2C_STATUS_REG & I2C_STATUS_READY) == 0);

        //if we tired to write to FIFO when already full 
        if (I2C_STATUS_REG & I2C_STATUS_WRITE_FIFO_OVERFLOW) {
            I2C_STATUS_REG &= ~I2C_STATUS_WRITE_FIFO_OVERFLOW;
            I2C_CONTROL_REG |= I2C_CONTROL_CLEAR_WR_FIFO; //clear buffer 
            continue;
        }
    
        //Assume load bit is reset automatically

        //1.7 execute 
        I2C_CONTROL_REG |= I2C_CONTROL_EXECUTE;

        //simulate signal of completed write 
        I2C_STATUS_REG |= I2C_STATUS_READY;

        //1.8 wait for ack
        while ((I2C_STATUS_REG & I2C_STATUS_READY) == 0);
        
        //1.9 check to see if it was a nack
        if (I2C_STATUS_REG & I2C_STATUS_NACK) {
            I2C_CONTROL_REG = 0x00000000; // restart the read again
            I2C_CONTROL_REG |= I2C_CONTROL_CLEAR_WR_FIFO; //clear buffer 
            I2C_STATUS_REG &= ~I2C_STATUS_WRITE_FIFO_OVERFLOW; //status of buffer
            maxnack -= 1;
            continue;
         }
        else {
            I2C_CONTROL_REG = 0x00000000;
            break;
        }
    }

    //STEP 2 Send a read to get temperature data (weve already checked to ensure controller is ready for operation
    uint16_t data;
    maxnack = 10;
    while (maxnack > 0) {
        //2.1 set initial values for regs
        I2C_DATA_REG = 0x00000000;

        uint32_t ctrlVal = 0;
        ctrlVal |= I2C_CONTROL_READ_MODE;  // as we are indidcating a read 
        ctrlVal |= ((uint32_t)TEMP_ADDRESS << 19);
        ctrlVal |= (0x8 << 10);  // as we want to read 8 bits from the 
        ctrlVal |= I2C_CONTROL_EXECUTE; //execute bit 
        
        //2.2 load in values in
        I2C_CONTROL_REG = ctrlVal;

        //simulate signal 
        I2C_STATUS_REG |= I2C_STATUS_READY; // Controller is now ready, and there is no nack 

        //2.4 wait for ack
        while ((I2C_STATUS_REG & I2C_STATUS_READY) == 0);

        //2.5 check to see if it was a nack
        if (I2C_STATUS_REG & I2C_STATUS_NACK) {
            I2C_CONTROL_REG = 0x00000000; // restart the read again
            maxnack -= 1;
            continue;
        }

        //assume execute bit automatically resets 
        
        //simulate data coming into I2C
        float simulate_data = i2cReadTemperatureSensor();
        int16_t rawTemp = (int16_t)(simulate_data / 0.0625f);
        uint16_t tmp101Format = ((uint16_t)rawTemp) << 4;
        I2C_DATA_REG = (tmp101Format >> 8);

        //2.6 read first byte from FIFO
        data = I2C_DATA_REG << 4; //shift the first bits into position so the final bit is at bit 11 (12 total)

        //2.7 get in next byte of data 
        I2C_CONTROL_REG |= I2C_CONTROL_READ_MODE;
        I2C_CONTROL_REG |= I2C_CONTROL_EXECUTE;

        //simulate done 
        I2C_STATUS_REG |= I2C_STATUS_READY;

        //2.8 wait for buffer to be ready
        while ((I2C_STATUS_REG & I2C_STATUS_READY) == 0);

        //simulate data
        I2C_DATA_REG = (tmp101Format & 0xFF);
        //collect the rest of the data in the variable 
        data = data | I2C_DATA_REG >> 4;
	break;	

    }
    int16_t temperature;
    if (data & 0x800) { //if a negative value, will be sign extended, ensure to set to correct decimal  
        temperature = (int16_t) (data |= 0xF000); // fill top bits to preserve 16-bit value
    }
    else {
        temperature = (int16_t)data;
    }

    float tempC = temperature * 0.0625f;

    LedState local;
    if (tempC >= TEMP_LOW_THRESHOLD && tempC <= TEMP_HIGH_THRESHOLD) {
        local = LED_OFF;
    }
    else {
        local = LED_ON;
    }

    return local;
}

static void i2cWritePcal(LedState temperature_led, LedState status_led) {

    int maxnack = 10;
    while (maxnack > 0) {

        //1.1 set initial values for control reg
        uint32_t ctrlVal = 0;
        // Bit31 = 0 => write, so do NOT set that bit. 
        ctrlVal |= ((uint32_t)PCAL6408_ADDRESS << 19);
        ctrlVal |= (8 << 10);  /* Data size = 1 byte. */

        I2C_CONTROL_REG = ctrlVal;


        if (status_led) { //status led will be bit 0 schema: 0010 0001
            pcal_led_state &= 0xFE; //turn led on by setting bit 0 to 0
        }
        else {
            pcal_led_state |= 0x01; //turn led off by setting bit 0 to 1
        }

        if (temperature_led) { //temperature led will be bit 5
            pcal_led_state &= 0xDF; //turn led on by setting bit 5 to 0
        }
        else {
            pcal_led_state |= 0x20; //turn led off by setting bit 5 to 1
        }

        //1.2 set data register values 
        I2C_DATA_REG = pcal_led_state; //zero as we want to read from the temperature register

        //1.3 load data from register into write FIFO
        I2C_CONTROL_REG |= I2C_CONTROL_DATA_LOAD;

        //simulate ack
        I2C_STATUS_REG |= I2C_STATUS_READY;

        //1.5 Wait for values to loaded
        while ((I2C_STATUS_REG & I2C_STATUS_READY) == 0);

        //if we tired to write to FIFO when already full 
        if (I2C_STATUS_REG & I2C_STATUS_WRITE_FIFO_OVERFLOW) {
            I2C_STATUS_REG &= ~I2C_STATUS_WRITE_FIFO_OVERFLOW;
            I2C_CONTROL_REG |= I2C_CONTROL_CLEAR_WR_FIFO; //clear buffer 
            continue;
        }

        //Assume load bit is reset automatically

        //1.7 execute 
        I2C_CONTROL_REG |= I2C_CONTROL_EXECUTE;

        //simulate 
        I2C_STATUS_REG |= I2C_STATUS_READY;

        //1.8 wait for ack
        while ((I2C_STATUS_REG & I2C_STATUS_READY) == 0 );

        //1.9 check to see if it was a nack
        if (I2C_STATUS_REG & I2C_STATUS_NACK) {
            I2C_CONTROL_REG = 0x00000000; // restart the read again
            I2C_CONTROL_REG |= I2C_CONTROL_CLEAR_WR_FIFO; //clear buffer 
            I2C_STATUS_REG &= ~I2C_STATUS_WRITE_FIFO_OVERFLOW; //status of buffer
            maxnack -= 1;
            continue;
        }
        else {
            I2C_CONTROL_REG = 0x00000000;
            break;
        }
    }
}

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvQueueI2CTask( void * pvParameters );
static void prvStatusLedTask( void * pvParameters );
static void prvTempTask(void* pvParameters);
/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/*** SEE THE COMMENTS AT THE TOP OF THIS FILE ***/
void main_temperature_monitor ( void )
{
    printf( "\r\nStarting the Temperature Monitor.\n");

    /* Create the queue. */
    xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );

    if( xQueue != NULL )
    {
        /* Start the two tasks as described in the comments at the top of this
         * file. */
        xTaskCreate( prvQueueI2CTask,             /* The function that implements the task. */
                     "Rx",                            /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                     configMINIMAL_STACK_SIZE,        /* The size of the stack to allocate to the task. */
                     NULL,                            /* The parameter passed to the task - not used in this simple case. */
                     mainQUEUE_RECEIVE_TASK_PRIORITY, /* The priority assigned to the task. */
                     NULL );                          /* The task handle is not required, so NULL is passed. */

        xTaskCreate( prvStatusLedTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_STATUS_TASK_PRIORITY, NULL );
        
        xTaskCreate( prvTempTask, "SX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_TEMP_TASK_PRIORITY, NULL);

        /* Start the tasks and timer running. */
        vTaskStartScheduler();
    }

    /* If all is well, the scheduler will now be running, and the following
     * line will never be reached.  If the following line does execute, then
     * there was insufficient FreeRTOS heap memory available for the idle and/or
     * timer tasks	to be created.  See the memory management section on the
     * FreeRTOS web site for more details. */
    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

static void prvStatusLedTask( void * pvParameters )
{
    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = TASK_LED_STATUS_MS;
    const uint32_t ulValueToSend = VALUE_SENT_FROM_STATUS_TASK;

    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();

    for( ; ; )
    {
        /* Place this task in the blocked state until it is time to run again.
         *  The block time is specified in ticks, pdMS_TO_TICKS() was used to
         *  convert a time specified in milliseconds into a time specified in ticks.
         *  While in the Blocked state this task will not consume any CPU time. */
        vTaskDelayUntil( &xNextWakeTime, xBlockTime );

        /* Send to the queue - causing the queue receive task to unblock and
         * write to the console.  0 is used as the block time so the send operation
         * will not block - it shouldn't need to block as the queue should always
         * have at least one space at this point in the code. */
        xQueueSend( xQueue, &ulValueToSend, 0U );
    }
}
/*-----------------------------------------------------------*/

static void prvTempTask(void* pvParameters)
{
    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = TEMP_CHECK_MS;
    const uint32_t ulValueToSend = VALUE_SENT_FROM_TMP_TASK;

    /* Prevent the compiler warning about the unused parameter. */
    (void)pvParameters;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();

    for (; ; )
    {
        /* Place this task in the blocked state until it is time to run again.
         *  The block time is specified in ticks, pdMS_TO_TICKS() was used to
         *  convert a time specified in milliseconds into a time specified in ticks.
         *  While in the Blocked state this task will not consume any CPU time. */
        vTaskDelayUntil(&xNextWakeTime, xBlockTime);

        /* Send to the queue - causing the queue receive task to unblock and
         * write to the console.  0 is used as the block time so the send operation
         * will not block - it shouldn't need to block as the queue should always
         * have at least one space at this point in the code. */
        xQueueSend(xQueue, &ulValueToSend, 0U);
    }
}


static void prvQueueI2CTask(void* pvParameters)
{
    uint32_t ulReceivedValue;
    LedState status_led = 0;
    LedState temperature_led = 0;

    /* Prevent the compiler warning about the unused parameter. */
    (void)pvParameters;

    for (; ; )
    {
        /* Wait until something arrives in the queue - this task will block
         * indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
         * FreeRTOSConfig.h.  It will not use any CPU time while it is in the
         * Blocked state. */
        xQueueReceive(xQueue, &ulReceivedValue, portMAX_DELAY);

        /* Enter critical section to use printf. Not doing this could potentially cause
         * a deadlock if the FreeRTOS simulator switches contexts and another task
         * tries to call printf - it should be noted that use of printf within
         * the FreeRTOS simulator is unsafe, but used here for simplicity. */
        taskENTER_CRITICAL();
        {
            //only one task could enter queue as this one is being completed so no need for other check
            if (ulReceivedValue == VALUE_SENT_FROM_TMP_TASK)
            {
                printf("TEMP Timer:");
                LedState curr_led = i2cTMP101();
                if (temperature_led != curr_led) {
                    temperature_led = curr_led;
                    i2cWritePcal(temperature_led, status_led);
                    if (temperature_led) {
                        printf("Temp LED is %d, Status Led is %d\n", temperature_led, status_led);
                    }
                    else {
                        printf("Temp LED is %d, Status Led is %d\n", temperature_led, status_led);
                    }

                }
            }
            else if (ulReceivedValue == VALUE_SENT_FROM_STATUS_TASK)
            {
                printf("Status Timer:");
                if (status_led) {
                    status_led = 0;
                    i2cWritePcal(temperature_led, status_led);
                    printf("Temp LED is %d, Status Led is %d\n", temperature_led, status_led);
                }
                else {
                    status_led = 1;
                    i2cWritePcal(temperature_led, status_led);
                    printf("Temp LED is %d, Status Led is %d\n", temperature_led, status_led);
                }
            }
        }
        taskEXIT_CRITICAL();
    }
}


