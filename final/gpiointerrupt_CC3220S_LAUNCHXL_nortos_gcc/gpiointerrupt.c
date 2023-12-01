/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/I2C.h>

/* Driver configuration */
#include "ti_drivers_config.h"
// ==============================================================================

// Globals
UART2_Handle uart;      // Driver Handle
char output[64];
int bytesToSend;

I2C_Handle i2c;         // Driver Handle
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}  sensors[3] = {
                { 0x48, 0x0000, "11X" },
                { 0x49, 0x0000, "116" },
                { 0x41, 0x0001, "006" }
};

Timer_Handle timer0;                    // Driver Handle
volatile unsigned char TimerFlag = 0;


// Constants for task periods and number of tasks
const unsigned char NUM_TASKS = 3;
const unsigned int TASK_PERIOD_GCD = 100000;
const unsigned int PERIOD_BUTTON = 200000;
const unsigned int PERIOD_TEMP = 500000;
const unsigned int PERIOD_SERVER = 1000000;
// ==============================================================================

// Macros
#define LED_ON GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON)
#define LED_OFF GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF)
#define DISPLAY(x) UART2_write(uart, &output, sizeof(output), x)
// ==============================================================================

// Variables
int16_t ambientTemp;
int16_t setPoint;
unsigned char heat;
unsigned int seconds;

// Functions Forward Declarations
void gpioButtonFxn0(uint_least8_t index);
void gpioButtonFxn1(uint_least8_t index);

void InitGPIOItems(void);       // Initialize GPIO dependent items
void InitUART2(void);           // Initialize UART2
void InitI2C(void);             // Initialize I2C
void InitTimer(void);           // Initialize Timer
void InitTasks(void);           // Initialize Tasks(3)

void TimerISR();                // Timer callback
void TaskScheduler();           // Schedule Tasks per period
unsigned char AT_SM(unsigned char AT_State);    // Ambient Temperature State Machine
unsigned char BP_SM(unsigned char BP_State);    // Button Press State Machine
unsigned char SD_SM(unsigned char SD_State);    // Send Data State Machine
int16_t readTemp(void);         // Get current ambientTemperature
// ==============================================================================

void *mainThread(void *arg0) {

    ambientTemp = 0;/*Current air temperature*/
    setPoint = 20;/*Desired temperature to stay at*/
    heat = 0;/*If true heat(LED) is on, else heat(LED) is off*/
    seconds = 0;/*Time in seconds since the board(thermostat) was reset*/

    // Driver, Button, LED Initialization Functions
    InitGPIOItems();
    InitUART2();
    InitI2C();

    // Task initialization
    InitTasks();

    // Timer initialization
    InitTimer();

    // main loop
    while(1) {
        // Call this every 100ms, 100000us
        TaskScheduler();

        // Wait for TimerFlag to activate
        while (!TimerFlag) {}
        TimerFlag = 0;
    }

    return (NULL);
}
// ==============================================================================

// Task structure
typedef struct task {
    unsigned char state;                        // Current state of task
    unsigned int period;                      // Rate task should tick
    unsigned int elapsedTime;                 // Time since previous tick in seconds (MS * .001)
    unsigned char ( *TickFct ) (unsigned char); // Function to call task's tick
    unsigned char flag;                         // Activated when elapsedTime >= period
} task;

// Initialize task array to 3 tasks
task tasks[3];

enum AT_States { AT_START, AT_OVER, AT_UNDER };
enum BP_States { BP_START, BP_STAY, BP_UP, BP_DOWN };
enum SD_States { SD_START, SD_WAIT, SD_SEND };

// ==============================================================================
// FUNCTION DEFINITIONS
// ==============================================================================

/* Button 0 Callback, Sets tasks[0].state to BP_UP */
void gpioButtonFxn0(uint_least8_t index) {

    tasks[0].state = BP_UP;

}
/* Button 1 Callback, Sets tasks[0].state to BP_DOWN */
void gpioButtonFxn1(uint_least8_t index) {

    tasks[0].state = BP_DOWN;

}

/*
 * Initialize GPIO,
 * Configure Buttons,
 * Configure LED
 */
void InitGPIOItems(void) {

    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    // GPIO Initialization Success
    LED_ON;

}
void InitUART2(void) {

    UART2_Params uartParams;

    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {

        /* UART2_open() failed */
        while (1) {}

    }
}
void InitI2C(void) {

    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Initialize the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while(1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Determine available sensor
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i = 0; i < 3; i++) {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }
    if(found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress));
    }
    else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}
void InitTimer(void) {

    Timer_Params params;

    Timer_init();

    Timer_Params_init(&params);
    params.period = TASK_PERIOD_GCD;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = TimerISR;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

void InitTasks(void) {

    unsigned char i = 0;

    tasks[i].state = BP_START;
    tasks[i].period = PERIOD_BUTTON;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &BP_SM;
    tasks[i].flag = 0;

    i++;

    tasks[i].state = AT_START;
    tasks[i].period = PERIOD_TEMP;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &AT_SM;
    tasks[i].flag = 0;

    i++;

    tasks[i].state = SD_START;
    tasks[i].period = PERIOD_SERVER;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &SD_SM;
    tasks[i].flag = 0;

}

void TimerISR() {

    TimerFlag = 1;

}

/*
 * For each task,
 * If the task's elapsed time equals the task's period,
 * reset the task's elapsed time,
 * activate the task's flag to 1
 * then call the task's current state function
 *
 * else add the timer period GCD to the task's elapsed time
 * and set flag to 0
 */
void TaskScheduler() {

    for (unsigned char i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].elapsedTime >= tasks[i].period) {

            tasks[i].elapsedTime = 0;
            tasks[i].flag = 1;
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
        }
        else {
            tasks[i].flag = 0;
        }
        tasks[i].elapsedTime += TASK_PERIOD_GCD;
    }
}

/*
 * Button Press (BP) States
 * =========================
 *
 *      BP_START:
 *          Initialization of state machine
 *
*       BP_STAY:
*           Wait for BP
*           BP will control state change
*
*       BP_UP:
*           Button1 pressed
*           setPoint++
*
*       BP_DOWN:
*           Button2 pressed
*           setPoint--
*
*       default:
*           Got to BP_START state
 */
unsigned char BP_SM(unsigned char BP_State) {

    /* State Switching*/
    switch(BP_State) {

        case BP_START:
            BP_State = BP_STAY;
            break;

        case BP_STAY:
            break;

        case BP_UP:
            break;

        case BP_DOWN:
            break;

        default:
            BP_State = BP_START;
            break;
    }

    switch (BP_State) {

        case BP_START:
            break;

        case BP_STAY:
            break;

        case BP_UP:
            setPoint++;
            BP_State = BP_STAY;
            break;

        case BP_DOWN:
            setPoint--;
            BP_State = BP_STAY;
            break;

        default:
            BP_State = BP_START;
            break;
    }
    return BP_State;

}


/*
 * Ambient Temperature (AT) States
 * ===============================
 *
 *      AT_START:
 *          Initialization of state machine
 *
*       AT_OVER:
*           AT is over the SetPoint temperature
*           LED (Heating) is OFF
*
*       AT_UNDER:
*           AT is under the SetPoint temperature
*           LED (Heating) is ON
*
*       default:
*           Go to AT_START state
 */
unsigned char AT_SM(unsigned char AT_State) {

    /* AT_SM State switching
     * OVER and UNDER do nothing here since their
     * changes are controlled by the I2C temperature sensor
     * and if no change, it will just loop
     */
    switch (AT_State) {

        case AT_START:
            if ((ambientTemp - setPoint) < 0) {
                heat = 1;
                AT_State = AT_UNDER;
            }
            else {
                heat = 0;
                AT_State = AT_OVER;
            }
            break;

        case AT_OVER:
            break;

        case AT_UNDER:
            break;

        default:
            AT_State = AT_START;
            break;
    }

    // AT_SM Functionality
    switch (AT_State) {

        case AT_START:
            break;

        case AT_OVER:
            LED_OFF;
            ambientTemp  = readTemp();
            if ((ambientTemp - setPoint) < 0) {
                heat = 1;
                AT_State = AT_UNDER;
            }
            else {
                heat = 0;
            }
            break;

        case AT_UNDER:
            LED_ON;
            ambientTemp  = readTemp();
            if ((ambientTemp - setPoint) >= 0) {
                heat = 0;
                AT_State = AT_OVER;
            }
            else {
                heat = 1;
            }
            break;

        default:
            break;
    }
    return AT_State;
}


/*
 * Send Data (SD) States
 * ======================
 *
 *      SD_START:
 *          Initialization of state machine
 *
*       SD_WAIT:
*           Wait for timer to reach 1000ms
*           Timer controls state change
*
*       SD_SEND:
*           Send data to server(UART2)
*           Form = "<%02d, %02d, %d, %04d>\n\r", temperature, setPoint, heat, seconds
*
*       default:
*           Go to SD_START state
 */
unsigned char SD_SM(unsigned char SD_State) {

    /* State switching */
    switch (SD_State) {

        case SD_START:
            SD_State = SD_WAIT;
            break;

        case SD_WAIT:
            if (tasks[2].flag) {
                SD_State = SD_SEND;
            }
            break;

        case SD_SEND:
            break;

        default:
            SD_State = SD_START;
            break;
    }

    switch (SD_State) {

        case SD_START:
            break;

        case SD_WAIT:
            break;

        case SD_SEND:
            seconds += (PERIOD_SERVER * 0.000001); // Convert microseconds to seconds and add to seconds passed since board reset

            /* This kept printing the address of the I2C sensor at the end, seemingly a remanent from :
             *                  DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress));
             *
             * Fixed it by filling the rest of the buffer with spaces at the end
             */
            DISPLAY (snprintf(output, 64, "\n\r<%02d, %02d, %d, %04d>           \n\r", ambientTemp, setPoint, heat, seconds));
            SD_State = SD_WAIT;
            break;

        default:
            SD_State = SD_START;
            break;
    }
    return SD_State;
}

int16_t readTemp(void) {

    int16_t temperature = 0;

    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    }
    else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Power cycle your board by unplugging USB and plugging back in.\n\r"));
    }

    return temperature;

}



































