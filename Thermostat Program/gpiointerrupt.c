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
#include <unistd.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char* id;
} sensors[3] = {
                { 0x48, 0x0000, "11X" },
                { 0x49, 0x0000, "116" },
                { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
UART_Handle uart;
I2C_Handle i2c;
Timer_Handle timer0;

// Defining display function key word for UART_write
#define DISPLAY(x) UART_write(uart, &output, x);

// UART Global Variables
char output[64];
int bytesToSend;

// Global flag variables for interrupt handling
volatile unsigned char BtnFlagG0 = 0;
volatile unsigned char BtnFlagG1 = 0;
volatile unsigned char TimerFlag = 0;

// Declare and initialize Global Variables
int16_t temp = 0;
int16_t setpoint = 0;
unsigned char heat = 0;
unsigned int seconds = 0;
unsigned int btn_counter0 = 0;
unsigned int btn_counter1 = 0;

// Task structure to create task scheduler
typedef struct task {
    int state;
    unsigned int period;
    unsigned int elapsedTime;
    int (*Tick)(int);
} task;

// Initialize task parameters: number of tasks and period constants
task tasks[2];
const unsigned short tasksNumber = 2;
const unsigned int taskGCD = 100000;
const unsigned int periodCheckButton = 200000;
const unsigned int periodCheckTemp = 500000;

// Declarations for the State Machine Functions; Implemented in the Task Scheduler
int Tick_CheckButtons(int state);
int Tick_CheckTemp(int state);

// Button States for SM
enum ButtonStates {
    B_Start,
    B_Wait,
    B0,
    B1
};

// Temperature States for SM
enum CheckTemp {
    T_Start,
    T_Check,
};

// Define the UART communication protocol
void initUART(void)
{
    UART_Params uartParams;
    // Init the driver
    UART_init();
    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}
// Define the I2C communication protocol
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i = 0; i < 3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
            if (I2C_transfer(i2c, &i2cTransaction))
            {
                DISPLAY(snprintf(output, 64, "Found\n\r"))
                found = true;
                break;
            }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }
    if (found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}
// Function to read the temperature
int16_t readTemp(void)
{
    // int j;
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
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
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor(%d)\n\r", i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

// Define the timer initialization function
void initTimer(void)
{
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    // Open the driver
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
/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    BtnFlagG0 = 1;  // Button flag to determine the state (See implementation in Tick_CheckButtons() function)
    btn_counter0++; // Button counter to keep track of button presses (See implementation in SetPoint() function)
}
/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    BtnFlagG1 = 1;  // Button flag to determine the state (See implementation in Tick_CheckButtons() function)
    btn_counter1++; // Button counter to keep track of button presses (See implementation in SetPoint() function)
}

/* ======= SetPoint =======
 * Function to set the target temperature from a default setting and updated from button presses
 * CONFIG_GPIO_BUTTON_0 (Right config) increases the setpoint by 1 degree Celsius
 * CONFIG_GPIO_BUTTON_1 (Left config) decreases the setpoint by 1 degree Celsius
 */
int16_t SetPoint()
{
    int16_t defaultSetting  = 26;

    if (btn_counter0 == 0 && btn_counter1 == 0) {
        setpoint = defaultSetting;
    }
    else if (btn_counter0 > btn_counter1) {
        setpoint = defaultSetting + (btn_counter0 - btn_counter1);
    }
    else if (btn_counter0 <= btn_counter1) {
        setpoint = defaultSetting - (btn_counter1 - btn_counter0);
    }
    return setpoint;
}

/* ======= RaiseTemp =======
 * Function to raise the target setpoint temperature
 * Function is called in Tick_CheckButtons() SM
 */

int16_t RaiseTemp(int16_t setpoint)
{
    setpoint++;
    return setpoint;
}
/* ======= LowerTemp =======
 * Function to lower the target setpoint temperature
 * Function is called in Tick_CheckButtons() SM
 */
int16_t LowerTemp(int16_t setpoint)
{
    setpoint--;
    return setpoint;
}
/*  ======= UpdateLed =======
 *  Function updates the Led based on the temperature sensor and the target setpoint
 *  The red LED on CONFIG_GPIO_LED_0 controls a heater
 *
 *  GPIO_LED_ON when the setpoint target is greater than the temperature
 *  GPIO_LED_OFF when the setpoint target is less than or equal to
 */

void UpdateLed(int16_t temp, int16_t setpoint)
{
    if (setpoint > temp)    // trigger for turning on Led
    {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); // turn on Red Led: Heat ON
    }
    else if (setpoint <= temp)  // trigger for turning off Led
    {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // turn off Red Led: Heat OFF
    }
}

/* ======= TaskScheduler =======
 * Function implements a task scheduler using the task structure
 * Called in mainThread()
 */

void TaskScheduler()
{
    unsigned char i = 0;
    tasks[i].state = B_Start;
    tasks[i].period = periodCheckButton;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].Tick = &Tick_CheckButtons;
    i++;
    tasks[i].state = T_Start;
    tasks[i].period = periodCheckTemp;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].Tick = &Tick_CheckTemp;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    usleep(1000000);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    initUART();         // initialize UART; must be called before initI2C()
    initI2C();          // initialize I2C
    initTimer();        // initialize the Timer
    TaskScheduler();    // call the Task Scheduler function

    // Get the current count of timer0 handle
    seconds = Timer_getCount(timer0) / 10000;       // convert useconds to seconds by dividing by 10000 us

    // Loop forever
    while (1) {

        TimerFlag = 0;

        unsigned char i;
        for (i=0; i < tasksNumber; ++i) {
            if (tasks[i].elapsedTime >= tasks[i].period)
            {
                tasks[i].state = tasks[i].Tick(tasks[i].state); //execute task tick
                tasks[i].elapsedTime = 0;
            }
            tasks[i].elapsedTime += taskGCD;
        }

        // Button flags are checked in the task scheduler SM code

        temp = readTemp();                      // get the current temperature from the temperature sensor
        setpoint = SetPoint();              // set the target temperature setpoint
        UpdateLed(temp, setpoint);              // update the LED from the temperature sensor and target setpoint
        heat = GPIO_read(CONFIG_GPIO_LED_0);    // read the LED to get the return value

        // print message to the server via UART
        DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temp, setpoint, heat, seconds));

        while(!TimerFlag) {}    // wait for the timer flag
        seconds++;  // increment seconds from timer counts
    }
}

// Defines the state machine to check the button flags and control the setpoint temperature
int Tick_CheckButtons(int state)
{
    switch (state)
    {   // State transitions
    case B_Start:
        state = B_Wait;
        break;

    // Check the Button flags to determine the state
    case B_Wait:
        if (BtnFlagG0 == 1) {       // Flag for Button_0
            state = B0;
        }
        else if (BtnFlagG1 == 1) {  // Flag for Button_1
            state = B1;
        }
        else if (BtnFlagG0 == 0 && BtnFlagG1 == 0) {    // Wait for a Button Press
            state = B_Wait;
        }
        break;

    // State Actions
    case B0:
        RaiseTemp(setpoint);    // Increment setpoint target temperature by 1 degree C
        break;

    case B1:
        LowerTemp(setpoint);    // Decrement setpoint target temperature by 1 degree C
        break;

    default:
        state = B_Start;
    }
    return state;
}

// Defines the state machine for reading the temperature
int Tick_CheckTemp(int state)
{
    switch (state)
    {   // State Transitions
    case T_Start:
        state = T_Check;
        break;
    case T_Check:
        state = T_Start;
        break;

    default:
        state = T_Start;
        break;
    }
    switch (state)
    {   // State Actions
    case T_Check:
        readTemp();     // Read the temperatures every 500 ms
        break;

    default:
        break;
    }
    return state;
}

