/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>

#define DISPLAY(x) UART_write(uart, &output, x);
/* Driver configuration */
#include "ti_drivers_config.h"

// I2C Global Variables
static const struct
{
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = { { 0x48, 0x0000, "11X" }, { 0x49, 0x0000, "116" }, { 0x41, 0x0001, "006" }};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// UART Global Variables
char output[64];
int bytesToSend;

// Temperature Control Globals
int tempSetPoint;
int tempNow;
unsigned char heat;
unsigned char checkTemp = 1;
unsigned char postSeconds = 1;
//Globals
int seconds;
int buttonPressed;

//Task Scheduler Variables
const unsigned char tasksNum = 3;
const unsigned long tasksPeriodGCD = 100000;
const unsigned long periodCheckTemp = 500000;
const unsigned long periodSetPoint = 200000;
const unsigned long periodCheckSeconds = 1000000;

// Driver Handles - Global variables

UART_Handle uart;
I2C_Handle i2c;

//Task Scheduler Struct
typedef struct task {
  char state; // Current state of the task
  unsigned long period; // Rate at which the task should tick
  unsigned long elapsedTime; // Time since task's previous tick
  char (*TickFct)(char); // Function to call for task's tick
} task;
task tasks[3];

// State Machine States
enum SP_States { SP_Start, SP_UP, SP_DOWN, SP_Hold} SP_States; //Set Point States
char Set_Point(char state);
enum CT_States { CT_Start, CT_ON, CT_OFF } CT_States; //Check Temperature States
char Check_Temp(char state);
enum CS_States { CS_Start, CS_ADD } CS_States; // Check Seconds States
char Check_Seconds(char state);

/*
 * --------------------Driver Interaction Code--------------------------------
 */
// UART Driver Interactions - GPIO output
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
    if (uart == NULL)
    {
        /* UART_open() failed */
        while (1);
    }
}
// I2C Driver Interactions - Accesses the temperature sensor.
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
// Function to read and return data from temperature sensor
int16_t readTemp(void)
{

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

        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
}

// Timer interrupt callback
// Is called during the set period interrupt variable tasksPeriodGCD and holds our task scheduler code.
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    //Task Scheduler
    unsigned int i = 0;
    for (i = 0; i < tasksNum; ++i) { // Heart of the scheduler code
       if ( tasks[i].elapsedTime >= tasks[i].period ) { // Ready
          tasks[i].state = tasks[i].TickFct(tasks[i].state);
          tasks[i].elapsedTime = 0;
       }
       tasks[i].elapsedTime += tasksPeriodGCD;
    }
}
// Timer Driver Interactions - creates/sets our timed interrupts
void initTimer(void)
{

    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = tasksPeriodGCD;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

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
 * --------------------------State Machines---------------------------
 */
/*
 * Set Point State Machine
 */
char Set_Point(char state){

    checkTemp = 1;
    //Set State Point - Add/Remove Based on Button Click
    switch(state) {
        case SP_Start:
            tempSetPoint = 28;
            break;
        case SP_UP:
            tempSetPoint+=1;
            state = SP_Hold;
            break;
        case SP_DOWN:
            tempSetPoint-=1;
            state = SP_Hold;
            break;
        case SP_Hold:
            break;
        default:
            break;
    }
    //Set State - Reset Button
    switch(buttonPressed) {
        case 0:
            state = SP_Hold;
            break;
        case 1:
            state = SP_UP;
            buttonPressed = 0;
            break;
        case 2:
            state = SP_DOWN;
            buttonPressed = 0;
            break;
    }
    return state;
}

/*
 * Check Temperature State Machine
 */
char Check_Temp(char state){

    //Check Temp and Turn On/Off LED
    switch(state) {
        case CT_Start:
            break;
        case CT_ON:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            heat = 1;
            break;
        case CT_OFF:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            heat = 0;
            break;
        default:
            break;
    }

    //Set Next State - Test tempNow against the tempSetPoint
    switch(state) {
        case CT_Start:
            if(tempNow > tempSetPoint){
                state = CT_OFF;
            }
            else{
                state = CT_ON;
            }
            break;
        case CT_ON:
            if(tempNow > tempSetPoint){
                state = CT_OFF;
            }
            else{
                state = CT_ON;
            }
            break;
        case CT_OFF:
            if(tempNow > tempSetPoint){
                state = CT_OFF;
            }
            else{
                state = CT_ON;
            }
            break;
        default:
            break;
    }
    return state;
}

/*
 * Check Seconds State Machine
 */
char Check_Seconds(char state){
    //Add Seconds to time and Set Post to True.
    switch(state){
        case CS_Start:
            state = CS_ADD;
            break;
        case CS_ADD:
            ++seconds;
            postSeconds = 1;
            break;
        default:
            break;
    }
    return state;
}


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle Set Point + 1 */
    buttonPressed = 1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle Set Point -1*/
    buttonPressed = 2;
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
    unsigned int i=0;
    //Check Time Task Schedule
    tasks[i].state = CT_Start;
    tasks[i].period = periodCheckTemp;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &Check_Temp;
    ++i;
    //Set Point Task Schedule
    tasks[i].state = SP_Start;
    tasks[i].period = periodSetPoint;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &Set_Point;
    ++i;
    //Check Seconds Task Schedule
    tasks[i].state = CS_Start;
    tasks[i].period = periodCheckSeconds;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &Check_Seconds;
    seconds = 0;//initiate seconds

    //Call drivers and interrupts
    initUART();
    initI2C();
    initTimer();

    //Check Temp and Post Out Through UART
    while(1){
        if(checkTemp){
            tempNow = readTemp();
            checkTemp = 0;
        }
        if(postSeconds){
            DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", tempNow, tempSetPoint, heat, seconds))
            postSeconds = 0;
        }
    }
}
