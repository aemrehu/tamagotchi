/* C Standard library */
#include <stdio.h>

#include <string.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "wireless/comm_lib.h"
#include "sensors/mpu9250.h"
#include "buzzer.h"
//#include <opt3001.h>
#include <ti/drivers/I2C.h>

/* Task */
#define STACKSIZE 2048
Char mpuTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char checkTaskStack[STACKSIZE];

//STATE MACHINE
enum state { WAITING=1, GET_DATA, SEND_MSG, PLAY_MSG, CHECK_DATA };
enum state programState = WAITING;

//Global variables
float ax, ay, az, gx, gy, gz;
float ayList[50] = {};
float azList[50] = {};
float gxList[50] = {};
int ayCount;
int azCount;
int gxCount;
int eatFlag = 0;
int petFlag = 0;
int exerciseFlag = 0;
int msgFlag = 0;

char uartBuffer[80];

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;
//static PIN_Handle led1Handle;
//static PIN_State led1State;

static PIN_Handle hBuzzer;
static PIN_State sBuzzer;

// MPU power pin global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

// MPU power pin
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

// Button config
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE, 
   PIN_TERMINATE
};

// LED PIN config
PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, 
   PIN_TERMINATE
};

//Buzzer pin
PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

void buzzerFunc(int toistot, uint16_t freq, int aika);

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    
    programState = GET_DATA;
}

void buzzerFunc(int toistot, uint16_t freq, int aika) {

    int i;
    for (i = 0; i < toistot; i++) {
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(freq);
        Task_sleep(aika / Clock_tickPeriod);
        buzzerClose();
        Task_sleep(50000 / Clock_tickPeriod);
    }
}

void uartFxn(UART_Handle uart, void *rxBuf, size_t len) {
    
    char str_test[10] = "525,BEEP\0";
    char *ret;
    ret = strstr(rxBuf, str_test);
    
    if (ret != NULL) {
        
        //System_printf("BEEP\n");
        //System_flush();
        
        programState = PLAY_MSG;
    }
    
    UART_read(uart, rxBuf, 80);
}

/* Task Functions */

Void checkTaskFxn(UArg arg0, UArg arg1) {

    while (1) {
        
        if (programState == CHECK_DATA) {
            int i;
            for (i = 0; i < 50; i++) {
                
                if ((ayList[i] > 2) || (ayList[i] < -2)) {
                    ayCount += 1;
                }

                if ((azList[i] > 2) || (ayList[i] < -2)) {
                    azCount += 1;
                }

                if ((gxList[i] > 220) || (ayList[i] < -220)) {
                    gxCount += 1;
                }
                /*
                char str_05[80];
                sprintf(str_05, "%d,  %.3f,%.3f,%.3f\n", i,ayList[i],azList[i],gxList[i]);
                System_printf(str_05);
                */
                Task_sleep(500 / Clock_tickPeriod);
                
            }
            
            eatFlag = 0;
            exerciseFlag = 0;
            petFlag = 0;

            if (ayCount >= 3) {
                
                exerciseFlag = ayCount / 3;
                
                buzzerFunc(1,5000,100000);
            } 
            
            if (azCount >= 3) {
                
                eatFlag = azCount / 3;
                
                buzzerFunc(2,4000,50000);
            } 
            
            if (gxCount >= 3) {
                
                petFlag = gxCount / 3;
                
                buzzerFunc(3,3000,50000);
            }
            
            if ((ayCount < 3) && (azCount < 3) && (gxCount < 3)) {
                buzzerFunc(1,1500,800000);
            }
            /*
            char str_04[80];
	        sprintf(str_04, "%d, %d, %d\n%d, %d, %d\n", ayCount, azCount, gxCount, exerciseFlag, eatFlag, petFlag);
	        System_printf(str_04);
	        System_flush();
            */
            ayCount = 0;
            azCount = 0;
            gxCount = 0;
            
            programState = SEND_MSG;
            
        }
        
        Task_sleep(10000 / Clock_tickPeriod);
    }
}

Void uartTaskFxn(UArg arg0, UArg arg1) {
    
    UART_Handle uart;
    UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = &uartFxn;
    uartParams.baudRate = 9600;
    uartParams.dataLength = UART_LEN_8;
    uartParams.parityType = UART_PAR_NONE;
    uartParams.stopBits = UART_STOP_ONE;
   
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        System_abort("Error opening the UART");
    }
    
    UART_read(uart, uartBuffer, 80);
    
    while (1) {
        
        if (programState == PLAY_MSG) {
            uint_t pinValue = PIN_getOutputValue( Board_LED0 );
            pinValue = !pinValue;
            PIN_setOutputValue( ledHandle, Board_LED0, pinValue );
            buzzerFunc(2,4000,200000);
            pinValue = !pinValue;
            PIN_setOutputValue( ledHandle, Board_LED0, pinValue );
            programState = WAITING;
        }
        
        if (programState == SEND_MSG) {
            
            if (eatFlag >= 1) {
                char eat[80];
                sprintf(eat,"id:525,EAT:%d\0",eatFlag);
                UART_write(uart, eat, strlen(eat)+1);
                //System_printf("EAT\n");
                //System_flush();
            }
            
            if (exerciseFlag >= 1) {
                char exercise[80];
                sprintf(exercise,"id:525,EXERCISE:%d\0",exerciseFlag);
                UART_write(uart, exercise, strlen(exercise)+1);
                //System_printf("EXERCISE\n");
                //System_flush();
            }
            
            if (petFlag >= 1) {
                char pet[80];
                sprintf(pet,"id:525,PET:%d\0",petFlag);
                UART_write(uart, pet, strlen(pet)+1);
                //System_printf("PET\n");
                //System_flush();
            }
            
            programState = WAITING;
        }
        
        Task_sleep(10000 / Clock_tickPeriod);
    }
}

Void mpuTaskFxn(UArg arg0, UArg arg1) {

	I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
	I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
	// Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
	Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // MPU setup and calibration
	System_printf("MPU9250: Setup and calibration...\n");
	System_flush();

	mpu9250_setup(&i2cMPU);

	System_printf("MPU9250: Setup and calibration OK\n");
	System_flush();
    
    // Loop forever
	while (1) {
	    
	    // MPU ask data
	    mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
	    
        if (programState == GET_DATA) {
            
            uint_t pinValue = PIN_getOutputValue( Board_LED0 );
            
            int i;
            for (i = 0; i < 50; i++) {
                
                if (i == 0) {
                    //System_printf("DATA START\n");
                    pinValue = !pinValue;
                    PIN_setOutputValue( ledHandle, Board_LED0, pinValue );
                    buzzerFunc(1,4000,50000);
                }
                
                mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
                
                ayList[i] = ay;
                azList[i] = az;
                gxList[i] = gx;
                
                if (i == 49) {
                    //System_printf("DATA END\n");
                    pinValue = !pinValue;
                    PIN_setOutputValue( ledHandle, Board_LED0, pinValue );
                    programState = CHECK_DATA;
                }
                
                Task_sleep(80000 / Clock_tickPeriod);
            }
            
        }
	    
    	Task_sleep(100000 / Clock_tickPeriod);
	}

}

Int main(void) {

    // Task variables
    Task_Handle mpuTaskHandle;
    Task_Params mpuTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    Task_Handle checkTaskHandle;
    Task_Params checkTaskParams;

    // Initialize board
    Board_initGeneral();
    Init6LoWPAN();
    Board_initI2C();
    Board_initUART();
    
    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
    	System_abort("Pin open failed!");
    }
    
    // Activate pins
    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
       System_abort("Error initializing button pins\n");
    }
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
       System_abort("Error initializing LED pins\n");
    }
    
    // Set interrupt handler 
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
       System_abort("Error registering button callback function");
    }

    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
        System_abort("Pin open failed!");
    }

    /* Tasks */
    Task_Params_init(&mpuTaskParams);
    mpuTaskParams.stackSize = STACKSIZE;
    mpuTaskParams.stack = &mpuTaskStack;
    mpuTaskParams.priority=2;
    mpuTaskHandle = Task_create(mpuTaskFxn, &mpuTaskParams, NULL);
    if (mpuTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }
    
    Task_Params_init(&checkTaskParams);
    checkTaskParams.stackSize = STACKSIZE;
    checkTaskParams.stack = &checkTaskStack;
    checkTaskParams.priority=2;
    checkTaskHandle = Task_create(checkTaskFxn, &checkTaskParams, NULL);
    if (checkTaskHandle == NULL) {
        System_abort("Task create failed!");
    }
    
    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
