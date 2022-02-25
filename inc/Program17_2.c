
#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/UART0.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/TimerA1.h"
#include "../inc/Nokia5110.h"
#include "../inc/SysTickInts.h"
#include "../inc/Reflectance.h"

// Macro to return a bounded value
#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ( (X) > (Max)? (Max) : (X) ) )

// bit-banding for LEDs
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define GREENLED (*((volatile uint8_t *)(0x42098064)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))

// proportional controller gain
// experimentally determine value that creates a stable system
static int32_t Kp = 20;


volatile static uint8_t IsControllerEnabled = 0;
// If controller is executed multiple times, run LCDOut.
volatile static uint8_t NumControllerExecuted = 0;


static void UpdateParameters(void) {

    while(LaunchPad_Input() || Bump_Read()) { // wait for release
        Clock_Delay1ms(200); LaunchPad_Output(0); // off
        Clock_Delay1ms(200); LaunchPad_Output(1); // red
    }

    Nokia5110_Clear();
    Nokia5110_SetCursor(0,0); Nokia5110_OutString("Update Kp");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("S1 for +   ");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("S2 for -   ");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("Kp:");
    Nokia5110_SetCursor(0,5);  Nokia5110_OutString("Bump to Exit");


    // Until bump is pressed.
    while(!Bump_Read()){
        // update the screen
        Nokia5110_SetCursor(3,3); Nokia5110_OutUDec(Kp);
        if (LaunchPad_Input() & 0x02 ) { // SW2 is pressed
            Kp++;
        }
        if (LaunchPad_Input() & 0x01 ) { // SW1 is pressed
            Kp--;
        }
        // flash the blue LED while desired speeds are updated.
        BLUELED ^= 1;
        Clock_Delay1ms(200);
    }

    // desired parameters are updated now.
    // flash yellow LED for 1 sec.
    for(int k = 0; k < 5; k++){
        LaunchPad_Output(0x03);
        Clock_Delay1ms(100);
        LaunchPad_Output(0x00);
        Clock_Delay1ms(100);
    }
}

#define ERROR_BUFF_SIZE  2000 // for 20 second data
static int32_t ErrorBuffer[ERROR_BUFF_SIZE];
static uint16_t ErrorBuffIndex = 0;

// send analysis data to PC.
static void TxBuffer(void) {

    while(LaunchPad_Input() || Bump_Read()) { // wait for release
        Clock_Delay1ms(200); LaunchPad_Output(0); // off
        Clock_Delay1ms(200); LaunchPad_Output(1); // red
    }

    uint8_t isTxEnabled = 0;

    Nokia5110_Clear();
    Nokia5110_OutString("Tx Buffer?");
    Nokia5110_SetCursor(0, 1);  Nokia5110_OutString("S2: Y/N");
    Nokia5110_SetCursor(0, 3);  Nokia5110_OutString("Bump 2 Enter");

    while(!Bump_Read()) {
        // update the screen
        if (isTxEnabled) {
            Nokia5110_SetCursor(11,0);
            Nokia5110_OutString("Y");
        } else {
            Nokia5110_SetCursor(11,0);
            Nokia5110_OutString("N");
        }

        if (LaunchPad_Input() & 0x02) {
            isTxEnabled ^= 1;
        }

        // flash the blue LED while desired speeds are updated.
        BLUELED ^= 1;
        Clock_Delay1ms(200);
    }

    if(isTxEnabled) {
        UART0_OutString("\n\r***Receiving buffer data***\n\r");
        for (int i = 0; i < ERROR_BUFF_SIZE; i++) {
            UART0_OutUDec(i);
            UART0_OutChar(',');
            UART0_OutSDec(ErrorBuffer[i]);
            UART0_OutString("\n\r");
        }

        Nokia5110_SetCursor(0, 3);
        Nokia5110_OutString("Done        ");
        Nokia5110_SetCursor(0, 4);
        Nokia5110_OutString("Bump 2 Exit");
        while(!Bump_Read());
        Clock_Delay1ms(200);
    }

    for(int k = 0; k < 10; k++){
        LaunchPad_Output(0x03);
        Clock_Delay1ms(100);
        LaunchPad_Output(0x00);
        Clock_Delay1ms(100);
    }
}


/**************Program17_2******************************************/
#define PWMNOMINAL 4000
#define SWING 3000
#define PWMIN (PWMNOMINAL-SWING)
#define PWMAX (PWMNOMINAL+SWING)

// Proportional controller to drive robot using line following
uint8_t LineData;       // direct measure from line sensor
int32_t Position;      // position in 0.1mm relative to center of line
static int16_t leftDuty_permyriad = PWMNOMINAL;
static int16_t rightDuty_permyriad = PWMNOMINAL;

static void LCDClear(void) {
    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear(); // erase entire display
    Nokia5110_OutString("17.2:Kp");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("Line Follow");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("D =  "); Nokia5110_OutUDec(0);
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("P = "); Nokia5110_OutSDec(0);
    Nokia5110_SetCursor(0,4); Nokia5110_OutString("UR=  "); Nokia5110_OutUDec(0);
    Nokia5110_SetCursor(0,5); Nokia5110_OutString("UL=  "); Nokia5110_OutUDec(0);
}

static void LCDOut(void) {
    Nokia5110_SetCursor(7,0); Nokia5110_OutUDec(Kp);
    Nokia5110_SetCursor(5,2); Nokia5110_OutUHex7(LineData);
    Nokia5110_SetCursor(4,3); Nokia5110_OutSDec(Position);
    Nokia5110_SetCursor(5,4); Nokia5110_OutUDec(rightDuty_permyriad);
    Nokia5110_SetCursor(5,5); Nokia5110_OutUDec(leftDuty_permyriad);
}


// Proportional controller to drive robot
// using line following
static void Controller(void){

    static uint16_t Time_ms = 0; // in 1 msec

    // Controller is disabled.  Do nothing
    if (!IsControllerEnabled) { return; }

    // If a SW1 or SW2 is pressed, disable Controller
    if (LaunchPad_Input()) {
        IsControllerEnabled = 0;
        return;
    }

    // If program reaches here, controller is enabled.
    // Execute controller.

    // ====================================================================
    // Complete the rest for lab17
    // ====================================================================

    // read values from line sensor, similar to
    // SysTick_Handler() in Lab10_Debugmain.c

    Time_ms++;

    if (Time_ms < 9) { // do nothing for Timer_ms = 0,1,...,8
        return;
    }

    if (9 == Time_ms) { // start Reflectance and return
        Reflectance_Start();
        return;
    }

    // If the program reaches here, it means Time_ms == 10
    Time_ms = 0;    // reset Time_ms

    // Read line data
    LineData = ~Reflectance_End();

    // find the position.
    // Use white line on black background for Maze.
    // Do not modify your reflectance.c
    Position = Reflectance_Position(LineData);


    // save error for performance analysis.
    if (ErrorBuffIndex < ERROR_BUFF_SIZE) {
        ErrorBuffer[ErrorBuffIndex++] = Position;
    }

    // update duty cycle based on proportional control
    rightDuty_permyriad = Kp*Position;
    leftDuty_permyriad = -1*Kp*Position;

    // check to ensure the input is bounded.
    rightDuty_permyriad = MINMAX(PWMIN, PWMAX, rightDuty_permyriad);
    leftDuty_permyriad = MINMAX(PWMIN, PWMAX, leftDuty_permyriad);

    // update motor values
    Motor_Forward(leftDuty_permyriad, rightDuty_permyriad);

    // Increment the number of controller executed.
    NumControllerExecuted++;
}


// proportional control, line following
void Program17_2(void){

    DisableInterrupts();
    Clock_Init48MHz();
    UART0_Init();
    LaunchPad_Init();
    Bump_Init();
    Reflectance_Init();
    Motor_Init();
    Nokia5110_Init();
    LCDClear();

    // user TimerA1 to run the controller at 1000 Hz
    uint16_t period_2us = 500;              // T = 1ms
    TimerA1_Init(&Controller, period_2us);  // f = 1000 Hz controller loop

    // NumControllerExecuted increments at 100 Hz
    // Updated LCD at 10 Hz --> every 10 controller runs
    const uint16_t LcdUpdateRate = 10;    // 100/10 Hz

    IsControllerEnabled = 0;
    NumControllerExecuted = 0;
    ErrorBuffIndex = 0;

    LCDClear();
    EnableInterrupts();

    while(1) {

        // low power mode while waiting for the next interrupt.
        WaitForInterrupt();

        // Nokia5110 is a slow device.
        // Do not add LCDOut inside ISR.
        // Update LCD at 10 Hz.
        if (NumControllerExecuted == LcdUpdateRate) {
            LCDOut();
            NumControllerExecuted = 0;
        }

        // If controller is enabled, skip the next and go to low power mode.
        if (IsControllerEnabled) {
            continue;
        }

        // If the program reaches here, controller is disabled.
        // We can update the desired speeds.
        LaunchPad_Output(0); // turn off RGB LED
        Motor_Coast();
        Clock_Delay1ms(300);

        UpdateParameters();
        TxBuffer();

        LCDClear();

        // Enable controller
        IsControllerEnabled = 1;
    }

}
