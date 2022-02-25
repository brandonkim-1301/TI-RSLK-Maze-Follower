
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
#include "../inc/LPF.h"
#include "../inc/Tachometer.h"


// Macro to return a bounded value
#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ( (X) > (Max)? (Max) : (X) ) )

// bit-banding for LEDs
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define GREENLED (*((volatile uint8_t *)(0x42098064)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))


// ------------ average ------------
// Simple math function that returns the average
// value of the data array.
// Input: data is an array of 16-bit unsigned numbers
//        data_length is the number of elements in data
// Output: the average value of the data
// Note: overflow is not considered
static uint16_t average(uint16_t *data, int data_length) {
    uint32_t sum = 0;
    for (int i = 0; i < data_length; i++) {
        sum += data[i];
    }
    return sum/data_length;
}


// Declare a storage class specifier, static, to limit the scope to the file.
// Otherwise, objects without a storage class specifier have external linkage (extern).
// store period for performance analysis
#define SPEED_BUFFER_SIZE 1000    // for 20 sec at 50Hz Controller
static uint32_t LeftPeriodBuffer[SPEED_BUFFER_SIZE];
static uint32_t RightPeriodBuffer[SPEED_BUFFER_SIZE];
static uint16_t BufferIndex = 0;

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
        for (int i = 0; i < BufferIndex; i++) {
            UART0_OutUDec(i);
            UART0_OutChar(',');
            UART0_OutSDec(LeftPeriodBuffer[i]);
            UART0_OutChar(',');
            UART0_OutSDec(RightPeriodBuffer[i]);
            UART0_OutString("\n\r");
        }
        Nokia5110_SetCursor(0, 3);
        Nokia5110_OutString("Done        ");
        Nokia5110_SetCursor(0, 4);
        Nokia5110_OutString("Bump 2 Exit");
        while(!Bump_Read());
    }

    for(int k = 0; k < 10; k++){
        LaunchPad_Output(0x03);
        Clock_Delay1ms(100);
        LaunchPad_Output(0x00);
        Clock_Delay1ms(100);
    }
}


// Proportional controller to drive straight with no sensor input
#define PWMNOMINAL 5000         // 50 %
#define PULSE2RPM 250000        // clock divider is 8
#define DESIRED_SPEED_RPM 120   // 120 rpm
static int16_t ErrorL = 0;
static int16_t ErrorR = 0;
static uint16_t LeftSpeed_rpm = 0;
static uint16_t RightSpeed_rpm = 0;

static void LCDClear(void){

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB8;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();
    Nokia5110_SetCursor(0, 0);
    Nokia5110_OutString("Desired(RPM)L     R     ");
    Nokia5110_SetCursor(1, 1);         // one leading space, second row
    Nokia5110_OutUDec(DESIRED_SPEED_RPM);
    Nokia5110_SetCursor(7, 1);         // seven leading spaces, second row
    Nokia5110_OutUDec(DESIRED_SPEED_RPM);
    Nokia5110_SetCursor(0, 2);
    Nokia5110_OutString("Actual (RPM)L     R     ");
    Nokia5110_SetCursor(0, 4);
    Nokia5110_OutString("Error(RPM)  L     R     ");
}


static void LCDOut(void){
    Nokia5110_SetCursor(1, 3);       // one leading space, fourth row
    Nokia5110_OutUDec(LeftSpeed_rpm);
    Nokia5110_SetCursor(7, 3);       // seven leading spaces, fourth row
    Nokia5110_OutUDec(RightSpeed_rpm);
    Nokia5110_SetCursor(0, 5);       // zero leading spaces, sixth row
    Nokia5110_OutSDec(ErrorL);
    Nokia5110_SetCursor(6, 5);       // six leading spaces, sixth row
    Nokia5110_OutSDec(ErrorR);
}


#define TACHBUFF_SIZE 10    // number of elements in tachometer array
#define PWMIN 2
#define PWMAX 9999

volatile static uint8_t IsControllerEnabled = 0;
volatile static uint8_t NumControllerExecuted = 0;

//**************************************************
// Proportional-integral controller to drive straight with input
// from the tachometers
static void Controller(void){

    static uint8_t nData = 0; // number of tachometer data read.
    static uint16_t LeftTachBuffer[TACHBUFF_SIZE];
    static uint16_t RightTachBuffer[TACHBUFF_SIZE];
    static int32_t PrevErrorL = 0;
    static int32_t PrevErrorR = 0;

    static int16_t leftDuty_permyriad = 0;
    static int16_t rightDuty_permyriad = 0;

    enum TachDirection LeftDir;
    enum TachDirection RightDir;

    int32_t LeftSteps_deg;      // left wheel steps in deg
    int32_t RightSteps_deg;     // right wheel steps in deg

    // Controller is disabled.  Do nothing
    if(!IsControllerEnabled) { return; }

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

    // Get values from the tachometer and store into variables
    Tachometer_Get(&LeftTachBuffer[nData], &LeftDir, &LeftSteps_deg,
                   &RightTachBuffer[nData], &RightDir, &RightSteps_deg);

    nData = (nData + 1) % TACHBUFF_SIZE;

    // use average of ten tachometer values
    uint16_t leftPeriod = average(LeftTachBuffer, TACHBUFF_SIZE);
    uint16_t rightPeriod = average(RightTachBuffer, TACHBUFF_SIZE);

    // determine actual speed (similar to Lab16)
    LeftSpeed_rpm = PULSE2RPM/leftPeriod;
    RightSpeed_rpm = PULSE2RPM/rightPeriod;

    // save periods for performance analysis.
    if (BufferIndex < SPEED_BUFFER_SIZE) {
        LeftPeriodBuffer[BufferIndex] = leftPeriod;
        RightPeriodBuffer[BufferIndex] = rightPeriod;
        BufferIndex++;
    }

    // Experimentally determine value that creates a stable system
    // This Ki is different than the lecture note Ki.
    // This Ki is normalized Ki, i.e., Ki(normalized) = Ki*DeltaT/2.
    // start with Ki = 0.
    // Then, increment Ki for removing steady state error.
    const uint16_t Kp = 9;
    const uint16_t Ki = 3;
    const int16_t Ka = Kp + Ki;
    const int16_t Kb = -Kp + Ki;

    // calculate Error
    // error = desired speed - actual (measured) speed
    ErrorR = DESIRED_SPEED_RPM - RightSpeed_rpm;
    ErrorL = DESIRED_SPEED_RPM - LeftSpeed_rpm;

    // Homework 17: use proportional control to update duty cycle
    // Use Kp and set Ki = 0.
//    rightDuty_permyriad = Ki + Kp*ErrorR;
//    leftDuty_permyriad = Ki + Kp*ErrorL;

    // Lab 17: use proportional-integral control to update duty cycle
    // Set Kp and Ki and use Ka and Kb here.
     rightDuty_permyriad += Ka*ErrorR + Kb*ErrorR;
     leftDuty_permyriad += Ka*ErrorL + Kb*ErrorL;


    // updated previous errors for the next iteration.
    PrevErrorR = ErrorR;
    PrevErrorL = ErrorL;

    // check min/max duty values
    // they must be bounded between PWMIN and PWMAX.
    // Use MINMAX defined in Line 18
    rightDuty_permyriad = MINMAX(PWMIN, PWMAX, rightDuty_permyriad);
    leftDuty_permyriad = MINMAX(PWMIN, PWMAX, leftDuty_permyriad);

    // update motor values
    Motor_Forward(leftDuty_permyriad, rightDuty_permyriad);

    // Increment the number of controller executed. .
    NumControllerExecuted++;
}


// Program17_1
void Program17_1(void){

    DisableInterrupts();
    Clock_Init48MHz();
    UART0_Init();
    LaunchPad_Init();
    Bump_Init();
    Motor_Init();
    Nokia5110_Init();
    LCDClear();

    Tachometer_Init();

    // user TimerA1 to run the controller at 100 Hz
    const uint16_t period_2us = 10000;              // T = 20ms
    TimerA1_Init(&Controller, period_2us);    // f = 50 Hz
    const uint16_t LcdUpdateRate = 5;         // f/5 = 50/5 = 10 Hz

    BufferIndex = 0;
    IsControllerEnabled = 0;

    EnableInterrupts();

    // foreground thread.
    while(1) {

        // low power mode while waiting for the next interrupt.
        WaitForInterrupt();

        // Nokia5110 is a slow device.
        // Do not add LCDOut inside ISR.
        // Updated LCD at 10 Hz.
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

        TxBuffer();

        LCDClear();

        BufferIndex = 0;
        // Enable controller
        IsControllerEnabled = 1;
  }
}
