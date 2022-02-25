
#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/UART0.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/ADC14.h"
#include "../inc/TimerA1.h"
#include "../inc/IRDistance.h"
#include "../inc/Nokia5110.h"
#include "../inc/LPF.h"
#include "../inc/SysTickInts.h"
#include "../inc/Tachometer.h"


// Macro to return a bounded value
#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ( (X) > (Max)? (Max) : (X) ) )

// bit-banding for LEDs
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define GREENLED (*((volatile uint8_t *)(0x42098064)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))

// proportional controller gain
// experimentally determine value that creates a stable system
// Kp for various responses: 3 (slow), 10 (fast w/o oscillation), 100 (fast w/ oscillation)
static int32_t Kp = 5;


volatile static uint8_t IsControllerEnabled = 0;

// If controller is executed, run LCDOut.
volatile static uint8_t NumControllerExecuted = 0;

volatile static uint8_t IsActuatorEnabled = 0;

#define ERROR_BUFF_SIZE  2000 // for 20 second data
static int32_t ErrorBuffer[ERROR_BUFF_SIZE];
static uint16_t ErrorBuffIndex = 0;

// update Kp and Ki in real-time.
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
    while(!Bump_Read()) {
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


static void EnableActuator(void) {

    while(LaunchPad_Input() || Bump_Read()) { // wait for release
        Clock_Delay1ms(200); LaunchPad_Output(0); // off
        Clock_Delay1ms(200); LaunchPad_Output(1); // red
    }

    Nokia5110_Clear();
    Nokia5110_SetCursor(0,0); Nokia5110_OutString("Motor:");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("S2: On/Off");
    Nokia5110_SetCursor(0,3);  Nokia5110_OutString("Bump to Exit");

    // Until bump is pressed.
    while(!Bump_Read()) {

        // update the screen
        if (IsActuatorEnabled) {
            Nokia5110_SetCursor(7,0);
            Nokia5110_OutString("ON ");
        } else {
            Nokia5110_SetCursor(7,0);
            Nokia5110_OutString("OFF");
        }

        if (LaunchPad_Input() & 0x02 ) { // SW2 is pressed
            IsActuatorEnabled ^= 1;
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
    }

    for(int k = 0; k < 10; k++){
        LaunchPad_Output(0x03);
        Clock_Delay1ms(100);
        LaunchPad_Output(0x00);
        Clock_Delay1ms(100);
    }
}


int32_t Left, Center, Right; // IR distances in mm
int32_t Error = 0;

static void LCDClear(void) {

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);
    Nokia5110_Clear(); // erase entire display
    Nokia5110_OutString("17.3:Kp");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("IR distance");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("L= ");
    Nokia5110_OutUDec(0); Nokia5110_OutString(" mm");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("C= ");
    Nokia5110_OutUDec(0); Nokia5110_OutString(" mm");
    Nokia5110_SetCursor(0,4); Nokia5110_OutString("R= ");
    Nokia5110_OutUDec(0); Nokia5110_OutString(" mm");
    Nokia5110_SetCursor(0,5); Nokia5110_OutString("E= ");
    Nokia5110_OutUDec(0); Nokia5110_OutString(" mm");
}

static void LCDOut(void){
    Nokia5110_SetCursor(7,0); Nokia5110_OutUDec(Kp);
    Nokia5110_SetCursor(3,2); Nokia5110_OutSDec(Left);
    Nokia5110_SetCursor(3,3); Nokia5110_OutSDec(Center);
    Nokia5110_SetCursor(3,4); Nokia5110_OutSDec(Right);
    Nokia5110_SetCursor(3,5); Nokia5110_OutSDec(Error);
}


void IRsampling(void){
    uint32_t raw17, raw14, raw16;
    ADC_In17_14_16(&raw17, &raw14, &raw16);
    uint32_t nr = LPF_Calc(raw17);
    uint32_t nc = LPF_Calc2(raw14);
    uint32_t nl = LPF_Calc3(raw16);
    Left = LeftConvert(nl);
    Center = CenterConvert(nc);
    Right = RightConvert(nr);
}


/**************Program17_3******************************************/
// Proportional controller to drive between two walls using IR sensors
// distances in mm

// constants for Controller
#define TOOCLOSE 110
#define DESIRED_DIST 172
#define TOOFAR 230
#define PWMNOMINAL 3500
#define SWING 1000
#define PWMIN (PWMNOMINAL-SWING)
#define PWMAX (PWMNOMINAL+SWING)

// Proportional controller to keep robot in
// center of two walls using IR sensors.
// 100 Hz
void SysTick_Handler(void){

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

    int32_t SetPoint;
    // Determine set point
    if (Left > DESIRED_DIST && Right > DESIRED_DIST ) {
        SetPoint = (Right + Left) / 2;                             // average of the two.
    } else {
        SetPoint = DESIRED_DIST;
    }

    // set error based off set point
    // if the robot is too close to the left wall, the error should be negative
    // if the robot is too close to the right wall, the error should be positive
    if (Left < Right) {
        Error = Left - SetPoint;
    } else {
        Error = SetPoint - Right;
    }

    // update duty cycle based on proportional control
    uint32_t rightDuty_permyriad = -Kp*Error/50;
    uint32_t leftDuty_permyriad = Kp*Error/50;

    // check to ensure not too big of a swing
    // You can use MINMAX defined in Line 60
    rightDuty_permyriad = MINMAX(PWMIN, PWMAX, rightDuty_permyriad);
    leftDuty_permyriad = MINMAX(PWMIN, PWMAX, leftDuty_permyriad);

    if(IsActuatorEnabled) {
        Motor_Forward(leftDuty_permyriad, rightDuty_permyriad);

        if (ErrorBuffIndex < ERROR_BUFF_SIZE) {
            ErrorBuffer[ErrorBuffIndex++] = Error;
        }
    }

    // Increment the number of controller executed.
    NumControllerExecuted++;

}


// proportional control, wall distance
void Program17_3(void){

    DisableInterrupts();
    Clock_Init48MHz();
    UART0_Init();
    LaunchPad_Init();
    Bump_Init();
    Motor_Init();
    Nokia5110_Init();
    LCDClear();

    // user TimerA1 to sample the IR sensors at 2000 Hz
    uint16_t period_2us = 250;      // T = 0.5ms
    TimerA1_Init(&IRsampling, period_2us);  // f = 2000 Hz sampling

    // initialize ADC channels 17,12,16
    ADC0_InitSWTriggerCh17_14_16();
    uint32_t raw17,raw14,raw16;
    ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
    LPF_Init(raw17,64);     // P9.0/channel 17
    LPF_Init2(raw14,64);    // P4.1/channel 12
    LPF_Init3(raw16,64);    // P9.1/channel 16

    // Use SysTick to run the controller at 100 Hz with a priority of 2
    uint32_t period_48th_us = 480000;   // T = 10000us = 10ms
    SysTick_Init(period_48th_us, 2);    // f = 100 Hz

    // NumControllerExecuted increments at 100 Hz
    // Updated LCD at 10 Hz --> every 10 controller runs
    const uint16_t LcdUpdateRate = 10;    // 100/10 Hz

    IsControllerEnabled = 0;
    NumControllerExecuted = 0;
    IsActuatorEnabled = 1;

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
        EnableActuator();
        TxBuffer();

        LCDClear();

        ErrorBuffIndex = 0;

        // Enable controller
        IsControllerEnabled = 1;
    }
}
