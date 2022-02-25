#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/ADC14.h"
#include "../inc/TimerA1.h"
#include "../inc/SysTickInts.h"
#include "../inc/Reflectance.h"
#include "../inc/Tachometer.h"
#include "../inc/Classifier.h"
#include "../inc/IRDistance.h"
#include "../inc/LPF.h"


// Macro to return a bounded value
#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ( (X) > (Max)? (Max) : (X) ) )

// Bit-banding for LEDs
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))

// LED configurations
static const uint16_t oneSecond_ms = 1000; // one second in units of ms
static const uint16_t halfSecond_ms = 500; // half second in units of ms

// Motor Commands for the FSM
enum State {
    Forward,
    TurnRight,
    TurnLeft,
    Blink
};
static enum State currentState = Forward;   // Initializes first state as Forward

// Motor configurations
#define TURN 110        // Wheel displacement for 90-deg turn in mm,
                        // assuming robot is 140mm wide and .25*circumference is 110mm

// Conversion for wheels from deg to mm
#define STEPS2DISTANCE 220/360

// Line sensor data
static uint8_t LineData;       // Direct measure from line sensor

// PWM Configurations
#define PWMNOMINAL 3000
#define SWING 2000
#define PWMIN (PWMNOMINAL-SWING)
#define PWMAX (PWMNOMINAL+SWING)

// Proportional controller settings
#define TOOCLOSECEN 150     // Center distance if too close to wall in mm
#define TOOCLOSESIDE 225    // Left/Right distance if too close to wall in mm
#define DES_DIS 220     // Desired distance from right wall in mm
static int32_t disError;       // Distance error relative to right wall in mm
static int16_t leftDuty_permyriad = PWMNOMINAL;
static int16_t rightDuty_permyriad = PWMNOMINAL;
static int32_t Kp = 10;        // Proportional controller gain

// Tachometer measurements
#define TACHBUFF_SIZE 10
static uint16_t LeftTachPeriod;        // Period of pulse on left wheel
static uint16_t RightTachPeriod;       // Period of pulse on right wheel
static enum TachDirection LeftDir;     // Direction of left wheel
static enum TachDirection RightDir;    // Direction of right wheel
static int32_t LeftSteps_deg;          // Left wheel steps in deg
static int32_t RightSteps_deg;         // Right wheel steps in deg

static uint16_t Time_ms = 0;           // Timer in ms

// Checks for the Treasure
// Assumes that there is line data and that it should not turn
static uint8_t checkTreasure(uint8_t lineData) {

    // If line data appears to be "random"
    return (lineData&0x3C) && (lineData&0xC0) && (lineData&0x03) && (lineData&0xFF);

}

// Reads and converts IR sensor distances
static int32_t disLeft, disCenter, disRight;   // IR distances in mm
void IRsampling(void){
    uint32_t raw17, raw14, raw16;
    ADC_In17_14_16(&raw17, &raw14, &raw16);
    uint32_t nr = LPF_Calc(raw17);
    uint32_t nc = LPF_Calc2(raw14);
    uint32_t nl = LPF_Calc3(raw16);
    disLeft = LeftConvert(nl);
    disCenter = CenterConvert(nc);
    disRight = RightConvert(nr);
}

// Proportional controller to drive robot using line following
void SysTick_Handler(void){

    // Finite State Machine
    switch(currentState) {
        case Forward:

            // Sets for a transition if it approaches a wall
            if(disCenter<TOOCLOSECEN) {

                // Transition to right turn, assuming it enters a tee-joint
                currentState = TurnRight;

                // Transition to left turn if it enters left-joint
                if(disRight<TOOCLOSESIDE) currentState = TurnLeft;

                // Resets tachometer steps
                Tachometer_ResetSteps();

                return;
            }

            // Increments the Timer_ms
            Time_ms++;

            // Does nothing for Timer_ms = 0,1,...,8ms
            if (Time_ms < 9) {
                return;
            }

            // Starts Reflectance for Timer_ms = 9ms
            if (9 == Time_ms) { // start Reflectance and return
                Reflectance_Start();
                return;
            }

            // Resets Time_ms when at 10ms
            Time_ms = 0;

            // Reads line data for a white line
            LineData = ~Reflectance_End();

            // Transitions to Blink if treasure detected
            if(checkTreasure(LineData)) {
                currentState = Blink;
            }

            // Calculates error distance from right wall
            disError = DES_DIS - disRight;

            // Updates duty cycle based on proportional control
            rightDuty_permyriad = PWMNOMINAL + Kp*disError;
            leftDuty_permyriad = PWMNOMINAL - Kp*disError;

            // Checks to ensure the speeds are bounded
            rightDuty_permyriad = MINMAX(PWMIN, PWMAX, rightDuty_permyriad);
            leftDuty_permyriad = MINMAX(PWMIN, PWMAX, leftDuty_permyriad);

            // Updates motor speeds
            Motor_Forward(leftDuty_permyriad, rightDuty_permyriad);

            return;

        case TurnRight:

            // Sets motor duty cycles to nominal
            leftDuty_permyriad = PWMNOMINAL;
            rightDuty_permyriad = PWMNOMINAL;

            // Updates motor speeds
            Motor_TurnRight(leftDuty_permyriad, rightDuty_permyriad);

            // Gets values from the tachometer and store into variables
            Tachometer_Get(&LeftTachPeriod, &LeftDir, &LeftSteps_deg,
                           &RightTachPeriod, &RightDir, &RightSteps_deg);

            // Transitions to Forward if robot completes a 90-deg right turn
            if((LeftSteps_deg*STEPS2DISTANCE > TURN) &&
                    (-1*RightSteps_deg*STEPS2DISTANCE > TURN)) {

                currentState = Forward;

                return;
            }

            return;

        case TurnLeft:

            // Sets motor duty cycles to nominal
            leftDuty_permyriad = PWMNOMINAL;
            rightDuty_permyriad = PWMNOMINAL;

            // Updates motor speeds
            Motor_TurnLeft(leftDuty_permyriad, rightDuty_permyriad);

            // Gets values from the tachometer and store into variables
            Tachometer_Get(&LeftTachPeriod, &LeftDir, &LeftSteps_deg,
                           &RightTachPeriod, &RightDir, &RightSteps_deg);

            // Transitions to Forward if robot completes a 90-deg left turn
            if((RightSteps_deg*STEPS2DISTANCE > TURN) &&
                    (-1*LeftSteps_deg*STEPS2DISTANCE > TURN)) {

                currentState = Forward;

                return;
            }

            return;

        case Blink:

            // Turns off motors
            Motor_Coast();

            // Increments Time_ms
            Time_ms++;

            // Flashes red and blue LED alternately, every half sec
            if(Time_ms % oneSecond_ms >= halfSecond_ms) {
                REDLED &= 0;
                BLUELED |= 1;
            } else {
                REDLED |= 1;
                BLUELED &= 0;
            }

            return;

        default:
            currentState = Blink;
            return;
    }

}

void Level3(void) {

    // Initializes clock, launchpad, sensors, and motors
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Bump_Init();
    Motor_Init();
    Reflectance_Init();
    Tachometer_Init();

    // Waits until SW1 is pressed
    while(LaunchPad_Input() != 0x01) {
        LEDOUT ^= 0x01;
        Clock_Delay1ms(1000);
    }

    // Uses TimerA1 to sample the IR sensors at 2 Hz
    uint16_t period_2us = 250;      // T = 0.5ms
    TimerA1_Init(&IRsampling, period_2us);  // f = 2 kHz sampling

    // Initializes ADC channels 17,12,16
    ADC0_InitSWTriggerCh17_14_16();
    uint32_t raw17,raw14,raw16;
    ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
    LPF_Init(raw17,64);     // P9.0/channel 17
    LPF_Init2(raw14,64);    // P4.1/channel 12
    LPF_Init3(raw16,64);    // P9.1/channel 16

    // Uses SysTick to run the controller at 1000 Hz with a priority of 2
    uint32_t period_48th_us = 48000;   // T = 1000us = 1ms
    SysTick_Init(period_48th_us, 2);    // f = 1000 Hz

    // Enables interrupts
    EnableInterrupts();

    // Runs through line-sensor FSM
    while(1) WaitForInterrupt();
}
