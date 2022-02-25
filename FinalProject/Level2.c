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
static enum State {
    Forward,
    Backward,
    TurnLeft,
    TurnAround,
    Blink
};
static enum State currentState = Forward;   // Initializes first state as Forward

// Motor state configurations
// Assumes robot is 140mm wide and circumference is 440mm
#define HALFTURN 80     // Wheel displacement for 90-deg turn in mm
#define FULLTURN 220    // Wheel displacement for 180-deg turn in mm
#define BACKREV  50     // Wheel displacement for reversing back in mm

// Conversion for wheels from deg to mm
#define STEPS2DISTANCE 220/360

// PWM Configurations
#define PWMNOMINAL 3000
#define SWING 2000
#define PWMIN (PWMNOMINAL-SWING)
#define PWMAX (PWMNOMINAL+SWING)

// Proportional controller settings
static uint8_t LineData;       // Direct measure from line sensor
static int32_t Position;       // Position relative to center of line in 0.1mm
static int16_t leftDuty_permyriad = PWMNOMINAL;
static int16_t rightDuty_permyriad = PWMNOMINAL;
static int32_t Kp = 3;        // Proportional controller gain

// Tachometer measurements
#define TACHBUFF_SIZE 10
static uint16_t LeftTachPeriod;        // Period of pulse on left wheel
static uint16_t RightTachPeriod;       // Period of pulse on right wheel
static enum TachDirection LeftDir;     // Direction of left wheel
static enum TachDirection RightDir;    // Direction of right wheel
static int32_t LeftSteps_deg;          // Left wheel steps in deg
static int32_t RightSteps_deg;         // Right wheel steps in deg
static int32_t prevLeftSteps_deg;      // Last measurement of left wheel steps in forward dir
static int32_t prevRightSteps_deg;     // Last measurement of right wheel steps in forward dir

// Timer count in ms
static uint16_t Time_ms = 0;

// Counts number of "no-line" reads
static uint8_t noLine = 0;

// Checks for the Treasure
// Assumes that there is line data and that it should not turn left
static uint8_t checkTreasure(uint8_t lineData) {

    // If line data appears to be "random"
    return (lineData&0x3C) && (lineData&0xC0) && (lineData&0x03) && (lineData&0xFF);

}

// Checks for if a left turn is available
static uint8_t checkLeft(uint8_t lineData) {
    return ((lineData&0xF0)>0xC0) && ((lineData&0xF0)<=0xF0);
}

// Proportional controller to drive robot using line following
static void Controller(void){

// Finite State Machine
    switch(currentState) {
        case Forward:

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

            // Finds the position on white line
            Position = Reflectance_Position(LineData);

            // Transitions to Backward if hits a wall or obstacle
            // For Bonus, transitions to Backward if no line is read
//            if(Bump_Read()) {
            if(!LineData) {

                noLine++;

                // If not the fifth consecutive "no-line read"
                if(noLine%5 != 0) return;

                currentState = Backward;

                // Resets tachometer steps
                Tachometer_ResetSteps();

                return;
            }

            // Transitions to TurnLeft if the line cuts to the left
            if(checkLeft(LineData)) {

                currentState = TurnLeft;

                // Stops the motors
                Motor_Coast();

                // Resets tachometer steps
                Tachometer_ResetSteps();

                return;
            }

            // Checks for Treasure
            if(checkTreasure(LineData)) {
                currentState = Blink;
                return;
            }

            // Updates duty cycle based on proportional control
            rightDuty_permyriad = PWMNOMINAL + Kp*Position;
            leftDuty_permyriad = PWMNOMINAL - Kp*Position;

            // Checks to ensure the speeds are bounded
            rightDuty_permyriad = MINMAX(PWMIN, PWMAX, rightDuty_permyriad);
            leftDuty_permyriad = MINMAX(PWMIN, PWMAX, leftDuty_permyriad);

            // Updates motor speeds
            Motor_Forward(leftDuty_permyriad, rightDuty_permyriad);

            return;

        case Backward:

            // Sets motor duty cycles to nominal
            leftDuty_permyriad = PWMNOMINAL;
            rightDuty_permyriad = PWMNOMINAL;

            // Updates motor speeds
            Motor_Backward(leftDuty_permyriad, rightDuty_permyriad);

            // Gets values from the tachometer and store into variables
            Tachometer_Get(&LeftTachPeriod, &LeftDir, &LeftSteps_deg,
                           &RightTachPeriod, &RightDir, &RightSteps_deg);

            // Transitions to TurnLeft if each wheel completes a set distance
            if((-1*LeftSteps_deg > BACKREV) && (-1*RightSteps_deg > BACKREV)) {

                currentState = TurnAround;

                // Resets tachometer steps
                Tachometer_ResetSteps();

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

            // Transitions to Forward if robot completes a 90-deg turn
            if((RightSteps_deg*STEPS2DISTANCE > HALFTURN) &&
                    (-1*LeftSteps_deg*STEPS2DISTANCE > HALFTURN)) {

                currentState = Forward;

                // Resets tachometer steps
                Tachometer_ResetSteps();

                return;
            }

            return;

        case TurnAround:

            // Sets motor duty cycles to nominal
            leftDuty_permyriad = PWMNOMINAL;
            rightDuty_permyriad = PWMNOMINAL;

            // Updates motor speeds
            Motor_TurnLeft(leftDuty_permyriad, rightDuty_permyriad);

            // Gets values from the tachometer and store into variables
            Tachometer_Get(&LeftTachPeriod, &LeftDir, &LeftSteps_deg,
                           &RightTachPeriod, &RightDir, &RightSteps_deg);

            // Transitions to Forward if robot completes a 180-deg turn
            if((RightSteps_deg*STEPS2DISTANCE > FULLTURN) &&
                    (-1*LeftSteps_deg*STEPS2DISTANCE > FULLTURN)) {

                currentState = Forward;

                // Resets tachometer steps
                Tachometer_ResetSteps();

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

void Level2(void) {

    // Initializes clock, launchpad, sensors, and motors
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Bump_Init();
    Reflectance_Init();
    Motor_Init();
    Tachometer_Init();

    // Uses TimerA1 to sample the IR sensors at 1000 Hz
    uint16_t period_2us = 500;              // T = 1ms
    TimerA1_Init(&Controller, period_2us);  // f = 1000 Hz sampling

    // Waits until SW1 is pressed
    while(LaunchPad_Input() != 0x01) {
        LEDOUT ^= 0x01;
        Clock_Delay1ms(1000);
    }

    // Enables interrupts
    EnableInterrupts();

    // Runs through line-sensor FSM
    while(1) WaitForInterrupt();
}
