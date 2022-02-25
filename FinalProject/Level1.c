#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/TimerA1.h"
#include "../inc/SysTickInts.h"
#include "../inc/Reflectance.h"
#include "../inc/Tachometer.h"


// Macro to return a bounded value
#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ( (X) > (Max)? (Max) : (X) ) )

// Bit-banding for LEDs
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))

// LED configurations
static uint8_t blinkEnabled = 0;   // semaphore for when to blink LED
static const uint16_t oneSecond_ms = 1000; // one second in units of ms
static const uint16_t halfSecond_ms = 500; // half second in units of ms

// Motor Commands for the FSM

enum State {
    Forward,
    Backward,
    Turn,
    Blink
};
static enum State currentState = Forward;   // Initializes first state as Forward

// Motor state configurations
#define ONE_REV 360     // Complete revolution of wheel in deg
#define HALF_TURN 220   // Wheel displacement for half turn in mm,
                        // assuming robot is 140mm wide and .5*circumference is 220mm

// Conversion for wheels from deg to mm
#define STEPS2DISTANCE 220/360

// PWM Configurations
#define PWMNOMINAL 4000
#define SWING 3000
#define PWMIN (PWMNOMINAL-SWING)
#define PWMAX (PWMNOMINAL+SWING)

// Proportional controller settings
static uint8_t LineData;       // Direct measure from line sensor
static int32_t Position;       // Position relative to center of line in 0.1mm
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
static int32_t prevLeftSteps_deg;      // Last measurement of left wheel steps in forward dir
static int32_t prevRightSteps_deg;     // Last measurement of right wheel steps in forward dir


static uint16_t Time_ms = 0; // Timer for Forward in ms

// Proportional controller to drive robot using line following
static void Controller(void){

//    static uint16_t Time_ms = 0; // Timer for Forward in ms

    switch(currentState) {
        case Forward:

            // Transitions to Backward or Blink if bumps into wall
            if(Bump_Read()) {

                // If the blink LED semaphore is high, transition to Blink
                if(blinkEnabled) {
                    currentState = Blink;
                    return;
                }

                //Otherwise, continue with Backward
                currentState = Backward;

                // Resets Tacho Steps
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

            // Finds the position
            Position = Reflectance_Position(LineData);

            // Updates duty cycle based on proportional control
            rightDuty_permyriad = Kp*Position;
            leftDuty_permyriad = -1*Kp*Position;

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

            // Transitions to Turn if each wheel completes a full revolution
            if((-1*LeftSteps_deg > ONE_REV) && (-1*RightSteps_deg > ONE_REV)) {

                currentState = Turn;

                // Reset Tacho Steps
                Tachometer_ResetSteps();

                return;
            }

            return;

        case Turn:

            // Sets motor duty cycles to nominal
            leftDuty_permyriad = PWMNOMINAL;
            rightDuty_permyriad = PWMNOMINAL;

            // Updates motor speeds
            Motor_TurnRight(leftDuty_permyriad, rightDuty_permyriad);

            // Gets values from the tachometer and store into variables
            Tachometer_Get(&LeftTachPeriod, &LeftDir, &LeftSteps_deg,
                           &RightTachPeriod, &RightDir, &RightSteps_deg);

            // Transitions to Forward if each robot completes a half-turn
            if((LeftSteps_deg*STEPS2DISTANCE > HALF_TURN) &&
                    (-1*RightSteps_deg*STEPS2DISTANCE > HALF_TURN)) {

                currentState = Forward;

                // Toggles blink LED semaphore
                blinkEnabled ^= 0x01;

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

void Level1(void) {

    // Initializes clock, launchpad, sensors, and motors
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Bump_Init();
    Reflectance_Init();
    Motor_Init();
    Tachometer_Init();

    // Sets a user TimerA1 to run the controller at 1000 Hz
    uint16_t period_2us = 500;              // T = 1ms
    TimerA1_Init(&Controller, period_2us);  // f = 1000 Hz controller loop

    // Wait until SW1 is pressed
    while(LaunchPad_Input() != 0x01) {
        LEDOUT ^= 0x01;
        Clock_Delay1ms(1000);
    }

    // Enables interrupts
    EnableInterrupts();

    // Runs through line-sensor FSM
    while(1);
}
