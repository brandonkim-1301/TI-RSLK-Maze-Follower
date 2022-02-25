/*
 * Classifier.h
 * Runs on MSP432
 *
 *
 * Conversion function for a GP2Y0A21YK0F IR sensor and classification function
 *  to take 3 distance readings and determine what the current state of the robot is.
 *
 *  Created on: Jul 24, 2020
 *  Author: Captain Steven Beyer
 *
 *  Modified on: Jul 20, 2021
 *  Author: Dr. Stan Baek
 *
 */

enum scenario {
    Error = 0,
    LeftTooClose = 1,
    RightTooClose = 2,
    CenterTooClose = 4,
    Straight = 8,
    LeftTurn = 9,
    RightTurn = 10,
    TeeJoint = 11,
    LeftJoint = 12,
    RightJoint = 13,
    CrossRoad = 14,
    Blocked = 15
};

typedef enum scenario scenario_t;

/**
 * <b>Calculate the distance in mm given the 14-bit ADC value, n.</b>:<br>
 * D = 195172/(n - 1058)
 * The maximum measurement distance for the sensor is 800 mm,
 * so if the ADC value is less than 2552 (IRMAX), your
 * function should return 800.
 * @param  14-bit ADC value.
 * @return distance in mm.
 * @brief  Calculate the distance in mm given the 14-bit ADC value.
 */
int32_t Convert(int32_t adc_value);


/**
 * <b>Classify</b>:<br>
 * Utilize three distance values from the left, center, and right
 * distance sensors to determine and classify the situation into one
 * of many scenarios (enumerated by scenario)
 * @param left: distance measured in mm by left sensor
 * @param center: distance measured in mm by center sensor
 * @param right: distance measured in mm by right sensor
 * @return enum value representing the scenario the robot
 * @brief  Calculate the distance in mm given the 14-bit ADC value.
 */
scenario_t Classify(int32_t left_mm, int32_t center_mm, int32_t right_mm);

