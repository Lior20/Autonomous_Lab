# Zumo32U4 Line Following and Obstacle Avoidance Robot

## Table of Contents
1. [Overview](#overview)
2. [Constants and Global Variables](#constants-and-global-variables)
3. [Object Instantiation](#object-instantiation)
4. [Helper Functions](#helper-functions)
5. [Main Control Functions](#main-control-functions)
6. [Setup Function](#setup-function)
7. [Main Loop](#main-loop)
   
## Overview

This project implements a line-following robot with obstacle avoidance capabilities using the Zumo32U4 robot platform. The robot uses various sensors to navigate along a line and detect obstacles, switching between line-following and obstacle avoidance modes as needed.

## Features

- Line following using reflectance sensors
- Obstacle detection using proximity sensors
- Autonomous switching between line-following and obstacle avoidance modes
- Real-time odometry calculation
- Data logging for analysis and debugging

## Constants and Global Variables

```cpp
const uint16_t maxSpeed = 300;
const uint16_t obstacleThreshold = 5;
const uint8_t NUM_SENSORS = 5;

int16_t lastError = 0;
unsigned int lineSensorValues[NUM_SENSORS];
enum State { LINE_FOLLOWING, OBSTACLE_AVOIDANCE };
State currentState = LINE_FOLLOWING;
uint32_t stateStartTime = 0;
float odometryX = 0, odometryY = 0, odometryTheta = 0;
uint32_t lastUpdateTime = 0;
int16_t lastLeftCount = 0, lastRightCount = 0;
```
These constants and variables define the robot's operational parameters and maintain its state information.

## Object Instantiation
```cpp
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonB buttonB;
Zumo32U4OLED display;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Encoders encoders;
```
These objects provide interfaces to various components of the Zumo32U4 robot.

## Helper Functions
loadCustomCharacters() - Loads custom characters for the OLED display to show bar graphs.
printBar(uint8_t height) - Prints a bar on the OLED display with the specified height.
calibrateSensors() - Calibrates the line sensors by rotating the robot in place.
showReadings() - Displays the current line sensor readings on the OLED display.
updateOdometry() - Updates the robot's estimated position and orientation based on wheel encoder data.
recordData() - Logs various data points including time, state, line position error, and odometry information.

## Main Control Functions
lineFollow() - Implements the line-following algorithm using a PID-like control system.
turnRight(), turnLeft(), forward(int path) - Helper functions for obstacle avoidance, implementing basic movement patterns.
avoidObstacle() - Executes a predefined sequence of movements to avoid an obstacle.

## Setup Function
```cpp
void setup()
```
Initializes sensors, calibrates the line sensors, and prepares the robot for operation. It includes:
- Sensor initialization
- Custom character loading for the display
- Line sensor calibration
- Initial display messages and sounds

## Main Loop
```cpp
void loop()
```
The main control loop of the robot. It performs the following tasks:
- Updates odometry
- Records data
- Reads proximity sensors
- Determines the current state (LINE_FOLLOWING or OBSTACLE_AVOIDANCE)
- Executes the appropriate behavior based on the current state

## State Transition Logic
- Switches to OBSTACLE_AVOIDANCE if an obstacle is detected while line following
- Returns to LINE_FOLLOWING after a set duration in OBSTACLE_AVOIDANCE mode

## Additional Notes
- The code uses the Zumo32U4 library extensively for interfacing with the robot's hardware.
- The obstacle avoidance algorithm is relatively simple and could be improved for more complex environments.
-The odometry calculations provide a basic estimate of the robot's position and orientation, but may accumulate errors over time.
- Data logging is implemented for debugging and analysis purposes, outputting data through the Serial interface.
