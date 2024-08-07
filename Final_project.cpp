/**
 * Zumo32U4 Line Following and Obstacle Avoidance Robot
 * 
 * This program implements a line-following robot with obstacle avoidance
 * capabilities using the Zumo32U4 robot platform. The robot uses line sensors
 * for following a line, proximity sensors for detecting obstacles, and wheel
 * encoders for odometry.
 */

#include <Wire.h>
#include <Zumo32U4.h>

// Constants
const uint16_t maxSpeed = 300;           // Maximum motor speed
const uint16_t obstacleThreshold = 5;    // Proximity sensor threshold for obstacle detection
const uint8_t NUM_SENSORS = 5;           // Number of line sensors

// Objects
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonB buttonB;
Zumo32U4OLED display;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Encoders encoders;

// Variables
int16_t lastError = 0;                   // Last calculated error for PID control
unsigned int lineSensorValues[NUM_SENSORS];  // Array to store line sensor readings
enum State { LINE_FOLLOWING, OBSTACLE_AVOIDANCE };  // Robot state enum
State currentState = LINE_FOLLOWING;     // Current robot state
uint32_t stateStartTime = 0;             // Time when the current state started
float odometryX = 0, odometryY = 0, odometryTheta = 0;  // Odometry variables
uint32_t lastUpdateTime = 0;             // Last time odometry was updated
int16_t lastLeftCount = 0, lastRightCount = 0;  // Last encoder counts

/**
 * Loads custom characters for the OLED display to show bar graphs.
 */
void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  display.loadCustomCharacter(levels + 0, 0);  // 1 bar
  display.loadCustomCharacter(levels + 1, 1);  // 2 bars
  display.loadCustomCharacter(levels + 2, 2);  // 3 bars
  display.loadCustomCharacter(levels + 3, 3);  // 4 bars
  display.loadCustomCharacter(levels + 4, 4);  // 5 bars
  display.loadCustomCharacter(levels + 5, 5);  // 6 bars
  display.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

/**
 * Prints a bar on the OLED display with the specified height.
 * @param height Height of the bar (0-8)
 */
void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  display.print(barChars[height]);
}

/**
 * Calibrates the line sensors by rotating the robot in place.
 */
void calibrateSensors()
{
  display.clear();

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

/**
 * Displays the current line sensor readings on the OLED display.
 */
void showReadings()
{
  display.clear();

  while(!buttonB.getSingleDebouncedPress())
  {
    lineSensors.readCalibrated(lineSensorValues);

    display.gotoXY(0, 0);
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
      printBar(barHeight);
    }
  }
}

/**
 * Updates the robot's estimated position and orientation based on wheel encoder data.
 */
void updateOdometry()
{
  uint32_t currentTime = millis();
  float deltaTime = (currentTime - lastUpdateTime) / 1000.0;
  lastUpdateTime = currentTime;

  int16_t leftCount = encoders.getCountsLeft();
  int16_t rightCount = encoders.getCountsRight();

  int16_t leftDelta = leftCount - lastLeftCount;
  int16_t rightDelta = rightCount - lastRightCount;

  lastLeftCount = leftCount;
  lastRightCount = rightCount;

  // Convert encoder counts to distance (you may need to adjust these values)
  float distanceLeft = leftDelta * 0.0001;  // Adjust this multiplier based on your wheel size and encoder resolution
  float distanceRight = rightDelta * 0.0001;

  float distance = (distanceLeft + distanceRight) / 2;
  float angleDelta = (distanceRight - distanceLeft) / 0.0865;  // Adjust 0.0865 based on your robot's wheel base width

  odometryX += distance * cos(odometryTheta + angleDelta / 2);
  odometryY += distance * sin(odometryTheta + angleDelta / 2);
  odometryTheta += angleDelta;

  // Normalize theta to be between -pi and pi
  while (odometryTheta > PI) odometryTheta -= 2 * PI;
  while (odometryTheta < -PI) odometryTheta += 2 * PI;
}

/**
 * Logs various data points including time, state, line position error, and odometry information.
 */
void recordData()
{
  static uint32_t lastRecordTime = 0;
  if (millis() - lastRecordTime > 100) { // Record every 100ms
    lastRecordTime = millis();
    
    int16_t position = lineSensors.readLine(lineSensorValues);
    int16_t deviation = position - 2000;

    Serial.print("Time: ");
    Serial.print(millis());
    Serial.print(",");
    Serial.print("Current State: ");
    Serial.print(currentState == LINE_FOLLOWING ? "LINE" : "OBSTACLE");
    Serial.print(",");
    Serial.print(" error: ");
    Serial.print(deviation);
    Serial.print(",");
    Serial.print(" Odometry - X: ");
    Serial.print(odometryX);
    Serial.print(",");
    Serial.print(" Odometry - Y: ");
    Serial.print(odometryY);
    Serial.print(",");
    Serial.print(" encoder left");
    Serial.print(encoders.getCountsLeft());
    Serial.print(",");
    Serial.print(" encoder right");
    Serial.println(encoders.getCountsRight());
  }
}

/**
 * Implements the line-following algorithm using a PID-like control system.
 */
void lineFollow()
{
  int16_t position = lineSensors.readLine(lineSensorValues);
  int16_t error = position - 2000;
  int16_t speedDifference = error / 5 + 2 * (error - lastError);
  lastError = error;

  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);
}

/**
 * Turns the robot to the right.
 */
void turnRight()
{
  motors.setSpeeds(200, -200);
  for (int i = 0; i < 10; i++) {
    delay(42);
    updateOdometry();
    recordData();
  }
  motors.setSpeeds(0, 0);
  delay(300);
  updateOdometry();
  recordData();
}

/**
 * Turns the robot to the left.
 */
void turnLeft()
{
  motors.setSpeeds(-200, 200);
  for (int i = 0; i < 10; i++) {
    delay(35);
    updateOdometry();
    recordData();
  }
  motors.setSpeeds(0, 0);
  delay(300);
  updateOdometry();
  recordData();
}

/**
 * Moves the robot forward for a specified distance.
 * @param path Distance to move forward
 */
void forward(int path)
{
  motors.setSpeeds(300, 300);
  for (int i = 0; i < 10; i++) {
    delay(path/10);
    updateOdometry();
    recordData();
  }
  motors.setSpeeds(0, 0);
  delay(300);
  updateOdometry();
  recordData();
}

/**
 * Executes the obstacle avoidance maneuver.
 */
void avoidObstacle()
{
  Serial.print("AvoidObstacle");

  // Simple obstacle avoidance: turn right for a bit, then go straight
  uint32_t elapsedTime = millis() - stateStartTime;
  motors.setSpeeds(0, 0);
  delay(1000);
  turnRight();
  forward(700);
  turnLeft();
  forward(900);
  turnLeft();
  forward(700);
  turnRight();
  lineFollow();
}

/**
 * Setup function: Initializes sensors, calibrates the line sensors,
 * and prepares the robot for operation.
 */
void setup()
{
  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();

  loadCustomCharacters();

  buzzer.play(">g32>>c32");

  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));
  buttonB.waitForButton();

  calibrateSensors();
  showReadings();

  display.clear();
  display.print(F("Go!"));
  buzzer.play("L16 cdegreg4");

  while(buzzer.isPlaying());

  stateStartTime = millis();
  lastUpdateTime = millis();
}

/**
 * Main loop: Controls the robot's behavior, switching between
 * line-following and obstacle avoidance modes as needed.
 */
void loop()
{
  updateOdometry();
  recordData();

  proxSensors.read();
  int16_t leftSensor = proxSensors.countsFrontWithLeftLeds();
  int16_t rightSensor = proxSensors.countsFrontWithRightLeds();

  // Check for state transitions
  if ((currentState == LINE_FOLLOWING) && ((proxSensors.countsFrontWithLeftLeds() > obstacleThreshold) || (proxSensors.countsFrontWithRightLeds() > obstacleThreshold))) {
    currentState = OBSTACLE_AVOIDANCE;
    stateStartTime = millis();
    Serial.println("Obstacle detected! Switching to OBSTACLE_AVOIDANCE");
  } else if (currentState == OBSTACLE_AVOIDANCE && millis() - stateStartTime > 5000) { // Adjust time as needed
    currentState = LINE_FOLLOWING;
    stateStartTime = millis();
    Serial.println("Switching back to LINE_FOLLOWING");
  }

  // Execute behavior based on current state
  if (currentState == LINE_FOLLOWING) {
    lineFollow();
  } else {
    avoidObstacle();
  }
}
