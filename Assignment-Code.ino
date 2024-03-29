/** This example uses the Zumo's line sensors to detect the white
border around a sumo ring.  When the border is detected, it
backs up and turns. */

#include <Wire.h>
#include <Zumo32U4.h>
#include <Zumo32U4LineSensors.h>


// This might need to be tuned for different lighting conditions,
// surfaces, etc.
#define QTR_THRESHOLD     800  // microseconds
// #define TURN_180_DURATION  700 // ms, this is an example value

// These might need to be tuned for different motor types.
#define REVERSE_SPEED     80  // 0 is stopped, 400 is full speed
#define TURN_SPEED        150
#define FORWARD_SPEED     100
#define REVERSE_DURATION  80  // ms
#define TURN_DURATION     250  // ms
#define TURN_180_DURATION 200 //I've set it a bit higher
#define LONGER_REVERSE_DURATION  400  // ms

// #define TRIGGER_PIN       4   // Ultrasonic sensor trigger pin
// #define ECHO_PIN          5   // Ultrasonic sensor echo pin


// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4OLED display;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

bool lastTurnWasRight = false; // Initially set to false so the first turn will be to the right

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];


bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;

void waitForButtonAndCountDown()
{
  ledYellow(1);
  display.clear();
  display.print(F("Press A"));

  buttonA.waitForButton();

  ledYellow(0);
  display.clear();

  // Play audible countdown.
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);
  delay(1000);
}
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

void setup()
{
  // Uncomment if necessary to correct motor directions:
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
  calibrateSensors();
  lineSensors.initThreeSensors();
  waitForButtonAndCountDown();
  proxSensors.initThreeSensors();
  // waitForButtonAndCountDown();
}

void loop() {
  if (buttonA.isPressed()) {
    // Stop the robot and wait for the next button press.
    motors.setSpeeds(0, 0);
    buttonA.waitForRelease();
    waitForButtonAndCountDown();
  }


  lineSensors.read(lineSensorValues);
  proxSensors.read();


  uint16_t proxLeftActive = proxSensors.countsLeftWithLeftLeds();
  uint16_t proxFrontActive = proxSensors.countsFrontWithLeftLeds();
  uint16_t proxRightActive = proxSensors.countsFrontWithRightLeds();
  uint16_t proxRightRightActive = proxSensors.countsRightWithRightLeds();


  Serial.print(proxLeftActive);
  Serial.print(proxFrontActive);
  Serial.print(proxRightActive);
  Serial.println(proxRightRightActive);


 if(proxFrontActive == 6){
    motors.setSpeeds(0, 0);
    // buzzer.playNote(NOTE_G(4), 500, 15); // Sound feedback after the turn
    buzzer.playFrequency(3000, 1000, 20); 
    delay(3000); // Pause for a moment to ensure a full stop
    // Start a longer reverse movement
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(LONGER_REVERSE_DURATION); // Reverse for a longer duration for more distance
    
    // Turn degrees
    if (lastTurnWasRight) {
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED); // Turn left to complete degrees
      lastTurnWasRight = false;
    } else {
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED); // Turn right to complete degrees
      lastTurnWasRight = true;
    }
    delay(TURN_180_DURATION); // Duration to complete the 180-degree turn
}

  // Adjusted condition to check for corners more effectively
  if ((lineSensorValues[0] > QTR_THRESHOLD && lineSensorValues[1] > QTR_THRESHOLD) || 
      (lineSensorValues[1] > QTR_THRESHOLD && lineSensorValues[2] > QTR_THRESHOLD)) {
    // Perform a corner escape maneuver by reversing and then turning 180 degrees.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);

    if (lastTurnWasRight) {
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED); // Turn left
      lastTurnWasRight = false;
    } else {
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED); // Turn right
      lastTurnWasRight = true;
    }
    delay(TURN_180_DURATION);
  } else if (lineSensorValues[0] > QTR_THRESHOLD) {
    // If the left sensor detects a line, turn right.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
  } else if (lineSensorValues[2] > QTR_THRESHOLD) {
    // If the right sensor detects a line, turn left.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
  } else {
    // No line detected under any sensor, proceed forward.
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
}