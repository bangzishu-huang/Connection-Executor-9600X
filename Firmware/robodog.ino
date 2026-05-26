#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#define SWITCH_PIN 19

#if defined(ESP32)
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN     100
#define SERVOMAX     500
#define CENTER       325
#define NUM_SERVOS   8
#define DELAY_TIME   5
#define SPEED_FACTOR 0.5f
#define MAX_STEP     8
#define MOVE_STEPS   40

#define TRIG_PIN 12
#define ECHO_PIN 13

#define NECK_SERVO 15


const float FREQUENCY = 0.006f;
const float AMPLITUDE = 70.0f;

const float off_set_walk[NUM_SERVOS] = { 0, -100, 0, 100, -50, -50, 50, 50 };

const float phase[NUM_SERVOS] = {
  PI  + PI/8,       PI/2 + PI  + PI/8,
  0   + PI/8,       PI/2       + PI/8,
  0   - PI/8,       PI/2       - PI/8,
  PI  - PI/8,       PI/2 + PI  - PI/8
};

unsigned long currentMillis = 0;
unsigned long oldMillis     = 0;
unsigned long innerTime     = 0;


float currentPos[NUM_SERVOS];
float targetPos[NUM_SERVOS];
int   lastPulse[NUM_SERVOS];

unsigned long lastCheck       = 0;
unsigned long lastBackwardScan = 0;
long distFwd   = 0;
long distLeft  = 0;
long distRight = 0;


enum DogState {
  STATE_FORWARD,
  STATE_LEFT,
  STATE_RIGHT,
  STATE_BACKWARD
};

DogState state = STATE_FORWARD;


String readCommand() {
  String input = "";
  if (Serial.available() > 0)
    input = Serial.readStringUntil('\n');

#if defined(ESP32)
  else if (SerialBT.available() > 0)
    input = SerialBT.readStringUntil('\n');
#endif

  input.trim();
  return input;
}



long readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return 999;

  long distance = duration * 0.034 / 2;

  if (distance <= 0) return 999;

  return distance;
}



void moveNeckSlow(int from, int to, int steps = 10, int delayMs = 50) {
  float step = (to - from) / (float)steps;
  for (int i = 0; i <= steps; i++) {
    pwm.setPWM(NECK_SERVO, 0, from + step * i);
    delay(delayMs);
  }
}



void returnNeckToCenter() {
  pwm.setPWM(NECK_SERVO, 0, CENTER);
  delay(200);
}



void runWalkSequence(const float ampScale[NUM_SERVOS]) {
  oldMillis     = currentMillis;
  currentMillis = millis();
  innerTime    += currentMillis - oldMillis;

  float t = FREQUENCY * innerTime;

  for (int s = 0; s < NUM_SERVOS; s += 2) {
    int pulse = CENTER + (int)off_set_walk[s]
              + (int)(ampScale[s] * AMPLITUDE * cosf(t + phase[s]));
    pwm.setPWM(s, 0, pulse);
    lastPulse[s] = pulse;
  }
  for (int s = 1; s < NUM_SERVOS; s += 2) {
    float c = fmaxf(0.0f, cosf(t + phase[s]));
    int pulse = CENTER + (int)off_set_walk[s]
              + (int)(ampScale[s] * AMPLITUDE * c);
    pwm.setPWM(s, 0, pulse);
    lastPulse[s] = pulse;
  }

  delay(DELAY_TIME);
}

void runWalkForward()  { static const float a[NUM_SERVOS] = {  1.f,  1.f, -1.f, -1.f,  1.f,  1.f, -1.f, -1.f }; runWalkSequence(a); }
void runWalkLeft()     { static const float a[NUM_SERVOS] = { 1.5f, 1.5f, -.2f, -.2f, 1.5f, 1.5f, -.2f, -.2f }; runWalkSequence(a); }
void runWalkRight()    { static const float a[NUM_SERVOS] = {  .2f,  .2f,-1.5f,-1.5f,  .2f,  .2f,-1.5f,-1.5f }; runWalkSequence(a); }
void runWalkBackward() { static const float a[NUM_SERVOS] = { -1.f, -1.f,  1.f,  1.f, -1.f, -1.f,  1.f,  1.f }; runWalkSequence(a); }

void moveUntilReachedAll() {
  float startPos[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) startPos[i] = currentPos[i];

  for (int step = 1; step <= MOVE_STEPS; step++) {
    float t = (float)step / (float)MOVE_STEPS;
    for (int i = 0; i < NUM_SERVOS; i++)
      currentPos[i] = startPos[i] + t * (targetPos[i] - startPos[i]);
    for (int i = 0; i < NUM_SERVOS; i++)
      pwm.setPWM(i, 0, (int)currentPos[i]);
    delay(DELAY_TIME);
  }
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPos[i] = targetPos[i];
    lastPulse[i]  = (int)targetPos[i];
    pwm.setPWM(i, 0, (int)currentPos[i]);
  }
}

void resetStance() {
  for (int i = 0; i < NUM_SERVOS; i++) targetPos[i] = CENTER;
  moveUntilReachedAll();
}



void setup() {
  Serial.begin(115200);

#if defined(ESP32)
  SerialBT.begin("ESP32_RobotDog");
  Serial.println("Bluetooth started. Device name: ESP32_RobotDog");
#endif

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPos[i] = CENTER;
    targetPos[i]  = CENTER;
    lastPulse[i]  = CENTER;
    pwm.setPWM(i, 0, CENTER);
  }

  pwm.setPWM(NECK_SERVO, 0, CENTER);

  resetStance();

  Serial.println("Robot dog autonomous mode started.");
#if defined(ESP32)
  SerialBT.println("Robot dog autonomous mode started.");
#endif
}




void loop() {
  if (digitalRead(SWITCH_PIN) == HIGH) {
    // Switch is OFF — hold still and do nothing
    return;
  }

  // Switch is ON — rest of your existing loop goes here unchanged
  switch (state) {
    case STATE_FORWARD:
      runWalkForward();

      if (millis() - lastCheck >= 1000) {
        lastCheck = millis();

        distFwd = readDistance();
        Serial.print("FORWARD - Distance: ");
        Serial.println(distFwd);

#if defined(ESP32)
        SerialBT.print("FORWARD - Distance: ");
        SerialBT.println(distFwd);
#endif

        if (distFwd < 30) {
          Serial.println("Obstacle detected! Scanning...");

          // Look RIGHT
          moveNeckSlow(CENTER, CENTER + 80, 10, 30);
          delay(300);
          distRight = readDistance();
          Serial.print("Right distance: ");
          Serial.println(distRight);

          if (distRight > 30) {
            Serial.println("Going RIGHT");
            returnNeckToCenter();
            state = STATE_RIGHT;
            break;
          }

          // Look LEFT
          moveNeckSlow(CENTER + 80, CENTER - 80, 16, 30);
          delay(300);
          distLeft = readDistance();
          Serial.print("Left distance: ");
          Serial.println(distLeft);

          if (distLeft > 30) {
            Serial.println("Going LEFT");
            returnNeckToCenter();
            state = STATE_LEFT;
            break;
          }

          // Both blocked
          Serial.println("Both blocked! Going BACKWARD");
          returnNeckToCenter();
          state = STATE_BACKWARD;
          lastBackwardScan = millis();
        }
      }
      break;

    // ====================================================================
    //                           RIGHT STATE
    // ====================================================================
    case STATE_RIGHT:
      runWalkRight();

      if (millis() - lastCheck >= 200) {
        lastCheck = millis();
        distFwd = readDistance();

        Serial.print("RIGHT - Forward distance: ");
        Serial.println(distFwd);

        if (distFwd > 60) {
          Serial.println("Path clear -> Going FORWARD");
          returnNeckToCenter();
          state = STATE_FORWARD;
        }
      }
      break;

    case STATE_LEFT:
      runWalkLeft();

      if (millis() - lastCheck >= 200) {
        lastCheck = millis();
        distFwd = readDistance();

        Serial.print("LEFT - Forward distance: ");
        Serial.println(distFwd);

        if (distFwd > 60) {
          Serial.println("Path clear -> Going FORWARD");
          returnNeckToCenter();
          state = STATE_FORWARD;
        }
      }
      break;

    case STATE_BACKWARD:
      runWalkBackward();

      if (millis() - lastBackwardScan >= 2000) {
        lastBackwardScan = millis();

        Serial.println("BACKWARD - Scanning for exit...");

        // Try RIGHT
        moveNeckSlow(CENTER, CENTER + 80, 10, 30);
        delay(300);
        distRight = readDistance();
        Serial.print("Right distance: ");
        Serial.println(distRight);

        if (distRight > 30) {
          Serial.println("Right path found! Going RIGHT");
          returnNeckToCenter();
          state = STATE_RIGHT;
          break;
        }

        // Try LEFT
        moveNeckSlow(CENTER + 80, CENTER - 80, 16, 30);
        delay(300);
        distLeft = readDistance();
        Serial.print("Left distance: ");
        Serial.println(distLeft);

        if (distLeft > 30) {
          Serial.println("Left path found! Going LEFT");
          returnNeckToCenter();
          state = STATE_LEFT;
          break;
        }

        returnNeckToCenter();
        Serial.println("No exit found, continuing BACKWARD");
      }
      break;
  }
}
      
