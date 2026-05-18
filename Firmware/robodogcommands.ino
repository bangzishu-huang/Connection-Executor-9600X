#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#if defined(ESP32)
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#endif

#define SERVOMIN     100
#define SERVOMAX     500
#define CENTER       325
#define NUM_SERVOS   8      
#define NECK_SERVO   15    
#define DELAY_TIME   5     
#define SPEED_FACTOR .5f  
#define MAX_STEP     8      
#define MOVE_STEPS   40    
#define PLOT_MODE    true    


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

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

float currentPos[NUM_SERVOS];
float targetPos[NUM_SERVOS];
int   lastPulse[NUM_SERVOS];  

enum Mode {
  MODE_IDLE,
  MODE_WALK_FORWARD,
  MODE_WALK_LEFT,
  MODE_WALK_RIGHT,
  MODE_SPOT_LEFT,
  MODE_SPOT_RIGHT
};
Mode currentMode = MODE_IDLE;

void moveUntilReachedAll() {
  float startPos[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    startPos[i] = currentPos[i];
  }

  for (int step = 1; step <= MOVE_STEPS; step++) {
    float t = (float)step / (float)MOVE_STEPS;   // 0.0 → 1.0

    for (int i = 0; i < NUM_SERVOS; i++) {
      currentPos[i] = startPos[i] + t * (targetPos[i] - startPos[i]);
    }

    for (int i = 0; i < NUM_SERVOS; i++) {
      lastPulse[i] = (int)currentPos[i];
      pwm.setPWM(i, 0, (int)currentPos[i]);
    }
    plotServos();
    delay(DELAY_TIME);
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPos[i] = targetPos[i];
    lastPulse[i]  = (int)targetPos[i];   // keep lastPulse always in sync
    pwm.setPWM(i, 0, (int)currentPos[i]);
  }
}

void updateAllServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    float diff = targetPos[i] - currentPos[i];
    if (fabsf(diff) > 0.5f) {
      currentPos[i] += constrain(diff * SPEED_FACTOR, -MAX_STEP, (float)MAX_STEP);
    }
  }
  for (int i = 0; i < NUM_SERVOS; i++) {
    pwm.setPWM(i, 0, (int)currentPos[i]);
  }
  delay(DELAY_TIME);
}


void syncCurrentPos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPos[i] = (float)lastPulse[i];
    targetPos[i]  = (float)lastPulse[i];   // avoid phantom motion
  }
}


String readCommand() {
  String input = "";
  if (Serial.available() > 0) {
    input = Serial.readStringUntil('\n');
  }
#if defined(ESP32)
  else if (SerialBT.available() > 0) {
    input = SerialBT.readStringUntil('\n');
  }
#endif
  input.trim();
  return input;
}

void sendReply(const String &msg) {
#if !PLOT_MODE
  Serial.println(msg);
#endif
#if defined(ESP32)
  SerialBT.println(msg);
#endif
}


void plotServos() {
#if PLOT_MODE
  Serial.print("Servo0:");
  Serial.print(lastPulse[0]);
  Serial.print(",Servo1:");
  Serial.println(lastPulse[1]);
#endif
}


void parseManualCommand(String input) {
  int startIdx = 0;
  while (startIdx < (int)input.length()) {
    int sepIdx = input.indexOf('-', startIdx);
    String part = (sepIdx == -1) ? input.substring(startIdx)
                                 : input.substring(startIdx, sepIdx);
    part.trim();

    if (part.length() > 0) {
      int commaIdx = part.indexOf(',');
      if (commaIdx > 0) {
        int servoNum = part.substring(0, commaIdx).toInt();
        int pos      = part.substring(commaIdx + 1).toInt();
        bool validPos = (pos >= SERVOMIN && pos <= SERVOMAX);
        bool legServo = (servoNum >= 0 && servoNum < NUM_SERVOS);
        bool neckServo = (servoNum == NECK_SERVO);

        if (validPos && (legServo || neckServo)) {
          if (neckServo) {
            pwm.setPWM(NECK_SERVO, 0, pos);
            sendReply("Neck (15) -> " + String(pos));
          } else {
            targetPos[servoNum] = pos;
            sendReply("Servo " + String(servoNum) + " -> " + String(pos));
          }
        } else {
          sendReply("Invalid: servo " + String(servoNum) + " pos " + String(pos));
        }
      }
    }

    if (sepIdx == -1) break;
    startIdx = sepIdx + 1;
  }
}


void runWalkSequence(const float ampScale[NUM_SERVOS]) {
  oldMillis     = currentMillis;
  currentMillis = millis();
  innerTime    += currentMillis - oldMillis;

  float t = FREQUENCY * innerTime;

  for (int s = 0; s < NUM_SERVOS; s += 2) {
    int pulse = CENTER + (int)off_set_walk[s]
              + (int)(ampScale[s] * AMPLITUDE * cosf(t + phase[s]));
    pwm.setPWM(s, 0, pulse); lastPulse[s] = pulse;
  }
  for (int s = 1; s < NUM_SERVOS; s += 2) {
    float c = fmaxf(0.0f, cosf(t + phase[s]));
    int pulse = CENTER + (int)off_set_walk[s]
              + (int)(ampScale[s] * AMPLITUDE * c);
    pwm.setPWM(s, 0, pulse); lastPulse[s] = pulse;
  }

  delay(DELAY_TIME);
}

void runWalkForward()   { static const float a[NUM_SERVOS] = { 1.f,  1.f,  -1.f,  -1.f,  1.f,  1.f,  -1.f,  -1.f  }; runWalkSequence(a); }
void runWalkLeft()      { static const float a[NUM_SERVOS] = { 1.5f, 1.5f, -.2f,  -.2f,  1.5f, 1.5f, -.2f,  -.2f  }; runWalkSequence(a); }
void runWalkRight()     { static const float a[NUM_SERVOS] = { .2f,  .2f,  -1.5f, -1.5f, .2f,  .2f,  -1.5f, -1.5f }; runWalkSequence(a); }
void runWalkSpotLeft()  { static const float a[NUM_SERVOS] = { 1.5f, 1.5f,  1.5f,  1.5f, 1.5f, 1.5f,  1.5f,  1.5f }; runWalkSequence(a); }
void runWalkSpotRight() { static const float a[NUM_SERVOS] = {-1.5f,-1.5f, -1.5f, -1.5f,-1.5f,-1.5f, -1.5f, -1.5f }; runWalkSequence(a); }


void runHandSequence() {
  sendReply("HAND...");
  targetPos[6] = 200; targetPos[7] = 500;
  moveUntilReachedAll();
  targetPos[6] = 500;
  moveUntilReachedAll();
  sendReply("HAND done");
}

void runDownSequence() {
  targetPos[0] = CENTER - 125; targetPos[1] = CENTER + 125;
  targetPos[2] = CENTER + 125; targetPos[3] = CENTER - 125;
  targetPos[4] = CENTER - 125; targetPos[5] = CENTER + 125;
  targetPos[6] = CENTER + 125; targetPos[7] = CENTER - 125;
  moveUntilReachedAll();
}

void runUpSequence() {
  sendReply("UP...");
  for (int i = 0; i < NUM_SERVOS; i++) targetPos[i] = CENTER;
  moveUntilReachedAll();
  sendReply("UP done");
}

void runTopSequence() {
  sendReply("TOP...");
  targetPos[0] = CENTER + 100; targetPos[1] = CENTER - 100;
  targetPos[2] = CENTER - 100; targetPos[3] = CENTER + 100;
  targetPos[4] = CENTER + 100; targetPos[5] = CENTER - 100;
  targetPos[6] = CENTER - 100; targetPos[7] = CENTER + 100;
  moveUntilReachedAll();
  sendReply("TOP done");
}

void runOnSequence() {
  sendReply("ON...");
  targetPos[0] = CENTER;  targetPos[1] = CENTER;
  targetPos[2] = CENTER;  targetPos[3] = CENTER;
  targetPos[4] = 200;     targetPos[5] = 500;
  targetPos[6] = 500;     targetPos[7] = 100;
  moveUntilReachedAll();
  sendReply("ON done");
}

void runBackSequence() {
  targetPos[0] = CENTER + 200; targetPos[1] = CENTER - 100;
  targetPos[2] = CENTER - 200; targetPos[3] = CENTER + 100;
  targetPos[4] = CENTER - 100; targetPos[5] = CENTER - 100;
  targetPos[6] = CENTER + 100; targetPos[7] = CENTER + 100;
  moveUntilReachedAll();
}

void runFrontSequence() {
  targetPos[0] = CENTER;       targetPos[1] = CENTER - 100;
  targetPos[2] = CENTER;       targetPos[3] = CENTER + 100;
  targetPos[4] = CENTER;       targetPos[5] = CENTER;
  targetPos[6] = CENTER;       targetPos[7] = CENTER;
  moveUntilReachedAll();
}

void runGallopSequence() {
  sendReply("GALLOP...");
  for (int i = 0; i < 8; i++) {
    runBackSequence();
    delay(150);
    runFrontSequence();
    delay(150);
  }
  sendReply("GALLOP done");
}


void setup() {
  Serial.begin(115200);

#if defined(ESP32)
  SerialBT.begin("ESP32_RobotDog");
  sendReply("Bluetooth ready: ESP32_RobotDog");
#endif

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

  sendReply("Ready! Commands:");
  sendReply("  WALK | LEFT | RIGHT | LS | RS | STOP");
  sendReply("  HAND | DOWN | UP | TOP | ON | BACK | GALLOP");
  sendReply("  Manual: 0,325-2,400-15,325  (index,position pairs)");
}

void loop() {
  String input = readCommand();

  if (input.length() > 0) {
    input.toUpperCase();

    if      (input == "WALK")  { currentMode = MODE_WALK_FORWARD; sendReply("Walking forward"); }
    else if (input == "LEFT")  { currentMode = MODE_WALK_LEFT;    sendReply("Turning left");    }
    else if (input == "RIGHT") { currentMode = MODE_WALK_RIGHT;   sendReply("Turning right");   }
    else if (input == "LS")    { currentMode = MODE_SPOT_LEFT;    sendReply("Spinning left");   }
    else if (input == "RS")    { currentMode = MODE_SPOT_RIGHT;   sendReply("Spinning right");  }
    else if (input == "STOP")  { currentMode = MODE_IDLE;         sendReply("Stopped");         }
    else if (input == "HAND")  { currentMode = MODE_IDLE; syncCurrentPos(); runHandSequence();   }
    else if (input == "DOWN")  { currentMode = MODE_IDLE; syncCurrentPos(); runDownSequence();   }
    else if (input == "UP")    { currentMode = MODE_IDLE; syncCurrentPos(); runUpSequence();     }
    else if (input == "TOP")   { currentMode = MODE_IDLE; syncCurrentPos(); runTopSequence();    }
    else if (input == "ON")    { currentMode = MODE_IDLE; syncCurrentPos(); runOnSequence();     }
    else if (input == "BACK")  { currentMode = MODE_IDLE; syncCurrentPos(); runBackSequence();   }
    else if (input == "GALLOP"){ currentMode = MODE_IDLE; syncCurrentPos(); runGallopSequence(); }
    else {
      currentMode = MODE_IDLE;
      parseManualCommand(input);
    }
  }

  switch (currentMode) {
    case MODE_WALK_FORWARD: runWalkForward();   break;
    case MODE_WALK_LEFT:    runWalkLeft();      break;
    case MODE_WALK_RIGHT:   runWalkRight();     break;
    case MODE_SPOT_LEFT:    runWalkSpotLeft();  break;
    case MODE_SPOT_RIGHT:   runWalkSpotRight(); break;
    default:
      updateAllServos();
      break;
  }
  plotServos();
}