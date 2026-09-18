#include "BluetoothSerial.h"

BluetoothSerial BT;

#define L_RPWM 25
#define L_LPWM 26

#define R_RPWM 18
#define R_LPWM 19

int speed = 250;
int turnSpeed = 250;

void stopAll() {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);
}


void forward() {
  analogWrite(L_RPWM, 0);
  analogWrite(R_RPWM, 0);

  analogWrite(L_LPWM, speed);
  analogWrite(R_LPWM, speed);
}


void backward() {
  analogWrite(L_LPWM, 0);
  analogWrite(R_LPWM, 0);

  analogWrite(L_RPWM, speed);
  analogWrite(R_RPWM, speed);
}


void turnLeft() {
  // Left side reverse
  analogWrite(L_LPWM, 0);
  analogWrite(L_RPWM, turnSpeed);

  // Right side forward
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, turnSpeed);
}


void turnRight() {
  // Left side forward
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, turnSpeed);

  // Right side reverse
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, turnSpeed);
}


void fwdLeft() {

  int leftSpeed = speed * 0.35;
  int rightSpeed = speed;

  // Left slow
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, leftSpeed);

  // Right fast
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, rightSpeed);
}


void fwdRight() {

  int leftSpeed = speed;
  int rightSpeed = speed * 0.35;

  // Left fast
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, leftSpeed);

  // Right slow
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, rightSpeed);
}


void revLeft() {

  int leftSpeed = speed * 0.35;
  int rightSpeed = speed;

  // Left slow reverse
  analogWrite(L_LPWM, 0);
  analogWrite(L_RPWM, leftSpeed);

  // Right fast reverse
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, rightSpeed);
}


void revRight() {

  int leftSpeed = speed;
  int rightSpeed = speed * 0.35;

  // Left fast reverse
  analogWrite(L_LPWM, 0);
  analogWrite(L_RPWM, leftSpeed);

  // Right slow reverse
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, rightSpeed);
}


// Autonomous drive from the Pi: "M<left>,<right>\n", each side -255..255.
// These must keep arriving: if they stop for DRIVE_TIMEOUT_MS the motors
// stop, so a crashed or unplugged Pi can't leave the rover driving.
const unsigned long DRIVE_TIMEOUT_MS = 500;
char driveBuf[16];
int driveLen = -1;  // -1 when not inside an M command
bool driveActive = false;
unsigned long lastDriveMs = 0;


// Positive = forward. The inactive pin is cleared first, as above.
void driveSide(int fwdPin, int revPin, int v) {
  if (v >= 0) {
    analogWrite(revPin, 0);
    analogWrite(fwdPin, v);
  } else {
    analogWrite(fwdPin, 0);
    analogWrite(revPin, -v);
  }
}


// Returns true if c was consumed as part of an M command.
bool feedDrive(char c) {

  if (c == 'M') {
    driveLen = 0;
    return true;
  }

  if (driveLen < 0) return false;

  if (c == '\n') {
    driveBuf[driveLen] = '\0';
    driveLen = -1;
    int l, r;
    if (sscanf(driveBuf, "%d,%d", &l, &r) == 2) {
      driveSide(L_LPWM, L_RPWM, constrain(l, -255, 255));
      driveSide(R_LPWM, R_RPWM, constrain(r, -255, 255));
      driveActive = true;
      lastDriveMs = millis();
    }
    return true;
  }

  if ((isDigit(c) || c == '-' || c == ',' || c == '\r') && driveLen < (int)sizeof(driveBuf) - 1) {
    driveBuf[driveLen++] = c;
    return true;
  }

  // anything else means this wasn't a drive command after all
  driveLen = -1;
  return false;
}


void handleCmd(char c) {

  if (feedDrive(c)) return;

  // a manual command takes over from the Pi
  driveActive = false;

  if (c >= '0' && c <= '9') {
    speed = map(c, '0', '9', 0, 255);
    Serial.print("Speed = ");
    Serial.println(speed);
    return;
  }

  if (c == 'q') {
    speed = 255;
    return;
  }

  switch (c) {

    case 'F':
      forward();
      Serial.println("FORWARD");
      break;

    case 'B':
      backward();
      Serial.println("BACKWARD");
      break;

    case 'L':
      turnLeft();
      Serial.println("LEFT");
      break;

    case 'R':
      turnRight();
      Serial.println("RIGHT");
      break;

    case 'G':
      fwdLeft();
      Serial.println("FWD-LEFT");
      break;

    case 'I':
      fwdRight();
      Serial.println("FWD-RIGHT");
      break;

    case 'H':
      revLeft();
      Serial.println("REV-LEFT");
      break;

    case 'J':
      revRight();
      Serial.println("REV-RIGHT");
      break;

    case 'S':
      stopAll();
      Serial.println("STOPPED");
      break;

    default:
      break;
  }
}


void setup() {

  Serial.begin(115200);

  BT.begin("Rover");

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);

  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);

  stopAll();

  Serial.println("=== ROVER READY ===");
  Serial.println("Pair Bluetooth 'Rover' from phone");
}


void loop() {

  if (Serial.available()) {
    handleCmd(Serial.read());
  }

  if (BT.available()) {
    handleCmd(BT.read());
  }

  if (driveActive && millis() - lastDriveMs > DRIVE_TIMEOUT_MS) {
    stopAll();
    driveActive = false;
    Serial.println("DRIVE TIMEOUT - STOPPED");
  }
}