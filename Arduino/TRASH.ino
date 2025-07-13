#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// Motors
#define PWM1 2
#define PWM2 4
#define DIR1 44
#define DIR2 47

// Encoder Signal x
#define ES1 3
#define ES2 9

// Bluetooth
#define TXD 11
#define RXD 10

// Servo
#define lEYE 5
#define rEYE 6
#define GATE 9

#define SERVO_MIN 120
#define SERVO_MAX 470
#define SERVO_FREQ 50

SoftwareSerial BT(TXD, RXD); 
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();

volatile long tickCount = 0;
int cmTarget=100, tickTarget;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
sensors_event_t orientationData;
float yaw;

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  Wire.begin();

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("Error: BNO055 not detected.");
    while (1);
  }
  Serial.println("BNO055 ready!");

  // Motors
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

  // Servo
  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);

  // Initial state
  servo.setPWM(GATE, 0, map(5, 0, 180, SERVO_MIN, SERVO_MAX));
  servo.setPWM(lEYE, 0, map(3, 0, 180, SERVO_MIN, SERVO_MAX));
  servo.setPWM(rEYE, 0, map(177, 0, 180, SERVO_MIN, SERVO_MAX));
  stopMotors();

  // Encoder
  pinMode(ES1, INPUT); 
  pinMode(ES2, INPUT); 
  attachInterrupt(digitalPinToInterrupt(ES1), encoderReading, RISING);
}

String BTstring;
char command;
void loop() {
  if (BT.available()) {
    BTstring = BT.readString();
    Serial.println(BTstring);
    command=BTstring.charAt(0);
    switch (command) {
      case 'f': // Forward
        goForward(BTstring.substring(1).toInt());
        break;
      case 'b': // Backward
        goBackward(BTstring.substring(1).toInt());
        break;
      case 'l': // Left
        turnLeft(BTstring.substring(1).toInt());
        break;
      case 'r': // Right
        turnRight(BTstring.substring(1).toInt());
        break;
      case 's': // Stop
        stopMotors();
        break;
      case 'o': // Open Bin
        open();
        break;
      case 'c': // Close Bin
        close();
        break;
      case '1': // Expressions of the Eyes
        face();
        break;
      case '2': // Final scene of Trash's performance
        final();
        break;
      default:
        Serial.println("Incorrect command received via Bluetooth.");
        break;
    }
  }
}

void open() {
  servo.setPWM(GATE, 0, map(100, 0, 180, SERVO_MIN, SERVO_MAX));
  BT.println("OK");
}

void close() {
  servo.setPWM(GATE, 0, map(5, 0, 180, SERVO_MIN, SERVO_MAX));
  BT.println("OK");
}
  
float gyroReading() {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  yaw = orientationData.orientation.x; // Yaw (orientation around the Z axis)
  delay(BNO055_SAMPLERATE_DELAY_MS);
  return yaw;
}

void face() {
  servo.setPWM(lEYE, 0, map(177, 0, 180, SERVO_MIN, SERVO_MAX));
  servo.setPWM(rEYE, 0, map(177, 0, 180, SERVO_MIN, SERVO_MAX));
  delay(500);
  for (int j=0;j<2;j++) {
    for (int i=177;i>2;i--) {
      servo.setPWM(lEYE, 0, map(i, 0, 180, SERVO_MIN, SERVO_MAX));
      servo.setPWM(rEYE, 0, map(i, 0, 180, SERVO_MIN, SERVO_MAX));
      delay(10);
    }
    for (int i=3;i<178;i++) {
      servo.setPWM(lEYE, 0, map(i, 0, 180, SERVO_MIN, SERVO_MAX));
      servo.setPWM(rEYE, 0, map(i, 0, 180, SERVO_MIN, SERVO_MAX));
      delay(10);
    }
      servo.setPWM(lEYE, 0, map(3, 0, 180, SERVO_MIN, SERVO_MAX));
      servo.setPWM(rEYE, 0, map(177, 0, 180, SERVO_MIN, SERVO_MAX));
  }
  BT.println("OK");
}

void encoderReading (){
  int B = digitalRead (ES2);
  tickCount += (B == 1) ? -1 : +1;
}

void moveR() {
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
}

void moveL() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
}

void moveForward() {
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
}

void moveBackward() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
}

void stopMotors() {
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
}

void turnLeft(int angle) {
  int reading;
  int initialAngle=(int) gyroReading();
  int degreesDone=0;
  
  // Activates Motors
  moveL();
  
  while (degreesDone<=angle) {
      reading=(int) gyroReading();
      degreesDone=(360-(reading-initialAngle))%360; // Calculates the degrees done
  }
  stopMotors();
  BT.println("OK");
}

void turnRight(int angle) {
  int reading;
  int initialAngle=(int) gyroReading();
  int degreesDone=0;
  
  // Activates Motors
  moveR();

  while (degreesDone<=angle) {
      reading=(int) gyroReading();
      degreesDone=(360+reading-initialAngle)%360; // Calculates the degrees done
  }
  stopMotors();
  BT.println("OK");
}

void goForward(int cmgoal) {
  tickCount=0; // From encoder
  cmTarget=cmgoal;
  tickTarget = 488 * cmTarget / 22; // Target number of ticks from the encoder needed to reach cmGoal

  moveForward();

  while (true) { // When hits tickTarget the motors stop
    if (tickTarget < abs(tickCount)) {
      stopMotors();
      break;
    }
  }
  BT.println("OK");
}

void goBackward(int cmgoal) {
  tickCount=0; // From encoder
  cmTarget=cmgoal;
  tickTarget = (488 * cmTarget) / 22; // Target number of ticks from the encoder needed to reach cmGoal

  moveBackward();

  while (true) { // When hits tickTarget the motors stop
    if (tickTarget < abs(tickCount)) {
      stopMotors();
      break;
    }
  }
  BT.println("OK");
}

// Final scene of Trash's performance
void final() {
  turnRight(70);
  delay(700);
  goForward(55);
  delay(700);
  goForward(55);
  delay(700);
  turnLeft(90);
  delay(700);
  face();
  open();
  delay(2500);
  close();
}
