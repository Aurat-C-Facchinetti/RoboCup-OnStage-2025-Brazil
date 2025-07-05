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

// Encoder x Signal y
#define E1S1 2
#define E1S2 4
#define E2S1 3
#define E2S2 9

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

//Debug
bool giro = true;
bool giroCurveDritto = false;
bool encoder = false;

SoftwareSerial BT(TXD, RXD); 
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();

volatile long conteggioTick = 0;
int cmTarget=100, tickTarget;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
sensors_event_t orientationData;
float yaw;

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  Wire.begin();

  if (!bno.begin()) {
    Serial.print("Errore: BNO055 non rilevato.");
    while (1);
  }
  Serial.println("BNO055 pronto!");

  // Motori
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

  // Encoder motori davanti
  pinMode(E1S1, INPUT); 
  pinMode(E1S2, INPUT); 
  //attachInterrupt(digitalPinToInterrupt(E1S1), leggiEncoder, RISING);
  pinMode(E2S1, INPUT); 
  pinMode(E2S2, INPUT); 
  attachInterrupt(digitalPinToInterrupt(E2S1), leggiEncoder2, RISING);
}

String BTstring;
char comando;
void loop() {
  if (giro == true) {
    Serial.println(leggiGiro());
    delay(100); 
  }

  if (encoder == true) {
    Serial.println(conteggioTick);
    delay(100);
  }

  if (BT.available()) {
    BTstring = BT.readString();
    Serial.println(BTstring);
    comando=BTstring.charAt(0);
    switch (comando) {
      case 'f': // Avanti
        goForward(BTstring.substring(1).toInt());
        break;
      case 'b': // Indietro
        goBackward(BTstring.substring(1).toInt());
        break;
      case 'l': // Sinistra
        turnLeft(BTstring.substring(1).toInt());
        break;
      case 'r': // Destra
        turnRight(BTstring.substring(1).toInt());
        break;
      case 's': // Ferma
        stopMotors();
        break;
      case 'o': //apre cestino
        servo.setPWM(GATE, 0, map(100, 0, 180, SERVO_MIN, SERVO_MAX));
        BT.println("OK");
        break;
      case 'c'://chiude cestino
        servo.setPWM(GATE, 0, map(5, 0, 180, SERVO_MIN, SERVO_MAX));
        BT.println("OK");
        break;
      case '1':
        face();
        break;
      case 'h':
        moveL();
        break;
      case 'j':
        moveBackward();
        break;
      case 'k':
        moveR();
        break;
      case 'u':
        moveForward();
        break;
      default:
        break;
    }
  }
}
  
float leggiGiro() {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  yaw = orientationData.orientation.x; // Yaw (orientazione sull'asse z)
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

void leggiEncoder (){
  int B = digitalRead (E1S2);
  //controlla il sengale B se è 1 per aumentare o diminuire il conteggio
  conteggioTick += (B == 1) ? -1 : +1;
}

void leggiEncoder2 (){
  int B = digitalRead (E2S2);
  //controlla il sengale B se è 1 per aumentare o diminuire il conteggio
  conteggioTick += (B == 1) ? -1 : +1;
}

void moveL() {
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
}

void moveR() {
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

void turnLeft(int gradii) {
  int lettura;
  int gradoIniziale=(int) leggiGiro();
  if (giroCurveDritto == true) {
    Serial.println(leggiGiro());
  }
  int gradiFatti=0;
  
  // Attiva Motori
  moveL();
  
  while (gradiFatti<=gradii) {
      lettura=(int) leggiGiro();
      gradiFatti=(360-(lettura-gradoIniziale))%360;
  }
  stopMotors();
  //delay(1000);
  BT.println("OK");
  if (giroCurveDritto == true) {
    Serial.println(leggiGiro());
  } 
}

void turnRight(int gradii) {
  int lettura;
  int gradoIniziale=(int) leggiGiro();
  int gradiFatti=0;
  
  // Attiva Motori
  moveR();

  while (gradiFatti<=gradii) {
      lettura=(int) leggiGiro();
      gradiFatti=(360+lettura-gradoIniziale);
      gradiFatti=gradiFatti%360;
  }
  stopMotors();
  //delay(1000);
  BT.println("OK");
  if (giroCurveDritto == true) {
    Serial.println(leggiGiro());
  }
}

void goForward(int gradi) {
  conteggioTick=0;
  cmTarget=gradi;
  tickTarget = 488 * cmTarget / 22;
  if (giroCurveDritto == true) {
      Serial.println(leggiGiro());
    }
  moveForward();
  while (true) {
    if (tickTarget < abs(conteggioTick)) {
      stopMotors();
      if (giroCurveDritto == true) {
        Serial.println(leggiGiro());
      }
      break;
    }
  }
  BT.println("OK");
}

void goBackward(int gradi) {
  conteggioTick=0;
  cmTarget=gradi;
  tickTarget = (488 * cmTarget) / 22;
  if (giroCurveDritto == true) {
      Serial.println(leggiGiro());
    }
  moveBackward();
  while (true) {
    if (tickTarget < abs(conteggioTick)) {
      stopMotors();
      if (giroCurveDritto == true) {
        Serial.println(leggiGiro());
      }
      break;
    }
  }
  BT.println("OK");
}
