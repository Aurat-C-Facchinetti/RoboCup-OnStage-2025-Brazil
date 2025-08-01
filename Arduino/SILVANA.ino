#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_PWMServoDriver.h>

// === Motor control pins ===
#define dir_DX 44
#define vel_DX 2
#define dir_SX 47
#define vel_SX 4
int vel_mot=200;
// === BNO055 Gyroscope setup ===
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

// === Encoder setup ===
const int Encoder1Signal1=3; 
const int Encoder1Signal2=9; 

volatile long encoder1Count = 0;

int cmTarget=100, tickTarget=(1000 * cmTarget) / 22; // default distance

// === Servo controller ===
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); 
// PWM configuration for servo control
#define SERVOMIN 120
#define SERVOMAX 470
#define SERVO_FREQ 50

int gradi_servo[13] = {0, 0, 0, 0, 0, 0, 0, 0, 90, 90, 90, 90, 90}; // Current angles for all servos

// Servo IDs for clarity
#define BASE 8
#define SHOULDER  9
#define ELBOW 10
#define WRIST 11
#define HAND 12
#define CAM 13

// Starting positions of each servo
#define BASE_START 90 //10 170
#define SHOULDER_START 95 //80 170
#define ELBOW_START 20 //40 18c0
#define WRIST_START 50 //100 180
#define HAND_START 0 //10aperto 65chiuso
#define CAM_START 95

#define DELAY_REACH_GOAL 30

//declaration of the starting posiiton of the robot (starting degrees of each servo in order to get a standing position of the robot)
int pos1[14] = {35, 27, 25, 25, 50, 125, 48, 134, BASE_START, SHOULDER_START, ELBOW_START, WRIST_START, HAND_START, CAM_START}; 

// === Gyro / Serial control variables ===
float yaw;
sensors_event_t orientationData;
uint8_t idx = 0;
uint8_t val_idx = 0;
char value[4] = "000"; // holds received angle string (e.g., "090")
char move;
int fatto, quanto, lettura, offset, val;

// === Position Presets ===
void posizione_braccio(){
  reach_goal(BASE, 0, DELAY_REACH_GOAL);
  reach_goal(SHOULDER, 95, DELAY_REACH_GOAL); 
  reach_goal(ELBOW, 0, DELAY_REACH_GOAL); 
  reach_goal(WRIST, 90, DELAY_REACH_GOAL); 
}

void posizione_camminata(){
  reach_goal(BASE, BASE_START, DELAY_REACH_GOAL);
  reach_goal(SHOULDER, SHOULDER_START, DELAY_REACH_GOAL);
  reach_goal(ELBOW, ELBOW_START, DELAY_REACH_GOAL);
  reach_goal(WRIST, WRIST_START, DELAY_REACH_GOAL);
  reach_goal(HAND, HAND_START, DELAY_REACH_GOAL);
}

// === Read yaw from BNO055 ===
void giro(){
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  yaw = orientationData.orientation.x; 
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

// === Servo Utilities ===
//read the current position of a specified servo
int leggi_servo(int motore){
  return gradi_servo[motore];
}

//moves a specified servo to a specified position in a controlled and fluid way
void reach_goal(int motore, int goal, int speed_servo){
  if(goal>=leggi_servo(motore)){
    // goes from the start point degrees to the end point degrees
    for (int pos = leggi_servo(motore); pos <= goal; pos += 1) { 
      muovi(motore, pos);     
      delay(speed_servo);                       
    }
  } else {
    // goes from the end point degrees to the start point degrees
    for (int pos = leggi_servo(motore); pos >= goal; pos -= 1) { 
      muovi(motore, pos);     
      delay(speed_servo);                       
    }
  }
}

//moves a specified servo to a specified position
void muovi(int motore, int pos) {
  int posizione = map(pos, 0, 180, SERVOMIN, SERVOMAX);
  servo.setPWM(motore, 0, posizione);
  gradi_servo[motore] = pos;
}

void posizioneIniziale() {
  for (int i = 0; i < sizeof(gradi_servo) / sizeof(gradi_servo[0]); i++) {  //each servo reachs its starting position 
    muovi(i, pos1[i]);
  }
}

void setup() {
  Serial.flush();
  Serial.begin(115200);

  servo.begin();
  servo.setOscillatorFrequency(27000000);  
  servo.setPWMFreq(SERVO_FREQ);           

  // Set motor control pins as output
  pinMode(vel_DX, OUTPUT);
  pinMode(vel_SX, OUTPUT);
  pinMode(dir_DX, OUTPUT);
  pinMode(dir_SX, OUTPUT);
  

  pinMode(Encoder1Signal1, INPUT); 
  pinMode(Encoder1Signal2, INPUT); 
  attachInterrupt(digitalPinToInterrupt(Encoder1Signal1), leggiEncoder, CHANGE);
 
   while (!Serial) delay(10); // Wait for serial to open

  if (!bno.begin()) {
    Serial.print("Errore: BNO055 non rilevato.");
    while (1);
  }
  posizioneIniziale();
  delay(1000);
  posizione_camminata();
  Serial.setTimeout(1); // timeout for serial reading
}

void loop() {
  if (Serial.available())
  {
    char chr = Serial.read();
    if(chr == 'g'){ // Orientation read
      idx = 10;
      val_idx=0;
      giro();
      Serial.print("Yaw:");
      Serial.println(yaw);
      
    }
    if(chr == 'w') // Movement command
    {
      idx = 8;
      move=chr;
      val_idx = 0;
    }
    if(chr == 'k'){
      idx = -1;
      val_idx = 0;
    }
     if(chr == 'l'){
      idx = -1;
      val_idx = 0;
    }
  if(chr == 'W') //avanti forte
    {
      idx = 8;
      move=chr;
      val_idx = 0;
    }
  if(chr == 's') //indietro piano
    {
      idx = 8;
      move=chr;
      val_idx = 0;
    }
  if(chr == 'S')//indietro forte
    {
      idx =8;
      move=chr;
      val_idx = 0;
    }
    // Joint controls
    // base motor
    if(chr == 'b')
    {
      idx = 0;
      val_idx = 0;
    }
    // shoulder motor
    else if(chr == 'v')
    {
      idx = 1;
      val_idx = 0;
    }
    // GOMITO motor
    else if(chr == 'c')
    {
      idx = 2;
      val_idx = 0;
    }
    // POLSO motor
    else if(chr == 'x')
    {
      idx = 3;
      val_idx = 0;
    }
    
    // MANO motor
    else if(chr == 'z')
    {
      idx = 4;
      val_idx = 0;
    }
    else if(chr == 'f')
    {
      idx = 7;
      val_idx = 0;
    }
    
    else if(chr == 'a')
    {
      idx = 5;
      val_idx = 0;
    }

    else if(chr == 'd')
    {
      idx = 6;
      val_idx = 0;
    }
    
    // Separator
    else if(chr == ',') {
      val = atoi(value); // Convert received number string to int
      
      Serial.flush();
      if(idx == 8)
      {
        switch(move){ // Movement based on selected command (forward/backward)
          case 'w':
            cmTarget=11;
            tickTarget = 488 * cmTarget / 22;
            moveForward();
            encoder1Count=0;
            while (true) {
              if (tickTarget < abs(encoder1Count)) {
                stopMotors();
                encoder1Count=0;
                break;
              }
            }
            
            break;
          case 'W':
            cmTarget=20;
            tickTarget = 488 * cmTarget / 22;
            moveForward();
            encoder1Count=0;
            while (true) {
              if (tickTarget < abs(encoder1Count)) {
                stopMotors();
                encoder1Count=0;
                break;
              }
            }
                  break;
          case 's':
              cmTarget=11;
            tickTarget = 488 * cmTarget / 22;
            moveBackward();
            encoder1Count=0;
            while (true) {
              if (tickTarget < abs(encoder1Count)) {
                stopMotors();
                encoder1Count=0;
                break;
              }
            }
            break;
          case 'S':
            cmTarget=20;
            tickTarget = 488 * cmTarget / 22;
            moveBackward();
            encoder1Count=0;
            while (true) {
              if (tickTarget < abs(encoder1Count)) {
                stopMotors();
                encoder1Count=0;
                break;
              }
            }
                  break;        
        }       
      
      Serial.print("Yaw:");
      Serial.println(yaw);
      }
      // Process joint movement
      else if(idx == 0)
      {
        val = map(val, 0, 180, 180, 0);
        reach_goal(BASE, val, DELAY_REACH_GOAL);
      }
      else if(idx == 1)
      {
        reach_goal(SHOULDER, val, DELAY_REACH_GOAL);
      }
      else if(idx == 2)
      { 
         val = map(val, 0, 180, 20, 157);
         reach_goal(ELBOW, val, DELAY_REACH_GOAL);
      }
      else if(idx == 3)
      { 
        reach_goal(WRIST, val, DELAY_REACH_GOAL);      }
      else if(idx == 4)
      {
        reach_goal(HAND, val, DELAY_REACH_GOAL);      }
      else if(idx == 7)
      { 
       reach_goal(CAM, val, DELAY_REACH_GOAL);
      }
      
      else if(idx == 5)
      { 
       turnLeft(val);
      giro();
      Serial.print("Yaw:");
      Serial.println(yaw);
        
      }
      else if(idx == 6)
      {
       
       turnRight(val);
       
        giro();
        Serial.print("Yaw:");
        Serial.println(yaw);
        
      }


      // reset the angle
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';
    }
    else // Store digits into value array
    {
      value[val_idx] = chr;
      val_idx++;
    }
  }
}

 void leggiEncoder() { // === Encoder ISR ===
  if (digitalRead(Encoder1Signal2) ==HIGH) 
    encoder1Count++;  // Count rising edges only when signal2 is high
}

// === Rotation using yaw angle ===
void turnLeft(int gradii) {
  int lettura;
  giro();
  int gradoIniziale=(int) yaw;
  int gradiFatti=0;
  
  moveLeft();

  // Wait until required degrees are rotated
  while (gradiFatti<=gradii) {
      giro();
      lettura=(int) yaw;
      gradiFatti=(360-(lettura-gradoIniziale))%360;
  }
  stopMotors();
}

void turnRight(int gradii) {
  int lettura;
  giro();
  
  int gradoIniziale=(int) yaw;
  int gradiFatti=0;
  
  moveRight();
  while (gradiFatti<=gradii) { // Wait until required degrees are rotated
      giro();
      lettura=(int) yaw;
      gradiFatti=(360+lettura-gradoIniziale);
      gradiFatti=gradiFatti%360;
  }
  stopMotors();
}

// === Motor actions ===

void moveForward() {
  digitalWrite(dir_DX,0);
  digitalWrite(dir_SX,0);
  analogWrite(vel_DX,vel_mot);
  analogWrite(vel_SX,vel_mot);
}

void moveBackward() {
  digitalWrite(dir_DX,1);
  digitalWrite(dir_SX,1);
  analogWrite(vel_DX,vel_mot);
  analogWrite(vel_SX,vel_mot);
}

void moveLeft(){
  digitalWrite(dir_DX,0);
  digitalWrite(dir_SX,1);
  analogWrite(vel_DX,vel_mot);
  analogWrite(vel_SX,vel_mot);
}

void moveRight(){
  digitalWrite(dir_DX,1);
  digitalWrite(dir_SX,0);
  analogWrite(vel_DX,vel_mot);
  analogWrite(vel_SX,vel_mot);
}

void stopMotors() {
  analogWrite(vel_DX,0);
  analogWrite(vel_SX,0);
}
