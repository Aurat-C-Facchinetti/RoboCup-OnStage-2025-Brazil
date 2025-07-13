#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PWM servo driver
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();

// Constants for PWM signal range for the servos
#define SERVOMIN 120
#define SERVOMAX 470
#define SERVO_FREQ 50
 
int gradi_servo[14] = {0, 0, 0, 0, 0, 0, 0, 0, 90, 90, 90, 90, 90, 90}; // Array storing the current angle for each of the 14 servos

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire); // Initialize the gyroscope BNO055 with ID 55 and address 0x29

// Indices for servo array corresponding to the robotic arm joints and the camera motor
#define BASE 8
#define SHOULDER  9
#define ELBOW 10
#define WRIST 11
#define HAND 12
#define CAM 13

// Default starting positions (in degrees) for the robotic arm joints
#define BASE_START 90 
#define SHOULDER_START 95
#define ELBOW_START 180
#define WRIST_START 50 
#define HAND_START 0 
#define CAM_START 45

// Leg servo indices
#define GIN_ANT_SX 0
#define GIN_ANT_DX 1
#define GIN_POST_DX 2
#define GIN_POST_SX 3
// Hip servo indices
#define ANCA_ANT_SX 4
#define ANCA_ANT_DX 5
#define ANCA_POST_DX 6
#define ANCA_POST_SX 7

// Movement timing constants
#define DELAY_LENTO 600 
#define DELAY_VELOCE 400
#define DELAY_REACH_GOAL 20

#define MOTOR_MAX_1 180
#define MOTOR_MAX_2 270

int gradi_per_rotazione = 0;

// Movement matrix: each pair contains a servo ID and a target degree or delay
// A pair with -1 as ID indicates a delay (in ms)
int posIndietro[26][2] = {{GIN_POST_SX, 35}, {-1, DELAY_VELOCE},  // Every row of the matrix has the servo's name and the degrees it has to reach
                        {GIN_POST_SX, 89}, {-1, DELAY_VELOCE},  // To identify a delay we use the value -1 and in the second coloumn we specify the duration of the delay
                        {GIN_POST_SX, 25}, {-1, DELAY_LENTO}, 
                        {GIN_ANT_SX, 55}, {GIN_POST_DX, 45}, {-1, DELAY_LENTO},
                        {ANCA_ANT_DX, 65}, {ANCA_POST_SX, 134}, {ANCA_POST_DX, 90}, {-1, DELAY_LENTO},
                        {GIN_ANT_SX, 35}, {GIN_POST_DX, 25}, {-1, DELAY_LENTO},
                        {GIN_ANT_DX, 47}, {GIN_POST_SX, 45}, {-1, DELAY_LENTO},
                        {ANCA_ANT_DX, 125}, {ANCA_POST_SX, 89}, {ANCA_ANT_SX, 100}, {ANCA_POST_DX, 48}, {-1, DELAY_LENTO},
                        {GIN_ANT_DX, 27}, {GIN_POST_SX, 25}}; 
int posAvanti[26][2] = {{GIN_ANT_SX, 45}, {-1, DELAY_VELOCE},
                          {ANCA_ANT_SX, 100}, {-1, DELAY_VELOCE},
                          {GIN_ANT_SX, 35}, {-1, DELAY_LENTO},
                          {GIN_ANT_DX, 47}, {GIN_POST_SX, 45}, {-1, DELAY_LENTO},
                          {ANCA_ANT_DX, 75}, {ANCA_POST_SX, 134}, {ANCA_POST_DX, 90}, {ANCA_ANT_SX, 55}, {-1, DELAY_LENTO},
                          {GIN_ANT_DX, 27}, {GIN_POST_SX, 25}, {-1, DELAY_LENTO},
                          {GIN_ANT_SX, 55}, {GIN_POST_DX, 45}, {-1, DELAY_LENTO},
                          {ANCA_ANT_DX, 125}, {ANCA_POST_SX, 89}, {ANCA_POST_DX, 48}, {-1, DELAY_LENTO},
                          {GIN_ANT_SX, 35}, {GIN_POST_DX, 25}};
int posGiraSx[23][2] = {{-1, DELAY_LENTO}, {GIN_ANT_SX, 55}, {GIN_POST_DX, 45}, {-1, DELAY_LENTO},
                        {ANCA_ANT_SX, -1}, {ANCA_POST_DX, -1}, {-1, DELAY_LENTO},
                        {GIN_ANT_SX, 35}, {GIN_POST_DX, 25}, {-1, DELAY_LENTO}, 
                        {GIN_ANT_DX, 47}, {GIN_POST_SX, 45}, {-1, DELAY_LENTO},
                        {ANCA_ANT_DX, -1}, {ANCA_POST_SX, -1}, {-1, DELAY_LENTO},
                        {GIN_ANT_DX, 27}, {GIN_POST_SX, 25}, {-1, DELAY_LENTO},
                        {ANCA_ANT_SX, -1}, {ANCA_POST_DX, -1}, {ANCA_ANT_DX, -1}, {ANCA_POST_SX, -1}};
int posGiraDx[23][2] = {{-1, DELAY_LENTO}, {GIN_ANT_SX, 55}, {GIN_POST_DX, 45}, {-1, DELAY_LENTO},
                        {ANCA_ANT_SX, -1}, {ANCA_POST_DX, -1}, {-1, DELAY_LENTO},
                        {GIN_ANT_SX, 35}, {GIN_POST_DX, 25}, {-1, DELAY_LENTO},
                        {GIN_ANT_DX, 47}, {GIN_POST_SX, 45}, {-1, DELAY_LENTO},
                        {ANCA_ANT_DX, -1}, {ANCA_POST_SX, -1}, {-1, DELAY_LENTO},
                        {GIN_ANT_DX, 27}, {GIN_POST_SX, 25}, {-1, DELAY_LENTO},
                        {ANCA_ANT_SX, -1}, {ANCA_POST_DX, -1}, {ANCA_ANT_DX, -1}, {ANCA_POST_SX, -1}};

// Declaration of the starting posiiton of the robot (starting degrees of each servo in order to get a standing position of the robot)
int pos1[14] = {35, 27, 25, 25, 30, 135, 88, 104, BASE_START, SHOULDER_START, ELBOW_START, WRIST_START, HAND_START, CAM_START}; 

// Global variables used in motion logic and serial input
float yaw;
sensors_event_t orientationData;
uint8_t idx = 0;
uint8_t val_idx = 0;
char value[4] = "000";
char move;
int fatto, quanto, lettura, offset, val;

// Read the last stored position of a given servo
int leggi_servo(int motore){
  return gradi_servo[motore];
}

// Move a servo immediately to a position scaled to PWM values
void muovi(int motore, int pos, int max) {
  int posizione = map(pos, 0, max, SERVOMIN, SERVOMAX);
  servo.setPWM(motore, 0, posizione);
  gradi_servo[motore] = pos;
}

// Smooth movement of a servo towards a goal position
void reach_goal(int motore, int goal, int speed_servo){
  if(goal>=leggi_servo(motore)){
    for (int pos = leggi_servo(motore); pos <= goal; pos += 1) { 
      muovi(motore, pos, MOTOR_MAX_1);     
      delay(speed_servo);                     
    }
  } else {
    for (int pos = leggi_servo(motore); pos >= goal; pos -= 1) { 
      muovi(motore, pos, MOTOR_MAX_1);     
      delay(speed_servo);          
    }
  }
}

// Read orientation (Yaw) from gyroscope
void giro(){
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  yaw = orientationData.orientation.x; 
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

// Move arm joints to walking start position
void posizione_camminata(){
  reach_goal(BASE, BASE_START, DELAY_REACH_GOAL);
  reach_goal(SHOULDER, SHOULDER_START, DELAY_REACH_GOAL);
  reach_goal(ELBOW, ELBOW_START, DELAY_REACH_GOAL);
  reach_goal(WRIST, WRIST_START, DELAY_REACH_GOAL);
  reach_goal(HAND, HAND_START, DELAY_REACH_GOAL);
} 

// Perform repeated hammer-like movements
void martellate() {
  reach_goal(WRIST, 90, DELAY_REACH_GOAL);
  val = map(0, 0, 180, 180, 0);
    reach_goal(BASE, val, DELAY_REACH_GOAL);
  for (int i = 0; i < 3; i++) {
    reach_goal(SHOULDER, 130, DELAY_REACH_GOAL);
    val = map(10, 0, 180, 20, 160);
    reach_goal(ELBOW, 180 - val, DELAY_REACH_GOAL);
    reach_goal(WRIST, 20, DELAY_REACH_GOAL);
    reach_goal(SHOULDER, 160, DELAY_REACH_GOAL);
    val = map(130, 0, 180, 20, 160);
    reach_goal(ELBOW, 180 - val, DELAY_REACH_GOAL);
    reach_goal(WRIST, 120, DELAY_REACH_GOAL);
    delay(1000);
  }
  Serial.print("Yaw:");
  Serial.println(yaw);
}

// Simulate a spraying action 
void spray() {
  reach_goal(WRIST, 90, DELAY_REACH_GOAL);
  val = map(0, 0, 180, 180, 0);
  reach_goal(BASE, val, DELAY_REACH_GOAL);
  reach_goal(SHOULDER, 130, DELAY_REACH_GOAL);
  val = map(10, 0, 180, 20, 160);
  reach_goal(ELBOW, 180 - val, DELAY_REACH_GOAL);
  reach_goal(WRIST, 20, DELAY_REACH_GOAL);
  for(int i = 0; i < 6; i++){
  val = map(0, 0, 180, 180, 0);
  reach_goal(BASE, val, DELAY_REACH_GOAL);
  val = map(45, 0, 180, 180, 0);
  reach_goal(BASE, val, DELAY_REACH_GOAL);
  }
}

// Lower the robot from scooter-ready pose to walking pose
void scendi_scooter(){
  muovi(GIN_ANT_SX, 35, MOTOR_MAX_1);
  muovi(GIN_ANT_DX, 27, MOTOR_MAX_1);
  muovi(GIN_POST_SX, 25, MOTOR_MAX_1);
  muovi(GIN_POST_DX, 25, MOTOR_MAX_1);
}

// Bring robot into scooter-ready pose (legs at 90°, so parallel to the ground)
void posizione_scooter(){
  reach_goal(GIN_ANT_SX, 90, DELAY_REACH_GOAL); 
  reach_goal(GIN_ANT_DX, 90, DELAY_REACH_GOAL);
  reach_goal(GIN_POST_SX, 90, DELAY_REACH_GOAL);
  reach_goal(GIN_POST_DX, 90, DELAY_REACH_GOAL);
}

// Picking a rose from the back of the robot
void rosa() {
  val = map(120, 0, 180, 180, 0);
  reach_goal(BASE, val, DELAY_REACH_GOAL);
  reach_goal(SHOULDER, 150, DELAY_REACH_GOAL);
  val = map(180, 0, 180, 20, 160);
  reach_goal(ELBOW, 180 - val, DELAY_REACH_GOAL);
  reach_goal(WRIST, 120, DELAY_REACH_GOAL);
  val = map(90, 0, 180, 180, 0);
  reach_goal(BASE, val, DELAY_REACH_GOAL);
  reach_goal(HAND, 92, DELAY_REACH_GOAL);

  delay(1000);
  reach_goal(WRIST, 90, DELAY_REACH_GOAL);
  val = map(90, 0, 180, 20, 160);
  reach_goal(ELBOW, 180 - val, DELAY_REACH_GOAL);
  reach_goal(SHOULDER, 90, DELAY_REACH_GOAL);
}

// Position the arm to place down an object
void riponi_oggetti(){
  reach_goal(CAM, 90, DELAY_REACH_GOAL);
  val = map(110, 0, 180, 180, 0);
  reach_goal(BASE, val, DELAY_REACH_GOAL);
  reach_goal(SHOULDER, 130, DELAY_REACH_GOAL);
  val = map(90, 0, 180, 20, 160);
  reach_goal(ELBOW, val, DELAY_REACH_GOAL);
  reach_goal(WRIST, 75, DELAY_REACH_GOAL);
  reach_goal(HAND, 60, DELAY_REACH_GOAL);
}

// Interprets and executes a movement matrix (like posAvanti or posIndietro)
void decode_matrix(int Matrix[][2], int righe, bool isSx) {
  for (int i = 0; i < righe; i++) {
    if (Matrix[i][0] != -1) {
      if(Matrix[i][1] == -1) {
        // Move in relative direction (left or right) for rotation
        if(isSx) {
          muovi(Matrix[i][0], pos1[Matrix[i][0]] + gradi_per_rotazione, MOTOR_MAX_1);
        } else {
          muovi(Matrix[i][0], pos1[Matrix[i][0]] - gradi_per_rotazione, MOTOR_MAX_1);
        }
      } else {
        muovi(Matrix[i][0], Matrix[i][1], MOTOR_MAX_1); // Move to specific angle
      }
    } else {
      delay(Matrix[i][1]); // Delay between steps
    }
  }
}

// Function to move all servos to their initial positions
void posizioneIniziale() {
  // Loop through all servo indices based on the size of the gradi_servo array
  for (int i = 0; i < sizeof(gradi_servo) / sizeof(gradi_servo[0]); i++) {  //each servo reachs its starting position 
    if(i == 10) {
    muovi(i, pos1[i] + 45, MOTOR_MAX_2); // Special case: if the servo index is 10, apply an extra offset of 45 degrees
    }
    else {
    muovi(i, pos1[i], MOTOR_MAX_1);
    }
  }
}

void setup() {
  Serial.flush(); // Clear the serial buffer
  Serial.begin(115200);
 
  while (!Serial) delay(10); // wait untill the serial port opens
 
  if (!bno.begin()) { //try to start the gyroscope
    Serial.print("Errore: BNO055 non rilevato.");
    while (1);
  }

  // Initialize the Adafruit PWM servo driver
  servo.begin();
  servo.setOscillatorFrequency(27000000); // Set oscillator frequency to 27MHz (standard for PCA9685)
  servo.setPWMFreq(SERVO_FREQ); // Set the PWM frequency for servos

  delay(1000);

  posizioneIniziale();
  
  delay(1000);
  
  Serial.setTimeout(1); // Set serial timeout to 1 millisecond for fast reads

  // Move servo 6 and 7 to specific positions after initial setup
  muovi(6, 120, MOTOR_MAX_1);
  muovi(7, 90, MOTOR_MAX_1);
}



void loop() {
  if (Serial.available()) // Check if there's incoming serial data
  {
    char chr = Serial.read();
    // -- HANDLE ACTIONS WITH SPECIFIC KEYS --
    if(chr == 'g'){ // Read and print the yaw angle from the gyro
      idx = -1;
      val_idx = 0;
      giro();
      Serial.print("Yaw:");
      Serial.println(yaw);
    } 
    if(chr == 'j'){ // Read yaw and start descent animation
      idx = -1;
      val_idx = 0;
      giro();
      scendi_scooter();
    }
    if(chr == 'm'){ // Read yaw and start hammering animation
      idx = -1;
      val_idx = 0;
      giro();
      martellate();
    }
    if(chr == 'z'){
      idx = -1;
      val_idx = 0;
      giro();
    }
    if(chr == 'k'){
      idx = -1;
      val_idx = 0;
      posizione_scooter();
    }
    if(chr == 'u'){ // Activate rose animation
      idx = -1;
      val_idx = 0;
      rosa();
    }
    if(chr == 'w') //avanti piano
    {
      idx = 8;
      move=chr;
      posizioneIniziale();
    }
  // -- MOVEMENT COMMANDS (WASD + CASE) --
  if(chr == 'W') // Move forward fast
    {
      idx = 8; // Special index for movement matrix
      move=chr; // Store movement type
      posizioneIniziale(); // Reset to initial posture
    }
  if(chr == 's') // Move backward slow
    {
      idx = 8;
      move=chr;
      posizioneIniziale();
    }
  if(chr == 'S') // Move backword fast
    {
      idx =8;
      move=chr;
      posizioneIniziale();
    }    
    // -- INDIVIDUAL SERVO SELECTION --
    // BASE motor
    if(chr == 'b')
    {
      idx = 0;
      val_idx = 0;
    }
    // SHOULDER motor
    else if(chr == 'v')
    {
      idx = 1;
      val_idx = 0;
    }
    // ELBOW motor
    else if(chr == 'c')
    {
      idx = 2;
      val_idx = 0;
    }
    // WRIST motor
    else if(chr == 'x')
    {
      idx = 3;
      val_idx = 0;
    }    
    // HAND motor
    else if(chr == 'z')
    {
      idx = 4;
      val_idx = 0;
    }
    // Turn left
    else if(chr == 'a')
    {
      idx = 5;
      val_idx = 0;
    }
    // Turn right
    else if(chr == 'd')
    {
      idx = 6;
      val_idx = 0;
    }
    // CAM motor
    else if (chr == 't')    
    {
      idx = 7;
      val_idx = 0;
    }
    // Put back object
    else if(chr == 'r'){
      idx = -1;
      val_idx = 0;
      riponi_oggetti();
    }
    else if(chr == 'o'){  // Spray action
      idx = -1;
      val_idx = 0;
      spray();
    }
    // -- DATA TERMINATOR: ',' Indicates end of numeric input --
    else if(chr == ',') {
      val = atoi(value); // Convert collected digits to integer
      Serial.flush();
      if(idx == 8)  // Movement forward/backward
      {
        switch(move){
          case 'w':
            decode_matrix(posAvanti, 26, true); // Play walking forward sequence
            delay(DELAY_LENTO);
            posizioneIniziale();
            break;
          case 's':
            decode_matrix(posIndietro, 26, true); // Play walking backward sequence
            delay(DELAY_LENTO); 
            posizioneIniziale();                
            break;     
        }
        val_idx=0;
        giro(); // Update yaw after motion
        Serial.print("Yaw:");
        Serial.println(yaw);
      }

      // Reach target angle for specific motors
      else if(idx == 7) { 
        reach_goal(CAM, val, DELAY_REACH_GOAL);
      }
      else if(idx == 0) {
        val = map(val, 0, 180, 180, 0);
        reach_goal(BASE, val, DELAY_REACH_GOAL);
      }
      else if(idx == 1) {
        reach_goal(SHOULDER, val, DELAY_REACH_GOAL);
      }
      else if(idx == 2) { 
        reach_goal(ELBOW, 180 - val, DELAY_REACH_GOAL);
      }
      else if(idx == 3) {
        reach_goal(WRIST, val, DELAY_REACH_GOAL);
      }
      else if(idx == 4){
        reach_goal(HAND, val, DELAY_REACH_GOAL);
      }
      else if(idx == 5){ // TURN LEFT with yaw correction
        giro();
        offset=yaw;
        quanto=val; // total degrees to rotate
        fatto=0; // degrees already rotated

        while(fatto<val){
          if (quanto/45 !=0) {
            gradi_per_rotazione = 30;
            decode_matrix(posGiraSx, 23, true);
            posizioneIniziale();
          }
          else {
            gradi_per_rotazione = quanto%45;
            decode_matrix(posGiraSx, 23, true);
            posizioneIniziale();
          }
          if (quanto > 10) {
          delay(2000);
          giro();
          lettura=yaw-offset;
          if (lettura<=0)
              fatto=abs(lettura)%360;
          else 
              fatto=abs(360-abs(lettura)%360);
              quanto=val-fatto;
          } else {
            fatto = val;
          }
        }
        giro();
        Serial.print("Yaw:");
        Serial.println(yaw);
        
      }
      else if(idx == 6) { // TURN RIGHT with yaw correction
        giro(); // Read initial yaw
        offset=yaw;
        quanto=val;
        fatto=0;
       
        while(fatto<val){
          if (quanto/45 !=0) {
            gradi_per_rotazione = 45;
            decode_matrix(posGiraDx, 23, false);
            posizioneIniziale();
          }
          else {
            gradi_per_rotazione = quanto%45;
            decode_matrix(posGiraSx, 23, false);
            posizioneIniziale();
          }
          if (quanto > 10) {
          delay(2000);
          giro();
          lettura=yaw;
          fatto=abs((lettura-offset)%360);
          if(lettura-offset<0)fatto=360-fatto;
          quanto=val-fatto;
          } else {
            fatto = val;
          }
        }
        giro();
        Serial.print("Yaw:");
        Serial.println(yaw);
        

      }
      // Reset input buffer
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';
    }
    // -- HANDLE DIGIT ENTRY (building up to a full number) --
    else {
      value[val_idx] = chr; // Store character in value buffer
      val_idx++; // Move to next position
    }
  }
}
