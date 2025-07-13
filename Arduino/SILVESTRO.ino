#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); // di default usa 0x40

//information for the use of servos 
#define SERVOMIN 120
#define SERVOMAX 470
#define SERVO_FREQ 50
 
int gradi_servo[14] = {0, 0, 0, 0, 0, 0, 0, 0, 90, 90, 90, 90, 90, 90}; //array with all the servo's degrees

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire); //creation of the gyroscope

// Declare the Arduino pin where each servo is connected
#define BASE 8
#define SHOULDER  9
#define ELBOW 10
#define WRIST 11
#define HAND 12
#define CAM 13

// Define the start configuration of the joint 
#define BASE_START 90 //10 170
#define SHOULDER_START 95 //80 170
#define ELBOW_START 180 //40 180
#define WRIST_START 50 //100 180
#define HAND_START 0 //10aperto 65chiuso
#define CAM_START 45

//definition of every servo's name
//Servo ginocchia
#define GIN_ANT_SX 0
#define GIN_ANT_DX 1
#define GIN_POST_DX 2
#define GIN_POST_SX 3
//Servo anche
#define ANCA_ANT_SX 4
#define ANCA_ANT_DX 5
#define ANCA_POST_DX 6
#define ANCA_POST_SX 7

//definition of two presetted delays
#define DELAY_LENTO 600 //900 nel codice vecchio
#define DELAY_VELOCE 400  //600 nel codice vecchio
#define DELAY_REACH_GOAL 20

#define MOTOR_MAX_1 180
#define MOTOR_MAX_2 270

int gradi_per_rotazione = 0;

//declaration of the movement matrixes
int posIndietro[26][2] = {{GIN_POST_SX, 35}, {-1, DELAY_VELOCE},  //every row of the matrix has the servo's name and the degrees it has to reach
                        {GIN_POST_SX, 89}, {-1, DELAY_VELOCE},  //to identify a delay we use the value -1 and in the second coloumn we specify the duration of the delay
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

              //0 1 2 3 4 5 6 7
              //13 23 33 43 12 22 32 42
//declaration of the starting posiiton of the robot (starting degrees of each servo in order to get a standing position of the robot)
int pos1[14] = {35, 27, 25, 25, 30, 135, 88, 104, BASE_START, SHOULDER_START, ELBOW_START, WRIST_START, HAND_START, CAM_START}; // rimetti in ordine

float yaw;
sensors_event_t orientationData;
uint8_t idx = 0;
uint8_t val_idx = 0;
char value[4] = "000";
char move;
int fatto, quanto, lettura, offset, val;

//read the current position of a specified servo
int leggi_servo(int motore){
  return gradi_servo[motore];
}
//moves a specified servo to a specified position
void muovi(int motore, int pos, int max) {
  int posizione = map(pos, 0, max, SERVOMIN, SERVOMAX);
  servo.setPWM(motore, 0, posizione);
  gradi_servo[motore] = pos;
}
//moves a specified servo to a specified position in a controlled and fluid way
void reach_goal(int motore, int goal, int speed_servo){
  if(goal>=leggi_servo(motore)){
    // goes from the start point degrees to the end point degrees
    for (int pos = leggi_servo(motore); pos <= goal; pos += 1) { 
      muovi(motore, pos, MOTOR_MAX_1);     
      delay(speed_servo);                     
    }
  } else {
    // goes from the end point degrees to the start point degrees
    for (int pos = leggi_servo(motore); pos >= goal; pos -= 1) { 
      muovi(motore, pos, MOTOR_MAX_1);     
      delay(speed_servo);          
    }
  }
}
//reads the yaw value from the gyroscope
void giro(){
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  yaw = orientationData.orientation.x; // Yaw (orientazione sull'asse z)
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void posizione_camminata(){
  reach_goal(BASE, BASE_START, DELAY_REACH_GOAL);
  reach_goal(SHOULDER, SHOULDER_START, DELAY_REACH_GOAL);
  reach_goal(ELBOW, ELBOW_START, DELAY_REACH_GOAL);
  reach_goal(WRIST, WRIST_START, DELAY_REACH_GOAL);
  reach_goal(HAND, HAND_START, DELAY_REACH_GOAL);
  //reach_goal(CAM, 160, 100);
} 

void martellate() {
  reach_goal(WRIST, 90, DELAY_REACH_GOAL);
  val = map(0, 0, 180, 180, 0);
    reach_goal(BASE, val, DELAY_REACH_GOAL);
  for (int i = 0; i < 3; i++) {
    //reach_goal(CAM, 180, 20);
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

void scendi_scooter(){
  muovi(GIN_ANT_SX, 35, MOTOR_MAX_1);
  muovi(GIN_ANT_DX, 27, MOTOR_MAX_1);
  muovi(GIN_POST_SX, 25, MOTOR_MAX_1);
  muovi(GIN_POST_DX, 25, MOTOR_MAX_1);
}
void posizione_scooter(){
  reach_goal(GIN_ANT_SX, 90, DELAY_REACH_GOAL); 
  reach_goal(GIN_ANT_DX, 90, DELAY_REACH_GOAL);
  reach_goal(GIN_POST_SX, 90, DELAY_REACH_GOAL);
  reach_goal(GIN_POST_DX, 90, DELAY_REACH_GOAL);
}

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

void decode_matrix(int Matrix[][2], int righe, bool isSx) {
  for (int i = 0; i < righe; i++) {
    if (Matrix[i][0] != -1) {
      if(Matrix[i][1] == -1) {
        if(isSx) {
          muovi(Matrix[i][0], pos1[Matrix[i][0]] + gradi_per_rotazione, MOTOR_MAX_1);
        } else {
          muovi(Matrix[i][0], pos1[Matrix[i][0]] - gradi_per_rotazione, MOTOR_MAX_1);
        }
      } else {
        muovi(Matrix[i][0], Matrix[i][1], MOTOR_MAX_1);
      }
    } else {
      delay(Matrix[i][1]);
    }
  }
}

void posizioneIniziale() {
  for (int i = 0; i < sizeof(gradi_servo) / sizeof(gradi_servo[0]); i++) {  //each servo reachs its starting position 
    if(i == 10) {
    muovi(i, pos1[i] + 45, MOTOR_MAX_2);
    }
    else {
    muovi(i, pos1[i], MOTOR_MAX_1);
    }
  }
}



void setup() {
  

  Serial.flush();
  Serial.begin(115200);
  Serial.println("inzio setup");
  while (!Serial) delay(10); // wait untill the serial port opens
  if (!bno.begin()) { //try to start the gyroscope
    Serial.print("Errore: BNO055 non rilevato.");
    while (1);
  }

  Serial.println("metà setup");
  //setting some parameters for the servos
  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);

  delay(1000);

  posizioneIniziale();
  
  delay(1000);
  
  Serial.setTimeout(1);
  //inizio_spettacolo();
  muovi(6, 120, MOTOR_MAX_1);
  muovi(7, 90, MOTOR_MAX_1);

  Serial.println("fine setup");

}



void loop() {
  if (Serial.available())
  {
    char chr = Serial.read();
    if(chr == 'g'){
      idx = -1;
      val_idx = 0;
      giro();
      Serial.print("Yaw:");
      Serial.println(yaw);
    }
    if(chr == 'j'){
      idx = -1;
      val_idx = 0;
      giro();
      scendi_scooter();
    }
    if(chr == 'y'){
      idx = -1; 
      val_idx = 0;
      giro();
      //triste();
    }
     if(chr == 'm'){
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
    if(chr == 'u'){
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
  if(chr == 'W') //avanti forte
    {
      idx = 8;
      move=chr;
      posizioneIniziale();
    }
  if(chr == 's') //indietro piano
    {
      idx = 8;
      move=chr;
      posizioneIniziale();
    }
  if(chr == 'S')//indietro forte
    {
      idx =8;
      move=chr;
      posizioneIniziale();
    }    
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
    //Curva a sinistra
    else if(chr == 'a')
    {
      idx = 5;
      val_idx = 0;
    }
    //Curva a destra
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
    else if(chr == 'r'){
      idx = -1;
      val_idx = 0;
      riponi_oggetti();
    }
    else if(chr == 'o'){
      idx = -1;
      val_idx = 0;
      spray();
    }
    // Separator
    else if(chr == ',') {
      val = atoi(value);//value.toInt();
      //Serial.println(val);
      Serial.flush();
      //Serial.println(idx);
      if(idx == 8)
      {
        switch(move){
          case 'w':
            decode_matrix(posAvanti, 26, true);
            delay(DELAY_LENTO);
            posizioneIniziale();
            break;
          case 's':
            decode_matrix(posIndietro, 26, true);   
            delay(DELAY_LENTO); 
            posizioneIniziale();                
            break;     
        }
        val_idx=0;
        giro();//non so se serve
        Serial.print("Yaw:");
        Serial.println(yaw);
      }
      
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
        //val = map(val, 0, 180, 20, 160);
        reach_goal(ELBOW, 180 - val, DELAY_REACH_GOAL);
      }
      else if(idx == 3) {
        reach_goal(WRIST, val, DELAY_REACH_GOAL);
      }
      else if(idx == 4){
        reach_goal(HAND, val, DELAY_REACH_GOAL);
      }
      else if(idx == 5){ 
        //Serial.println(val);
        //sinistra antiorario
        giro();
        offset=yaw;
        //per ragno lievemente diverso 
        quanto=val;
        fatto=0;
        //per ruota gira qui
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
      else if(idx == 6) {
        //Serial.println("val inizio:"+val);
        //destra orario
        giro();
        offset=yaw;
        //per ragno lievemente diverso 
        quanto=val;
        fatto=0;
        //Serial.println(val);
        //per ruota gira qui
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
      // reset the angle
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';
    }
    // Plain number
    else {
      value[val_idx] = chr;
      val_idx++;
    }
  }
}
