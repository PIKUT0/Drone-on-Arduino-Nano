//------------------- TO_DO LIST ------------------//
// Finish PID and test
// Smile :)
// Write all comments
// Commit to github
// Write papers
// Be happy
//------------------- CREDITS ------------------//
// Author: Pikuto
// GitHub: https://github.com/PIKUT0
// Project GitHUb: https://github.com/PIKUT0/Drone-on-Arduino-Nano
// 12.06.20 Started 
// 07.08.24 Finished
// Yeah, I was lazy asf.
// This code is for Drone = Receiver
//-------------------- CODE -------------------//
//------------ LIBRARIES ------------//
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//------------ "DEFINES" ------------//
MPU6050 mpu;
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" and "PID_YPR" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO and if you want to controll your drone via YPR PID. 
// Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
#define PID_YPR

// uncomment "OUTPUT_READABLE_EULER" and "PID_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO
// and if you want to controll your drone via Euler PID.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER
//#define PID_EULER 

#define MAX_SPEED 2000  // Motor Max speed = Dangerous     
#define IDLE_SPEED 1100 // Motor Idling speed = min_speed
#define NO_SPEED 1000   // No Rotation = No speed 
#define DIFF 100

Servo Motor1, Motor2, Motor3, Motor4;
RF24 radio(8, 10);      // Make sure you connected radio module correctly. Can change this numbers.
                        // {RU} ["создать" модуль на пинах 9 и 10 Для Уно]

//------------ VARIABLES ------------//
//-------- For MPU6050 --------//
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO Buffer, Here we store all data
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int yaw, pitch, roll;   // our yaw pitch and roll and other shit from GY521
//int e_psi;            // Euler Psi angle in degrees   ["e" stands for Euler]
//int e_theta;          // Euler Theta angle in degrees ["e" stands for Euler]
//int e_phi;            // Euler Phi angle in degrees   ["e" stands for Euler]
//--------- For PID ---------//
float PIDReturn[]={0, 0, 0}; //PID output
//Angle
int des_AngleRoll;      // Desired Roll angle
int des_AnglePitch;     // Desired pitch angle
int des_AngleYaw;       // Desired yaw angle

float er_AngleRoll = 0; // Error for roll angle in degrees
float er_AnglePitch= 0; // Error for pitch angle in degress
float er_AngleYaw = 0;  // Error for yaw angle in degrees 

float p_a_AngleRoll=0;  // p_e{..} = Previous Error for roll angle
float p_a_AnglePitch=0; // p_e{..} = Previous Error for pitch angle
float p_a_AngleYaw=0;   // p_e{..} = Previous Error for Yaw angle

float S_I_AngleRoll = 0;// p_e{..} = Previous Integral sum of Error for roll angle   
float S_I_AnglePitch =0;// p_e{..} = Previous Integral sum of Error for pitch angle
float S_I_AngleYaw = 0; // p_e{..} = Previous Integral sum of Error for Yaw angle
//--- For MOTORS and Radio ---//
int x1,y1,x2,y2;        // Our radio joystick variables
int RollPID, PitchPID, YawPID;
int FL, FR, BL, BR = 0; // Motor Inputs
      // [FL - Front Left  Motor-4 (передний левый  (4 мотор))]
      // [FR - Front Right Motor-1 (передний правый (1 мотоР))]
      // [BL - Back Left   Motor-3 (задний левый    (3 мотор))]
      // [BR - Back Right  Motor-2 (задний правый   (2 мотор))]
byte SAFETY = 0;        // Safety goes first

byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //Tube address
unsigned long gotByte;     // Radio data

//--------------- FUNCTIONS ---------------//
// This Function will return joystick angles [Rate, Yaw, pitch] [Psi, Theta, Phi]
// I'm Still not sure how to use it correctly, need to check in reality
void desiredAngle(){
  // -- Yaw -- //
  des_AngleYaw = map(x2, 0, 255, 1000,2000);
  des_AngleYaw = int(0.1*(des_AngleYaw - 1500));
  if(des_AngleYaw == -1) des_AngleYaw = 0;
  // -- Picth -- //
  des_AnglePitch = map(y1, 0, 255, 1000,2000);
  des_AnglePitch = int(0.1*(des_AnglePitch - 1500));
  if(des_AnglePitch == -1) des_AnglePitch = 0;
  // -- Roll -- //
  des_AngleRoll = map(x1, 0, 255, 1000,2000);
  des_AngleRoll = int(0.1*(des_AngleRoll - 1500));
  if(des_AngleRoll == -1.) des_AngleRoll = 0;
}

//This Function is PID function
void pid_equation(float Error, float KP, float KI, float KD, float PrevData, float ISum, byte k){
  int Data = 0;
  float uP, uI, uD, PID;
  uP = KP * Error;
  ISum = ISum + Error;
  if(Error == 0) ISum = 0;
  if(ISum < -200)ISum = -200;
  if(ISum > 200)ISum = 200; 
  uI = KI * ISum;
  gyro_signals();
  if(k == 0) Data = roll;
  if(k == 1) Data = pitch;
  if(k == 2) Data = yaw;
  uD = KD * (Data - PrevData);
  PID = uP + uI + uD;
  if (PID>200) PID=200;
  else if (PID <-200) PID=-200;
  PIDReturn[0] = PID;
  PIDReturn[1] = Error;
  PIDReturn[2] = ISum;
}

//This Function will calculate Roll, Yawm and Pitch PID
void pid_calculate(){
  // --- ROLL --- //
  Serial.print("ROLL\n");
  er_AngleRoll = des_AngleRoll - roll; 
  p_a_AngleRoll = roll;
  pid_equation(er_AngleRoll, 0.35, 0.3, 0.04, p_a_AngleRoll, S_I_AngleRoll, 0);
  p_a_AngleRoll = PIDReturn[1];
  S_I_AngleRoll = PIDReturn[2];
  RollPID = int(PIDReturn[0]);
  
  // --- YAW --- //
  //No Need
  // --- PITCH --- //
  Serial.print("PITCH\n");
  er_AnglePitch = des_AnglePitch - pitch;
  p_a_AnglePitch = pitch;
  pid_equation(er_AnglePitch, 0.35, 0.3, 0.04, p_a_AnglePitch,S_I_AnglePitch, 1);
  p_a_AnglePitch = PIDReturn[1];
  S_I_AnglePitch = PIDReturn[2];
  PitchPID = int(PIDReturn[0]);
}

//This Function will calculate Roll, Yawm and Pitch PID to "Return Home"
void pid_Home(){
  // --- ROLL --- //
  er_AngleRoll = 0 - roll; 
  p_a_AngleRoll = roll;
  pid_equation(er_AngleRoll, 0.35, 0.3, 0.04, p_a_AngleRoll, S_I_AngleRoll, 0);
  p_a_AngleRoll = PIDReturn[1];
  S_I_AngleRoll = PIDReturn[2];
  RollPID = int(PIDReturn[0]);
  // --- YAW --- //
  er_AngleYaw = 0 - yaw;
  p_a_AngleYaw = yaw;
  pid_equation(er_AngleYaw, 0.35, 0.3, 0.02 ,p_a_AngleYaw,S_I_AngleYaw, 2);
  p_a_AngleYaw = PIDReturn[1];
  S_I_AngleYaw = PIDReturn[2];
  YawPID = int(PIDReturn[0]); 
  // --- PITCH --- //
  er_AnglePitch = 0 - pitch;
  p_a_AnglePitch = pitch;
  pid_equation(er_AnglePitch, 0.35, 0.3, 0.04, p_a_AnglePitch,S_I_AnglePitch, 1);
  p_a_AnglePitch = PIDReturn[1];
  S_I_AnglePitch = PIDReturn[2];
  PitchPID = int(PIDReturn[0]);
}
//This Function is PID error reset function
void reset_pid(){
  p_a_AngleRoll = 0; p_a_AnglePitch = 0;  p_a_AngleYaw = 0;
  S_I_AngleRoll = 0; S_I_AnglePitch = 0;  S_I_AngleYaw = 0;
  des_AngleRoll = 0; des_AnglePitch = 0;  des_AngleYaw = 0;
}
//This Function will return the angle(yaw, pitch, roll)[Wire], Euler angles(DMP) = pure shit
void gyro_signals(void) { 
  if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_EULER
          #ifdef PID_EULER
            // display Euler angles in degrees
             mpu.dmpGetQuaternion(&q, fifoBuffer);
             mpu.dmpGetEuler(euler, &q);
             e_psi = int(euler[0] * 180/M_PI);
             e_theta = int(euler[1] * 180/M_PI);
             e_phi = (-1)* int(euler[2] * 180/M_PI);
          #endif
        #endif
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
          #ifdef PID_YPR
              // display Euler angles in degrees
             mpu.dmpGetQuaternion(&q, fifoBuffer);
             mpu.dmpGetGravity(&gravity, &q);
             mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
             yaw = int(ypr[0] * 180/M_PI);
             pitch =(-1)*int(ypr[1] * 180/M_PI);
             roll = int(ypr[2] * 180/M_PI);
             Serial.print("2_Euler\tyaw: ");
             Serial.print(yaw);
             Serial.print("\tpitch: ");
             Serial.print(pitch);
             Serial.print("\troll: ");
             Serial.println(roll);
             Serial.print("\n ");
          #endif
        #endif
     }
}

//------------ SETUP TIME ------------//
void setup() {
  Serial.begin(9600);
  //------------ MOTOR INITIALIZATION ------------//
  Motor1.attach(5, 0, 2000); // Front Right 
  Motor2.attach(6, 0, 2000); // Back Right
  Motor3.attach(9, 0, 2000); // Back Left
  Motor4.attach(3, 0, 2000); // Front Right
  //------------ MPU6050 INITIALIZATION ------------//
  mpu.initialize();
  mpu.testConnection() ? digitalWrite(LED_BUILTIN,1) : digitalWrite(LED_BUILTIN,0);
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(328);
  mpu.setYGyroOffset(24);
  mpu.setZGyroOffset(-558);
  mpu.setXAccelOffset(-1669);
  mpu.setYAccelOffset(-1430);
  mpu.setZAccelOffset(1553);
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    } 
  else {
    Motor1.write(1000);//FR
    Motor2.write(1000);//BR
    Motor3.write(1000);//BL 
    Motor4.write(1000);//FL
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    B:for(byte n = 0; n<devStatus; n++){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN,LOW);
      delay(500);
    }
   delay(5000); 
   goto B;
  }
  //------------ RADIO INITIALIZATION ------------//
  radio.begin();              // Activate module [активировать модуль]
  radio.setAutoAck(1);        // [режим подтверждения приёма, 1 вкл 0 выкл]
  radio.setRetries(0, 15);    // (Time between attempts, attempts amount)[(время между попыткой достучаться, число попыток]
  radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(4);    // размер пакета, в байтах
  radio.openReadingPipe(1, address[0]);   // хотим слушать трубу 0
  radio.setChannel(0x60);     // выбираем канал (в котором нет шумов!)
  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!
  radio.powerUp();        // начать работу
  radio.startListening(); // начинаем слушать эфир, мы приёмный модуль
  //------------ MOTORS MUSN'T WORK ------------//
  Motor1.write(1000);//FR
  Motor2.write(1000);//BR
  Motor3.write(1000);//BL 
  Motor4.write(1000);//FL
}
//------------ SETUP ENDS ------------//

//------------ LOOP TIME ------------//
void loop() {
  B:byte pipeNo;
  unsigned int OldTimer;
  byte n;
  //------------ WHILE RADIO WORKS ------------//
  A: while (radio.available(&pipeNo)) {        // слушаем эфир со всех труб
    //------------ GET THE DATA ------------//
    // Here we are getting data from transmitter and playing with it
    radio.read(&gotByte, sizeof(gotByte));  // чиатем входящий сигнал
    y2 = gotByte & 255;
    gotByte = gotByte >> 8;
    x2 = gotByte & 255;
    gotByte = gotByte >> 8;
    y1 = gotByte & 255;
    gotByte = gotByte >> 8;
    x1 = gotByte & 255;
    desiredAngle();
    x2 = map(x2, 0, 255, -300, 300);
    y2 = map(y2, 0, 255, -300, 300);
    x1 = map(x1, 0, 255, -300, 300);
    y1 = map(y1, 0, 255, -300, 300);
    if(x2 == -6) x2 = 0;
    if(y2 == -6) y2 = 0;
    if(x1 == -6) x1 = 0;
    if(y1 == -6) y1 = 0;
    while(SAFETY == 0){
      FR = NO_SPEED; 
      FL = NO_SPEED;
      BR = NO_SPEED;
      BL = NO_SPEED;
      Motor1.writeMicroseconds(FR);
      Motor2.writeMicroseconds(BR);
      Motor3.writeMicroseconds(BL); 
      Motor4.writeMicroseconds(FL);
      if(y2 < 0){
        SAFETY = 1;
        unsigned int oldtimer = 0;
        for(byte i = 0; i<255; i++);
        Serial.println("SAFETY = 1");
      }
      goto A;
    }
    //------------ GET THE MPU6050 DATA ------------//
    gyro_signals();
    //------------ USE PID CONTROLLER ------------//
    pid_calculate();
    //Debug
    Serial.print("PID\tYaw: ");
    Serial.print(YawPID);
    Serial.print("\tPitch: ");
    Serial.print(PitchPID);
    Serial.print("\tRoll: ");
    Serial.println(RollPID);
    //------------ FORMULA FOR DRONE ------------//
    FR = y2 + x2 - y1 - x1 - PitchPID - RollPID; 
    FL = y2 - x2 - y1 + x1 - PitchPID + RollPID;
    BR = y2 - x2 + y1 - x1 + PitchPID - RollPID;
    BL = y2 + x2 + y1 + x1 + PitchPID + RollPID;
    FR = map(FR, -1200, 1200, 1000, 2000);
    FL = map(FL, -1200, 1200, 1000, 2000);
    BR = map(BR, -1200, 1200, 1000, 2000);
    BL = map(BL, -1200, 1200, 1000, 2000);
    //------------ SOME CONSTRAINS ------------//
    if (FR > 2000)FR = 2000;
    if (BR > 2000)BR = 2000; 
    if (BL > 2000)BL = 2000; 
    if (FL > 2000)FL = 2000;
    if (FR < IDLE_SPEED) FR =  IDLE_SPEED;
    if (BR < IDLE_SPEED) BR =  IDLE_SPEED;
    if (BL < IDLE_SPEED) BL =  IDLE_SPEED;
    if (FL < IDLE_SPEED) FL =  IDLE_SPEED;
    if (y2<-290) {
      FR=NO_SPEED; 
      BR=NO_SPEED;
      BL=NO_SPEED; 
      FL=NO_SPEED;
      reset_pid();
    } 
    //Debug
    Serial.print("FR: ");
    Serial.print(FR);    // z - axis
    Serial.print("\tFL: ");
    Serial.print(FL);    // z - axis
    Serial.print("\tBR: ");
    Serial.print(BR);    // y - axis
    Serial.print("\tBL; ");
    Serial.println(BL);
    Serial.print("\n");
    //------------ APPLY VALUES TO MOTORS ------------//
    Motor1.writeMicroseconds(FR);
    Motor2.writeMicroseconds(BR);
    Motor3.writeMicroseconds(BL); 
    Motor4.writeMicroseconds(FL);
  }
  //------------ WHILE RADIO DOESN'T WORK ------------//
  // Here drone is on it's own with MPU6050.
  // Its job is to lower height every n seconds
  // while being stable in the air
  OldTimer = millis();
 C:while (!radio.available(&pipeNo)){// слушаем эфир со всех труб
    //Serial.println(" ----------- ");
    gyro_signals();
    reset_pid();
    pid_Home();
    //------------ FORMULA FOR DRONE ------------//
    FR = YawPID - PitchPID - RollPID; 
    FL = YawPID - PitchPID + RollPID;
    BR = YawPID + PitchPID - RollPID;
    BL = YawPID + PitchPID + RollPID;
    FR = map(FR, -1200, 1200, 1000, 2000);
    FL = map(FL, -1200, 1200, 1000, 2000);
    BR = map(BR, -1200, 1200, 1000, 2000);
    BL = map(BL, -1200, 1200, 1000, 2000);
    //------------ SOME CONSTRAINS ------------//
    if (FR > 2000)FR = 2000;
    if (BR > 2000)BR = 2000; 
    if (BL > 2000)BL = 2000; 
    if (FL > 2000)FL = 2000;
    for(n; n<250;){
      FR = FR - n;
      BR = BR - n;
      BL = BL - n;
      FL = FL - n;
      Motor1.writeMicroseconds(FR);
      Motor2.writeMicroseconds(BR);
      Motor3.writeMicroseconds(BL); 
      Motor4.writeMicroseconds(FL);
      if(millis() - OldTimer >= 2000){
        OldTimer = millis();
        n = n + 50;
        Serial.println(n);
      }
      goto C;
      if(n == 400){
        Motor1.writeMicroseconds(NO_SPEED);
        Motor2.writeMicroseconds(NO_SPEED);
        Motor3.writeMicroseconds(NO_SPEED); 
        Motor4.writeMicroseconds(NO_SPEED);
        while(1){Serial.println("END!");}
      }
      else if(radio.available(&pipeNo)){
        OldTimer = millis();
        n = 0;
        goto B;
        }
    }
  }
  n = 0;
}