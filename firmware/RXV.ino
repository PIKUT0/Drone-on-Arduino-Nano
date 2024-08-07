2//------------------- TO_DO LIST ------------------//
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
//int e_psi;              // Euler Psi angle in degrees   ["e" stands for Euler]
//int e_theta;            // Euler Theta angle in degrees ["e" stands for Euler]
//int e_phi;              // Euler Phi angle in degrees   ["e" stands for Euler]
//--------- For PID ---------//
float PIDReturn[]={0, 0, 0}; //PID output
//Angle
float des_AngleRoll;      // Desired Roll angle
float des_AnglePitch;     // Desired pitch angle
float des_AngleYaw;       // Desired yaw angle

float error_AngleRoll;  // Error for roll angle in degrees
float error_AnglePitch; // Error for pitch angle in degress
float error_AngleYaw;   // Error for yaw angle in degrees 

float p_e_AngleRoll;    // p_e{..} = Previous Error for roll angle
float p_e_AnglePitch;   // p_e{..} = Previous Error for pitch angle
float p_e_AngleYaw;     // p_e{..} = Previous Error for Yaw angle

float p_I_AngleRoll;    // p_e{..} = Previous Integral Error for roll angle
float p_I_AnglePitch;   // p_e{..} = Previous Integral Error for pitch angle
float p_I_AngleYaw;     // p_e{..} = Previous Integral Error for Yaw angle

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
  des_AngleYaw = 0.1*(des_AngleYaw - 1500);
  if(des_AngleYaw == -1.0) des_AngleYaw = 0;
  // -- Picth -- //
  des_AnglePitch = map(y1, 0, 255, 1000,2000);
  des_AnglePitch = 0.1*(des_AnglePitch - 1500);
  if(des_AnglePitch == -1.0) des_AnglePitch = 0;
  // -- Roll -- //
  des_AngleRoll = map(x1, 0, 255, 1000,2000);
  des_AngleRoll = 0.1*(des_AngleRoll - 1500);
  if(des_AngleRoll == -1.0) des_AngleRoll = 0;
  // Debug
  Serial.print("1_desiredRateYaw: ");
  Serial.print(des_AngleYaw);
  Serial.print("\tdesiredRatePitch: ");
  Serial.print(des_AnglePitch);
  Serial.print("\t desiredRateRoll: ");
  Serial.println(des_AngleRoll);
}

//This Function is PID function
void pid_equation(float Error, float KP, float KI, float KD, float PrevError, float PrevIterm){
  float P = KP * Error;
  float I = KI*(PrevIterm + Error);
  if (I > 200) I=200;
  else if (I < -200) I=-200;
  float D = D * (Error - PrevError);
  float PID = P+I+D;
  if (PID>200) PID=200;
  else if (PID <-200) PID=-200;
  PIDReturn[0] = PID;
  PIDReturn[1] = Error;
  PIDReturn[2] = I;
}

//This Function is PID error reset function
void reset_pid(){
  //error_AngleRoll = 0; error_AnglePitch = 0; error_AngleYaw = 0;
  p_e_AngleRoll = 0; p_e_AnglePitch = 0;  p_e_AngleYaw = 0;
  p_I_AngleRoll = 0; p_I_AnglePitch = 0;  p_I_AngleYaw = 0;
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
             Serial.print("1_Euler\tpsi: ");
             Serial.print(e_psi);
             Serial.print("\ttheta: ");
             Serial.print(e_theta);
             Serial.print("\t phi: ");
             Serial.println(e_phi);
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
          #endif
        #endif

       /* #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif */
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
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
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
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
   Serial.print(F("DMP Initialization failed (code "));
   Serial.print(devStatus);
   Serial.println(F(")"));
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
  byte pipeNo;
  unsigned int OldTimer;
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
    Serial.print("x1: ");
    Serial.print(x1);    // z - axis
    Serial.print("\ty1: ");
    Serial.print(y1);    // z - axis
    Serial.print("\tx2: ");
    Serial.print(x2);    // y - axis
    Serial.print("\ty2; ");
    Serial.println(y2); //x - axis
    x2 = map(x2, 0, 255, -300, 300);
    y2 = map(y2, 0, 255, -300, 300);
    x1 = map(x1, 0, 255, -300, 300);
    y1 = map(y1, 0, 255, -300, 300);
    if(x2 == -6) x2 = 0;
    if(y2 == -6) y2 = 0;
    if(x1 == -6) x1 = 0;
    if(y1 == -6) y1 = 0;
    constrain(x1,-1000,1000);
    constrain(y1,-1000,1000);
    constrain(x2,-1000,1000);
    constrain(y2,-1000,1000);
    Serial.print("x1: ");
    Serial.print(x1);    // z - axis
    Serial.print("\ty1: ");
    Serial.print(y1);    // z - axis
    Serial.print("\tx2: ");
    Serial.print(x2);    // y - axis
    Serial.print("\ty2; ");
    Serial.println(y2);  //x - axis
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
        for(byte i = 0; i<=100; i++){
          while(millis() - oldtimer < 100) oldtimer = millis();
        }
        Serial.println("SAFETY = 1");
      }
      goto A;
    }
    //------------ GET THE MPU6050 DATA ------------//
    gyro_signals();
    //------------ USE PID CONTROLLER ------------//
    // --- ROLL --- //
    error_AngleRoll = des_AngleRoll - roll; 
    pid_equation(error_AngleRoll, 0.6, 3.5, 0.05, p_e_AngleRoll, p_I_AngleRoll);
    p_e_AngleRoll = PIDReturn[1];
    p_I_AngleRoll = PIDReturn[2];
    RollPID = int(PIDReturn[0]);
    // --- YAW --- //
    error_AngleYaw = des_AngleYaw - yaw;
    pid_equation(error_AngleYaw, 1.4, 5, 0.02 ,p_e_AngleYaw,p_I_AngleYaw);
    p_e_AngleYaw = PIDReturn[1];
    p_I_AngleYaw = PIDReturn[2];
    YawPID = int(PIDReturn[0]); 
    // --- PITCH --- //
    error_AnglePitch = des_AnglePitch - pitch;
    pid_equation(error_AnglePitch, 0.6, 3.5, 0.05, p_e_AnglePitch,p_I_AnglePitch);
    p_e_AnglePitch = PIDReturn[1];
    p_I_AnglePitch = PIDReturn[2];
    PitchPID = int(PIDReturn[0]);
    //Debug
    Serial.print("PID\tYaw: ");
    Serial.print(YawPID);
    Serial.print("\tPitch: ");
    Serial.print(PitchPID);
    Serial.print("\tRoll: ");
    Serial.println(RollPID);
    //------------ FORMULA FOR DRONE ------------//
    FR = y2 + x2 + y1 + x1 + YawPID - PitchPID + RollPID; 
    FL = y2 - x2 + y1 - x1 - YawPID - PitchPID - RollPID;
    BR = y2 - x2 - y1 + x1 - YawPID + PitchPID + RollPID;
    BL = y2 + x2 - y1 - x1 + YawPID + PitchPID - RollPID;
   // FR = map(FR, -1200, 1200, 1000, 2000);
   // FL = map(FL, -1200, 1200, 1000, 2000);
   // BR = map(BR, -1200, 1200, 1000, 2000);
   // BL = map(BL, -1200, 1200, 1000, 2000);
    FR = map(FR, -2000, 2000, 1000, 2000);
    FL = map(FL, -2000, 2000, 1000, 2000);
    BR = map(BR, -2000, 2000, 1000, 2000);
    BL = map(BL, -2000, 2000, 1000, 2000);
    //------------ SOME CONSTRAINS ------------//
    constrain(FR,1000,2000);
    constrain(FL,1000,2000);
    constrain(BR,1000,2000);
    constrain(BL,1000,2000);
    /*if (FR > 2000)FR = 2000;
    if (BR > 2000)BR = 2000; 
    if (BL > 2000)BL = 2000; 
    if (FL > 2000)FL = 2000; */
    if (FR < IDLE_SPEED) FR =  IDLE_SPEED;
    if (BR < IDLE_SPEED) BR =  IDLE_SPEED;
    if (BL < IDLE_SPEED) BL =  IDLE_SPEED;
    if (FL < IDLE_SPEED) FL =  IDLE_SPEED;
    if (y2<-290) {
      FR=NO_SPEED; 
  22    BR=NO_SPEED;
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
 B:while (!radio.available(&pipeNo)) {// слушаем эфир со всех труб
    //Serial.println(" ----------- ");
    gyro_signals();
    reset_pid();
    des_AngleRoll = 0;
    des_AngleYaw = 0;
    des_AnglePitch = 0;
    // --- ROLL --- //
    error_AngleRoll = des_AngleRoll - roll; 
    pid_equation(error_AngleRoll, 0.6, 3.5, 0.05, p_e_AngleRoll, p_I_AngleRoll);
    p_e_AngleRoll = PIDReturn[1];
    p_I_AngleRoll = PIDReturn[2];
    RollPID = int(PIDReturn[0]);
    // --- YAW --- //
    error_AngleYaw = des_AngleYaw - yaw;
    pid_equation(error_AngleYaw, 1.4, 5, 0.02 ,p_e_AngleYaw,p_I_AngleYaw);
    p_e_AngleYaw = PIDReturn[1];
    p_I_AngleYaw = PIDReturn[2];
    YawPID = int(PIDReturn[0]); 
    // --- PITCH --- //
    error_AnglePitch = des_AnglePitch - pitch;
    pid_equation(error_AnglePitch, 0.6, 3.5, 0.05, p_e_AnglePitch,p_I_AnglePitch);
    p_e_AnglePitch = PIDReturn[1];
    p_I_AnglePitch = PIDReturn[2];
    PitchPID = int(PIDReturn[0]);
    FR = 1500 + YawPID + PitchPID + RollPID; 
    FL = 1500 - YawPID + PitchPID - RollPID;
    BR = 1500 - YawPID - PitchPID + RollPID;
    BL = 1500 + YawPID - PitchPID - RollPID;
    for(unsigned int n = 0; n<400;){
      FR = FR - n;
      BR = BR - n;
      BL = BL - n;
      FL = FL - n;
      Motor1.writeMicroseconds(FR);
      Motor2.writeMicroseconds(BR);
      Motor3.writeMicroseconds(BL); 
      Motor4.writeMicroseconds(FL);
      if(millis() - OldTimer >= 10000){
        OldTimer = millis();
        n = n + 50;
        Serial.println(n);
      }
      if(FR == 1100 && BR == 1100 && BL == 1100 && FL == 1100){
        Motor1.writeMicroseconds(NO_SPEED);
        Motor2.writeMicroseconds(NO_SPEED);
        Motor3.writeMicroseconds(NO_SPEED); 
        Motor4.writeMicroseconds(NO_SPEED);
        //while(1){Serial.println("END!");}
      }
      else if(radio.available(&pipeNo)){
        OldTimer = millis();
        n = 0;
        goto B;
        }
    }
  }
}