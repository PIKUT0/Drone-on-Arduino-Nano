//------------------- CREDITS ------------------//
// Author: Pikuto
// GitHub: https://github.com/PIKUT0
// Project GitHUb: https://github.com/PIKUT0/Drone-on-Arduino-Nano
// 12.06.20 Started 
// 06.08.24 Finished
// Yeah, I was lazy asf.
// This code is for Controller = Transmitter
//-------------------- CODE -------------------//
//------------ LIBRARIES ------------//
#include <SPI.h>          // библиотека для работы с шиной SPI
#include "nRF24L01.h"     // библиотека радиомодуля
#include "RF24.h"         // ещё библиотека радиомодуля
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//------------ "DEFINES" ------------//
MPU6050 mpu;
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
#define buttonPin 2
#define RedFlag 5
RF24 radio(9, 10); // "создать" модуль на пинах 9 и 10 Для Уно
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
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
int OldRoll, OldPitch, OldYaw;
//-------- For RADIO --------//
volatile bool Role = 0;   //
int buttonState = 0;
int roll, pitch, yaw;
int p_roll, p_pitch, p_yaw;
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб
unsigned long sent;  //Данные на отправку
long x1, y1, x2, y2; //значения с джойстиков

//--------------- FUNCTIONS ---------------//
//This Function will return the angle(yaw, pitch, roll)[Wire], Euler angles(DMP) = pure shit
void gyro_signals(){
  if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);            
            yaw = int(ypr[0] * 180/M_PI);   //Z-axis
            pitch = int(ypr[1] * 180/M_PI); //Y -axis
            roll = int(ypr[2] * 180/M_PI);  //X - asis
        #endif
    }
}
//This Function will change the way you control your drone.
//Role = 0 -> Jostick
//Role = 1 -> MPU6050 + 1 left Joystick axis (acceleration)
void Change_Roles(){
  Role = !Role;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  OldYaw = int(ypr[0] * 180/M_PI); //Z-axis
  OldPitch = int(ypr[1] * 180/M_PI); //Y -axis
  OldRoll = int(ypr[2] * 180/M_PI);//X - asis
}

//------------ SETUP TIME ------------//
void setup() {
  byte lds = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RedFlag, OUTPUT);
  //Blink for n times to see if its working / rebooted
  for(byte i = 0; i< 10; i++){
    digitalWrite(LED_BUILTIN, lds);
    digitalWrite(RedFlag, lds);
    lds = !lds;
    delay(100);
  }
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif
  //Buttons
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(0, Change_Roles, FALLING);
  //------------ MPU6050 INITIALIZATION ------------//
  mpu.initialize();
  mpu.testConnection() ? digitalWrite(LED_BUILTIN,1) : digitalWrite(LED_BUILTIN,0);
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(212);
  mpu.setYGyroOffset(50);
  mpu.setZGyroOffset(-83);
  mpu.setZAccelOffset(1788);
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
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
   noInterrupts();
   B:for(byte n = 0; n<devStatus; n++){
     digitalWrite(LED_BUILTIN,HIGH);
     delay(500);
     digitalWrite(LED_BUILTIN,LOW);
     delay(500);
   }
   delay(5000); 
   goto B;
  }
  // Get that first angles and mark them as old
  gyro_signals();
  OldYaw = yaw;     //Z -axis
  OldPitch = pitch; //Y -axis
  OldRoll = roll;   //X -asis
  //------------ RADIO INITIALIZATION ------------//
  radio.begin();              // активировать модуль
  radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(4);   // размер пакета, в байтах
  radio.openWritingPipe(address[0]);  // мы - труба 0, открываем канал для передачи данных
  radio.setChannel(0x60);             // выбираем канал (в котором нет шумов!)
  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!
  radio.powerUp();        // начать работу
  radio.stopListening();  // не слушаем радиоэфир, мы передатчик
  Serial.begin(9600);
}
//------------ SETUP ENDS ------------//

//------------ LOOP TIME ------------//
void loop() {
  //------------ ONLY JOYSTICK ------------//
  digitalWrite(RedFlag, LOW);
  while(Role == 0){
    x1 = (analogRead(A1) / 4);
    y1 = 255 - (analogRead(A0) / 4);
    x2 = 255 - analogRead(A3) / 4;
    y2 = analogRead(A2) / 4;
    constrain(x1,0,255);
    constrain(y1,0,255);
    constrain(x2,0,255);
    constrain(y2,0,255);
    if(x1 == 124){x1 = 125;}
    if(y1 == 128){y1 = 125;}
    if(x2 == 128){x2 = 125;}
    if(y2 == 125){y2 = 125;}
    sent = ((x1 << 24) + (y1 << 16) + (x2 << 8) + y2);
    radio.write(&sent, sizeof(sent));
    delay(1);
  }
  //------------ MPU6050 + joystick Y2 axis ------------//
  digitalWrite(RedFlag, HIGH);
  while(Role == 1){
    y2 = analogRead(A2) / 4;
    gyro_signals();
    if(OldYaw == 180)OldYaw = 0;
    roll = OldRoll - roll;
    yaw = OldYaw - yaw;
    pitch = OldPitch - pitch;
    //idk but constrain wont work
    if(pitch >= 50) pitch = 50;
    if(pitch <= -50) pitch = -50;
    if(roll >= 50) roll = 50;
    if(roll <= -50) roll = -50;
    if(yaw >= 50) yaw = 50;
    if(yaw <= -50) yaw = -50;
    roll = map(roll, -50, 50, 255, 0);
    pitch = map(pitch, -50, 50, 0, 255);
    yaw = map(yaw, -50, 50, 255, 0); 
    if(roll == 128) roll = 125;
    if(pitch == 127) pitch = 125;
    if(yaw == 128) yaw = 125;
    if(!(roll - 125 >= 15 || roll - 125 <= -15)) roll = 125;
    if(!(pitch - 125 >= 15 || pitch - 125 <= -15)) pitch = 125;
    if(!(yaw - 125 >= 25 || yaw - 125 <= -25)) yaw = 125;
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println(roll);
    sent = ((long(roll) << 24) + (long(pitch) << 16) + (long(yaw) << 8) + y2);
    radio.write(&sent, sizeof(sent));
    }
}