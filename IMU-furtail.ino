#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2

#define XaccOFFSET  2147
#define YaccOFFSET  -995
#define ZaccOFFSET   2141
#define XgryoOFFSET  -63
#define YgryoOFFSET 163
#define ZgryoOFFSET 40

 

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float pypr[3];          // previous ypr
float eypr[3];          // ypr error
float sum_e;            // error sum

int Lefturn_TH =          8 ;
int Rightturn_TH =       -8 ;
int Upturn_TH =           8 ;
int Downturn_TH =        -20 ;
int sum_TH =              15 ;

int counts = 0;

#include <Servo.h>
Servo servo_2;
Servo servo_8;




void setup() {
  // put your setup code here, to run once:

  servo_2.attach(5);
    servo_8.attach(11);

      servo_8.write(90);
      servo_2.write(90);
      delay(500);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  /*
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  */

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(XgryoOFFSET);
    mpu.setYGyroOffset(XgryoOFFSET);
    mpu.setZGyroOffset(XgryoOFFSET);
    mpu.setXAccelOffset(XaccOFFSET);
    mpu.setYAccelOffset(YaccOFFSET);
    mpu.setZAccelOffset(ZaccOFFSET);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    //Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    delay(500);

    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }
      servo_8.write(45);
      servo_2.write(135);
      delay(1000);

}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    

    for(int i=0;i<3;i++){
      eypr[i] = (ypr[i]-pypr[i]) * 180 / M_PI;
      pypr[i] = ypr[i];    
    }

    sum_e = eypr[0]+ eypr[1]+ eypr[2] ;

  /*
    Serial.print("Error ypr\t");
    Serial.print(eypr[0]);
    Serial.print("\t");
    Serial.print(eypr[1]);
    Serial.print("\t");
    Serial.print(eypr[2]);
    Serial.print("\t");
    Serial.print(sum_e);
    Serial.println();
  */
  

    if (eypr[2] < Rightturn_TH) {
      servo_8.write(10);
      servo_2.write(40);
      //delay(100);
      counts = 0;
    }

    if (eypr[2] > Lefturn_TH) {
      servo_8.write(140);
      servo_2.write(170);
      //delay(100);
      counts = 0;
    }

    if (eypr[1] > Upturn_TH) {
      servo_8.write(0);
      servo_2.write(180);
      //delay(100);
      counts = 0;
    }

    if (eypr[1] < Downturn_TH) {
      servo_8.write(170);
      servo_2.write(10);
      //delay(100);
      counts = 0;
    }

    if (sum_e < sum_TH){
      counts++;
    }

    if (counts > 30){
      servo_8.write(60);
      servo_2.write(120);
      counts = 0;
    }

  }

  delay(200);


}
