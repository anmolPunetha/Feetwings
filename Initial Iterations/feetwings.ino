#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>   
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#include  <LiquidCrystal_I2C.h>
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL  

const int PulseWire = 0;       
const int LED13 = 13;          
int Threshold = 20;                           
PulseSensorPlayground pulseSensor; 
LiquidCrystal_I2C lcd(0x27,16,2);

bool dmpReady = false; 
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;   
uint16_t fifoCount;    
uint8_t fifoBuffer[64];
Quaternion q;           
VectorFloat gravity;  
float ypr[3];     
volatile bool mpuInterrupt = false;    
void dmpDataReady() {
    mpuInterrupt = true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {   

  Serial.begin(9600);          
  pulseSensor.analogInput(PulseWire);   
  pulseSensor.blinkOnPulse(LED13);      
  pulseSensor.setThreshold(Threshold);   

   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    mpu.initialize();
    
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(148);
    mpu.setYGyroOffset(-29);
    mpu.setZGyroOffset(63);
    mpu.setZAccelOffset(2048);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    pinMode(6, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);

    lcd.init();                   
    lcd.backlight();
}
void loop() {

   if (!dmpReady) return;
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();}
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);            
        #endif
    }
    if (((ypr[0] * 180/M_PI) > 30.00) || ((ypr[0] * 180/M_PI) < -30.00)){
              digitalWrite(6, LOW);
              lcd.print("Threat Detected");
              }
    else{
          digitalWrite(6, HIGH);
        }
        
     if (((ypr[1] * 180/M_PI) > 30.00) || ((ypr[1] * 180/M_PI) < -30.00)){
              digitalWrite(4, LOW);
              lcd.print("Emergency");
              }
    else{
          digitalWrite(4, HIGH);
        }
        
      if (((ypr[2] * 180/M_PI) > 30.00) || ((ypr[2] * 180/M_PI) < -30.00)){
              digitalWrite(5, LOW);
              lcd.print("Call Home");
              }
    else{
          digitalWrite(5, HIGH);
        }
             

 int myBPM = pulseSensor.getBeatsPerMinute();  
 if (pulseSensor.sawStartOfBeat()) {      
 Serial.print("BPM ♥: ");                        
 Serial.println(myBPM);                        // Print the value inside of myBPM. 
}
  delay(20);                   
}
