/*Code developed by Anmol Punetha*/
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
int Threshold = 510; 
const int tap_threshold = 12; // at 5k resistance
const int piezo = 0;
int count = 1;
int sensorReading = 0;                          
PulseSensorPlayground pulseSensor; 
LiquidCrystal_I2C lcd(0x27,16,2);


unsigned int time_elapsed;
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
 // pulseSensor.blinkOnPulse(LED13);      
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
//            lcd.print(ypr[0]* 180/M_PI);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);            
        #endif
    }
//    lcd.setCursor(4,0);
//    lcd.print("Hi buddy!!!");
      lcd.setCursor(0,0);
      lcd.print("No of Steps:");
    lcd.setCursor(0,1);
    lcd.print("                ");
    if (((ypr[0] * 180/M_PI) > 30.00) || ((ypr[0] * 180/M_PI) < -30.00)){
                digitalWrite(6, LOW);
                lcd.setCursor(3,1);
                lcd.print("Emergency!");
              }
    else{
          digitalWrite(6, HIGH);
        }
        
     if (((ypr[1] * 180/M_PI) > 30.00) || ((ypr[1] * 180/M_PI) < -30.00)){
                digitalWrite(4, LOW);
                lcd.setCursor(3,1);
                lcd.print("Fallen!   ");
              }
    else{
          digitalWrite(4, HIGH);
        }
        
      if (((ypr[2] * 180/M_PI) > 30.00) || ((ypr[2] * 180/M_PI) < -30.00)){
                digitalWrite(5, LOW);
                lcd.setCursor(3,1);
                lcd.print("Rolled!    ");
              
              }
    else{
          digitalWrite(5, HIGH);
        }


        sensorReading = analogRead(piezo);
          if (sensorReading >= tap_threshold) {
             if(count>=1){
//      lcd.setCursor(0,0);
//      lcd.print("No. of Steps: ");
      lcd.setCursor(12,0);
      lcd.print(count/2);
         count=count+1;}
     }
             

 int myBPM = pulseSensor.getBeatsPerMinute();  
 if (pulseSensor.sawStartOfBeat()) { 
// lcd.setCursor(0,1);     
// lcd.print("   BPM ♥: ");                        
// lcd.print(myBPM);      
 // Print the value inside of myBPM. 
}
 // delay(20);                   
}
