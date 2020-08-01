/*Code developed by Anmol Punetha*/

#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>   
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL  

// declaring the pins
const int PulseWire = A2;                
const int rxPin = 4;
const int txPin = 3;
const int piezo = A3;

// creating instances required
PulseSensorPlayground pulseSensor; 
LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial BTserial(rxPin, txPin);

// variables used
uint8_t count = 1;
const int pulse_threshold = 505;
const int tap_threshold = 16; // at 5k resistance
uint8_t  sensorReading = 0;  

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
void dmpDataReady() 
   {   mpuInterrupt = true; }
         
/*********************************************************MAIN-CODE***********************************************************************/

void setup() {   

  Serial.begin(9600); 
  BTserial.begin(9600);         
  pulseSensor.analogInput(PulseWire);         
  pulseSensor.setThreshold(pulse_threshold);   

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
//            BTserial.print(ypr[0]* 180/M_PI);                     // value 1
//            BTserial.print(",");
//            BTserial.print(ypr[1]* 180/M_PI);                     // value 2
//            BTserial.print(",");
//            BTserial.print(ypr[2]* 180/M_PI);                     // value 3
//            BTserial.print(",");
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[2] * 180/M_PI);            
        #endif
    }
           BTserial.print(ypr[0]* 180/M_PI);                     // value 1
           BTserial.print(",");
           BTserial.print(ypr[1]* 180/M_PI);                     // value 2
            BTserial.print(",");
            BTserial.print(ypr[2]* 180/M_PI);                     // value 3
            BTserial.print(",");
            

      // Setting the lcd's cursor for constant texts
      lcd.setCursor(0,0);
      lcd.print("Steps:");
      lcd.setCursor(0,1);
      lcd.print("                ");


      // Setting the number of steps (by piezo)
      sensorReading = analogRead(piezo);
      Serial.print("\t");
      Serial.println(sensorReading);
      if (sensorReading >= tap_threshold) 
           {  if(count>=1){
              lcd.setCursor(8,0);
              lcd.print(count);
//              BTserial.print(count);                                // value 4
//              BTserial.print(";");
              count=count+1;}
           }

           


      // Setting the gesture control system
      if (((ypr[0] * 180/M_PI) > 40.00) || ((ypr[0] * 180/M_PI) < -40.00))
           {  digitalWrite(6, LOW);
              lcd.setCursor(3,1);
              lcd.print("Emergency!");
           }
       else{
              digitalWrite(6, HIGH);
           }
        
       if (((ypr[1] * 180/M_PI) > 30.00) || ((ypr[1] * 180/M_PI) < -30.00))
           {  digitalWrite(4, LOW);
              lcd.setCursor(3,1);
              lcd.print("Fallen!   ");
           }
           
       else{
              digitalWrite(4, HIGH);
           }
        
       if (((ypr[2] * 180/M_PI) > 30.00) || ((ypr[2] * 180/M_PI) < -30.00))
           {  digitalWrite(5, LOW);
              lcd.setCursor(3,1);
              lcd.print("Rolled!    ");
           }
       else{
              digitalWrite(5, HIGH);
           }
             
  
 //   int myBPM = pulseSensor.getBeatsPerMinute();  
      int val = analogRead(PulseWire);
//    if (pulseSensor.sawStartOfBeat())  
//        {  Serial.print("BPM: ");
            // Serial.println(val);
        //   lcd.setCursor(11,0);    
        //   lcd.print("♥: ");                        
        //   lcd.print(myBPM);      
        //   Print the value inside of myBPM. 
//        }
//    delay(20);                   
}
