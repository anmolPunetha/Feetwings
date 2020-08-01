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

// declaring the   pins
const int PulseWire = A2;                
const int rxPin = 3;
const int txPin = 4;
const int piezo = A2;

// creating instances required
PulseSensorPlayground pulseSensor; 
LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial BTserial(rxPin, txPin);

// variables used
uint8_t steps ;  // set =1 if necesasry
int count=1 ;
const int pulse_threshold = 505;
const int tap_threshold = 16; // at 5k resistance
uint8_t  sensorReading = 0;  
String yaw_check ;
String pitch_check ;
String roll_check;
uint8_t beats ;

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
   mpu.setXGyroOffset(146);
   mpu.setYGyroOffset(-29);
   mpu.setZGyroOffset(76);
   mpu.setZAccelOffset(2048);

   if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);

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
        #endif
    }
    
      // Setting the lcd's cursor for constant texts
      lcd.setCursor(0,0);
      lcd.print("Steps:");
      lcd.setCursor(0,1);
      lcd.print("                ");


      // Sending data to bluetooth
      steps = stepsCount();
      BTserial.print(steps);                                      // value 1 = steps
      BTserial.print(",");                       

      yaw_check = yawCheck();
      BTserial.print(yaw_check);                                  // value 2 = yaw check
      BTserial.print(",");
      
      pitch_check = pitchCheck();
      BTserial.print(pitch_check);                                // value 3 = pitch check
      BTserial.print(",");
      
      roll_check = rollCheck();
      BTserial.print(roll_check);                                 // value 4 = roll check
      BTserial.print(",");

      beats = pulseBPM();
      BTserial.print(beats);                                      // value 5 = BPM
      BTserial.print(",");

      BTserial.print(ypr[0]* 180/M_PI);                            // value 6 = yaw
      BTserial.print(",");
      BTserial.print(ypr[1]* 180/M_PI);                            // value 7 = pitch
      BTserial.print(",");
      BTserial.print(ypr[2]* 180/M_PI);                            // value 8 = roll
      BTserial.print(";");
                 
}
int stepsCount(){
     sensorReading = analogRead(piezo);
      if (sensorReading >= tap_threshold) 
           {  if(count>=1){
              lcd.setCursor(8,0);
              lcd.print(count);
              count=count+1;}
           }
      return count-1;
}

String yawCheck(){
  String y;
      if (((ypr[0] * 180/M_PI) > 40.00) || ((ypr[0] * 180/M_PI) < -40.00))
           {  
              digitalWrite(8, HIGH);
              lcd.setCursor(3,1);
              lcd.print("Emergency!");
              y = "Emergency!   " ;
           }
       else{
              digitalWrite(8, LOW);
              y = "Normal" ;
           }

         return y ;  
}

String pitchCheck(){
  String p;
    if (((ypr[1] * 180/M_PI) > 30.00) || ((ypr[1] * 180/M_PI) < -30.00))
           {  
              digitalWrite(7, HIGH);
              lcd.setCursor(3,1);
              lcd.print("Ankle Twisted!");
              p = "Ankle Twisted!";
           }
           
       else{
              digitalWrite(7, LOW);
              p = "Normal" ;
           }

           return p;
}

String rollCheck(){
  String r;
       if (((ypr[2] * 180/M_PI) > 40.00) || ((ypr[2] * 180/M_PI) < -40.00))
           {  
              digitalWrite(6, HIGH);
              lcd.setCursor(3,1);
              lcd.print("Fallen!     ");
              r = "Fallen !" ;
           }
       else{
              digitalWrite(6, LOW);
              r = "Normal";
           }
           return r;
}

int pulseBPM(){  
      int myBPM = pulseSensor.getBeatsPerMinute();  
      int val = analogRead(PulseWire);
      if (pulseSensor.sawStartOfBeat())  
        {  Serial.print("BPM: ");
           Serial.println(val);
           lcd.setCursor(11,0);    
           lcd.print("♥: ");                        
           lcd.print(myBPM);      
        }
      return myBPM;
}
