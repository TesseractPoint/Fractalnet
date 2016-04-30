/* 

=============================================================     

  FractalNet DataGlove code for trasmitting data to devices.  Uses RF24 Library provided by TMRh20.  
  See below for details.  Submitted for 2016 NASA International Space Apps Challenge for "Rock-it Fashion and Design."

  HARDWARE USED:
  Nintendo Power Glove
  Arduino Nano (clone)
  nRF24L01 Radio Module
  HiLetgo CD74HC4067 CMOS 16 Channel Digital Analog Multiplexer Breakout Module (not coded yet)
  Keyestudio I2C LCD Display Module 
  HiLetgo GY-521 MPU-6050 MPU6050 3 Axis Accelerometer Gyroscope Module 6 DOF Module
  Asiawill® Pulsesensor Pulse Heart RatEmic 2 Text-to-Speech Module - 30016e Sensor 
  Emic 2 Text-to-Speech Module - 30016
  8 ohm speaker (1 inch diameter)
  Quimat QK18 Hc-sr04 Ultrasonic Distance Measuring Sensor Module (not coded yet)
  
=============================================================  

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

=============================================================  

   I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
   6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
   Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
  
   Changelog:
        2013-05-08 - added seamless Fastwire support
                   - added note about gyro calibration
        2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
        2012-06-20 - improved FIFO overflow handling and simplified read process
        2012-06-19 - completely rearranged DMP initialization code and simplification
        2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
        2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
        2012-06-05 - add gravity-compensated initial reference frame acceleration output
                   - add 3D math helper file to DMP6 example sketch
                   - add Euler output and Yaw/Pitch/Roll output formats
        2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
        2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
        2012-05-30 - basic DMP initialization working

  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

=============================================================  

   - RADIO CONNECTIONS:
   - nRF24L01 Radio Module: See http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 7
   4 - CSN to Arduino pin 8
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED

  - V2.12 02/08/2016
   - Uses the RF24 Library by TMRH20 and Maniacbug: https://github.com/TMRh20/RF24 (Download ZIP)
   Questions: terry@yourduino.com 
   
 =============================================================

  Ardunio Nano Connections:
  
  TX0 - TO EMIC 2 SIN   // DISCONNECT WHEN UPLOADING VIA ARDUINO IDE
  RX1 - TO EMIC 2 SOUT   // DISCONNECT WHEN UPLOADING VIA ARDUINO IDE
  RST - NOT USED
  GND - RF24 MODULE 
  D2 - GYRO (INTERRUPT)
  D3 - TO MULTIPLEXER - S0
  D4 - TO MULTIPLEXER - S1
  D5 - TO MULTIPLEXER - S2
  D6 - TO MULTIPLEXER - S3
  D7 - RF24 MODULE
  D8 - RF24 MODULE
  D9 - ULTRASONIC PING
  D10 -  ULTRASONIC LISTEN
  D11 - RF24 MODULE
  D12 - RF24 MODULE
  
  D13 - RF24 MODULE 
  3V3 - RF24 MODULE
  REF -  NOT USED
  A0 - THUMB SENSOR
  A1 - INDEX SENSOR
  A2 - MIDDLE SENSOR
  A3 - RING SENSOR
  A4 - GYRO (I2C - SDA)
  A5 - GYRO (I2C - SCL) 
  A6 - HEART RATE SENSOR
  A7 - SIGNAL FROM MULTIPLEXOR
  5V - VCC FOR GYRO/FINGER SENSORS
  RST -  NOT USED
  GND - GROUND FOR GYRO/FINGER SENSORS 
  VIN - 9V BATTERY IN

=============================================================   

   Special Thanks to NJKL44's Power Glove Hack Tutorial: http://www.instructables.com/id/Hacking-a-Powerglove/
   Special Thanks to BIPHENYL's Power Glove Hack Tutorial: http://www.instructables.com/id/Power-Glove-20th-Anniversary-Edition/
   
 =============================================================   
  
*/
   
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

///////////////////// START OF GLOBAL VARIABLES FOR FINGER SENSORS
// These constants won't change.  They're used to give names
// to the pins used:
const int ThumbAnalogInPin = A0;     // Analog input pin that the finger/heart rate sensor is attached to
const int IndexAnalogInPin = A1; 
const int MiddleAnalogInPin = A2; 
const int RingAnalogInPin = A3; 
const int HeartRatePin = A6;

///////////////////// END OF GLOBAL VARIABLES FOR FINGER SENSORS


///////////////////// START OF RF 24 
/*-----( Import needed libraries )-----*/
#include <SPI.h>   // Comes with Arduino IDE
#include "RF24.h"  // Download and Install (See above)
//#include "printf.h" // Needed for "printDetails" Takes up some memory
/*-----( Declare Constants and Pin Numbers )-----*/
#define  CE_PIN  7   // The pins to be used for CE and SN
#define  CSN_PIN 8
/*-----( Declare objects )-----*/
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus (usually) pins 7 & 8 (Can be changed) */
RF24 radio(CE_PIN, CSN_PIN);

/*-----( Declare Variables )-----*/
byte addresses[][6] = {"1Node", "2Node"}; // These will be the names of the "Pipes"

unsigned long timeNow;  // Used to grab the current time, calculate delays
unsigned long started_waiting_at;
boolean timeout;       // Timeout? True or False

/**
  Create a data structure for transmitting and receiving data
  This allows many variables to be easily sent and received in a single transmission
  See http://www.cplusplus.com/doc/tutorial/structures/
*/

struct dataStruct {
  unsigned long _micros;     // to save response times
  int yaw;                   // initialize format for saving DataGlove metrics into variables
  int pitch;
  int roll;          
  int ThumbSensorValue;
  int IndexSensorValue;
  int MiddleSensorValue;
  int RingSensorValue; 
  int HeartRate;
} myData;                 // This can be accessed in the form:  myData.yaw  etc.


///////////////////// END OF RF 24 


//////////////////  INCLUDE LIQUID CRYSTAL I2C LIBRARY
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

//////////////////  END OF LIQUID CRYSTAL


/////////////////////////////////// START OF DISPLAY FOR HRS

int range = 0;
int sensorValue = 0;       
int outputValue = 0;        

int MyMinSensorValue = 498;          // minimum raw output for a good signal - based on user
int MyMaxSensorValue = 543;          // maximum raw output for a good signal - based on user
int MyThresholdSensorValue = 525;    // threshold for a good signal - based on user

/////////////////////////////////// END OF DISPLAY FOR HRS

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  // initialize the LCD
  lcd.begin();
  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.print("DataGlove v1.0");

  delay(50); 
  
    // initialize serial communication
    Serial.begin(9600); 
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // send text to Emic 2 to speak
    Serial.print('\n');             // Send a CR in case the system is already up
    while (Serial.read() != ':') {//;   // When the Emic 2 has initialized and is ready, it will send a single ':' character, so wait here until we receive it
        lcd.setCursor(0,0);
        lcd.print("Waiting For Emic      ");
        lcd.setCursor(0,1);
        lcd.print("Check Connection      ");
        delay(50); 
      }
      
  delay(10);                          // Short delay
  
  Serial.flush();                 // Flush the receive buffer
  Serial.print('S');
  Serial.print("Initializing DataGlove.");  // Send the desired string to convert to speech
  Serial.print('\n');

  lcd.setCursor(0,0);
  lcd.print("Initializing       ");
  lcd.setCursor(0,1);
  lcd.print("DataGlove      ");
  delay(50); 
  
  mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity.  see library and demos for detail
    mpu.setXGyroOffset(115);
    mpu.setYGyroOffset(224);
    mpu.setZGyroOffset(-48);
    mpu.setXAccelOffset(-2970);
    mpu.setYAccelOffset(4466);
    mpu.setZAccelOffset(570); // 1688 factory default for my test chip /570

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        //////////////////
        while (Serial.read() != ':');   // When the Emic 2 has initialized and is ready, it will send a single ':' character, so wait here until we receive it
        delay(10);                          // Short delay
        Serial.flush();  
        Serial.print('S');
        Serial.print("Initialization successful...");
        Serial.print('\n');
        
        lcd.setCursor(0,0);
        lcd.print("Initialization      ");
        lcd.setCursor(0,1);
        lcd.print("successful...        ");
        delay(50);     
        //////////////////////////

        
    } else {
              // ERROR!
              // 1 = initial memory load failed
              // 2 = DMP configuration updates failed
              // (if it's going to break, usually the code will be 1)
             // Serial.print(F("DMP Initialization failed (code "));
             // Serial.print(devStatus);
             // Serial.println(F(")"));
      
              //////////////////
              while (Serial.read() != ':');   // When the Emic 2 has initialized and is ready, it will send a single ':' character, so wait here until we receive it
              delay(10);                          // Short delay
              Serial.flush();  
              Serial.print('S');
              Serial.print("Initialization failed...");
              Serial.print('\n');
              
              lcd.setCursor(0,0);
              lcd.print("Initialization      ");
              lcd.setCursor(0,1);
              lcd.print("failed...        ");
              delay(50);     
              //////////////////////////
         }

  lcd.setCursor(0,0);
  lcd.print("                      ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  delay(50);     
        
  radio.begin();          // Initialize the nRF24L01 Radio
  radio.setChannel(108);  // Above most WiFi frequencies
  radio.setDataRate(RF24_250KBPS); // Fast enough.. Better range

  // Set the Power Amplifier Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  // PALevelcan be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_LOW);
  //  radio.setPALevel(RF24_PA_MAX);

  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);

  // Start the radio listening for data
  //radio.startListening();

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
       // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);           
        #endif

/////////////////////////  START OF FINGER SENSOR 

    myData.yaw = ypr[0] * 180/M_PI;
    myData.pitch = ypr[1] * 180/M_PI;
    myData.roll  = ypr[2] * 180/M_PI;  // Invert the pulldown switch
    myData.ThumbSensorValue = analogRead(ThumbAnalogInPin);
    myData.IndexSensorValue = analogRead(IndexAnalogInPin);
    myData.MiddleSensorValue = analogRead(MiddleAnalogInPin);
    myData.RingSensorValue = analogRead(RingAnalogInPin); 
    myData.HeartRate = analogRead(HeartRatePin);
    myData._micros = micros();  // Send back for timing


    // Serial.print(F("Now sending  -  "));

  if (!radio.write( &myData, sizeof(myData) )) {            // Send data, checking for error ("!" means NOT)
        // Serial.println(F("Transmit failed "));
         /*   
        //////////////////
        while (Serial.read() != ':');   // When the Emic 2 has initialized and is ready, it will send a single ':' character, so wait here until we receive it
        delay(10);                          // Short delay
        Serial.flush();  
        Serial.print('S');
        Serial.print("Transmit failed...");
        Serial.print('\n');
        */
          
        lcd.setCursor(0,0);
        lcd.print("Transmit      ");
        lcd.setCursor(0,1);
        lcd.print("failed...        ");
        delay(50);     
        //////////////////////////
    }

  radio.startListening();                                    // Now, continue listening

  started_waiting_at = micros();               // timeout period, get the current microseconds
  timeout = false;                            //  variable to indicate if a response was received or not

  while ( ! radio.available() ) {                            // While nothing is received
    if (micros() - started_waiting_at > 200000 ) {           // If waited longer than 200ms, indicate timeout and exit while loop
      timeout = true;
      break;
    }
  }

  if ( timeout )
  { // Describe the results
    //Serial.println(F("Response timed out -  no Acknowledge."));
    
         /*       
        //////////////////    
        while (Serial.read() != ':');   // When the Emic 2 has initialized and is ready, it will send a single ':' character, so wait here until we receive it
        delay(10);                          // Short delay
        Serial.flush();  
        Serial.print('S');
        Serial.print("Response timed out -  no Acknowledge.");
        Serial.print('\n');
        */
        
        lcd.setCursor(0,0);
        lcd.print("Response timed out      ");
        lcd.setCursor(0,1);
        lcd.print("no Acknowledge....        ");
        delay(50);     
        //////////////////////////
  }
  else
  {
    // Grab the response, compare, and send to Serial Monitor
    radio.read( &myData, sizeof(myData) );
    timeNow = micros();

    // Show it
    /*
    Serial.print(F("Sent "));
    Serial.print(timeNow);
    Serial.print(F(", Got response "));
    Serial.print(myData._micros);
    Serial.print(F(", Round-trip delay "));
    Serial.print(timeNow - myData._micros);
    Serial.println(F(" microseconds "));
    */
  }
  
radio.stopListening();


////////////////////////////// START OF HEART RATE SENSOR ////////////////////////////
 
  sensorValue = analogRead(HeartRatePin);
  // map it to the range of the analog out:
  range = map(sensorValue, MyMinSensorValue, MyMaxSensorValue, 0, 11);

  if (sensorValue < MyMinSensorValue) {
   // Serial.println("below..." );
   // return;
  }

  if (sensorValue > MyMaxSensorValue) {
    //Serial.println("above..." );
   // return;
  }

  /*
  
  //  Serial.print("sensor = " );
    Serial.print(sensorValue);
  //  Serial.print("\t output = ");
  //  Serial.println(outputValue);
  */

  lcd.setCursor(0,0);
  lcd.print("HEARTBEAT:      ");
  lcd.setCursor(0,1);
  lcd.print("                  ");
             
  // diplay lines varying in length depending on the range value:
  switch (range) {
  case 0:     
   // Serial.println("");     /////ASCII Art Madness
             lcd.setCursor(0,0);
      // lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
            lcd.setCursor(0,1);
        lcd.print("                  ");
    break;
  case 1:   
   // Serial.println("---");
             lcd.setCursor(0,0);
       // lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
            lcd.setCursor(0,1);
        lcd.print("-                  ");
    break;
  case 2:    
   // Serial.println("------");
                lcd.setCursor(0,0);
       // lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
             lcd.setCursor(0,1);
        lcd.print("--                  ");
    break;
  case 3:    
   // Serial.println("---------");
                    lcd.setCursor(0,0);
      //  lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
             lcd.setCursor(0,1);
        lcd.print("---                  ");
    break;
  case 4:   
   // Serial.println("------------");
                     lcd.setCursor(0,0);
       // lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
            lcd.setCursor(0,1);
        lcd.print("----                  ");
    break;
  case 5:   
  //  Serial.println("--------------|-");
                    lcd.setCursor(0,0);
       // lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
             lcd.setCursor(0,1);
        lcd.print("-----                  ");
    break;
  case 6:   
   // Serial.println("--------------|---");
                     lcd.setCursor(0,0);
        //lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
            lcd.setCursor(0,1);
        lcd.print("------                  ");
    break;
  case 7:   
  //  Serial.println("--------------|-------");
                   lcd.setCursor(0,0);
        //lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
              lcd.setCursor(0,1);
        lcd.print("-------                  ");
    break;
  case 8:  
  //  Serial.println("--------------|----------");
                   lcd.setCursor(0,0);
       // lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
              lcd.setCursor(0,1);
        lcd.print("---------                  ");
    break;
  case 9:    
   // Serial.println("--------------|----------------");
                   lcd.setCursor(0,0);
       // lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
              lcd.setCursor(0,1);
        lcd.print("-----------                  ");
    break;
  case 10:   
  //  Serial.println("--------------|-------------------");
                   lcd.setCursor(0,0);
        //lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
              lcd.setCursor(0,1);
        lcd.print("-------------                  ");
    break;
  case 11:   
   // Serial.println("--------------|-----------------------");
                  lcd.setCursor(0,0);
        //lcd.print("                   ");
        lcd.print("HEARTBEAT: ");
               lcd.setCursor(0,1);
        lcd.print("----------------                  ");
    break;
  }

////////////////////////////// END OF HEART RATE SENSOR ////////////////////////////
   
    }

  delay(50);

}
