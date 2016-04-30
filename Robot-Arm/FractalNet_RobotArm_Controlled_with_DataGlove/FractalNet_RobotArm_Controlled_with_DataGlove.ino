/* 

=============================================================  
  
  FractalNet Robot Arm code for receiving data from the DataGlove.  Uses RF24 Library provided by TMRh20.  
  See below for details.  Submitted for 2016 NASA International Space Apps Challenge for "Rock-it Fashion and Design."

  HARDWARE USED:
  Arduino Uno (clone)
  nRF24L01 Radio Module
  Five (5) Deluxe HiTEC HS-322HD Servos with Pan and Tilt hardware
  SainSmart Sensor Shield v4

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
   - nRF24L01 Radio Module: See http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   - RADIO CONNECTIONS:
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
   Questions: terry@yourduino.com */

#include <Servo.h>

Servo Pan0Servo;          // create servo objects to control 5 servos
Servo Pan1Servo;
Servo TiltServo;
Servo RollServo;
Servo GripServo;

int Pan0Value = 90;       // min: 0 - max: 180 - initial: 90
int Pan1Value = 90;       // min: 0 - max: 180 - initial: 90
int TiltValue = 30;       // min: 0 - max: 130 - initial: 30
int RollValue = 90;       // min: 0 - max: 180 - initial: 90
int GripValue = 60;       // min: 60 - max: 155 - initial: 60 
  
int OldPan0Value = Pan0Value;
int OldPan1Value = Pan1Value;
int OldTiltValue = TiltValue;
int OldRollValue = RollValue;

int Pan0Posl;
int Pan0Posm;
int Pan1Posl;
int Pan1Posm;
int TiltPosl;
int TiltPosm;
int RollPosl;
int RollPosm;
int GripPos;


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


/**
  Create a data structure for transmitting and receiving data
  This allows many variables to be easily sent and received in a single transmission
  See http://www.cplusplus.com/doc/tutorial/structures/
*/
struct dataStruct {
  unsigned long _micros;         // to save response times
  int yaw;                       // initialize format for saving DataGlove metrics into variables 
  int pitch;
  int roll;          
  int ThumbSensorValue;
  int IndexSensorValue;
  int MiddleSensorValue;
  int RingSensorValue;   
  int HeartRate;
} myData;                 // This can be accessed in the form:  myData.yaw  etc.




void setup()   /****** SETUP: RUNS ONCE ******/
{

  Serial.begin(9600);   // NOTE: The "F" in the print statements means "unchangable data; save in Flash Memory to conserve SRAM"
  Serial.println(F("Receiving DataGlove data by nRF24L01 radio"));

  Pan0Servo.attach(3);  // assign pins to each servo for PWM
  Pan1Servo.attach(5);
  TiltServo.attach(6);
  RollServo.attach(9);
  GripServo.attach(10);

  Pan0Servo.write(Pan0Value);   // send initial values to servos
  Pan1Servo.write(Pan1Value);   
  TiltServo.write(TiltValue);  
  RollServo.write(RollValue);  
  GripServo.write(GripValue);   

  radio.begin();          // Initialize the nRF24L01 Radio
  radio.setChannel(108);  // 2.508 Ghz - Above most Wifi Channels
  radio.setDataRate(RF24_250KBPS); // Fast enough.. Better range

  // Set the Power Amplifier Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  // PALevelcan be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_LOW);
  //   radio.setPALevel(RF24_PA_MAX);

  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);

  // Start the radio listening for data
  radio.startListening();
//  radio.printDetails(); //Uncomment to show LOTS of debugging information
  
}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{

  if ( radio.available())
  {

    while (radio.available())   // While there is data ready to be retrieved from the receive pipe
      {
        radio.read( &myData, sizeof(myData) );             // Get the data
      }

    radio.stopListening();                               // First, stop listening so we can transmit
    radio.write( &myData, sizeof(myData) );              // Send the received data back.
    radio.startListening();                              // Now, resume listening so we catch the next packets.

    Pan0Value = map(myData.yaw, -170, 170, 0, 180);      // Map the variables to servo PWM values
    Pan1Value = Pan0Value;
    TiltValue = map(myData.pitch, -60, 80, 130, 0);
    RollValue = map(myData.roll, -60, 60, 180, 0);

    TiltValue = TiltValue + 15;                          // add a little to make the arm 'straight'
    
    if (Pan0Value < 1) {                                 // making sure the servos don't go out of range
      Pan0Value = 0;
    }
      if (Pan1Value < 1) {
      Pan1Value = 0;
    }
       if (Pan0Value > 179) {
      Pan0Value = 180;
    }
      if (Pan1Value > 179) {
      Pan1Value = 180;
    }
       if (TiltValue < 1) {
      TiltValue = 0;
    }
       if (TiltValue > 130) {
      TiltValue = 130;
    }    
       if (RollValue < 1) {
      RollValue = 0;
    }
       if (RollValue > 180) {
      RollValue = 180;
    }  
    
    if (myData.ThumbSensorValue < 220 && myData.IndexSensorValue < 430 && myData.MiddleSensorValue < 200 && myData.RingSensorValue < 250) {
        GripValue = 160;      // close grip
    }
    else {
         GripValue = 60;      // open grip
    }

    if (Pan0Value < OldPan0Value) {    // in steps of 1 degree
           for(Pan0Posl = OldPan0Value; Pan0Posl > Pan0Value; Pan0Posl -= 1) {
               Pan0Servo.write(Pan0Posl);              
               Pan1Servo.write(Pan0Posl);
               delay(15);
             }
        }  
    if (Pan0Value > OldPan0Value) {        // in steps of 1 degree
           for(Pan0Posm = OldPan0Value; Pan0Posm < Pan0Value; Pan0Posm += 1) {                          
               Pan0Servo.write(Pan0Posm);              
               Pan1Servo.write(Pan0Posm);
               delay(15);
             }
        }

    if (TiltValue < OldTiltValue) {       // in steps of 1 degree
         for(TiltPosl = OldTiltValue; TiltPosl > TiltValue; TiltPosl -= 1) {             
              TiltServo.write(TiltPosl);           
              delay(15);
            }
        }   
    if (TiltValue > OldTiltValue) {        // in steps of 1 degree
         for(TiltPosm = OldTiltValue; TiltPosm < TiltValue; TiltPosm += 1) { 
               TiltServo.write(TiltPosm);              
               delay(15);
            }
        }
    
    if (RollValue < OldRollValue) {        // in steps of 1 degree
         for(RollPosl = OldRollValue; RollPosl > RollValue; RollPosl -= 1) {
              RollServo.write(RollPosl);              
              delay(15);
            }
        }  
    if (RollValue > OldRollValue) {        // in steps of 1 degree
         for(RollPosm = OldRollValue; RollPosm < RollValue; RollPosm += 1) {              
              RollServo.write(RollPosm);              
              delay(15);
            }
        }
    
      GripServo.write(GripValue);
      delay(15);
      
      OldPan0Value = Pan0Value;
      OldPan1Value = Pan1Value;
      OldTiltValue = TiltValue;
      OldRollValue = RollValue;

  } // END radio available

delay(10);

}//--(end main loop )---

