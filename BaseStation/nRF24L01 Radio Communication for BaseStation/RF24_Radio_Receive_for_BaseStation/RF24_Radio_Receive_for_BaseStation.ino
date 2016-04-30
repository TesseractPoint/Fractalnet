/* 

=============================================================    
  FractalNet BaseStation code for receiving data from the DataGlove.  Uses RF24 Library provided by TMRh20.  
  See below for details.  Submitted for 2016 NASA International Space Apps Challenge for "Rock-it Fashion and Design."

  HARDWARE USED:
  Arduino Uno (clone)
  nRF24L01 Radio Module

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
   Questions: terry@yourduino.com */

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


/*
  Create a data structure for transmitting and receiving data
  This allows many variables to be easily sent and received in a single transmission
  See http://www.cplusplus.com/doc/tutorial/structures/
*/

struct dataStruct {
  unsigned long _micros;       // to save response times
  int yaw;                     // initialize format for saving DataGlove metrics into variables 
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
  Serial.println(F("Receiving DataGlovek data by nRF24L01 radio from another Arduino"));
//  Serial.println(F("and control servos if attached (Check 'hasHardware' variable"));
//  printf_begin(); // Needed for "printDetails" Takes up some memory
  /*-----( Set up servos )-----*/


  radio.begin();                      // Initialize the nRF24L01 Radio
  radio.setChannel(108);              // 2.508 Ghz - Above most Wifi Channels
  radio.setDataRate(RF24_250KBPS);    // Fast enough.. Better range

  // Set the Power Amplifier Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  // PALevelcan be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_LOW);
  //   radio.setPALevel(RF24_PA_MAX);

  radio.openWritingPipe(addresses[1]);   // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openReadingPipe(1, addresses[0]);
  
  radio.startListening();   // Start the radio listening for data
  
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

    Serial.print(myData.yaw);                            // Send all DataGlove metrics to computer via Serial port.
    Serial.print(",");
    Serial.print(myData.pitch);
    Serial.print(",");
    Serial.print(myData.roll);
    Serial.print(",");
    Serial.print(myData.ThumbSensorValue);
    Serial.print(",");
    Serial.print(myData.IndexSensorValue);
    Serial.print(",");
    Serial.print(myData.MiddleSensorValue);
    Serial.print(",");
    Serial.print(myData.RingSensorValue);
    Serial.print(",");
    Serial.println(myData.HeartRate);
  }

delay(2);

}//--(end main loop )---

