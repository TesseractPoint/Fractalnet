/*

============================================================= 

  FractalNet BaseStation code for receiving GPS and Weather data to send to computer via Serial port.
  See below for details.  Submitted for 2016 NASA International Space Apps Challenge for "Rock-it Fashion and Design."

  This code was written specifically for the following hardware:
  Arduino Uno (clone)
  iTead Studio GPS Shield for Arduino 
  Sparkfun USB Weather Board v2
  
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
 */

 
#include <SoftwareSerial.h>

SoftwareSerial myGPSSerial(2, 3); // RX, TX
SoftwareSerial myWeatherSerial(4, 5); // RX, TX

long int i;    // setup variable for "for loop."  Needs to be 'long' due to size.

void setup()   /****** SETUP: RUNS ONCE ******/
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // set the data rate for the SoftwareSerial ports
  myGPSSerial.begin(9600);
  myWeatherSerial.begin(9600);
  
}

void loop()    /****** LOOP: RUNS CONSTANTLY ******/
{
 
  myGPSSerial.listen();                         // Start listening to the GPS port and flush any bytes 
  myGPSSerial.flush();
  delay(500);
  
      for(i = 1; i < 80000; i += 1) {           // reads the GPS data for a short time
         if (myGPSSerial.available()){
            Serial.write(myGPSSerial.read());
             }
         }
         
  Serial.println();
  
  delay(200);
  
  myWeatherSerial.listen();                    // Start listening to the Weather port and flush any bytes
  myWeatherSerial.flush();
  
  delay(500);
  
      for(i = 1; i < 100000; i += 1) {          // reads the Weather data for a short time
         if (myWeatherSerial.available()){
          Serial.write(myWeatherSerial.read()); 
            }      
         }

  Serial.println();
  
  delay(500);
  
}              //--(end main loop )---

