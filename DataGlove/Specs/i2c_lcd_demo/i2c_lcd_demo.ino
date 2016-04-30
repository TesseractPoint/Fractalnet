#include "Wire.h"
//////////////////  INCLUDE LIQUID CRYSTAL I2C LIBRARY
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

//////////////////  END OF LIQUID CRYSTAL


void setup() {
  // put your setup code here, to run once:
    // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.print("DataGlove v1.0");

}

void loop() {
  // put your main code here, to run repeatedly:

}
