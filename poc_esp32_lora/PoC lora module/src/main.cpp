#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>

#define GREENLED (25)

#define OLED_RESET U8X8_PIN_NONE
#define OLED_SDA (21)
#define OLED_SCL (22)

U8X8_SSD1306_128X64_NONAME_HW_I2C display = U8X8_SSD1306_128X64_NONAME_HW_I2C(OLED_SCL, OLED_SDA, OLED_RESET);
uint8_t col = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  display.begin();  // initialize U8x8 text library for selected display module
  display.setFont(u8x8_font_victoriabold8_r);  // set text font (8x8 pixels)

  delay(1000);
  pinMode(GREENLED, OUTPUT);
  Serial.println("Hello");

  display.clear();
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(GREENLED, LOW);
  Serial.println("Green LED off");
  delay(100);

  digitalWrite(GREENLED, HIGH);
  Serial.println("Green LED on");
  
  display.drawString(0, col, "Hello, Max !");  // print text to screen (column 0, row 0)
  delay(100);
  //display.clearLine(col);
  //display.clear();
  col = col + 1 % 8;
}