/*
 * UART tester 1.
 * Send byte from Arduino console to Hardware UART TX then read it with RX and output it to Arduino console.
 * 
 */

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h> 

int UART_RX = 16; // ESP RX2
int UART_TX = 17; // ESP TX2

int LED_BLUE = 2;

HardwareSerial MySerial(1);

/////// SCREEN start
#define TFT_CS    22
#define TFT_RST   4 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC    2
#define TFT_SCLK  18  // Clock out
#define TFT_MOSI  23  // Data out

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

void setup() {
  pinMode(UART_RX, INPUT);
  pinMode(UART_TX, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  Serial.begin(9600);
  Serial.setTimeout(1);
  MySerial.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
  MySerial.setTimeout(1);

  tft.init(240, 240);   

  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(2);
  tft.println("Hello World!");

  tft.setTextColor(ST77XX_WHITE);
}

int screenLine = 0;
int lineHeight = 20;

void loop() {
  //Serial.println("Loop start");
  
  while (Serial.available()) {
    byte consoleByte[1];
    int len = Serial.readBytes(consoleByte, 1);
    if (len > 0) {
      //Serial.print("Sending [");
      //Serial.print(consoleByte[0]);
      //Serial.println("] to UART");

      //Serial.write(consoleByte);
      MySerial.write(consoleByte, 1);
    }
  }

  while (MySerial.available()) {
    byte uartByte[20] = "";
    int len = MySerial.readBytesUntil('\n', uartByte, 20);
    if (len > 0) {
      //Serial.print("Reading [");
      //Serial.print(uartByte[0]);
      //Serial.println("] from UART");

      String message = String((char*)uartByte);

      Serial.println(message);

      int y = (screenLine * lineHeight) % (240);

      //tft.setTextWrap(false);
      //tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(0, y);
      //tft.setTextColor(ST77XX_WHITE);
      //tft.setTextSize(2);
      tft.fillRect(0, y, 240, lineHeight, ST77XX_BLACK);
      tft.println(message);

      screenLine += 1;
    }
  }

  //Serial.println("Loop end");
}

void blink() {
  digitalWrite(LED_BLUE, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BLUE, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
  
}
