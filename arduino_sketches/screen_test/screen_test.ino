/*
 * UART tester 1.
 * Send byte from Arduino console to Hardware UART TX then read it with RX and output it to Arduino console.
 * 
 */

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h> 

int BLUE_LED = 2;

HardwareSerial MySerial(2);

/////// SCREEN start
#define TFT_CS    -1
#define TFT_RST   22 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC    15
#define TFT_SCLK  18  // Clock out
#define TFT_MOSI  19  // Data out

static const int spiClk = 1000000; // 1 MHz

void setup() {
  Serial.begin(115200);

  #ifdef ESP32
  Serial.println("ESP32 defined");
  #endif
  
  #ifdef SPI_HAS_TRANSACTION
  Serial.println("SPI_HAS_TRANSACTION defined");
  #endif
  
  #ifdef USE_FAST_PINIO
  Serial.println("USE_FAST_PINIO defined");
  #endif
  
  #ifdef __AVR__
  Serial.println("__AVR__ defined");
  #endif
  
  #ifdef USE_SPI_DMA
  Serial.println("USE_SPI_DMA defined");
  #endif
  
  //SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_DC);
  SPIClass * vspi = new SPIClass(VSPI);
  //vspi->setClockDivider(SPI_CLOCK_DIV32);
  vspi->setFrequency(100000);
  vspi->begin(TFT_SCLK, -1, TFT_MOSI, TFT_DC);

  // Configure SPI slave select has output
  pinMode(5, OUTPUT);
  pinMode(TFT_DC, OUTPUT);

  digitalWrite(5, HIGH);
  digitalWrite(TFT_DC, HIGH);

  vspi->end();
 
  //vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(TFT_DC, LOW);

  //Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
  //Adafruit_ST7789 tft = Adafruit_ST7789(vspi, TFT_CS, TFT_DC, TFT_RST);
  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
  
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW); // switch off blue LED

  tft.init(240, 240);   
  tft.setRotation(2);

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("ping:");
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_BLUE);
  tft.println("loop:");
  tft.setCursor(0, 60);
  tft.setTextColor(ST77XX_RED);
  tft.println("errors:");
  tft.setCursor(0, 90);
  tft.setTextColor(ST77XX_WHITE);
  tft.println("Sent message:");
  tft.setCursor(0, 150);
  tft.println("Received message:");

  vspi->endTransaction();
}

void loop() {

}
