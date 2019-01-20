/*
 * UART tester 1.
 * Send byte from Arduino console to Hardware UART TX then read it with RX and output it to Arduino console.
 * 
 */

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h> 

//#define DEBUG
#define REPEATER

int UART_RX = 16; // ESP RX2
int UART_TX = 17; // ESP TX2

int BLUE_LED = 2;

HardwareSerial MySerial(2);

/////// SCREEN start
#define TFT_CS    13
#define TFT_RST   4 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC    15
#define TFT_SCLK  18  // Clock out
#define TFT_MOSI  23  // Data out

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
/////// SCREEN END

/////// LoRa start
#define LORA_M0   19
#define LORA_M1   21   
#define LORA_AUX  5 

const int LORA_BUFFER_SIZE = 58;
byte LORA_BUFFER[LORA_BUFFER_SIZE];


const int ERRORS_WINDOW_SIZE = 100;
boolean ERRORS[ERRORS_WINDOW_SIZE];

/////// LoRa end

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);

  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW); // switch off blue LED
  
  tft.init(240, 240);   

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

  // LoRa
  for(int k=0; k<LORA_BUFFER_SIZE; k++) {
    LORA_BUFFER[k] = 0;
  }
  
  configLora();

  for (int k=0; k<ERRORS_WINDOW_SIZE; k++) {
    ERRORS[k] = false;
  }
}

int screenLine = 0;
int lineHeight = 20;

int errorIndex = 0;
void loop() {

  #ifdef REPEATER
  
  // LoRa repeater
  int len = 0;
  while(len < 1) {
    len = receiveLora(LORA_BUFFER);
  }
  
  if (len > 0) {
    digitalWrite(BLUE_LED, HIGH); // switch on blue LED
    sendLora(String((char*) LORA_BUFFER).substring(0, len));
    waitLoraAvailable();
    digitalWrite(BLUE_LED, LOW); // switch off blue LED
  }
  
  #else

  // LoRa emeter
  String message = "ping " + String(esp_random());
  digitalWrite(BLUE_LED, HIGH); // switch on blue LED
  sendLora(message);
  long pingTime;
  int counter = 0;
  String receivedMessage = "";
  long endTime;

  waitLoraAvailable();
  digitalWrite(BLUE_LED, LOW); // switch off blue LED
  
  long startTime = esp_timer_get_time();
  
  int len = 0; //receiveLora(LORA_BUFFER);
  
  while (len < 1 && counter < 50) {
    len = receiveLora(LORA_BUFFER);
    counter ++;
  }

  if (len > 0) {
    endTime = esp_timer_get_time();
    pingTime = (endTime - startTime)/1000;
    receivedMessage = String((char*) LORA_BUFFER).substring(0, len);
    ERRORS[errorIndex] = message != receivedMessage;
    
  } else {
    ERRORS[errorIndex] = true;
    pingTime = 88888;
  }

  int errorsCount = 0;
  for (int k=0; k<ERRORS_WINDOW_SIZE; k++) {
    if (ERRORS[k]) {
      errorsCount ++;
    }
  }

  updateScreenDisplay(pingTime, counter, errorsCount, message, receivedMessage);
  
  delay(1000);

  errorIndex = (errorIndex + 1) % ERRORS_WINDOW_SIZE;

  #endif
}

void updateScreenDisplay(int pingTime, int counter, int errorsCount, String sentMsg, String receivedMsg) {
  tft.fillRect(100, 0, 140, 90, ST77XX_BLACK);
  tft.fillRect(0, 120, 240, 30, ST77XX_BLACK);
  tft.fillRect(0, 180, 240, 30, ST77XX_BLACK);
  tft.setCursor(100, 0);
  tft.setTextColor(ST77XX_GREEN);
  tft.println(String(pingTime) + " ms");
  tft.setCursor(100, 30);
  tft.setTextColor(ST77XX_BLUE);
  tft.println(String(counter));
  tft.setCursor(100, 60);
  tft.setTextColor(ST77XX_RED);
  tft.println(String(errorsCount) + " %");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 120);
  tft.println(sentMsg);
  tft.setCursor(10, 180);
  tft.println(receivedMsg);
}

int receiveLora(byte *buffer) {
  int len = readLora(buffer);

  //waitLoraAvailable();

  return len;
}

int sendLora(String message) {
  waitLoraAvailable();
  
  message.getBytes(LORA_BUFFER, LORA_BUFFER_SIZE);
  int len = writeLora(LORA_BUFFER, message.length());

  return len;
}

void configLora() {
  // serial port 9600 & 8N1 for sleep mode only

  Serial.println("Setup LoRa device ...");
  // Set Mode 3 Sleep
  digitalWrite(LORA_M0, HIGH); 
  digitalWrite(LORA_M1, HIGH); 
  
  //pinMode(UART_RX, INPUT_PULLUP);
  //pinMode(UART_RX, INPUT);
  //pinMode(UART_TX, OUTPUT);
  //pinMode(UART_TX, INPUT);
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  //pinMode(LORA_AUX, INPUT_PULLUP);// Pull UP resistor on LORA_AUX
  pinMode(LORA_AUX, INPUT);

  MySerial.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
  MySerial.setTimeout(100);
  
  // Wait AUX to be High indicating the RESET is complete. 
  waitLoraAvailable();

  Serial.println("LoRa is in sleep mode.");

  // C0: Save config beyond reset
  // 0000: Address
  // 1A: "Speed" 9600 baups (UART) + 2.4k baups (Air rate) (SF ?)
  // 06: Channel 06
  // 44: "Options" TXD and AUX push-pull outputs, RXD pull-up inputs + Turn on FEC (default) for better range
  
  //byte config[] = {0xC0, 0x00, 0x00, 0x1A, 0x06, 0x44}; // air data rate: 2k4
  //byte config[] = {0xC0, 0x00, 0x00, 0x19, 0x06, 0x44}; // air data rate: 1.2k
  byte config[] = {0xC0, 0x00, 0x00, 0x18, 0x06, 0x44}; // air data rate: 0.3k

  byte askSavedParameters[] = {0xC1, 0xC1, 0xC1};
  byte askDeviceVersion[] = {0xC3, 0xC3, 0xC3};
  byte resetDevice[] = {0xC4, 0xC4, 0xC4};

  //MySerial.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
  //MySerial.setTimeout(1);
  
  //MySerial.write(askDeviceVersion, 3);
  writeLora(askDeviceVersion, 3);
  
  waitLoraAvailable();

  readLora(LORA_BUFFER);
  
  waitLoraAvailable();

  writeLora(config, 6);
  
  waitLoraAvailable();

  readLora(LORA_BUFFER);

  waitLoraAvailable();

  writeLora(askSavedParameters, 3);
  
  waitLoraAvailable();
  
  readLora(LORA_BUFFER);

  // Set Mode 0 Normal
  digitalWrite(LORA_M0, LOW); 
  digitalWrite(LORA_M1, LOW); 
    
  waitLoraAvailable();

  Serial.println("LoRa is in normal mode.");
}

int writeLora(byte *data, int length) {
  int len = MySerial.write(data, length);

  #ifdef DEBUG
  Serial.print("Written LoRa message: [");
  PrintHex8(data, len);
  Serial.println("]");
  #endif
  
  return len;
}

int readLora(byte *buffer) {
  int len = MySerial.readBytes(buffer, LORA_BUFFER_SIZE);
  
  #ifdef DEBUG
  Serial.print("Read LoRa message: [");
  PrintHex8(buffer, len);
  Serial.println("]");
  #endif
  
  return len;
}

void waitLoraAvailable() {
  // FIXME use rising edge on AUX ?
  delay(1);
  while (digitalRead(LORA_AUX) == LOW) {
    delay(1);
    #ifdef DEBUG
    Serial.println("LoRa AUX is LOW.");
    delay(1000);
    #endif
  }
  delay(3);
}

// prints 8-bit data in hex with leading zeroes
void PrintHex8(uint8_t *data, uint8_t length) {
  char tmp[16];
  for (int i=0; i<length; i++) {
    sprintf(tmp, "0x%.2X",data[i]);
    Serial.print(tmp); Serial.print(" ");
  }
}
