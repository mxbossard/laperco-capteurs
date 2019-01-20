/*
 * UART tester 1.
 * Send byte from Arduino console to Hardware UART TX then read it with RX and output it to Arduino console.
 * 
 */

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h> 

//#define DEBUG_NET
#define DEBUG_PROTOCOL

//#define REPEATER

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


/////// PROTOCOL
const int LORA_UART_TIMEOUT = 20;

const int MIN_AIR_RATE = 0;
const int MAX_AIR_RATE = 5;
const int AIR_DATA_RATE[6] = {300, 1200, 2400, 4800, 9600, 19200};
const int AIR_RATE_TIMEOUT_MULTIPLIER[6] = {30, 20, 10, 5, 4, 3}; // Affect WAIT_ACK_TIMEOUT and TOO_FAST_TIMEOUT, but does not affect LINK_DEAD_TIMEOUT

const int START_SEQ_ID = 42;

#ifdef REPEATER
const int SPEED_INC_VALID_MSG_COUNT = 5;
const int SPEED_INC_REQ_COUNT = 2;
const long WAIT_ACK_TIMEOUT = 250000; // multiplied by AIR_RATE_TIMEOUT_MULTIPLIER
const long TOO_FAST_TIMEOUT = 3 * WAIT_ACK_TIMEOUT + 1000000;
const long LINK_DEAD_TIMEOUT = 23000000; //(TOO_FAST_TIMEOUT * 3 + 1000000) * 1.2;
#else
const int SPEED_INC_VALID_MSG_COUNT = 5;
const int SPEED_INC_REQ_COUNT = 2;
const long WAIT_ACK_TIMEOUT = 250000; // multiplied by AIR_RATE_TIMEOUT_MULTIPLIER
const long TOO_FAST_TIMEOUT = 3 * WAIT_ACK_TIMEOUT + 1000000;
const long LINK_DEAD_TIMEOUT = 20000000; //TOO_FAST_TIMEOUT * 3 + 1000000;
const long WAIT_BEFORE_SPEED_INC_INTERVAL = 20000000;
#endif

const int ERRORS_WINDOW_SIZE = 100;
boolean ERRORS[ERRORS_WINDOW_SIZE];

/////// LoRa end

void setup() {
  #if  defined (DEBUG_NET)  || defined (DEBUG_PROTOCOL)
  Serial.begin(9600);
  Serial.setTimeout(1);
  #endif

  #ifndef REPEATER
  initScreenDisplay();
  #endif
  
}

void init() {
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW); // switch off blue LED

  // LoRa
  for(int k=0; k<LORA_BUFFER_SIZE; k++) {
    LORA_BUFFER[k] = 0;
  }
  
  configLora(0);

  for (int k=0; k<ERRORS_WINDOW_SIZE; k++) {
    ERRORS[k] = false;
  }
}


int errorIndex = 0;

void loop() {

  // If we loop reinit.
  init();

  // blink led for 500ms
  digitalWrite(BLUE_LED, HIGH);
  delay(500);
  digitalWrite(BLUE_LED, LOW);

  #ifdef REPEATER
  // LoRa repeater
    
  #ifdef DEBUG_PROTOCOL
  Serial.println("LoRa Gateway repeater mode.");
  #endif
  
  String sessionId = "";
  int seqId = START_SEQ_ID - 2; // ID of last acquitted message
  int airRate = 0;
  int sequentialValidMessages = 0;
  int speedIncreaseReqCount = 0;
  int tooFastAttempt = 0;

  long lastMessageReceivedTimestamp = esp_timer_get_time();
  long lastValidMessageReceivedTimestamp = esp_timer_get_time();

  #ifdef DEBUG_PROTOCOL
  Serial.println("Message session reset.");
  #endif
  
  while ((lastValidMessageReceivedTimestamp + LINK_DEAD_TIMEOUT) > esp_timer_get_time()) {
  
    tooFastAttempt ++;
    //int tooFastTimeout = (TOO_FAST_TIMEOUT * AIR_RATE_TIMEOUT_MULTIPLIER[airRate] * max(1, tooFastAttempt));
    int tooFastDeadline = esp_timer_get_time() + TOO_FAST_TIMEOUT * AIR_RATE_TIMEOUT_MULTIPLIER[airRate];
    
    //while ((lastMessageReceivedTimestamp + tooFastTimeout) > esp_timer_get_time() && (lastValidMessageReceivedTimestamp + LINK_DEAD_TIMEOUT) > esp_timer_get_time()) {
    while (tooFastDeadline >= esp_timer_get_time() && (lastValidMessageReceivedTimestamp + LINK_DEAD_TIMEOUT) > esp_timer_get_time()) {
    
      String receivedMsg = receiveLoraMesssage();
      if (receivedMsg == "") {
        // Continue looping if no message
        continue;
      }
      int firstMsgSep = receivedMsg.indexOf('_', 0);
      int secondMsgSep = receivedMsg.indexOf('_', firstMsgSep + 1);
      if (firstMsgSep < 1 || secondMsgSep < 3 || firstMsgSep == secondMsgSep) {
        // Continue looping if no well formatted message
        
        #ifdef DEBUG_PROTOCOL
        Serial.println("ERROR: Received a malformed message: [" + receivedMsg + "]");
        #endif
        
        continue;
      }

      lastMessageReceivedTimestamp = esp_timer_get_time();
      tooFastDeadline = lastMessageReceivedTimestamp + TOO_FAST_TIMEOUT * AIR_RATE_TIMEOUT_MULTIPLIER[airRate];
      
      String msgSessionId = receivedMsg.substring(0, firstMsgSep);
      long msgSeqId = receivedMsg.substring(firstMsgSep + 1, secondMsgSep).toInt();
      int msgRequestedAirRate = receivedMsg.substring(secondMsgSep + 1, receivedMsg.length()).toInt();
      
      #ifdef DEBUG_PROTOCOL
      Serial.println("Received a wellformed message: [" + receivedMsg + "] => {sessionId: " + msgSessionId + ", seqId: " + String(msgSeqId, DEC) + ", airRateReq: " + msgRequestedAirRate + "}");
      #endif

      tooFastAttempt = 0;

      //if (sessionId == "" && msgSeqId == 1) {
      if (msgSeqId == START_SEQ_ID) {
        // We get a new sessionId
        sessionId = msgSessionId;
        seqId = START_SEQ_ID - 2;
        #ifdef DEBUG_PROTOCOL
        Serial.println("New sessionId: [" + sessionId + "]");
        #endif
      }

      if (sessionId == msgSessionId && msgSeqId <= seqId + 2) {
        // If already acked seqId we respond as well

        lastValidMessageReceivedTimestamp = lastMessageReceivedTimestamp;
        String ack = msgSessionId + "_" + String(msgSeqId + 1, DEC) + "_";
        
        //if (msgSeqId == seqId + 1 && msgRequestedAirRate > 0 && msgRequestedAirRate == airRate + 1) {
        if (msgRequestedAirRate > 0 && msgRequestedAirRate == airRate + 1) {
          speedIncreaseReqCount ++;
          ack = ack + String(airRate + 1, DEC);
        } else {
          speedIncreaseReqCount = 0;
        }
          
        sendLoraMessage(ack);
        
        #ifdef DEBUG_PROTOCOL
        Serial.println("Sent message: [" + ack + "]");
        #endif
        
        if (speedIncreaseReqCount >= SPEED_INC_REQ_COUNT) {
          speedIncreaseReqCount = 0;
          airRate = min(airRate + 1, MAX_AIR_RATE);

          #ifdef DEBUG_PROTOCOL
          Serial.println("Try to increase air rate to: " + String(airRate, DEC));
          #endif
              
          configLora(airRate);

          
          tooFastDeadline = lastMessageReceivedTimestamp + TOO_FAST_TIMEOUT * AIR_RATE_TIMEOUT_MULTIPLIER[airRate];
        }
          
      }

      if (sessionId == msgSessionId && msgSeqId == seqId + 2) {
        // Increment seqId only if the message was after our previously registered seqId
        seqId = msgSeqId;
      }
    }
    
    //if ((lastMessageReceivedTimestamp + tooFastTimeout) <= esp_timer_get_time()) {
    if (tooFastDeadline < esp_timer_get_time()) {
      sequentialValidMessages = 0;
      int newAirRate = max(airRate - 1, MIN_AIR_RATE);
      
      #ifdef DEBUG_PROTOCOL
      Serial.println("No ACK received for TOO_FAST_TIMEOUT long, so we decrease Air Rate to: " + String(newAirRate, DEC));
      #endif
      
      if (newAirRate != airRate) {
        airRate = newAirRate;
        configLora(airRate);
      }
    }
      
  }
    
  
  #else
  // LoRa emeter
  
  #ifdef DEBUG_PROTOCOL
  Serial.println("LoRa Emiter mode.");
  #endif
  
  String sessionId = String(esp_random());
  int seqId = START_SEQ_ID;
  int airRate = 0;
  int sequentialValidMessages = 0;
  int messageCountForFixedAirRate = 0;
  int speedIncreaseReqCount = 0;
  int ackTimeoutCount = 0;
  
  long lastMessageReceivedTimestamp = esp_timer_get_time();
  long lastValidMessageReceivedTimestamp = esp_timer_get_time();
  long lastMessageSentTimestamp = esp_timer_get_time();
  long lastTooFastTimeout = 0;

  #ifdef DEBUG_PROTOCOL
  Serial.println("Message session reset. New sessionId: " + sessionId);
  #endif

  int tooFastAttempt = 0;
  int ackAttempt = 0;

  // init screen display
  updateReceivedScreenDisplay("-", airRate, "init", "init", "-");
    
  while (lastValidMessageReceivedTimestamp + LINK_DEAD_TIMEOUT > esp_timer_get_time()) {
    
    tooFastAttempt ++;
    //int tooFastTimeout = (TOO_FAST_TIMEOUT * AIR_RATE_TIMEOUT_MULTIPLIER[airRate] * max(1, tooFastAttempt));
    int tooFastDeadline = esp_timer_get_time() + TOO_FAST_TIMEOUT * AIR_RATE_TIMEOUT_MULTIPLIER[airRate];
    
    //while ((lastMessageReceivedTimestamp + tooFastTimeout) > esp_timer_get_time() && (lastValidMessageReceivedTimestamp + LINK_DEAD_TIMEOUT) > esp_timer_get_time()) {
    while (tooFastDeadline >= esp_timer_get_time() && (lastValidMessageReceivedTimestamp + LINK_DEAD_TIMEOUT) > esp_timer_get_time()) {
      // Try to send a message and get an ACK for TOO_FAST_TIMEOUT

      boolean notAckedMessage = true;
      String message = sessionId + "_" + seqId + "_";
      if (airRate < MAX_AIR_RATE && (lastTooFastTimeout + WAIT_BEFORE_SPEED_INC_INTERVAL) < esp_timer_get_time() && messageCountForFixedAirRate >= SPEED_INC_VALID_MSG_COUNT) {
        message = message + String(airRate + 1);
      }
    
      notAckedMessage = true;
      sendLoraMessage(message);
      lastMessageSentTimestamp = esp_timer_get_time();
      updateSentScreenDisplay(airRate, (WAIT_ACK_TIMEOUT * AIR_RATE_TIMEOUT_MULTIPLIER[airRate]) / 1000, message);
      
      #ifdef DEBUG_PROTOCOL
      Serial.println("Sent message: [" + message + "] with air rate: " + airRate);
      #endif 
      
      ackAttempt ++;
      int waitAckTimeout = WAIT_ACK_TIMEOUT * AIR_RATE_TIMEOUT_MULTIPLIER[airRate];
      
      //while (notAckedMessage && (lastMessageSentTimestamp + waitAckTimeout) > esp_timer_get_time() && (lastMessageReceivedTimestamp + tooFastTimeout) > esp_timer_get_time() && (lastValidMessageReceivedTimestamp + LINK_DEAD_TIMEOUT) > esp_timer_get_time()) {
      while (notAckedMessage && (lastMessageSentTimestamp + waitAckTimeout) > esp_timer_get_time() && tooFastDeadline >= esp_timer_get_time() && (lastValidMessageReceivedTimestamp + LINK_DEAD_TIMEOUT) > esp_timer_get_time()) {
        // Try to get an ACK for WAIT_ACK_TIMEOUT

        String receivedMsg = receiveLoraMesssage();
        if (receivedMsg == "") {
          // Continue looping if no message
          continue;
        }

        int firstMsgSep = receivedMsg.indexOf('_', 0);
        int secondMsgSep = receivedMsg.indexOf('_', firstMsgSep + 1);
        if (firstMsgSep < 1 || secondMsgSep < 3) {
          // Continue looping if no well formatted message
          #ifdef DEBUG_PROTOCOL
          Serial.println("ERROR: Received a malformed message: [" + receivedMsg + "]");
          #endif
          continue;
        }

        lastMessageReceivedTimestamp = esp_timer_get_time();
        tooFastDeadline = lastMessageReceivedTimestamp + TOO_FAST_TIMEOUT * AIR_RATE_TIMEOUT_MULTIPLIER[airRate];
        
        String msgSessionId = receivedMsg.substring(0, firstMsgSep);
        long msgSeqId = receivedMsg.substring(firstMsgSep + 1, secondMsgSep).toInt();
        int msgRequestedAirRate = receivedMsg.substring(secondMsgSep + 1, receivedMsg.length()).toInt();
        
        #ifdef DEBUG_PROTOCOL
        Serial.println("Received a wellformed message: [" + receivedMsg + "] => {sessionId: " + msgSessionId + ", seqId: " + String(msgSeqId, DEC) + ", airRateReq: " + msgRequestedAirRate + "}");
        #endif

        tooFastAttempt = 0;
          
        if (msgSessionId == sessionId && msgSeqId == seqId + 1) {
          // Ack is valid
          sequentialValidMessages ++;
          messageCountForFixedAirRate ++;
   
          notAckedMessage = false;
          ackAttempt = 0;
          seqId = msgSeqId + 1;
          lastValidMessageReceivedTimestamp = lastMessageReceivedTimestamp;

          long pingTime = lastValidMessageReceivedTimestamp - lastMessageSentTimestamp;
          updateReceivedScreenDisplay(String(pingTime/1000, DEC), airRate, String(messageCountForFixedAirRate, DEC) + " / " + String(sequentialValidMessages, DEC), String(ackAttempt, DEC) + " / " + String(ackTimeoutCount, DEC), receivedMsg);

          if (msgRequestedAirRate > 0 && msgRequestedAirRate == airRate + 1) {
            // Received an ack for a changing air rate request
            speedIncreaseReqCount ++;
   
            if (speedIncreaseReqCount >= SPEED_INC_REQ_COUNT) {
              speedIncreaseReqCount = 0;
              airRate = min(airRate + 1, MAX_AIR_RATE);
              
              #ifdef DEBUG_PROTOCOL
              Serial.println("Try to increase air rate to: " + String(airRate, DEC));
              #endif

              messageCountForFixedAirRate = 0;
              configLora(airRate);

              tooFastDeadline = lastMessageReceivedTimestamp + TOO_FAST_TIMEOUT * AIR_RATE_TIMEOUT_MULTIPLIER[airRate];
            }
          }
   
        } else {
          sequentialValidMessages = 0;
        }
      }

      if (notAckedMessage) {
        speedIncreaseReqCount = 0;
        messageCountForFixedAirRate = 0;
        ackTimeoutCount ++;
        updateReceivedScreenDisplay("-", airRate, String(messageCountForFixedAirRate, DEC) + " / " + String(sequentialValidMessages, DEC), String(ackAttempt, DEC) + " / " + String(ackTimeoutCount, DEC), "-");
        //updateScreenDisplay("-", airRate, sequentialValidMessages, message, "-");
      }

    }
   
    //if ((lastMessageReceivedTimestamp + tooFastTimeout) <= esp_timer_get_time()) {
    if (tooFastDeadline < esp_timer_get_time()) {
      sequentialValidMessages = 0;
      int newAirRate = max(airRate - 1, MIN_AIR_RATE);

      if (airRate != 0) {
        lastTooFastTimeout = esp_timer_get_time();
      }
      
      #ifdef DEBUG_PROTOCOL
      Serial.println("No ACK received for TOO_FAST_TIMEOUT long, so we decrease Air Rate to: " + String(newAirRate, DEC));
      #endif
      
      if (newAirRate != airRate) {
        airRate = newAirRate;
        messageCountForFixedAirRate = 0;
        configLora(airRate);
      }

      
    }
    
    //delay(1000);
  }

  //delay(1000);

  #endif
}

void initScreenDisplay() {
  tft.init(240, 240);   

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_CYAN);
  tft.println("ping:");
  tft.setCursor(0, 24);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("rate:");
  tft.setCursor(0, 48);
  tft.setTextColor(ST77XX_YELLOW);
  tft.println("timeout:");
  tft.setCursor(0, 72);
  tft.setTextColor(ST77XX_ORANGE);
  tft.println("streak:");
  tft.setCursor(0, 96);
  tft.setTextColor(ST77XX_RED);
  tft.println("miss:");
  tft.setCursor(0, 120);
  tft.setTextColor(ST77XX_WHITE);
  tft.println("Sent message:");
  tft.setCursor(0, 180);
  tft.println("Received message:");
}

void updateSentScreenDisplay(int airRate, int timeout, String sentMsg) {
  tft.fillRect(100, 24, 140, 48, ST77XX_BLACK);
  tft.fillRect(0, 150, 240, 30, ST77XX_BLACK);
  tft.setCursor(100, 24);
  tft.setTextColor(ST77XX_GREEN);
  tft.println(String(AIR_DATA_RATE[airRate], DEC) + " bps");
  tft.setCursor(100, 48);
  tft.setTextColor(ST77XX_YELLOW);
  tft.println(String(timeout) + " ms");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 150);
  tft.println(sentMsg);
}

void updateReceivedScreenDisplay(String pingTime, int airRate, String streak, String miss, String receivedMsg) {
  tft.fillRect(100, 0, 140, 48, ST77XX_BLACK);
  tft.fillRect(100, 72, 140, 48, ST77XX_BLACK);
  tft.fillRect(0, 210, 240, 30, ST77XX_BLACK);
  tft.setCursor(100, 0);
  tft.setTextColor(ST77XX_CYAN);
  tft.println(pingTime + " ms");
  tft.setCursor(100, 24);
  tft.setTextColor(ST77XX_GREEN);
  tft.println(String(AIR_DATA_RATE[airRate], DEC) + " bps");
  tft.setCursor(100, 72);
  tft.setTextColor(ST77XX_ORANGE);
  tft.println(streak);
  tft.setCursor(100, 96);
  tft.setTextColor(ST77XX_RED);
  tft.println(miss);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 210);
  tft.println(receivedMsg);
}

void updateScreenDisplay(String pingTime, int airRate, String streak, String sentMsg, String receivedMsg) {
  tft.fillRect(100, 0, 140, 90, ST77XX_BLACK);
  tft.fillRect(0, 120, 240, 30, ST77XX_BLACK);
  tft.fillRect(0, 180, 240, 30, ST77XX_BLACK);
  tft.setCursor(100, 0);
  tft.setTextColor(ST77XX_GREEN);
  tft.println(pingTime + " ms");
  tft.setCursor(100, 30);
  tft.setTextColor(ST77XX_BLUE);
  tft.println(String(AIR_DATA_RATE[airRate], DEC) + " bps");
  tft.setCursor(100, 60);
  tft.setTextColor(ST77XX_RED);
  tft.println(streak);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 120);
  tft.println(sentMsg);
  tft.setCursor(10, 180);
  tft.println(receivedMsg);
}

String receiveLoraMesssage() {
  String msg = "";
  int len = receiveLora(LORA_BUFFER);
  if (len > 0) {
    msg = String((char*)LORA_BUFFER).substring(0, len);
  }

  return msg;
}

int receiveLora(byte *buffer) {
  int len = readLora(buffer);

  //waitLoraAvailable();

  return len;
}

int sendLoraMessage(String message) {
  digitalWrite(BLUE_LED, HIGH); // switch on blue LED
  
  waitLoraAvailable();
  
  message.getBytes(LORA_BUFFER, LORA_BUFFER_SIZE);
  int len = writeLora(LORA_BUFFER, message.length());

  waitLoraAvailable();
  digitalWrite(BLUE_LED, LOW); // switch off blue LED
    
  return len;
}

void configLora(int airRate) {
  // serial port 9600 & 8N1 for sleep mode only

  #ifdef DEBUG_PROTOCOL
  Serial.println("Setup LoRa device...");
  Serial.println("Selected airRate: " + String(airRate, DEC));
  #endif

  waitLoraAvailable();
  
  // Set Mode 3 Sleep
  digitalWrite(LORA_M0, HIGH); 
  digitalWrite(LORA_M1, HIGH); 
  
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);

  MySerial.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
  MySerial.setTimeout(LORA_UART_TIMEOUT);
  
  // Wait AUX to be High indicating the RESET is complete. 
  waitLoraAvailable();

  #ifdef DEBUG_PROTOCOL
  Serial.println("LoRa is in sleep mode.");
  #endif
  
  airRate = max(airRate, MIN_AIR_RATE);
  airRate = min(airRate, MAX_AIR_RATE);

  // C0: Save config beyond reset
  // 0000: Address
  // 1A: "Speed" 9600 baups (UART) + 2.4k baups (Air rate) (SF ?)
  // 06: Channel 06
  // 44: "Options" TXD and AUX push-pull outputs, RXD pull-up inputs + Turn on FEC (default) for better range
  
  //byte config[] = {0xC0, 0x00, 0x00, 0x1A, 0x06, 0x44}; // air data rate: 2k4
  //byte config[] = {0xC0, 0x00, 0x00, 0x19, 0x06, 0x44}; // air data rate: 1.2k
  //byte config[] = {0xC0, 0x00, 0x00, 0x18, 0x06, 0x44}; // air data rate: 0.3k
  byte config[] = {0xC0, 0x00, 0x00, 24 + airRate, 0x06, 0x44}; // air data rate: 0.3k

  byte askSavedParameters[] = {0xC1, 0xC1, 0xC1};
  byte askDeviceVersion[] = {0xC3, 0xC3, 0xC3};
  byte resetDevice[] = {0xC4, 0xC4, 0xC4};

  int len;

  /*
  waitLoraAvailable();
  
  writeLora(askDeviceVersion, 3);
  
  waitLoraAvailable();

  readLora(LORA_BUFFER);
  */
  
  do {
    waitLoraAvailable();
  
    writeLora(config, 6);
    
    waitLoraAvailable();
  
    len = readLora(LORA_BUFFER);
    #ifdef DEBUG_PROTOCOL
    Serial.print("LoRa config returned: ");
    PrintHex8(LORA_BUFFER, len);
    Serial.println("");
    #endif
    
    //delay(10);
  } while(config[3] != LORA_BUFFER[3]);

  /*
  waitLoraAvailable();

  writeLora(askSavedParameters, 3);
  
  waitLoraAvailable();

  
  len = readLora(LORA_BUFFER);
  #ifdef DEBUG_PROTOCOL
  Serial.print("LoRa saved parameters: ");
  PrintHex8(LORA_BUFFER, len);
  Serial.println("");
  #endif
  */
    
  // Set Mode 0 Normal
  digitalWrite(LORA_M0, LOW); 
  digitalWrite(LORA_M1, LOW); 
    
  waitLoraAvailable();

  #ifdef DEBUG_PROTOCOL
  Serial.println("LoRa is in normal mode.");
  #endif
}

int writeLora(byte *data, int length) {
  int len = MySerial.write(data, length);

  #ifdef DEBUG_NET
  Serial.print("Written LoRa message: [");
  PrintHex8(data, len);
  Serial.println("]");
  #endif
  
  return len;
}

int readLora(byte *buffer) {
  int len = MySerial.readBytes(buffer, LORA_BUFFER_SIZE);
  
  #ifdef DEBUG_NET
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
    #ifdef DEBUG_NET
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
