/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include "EEPROM.h"

// ##### LMIC (LoRaWAN) #####
#define ARDUINO_LMIC_PROJECT_CONFIG_H /home/maxbundy/git/laperco-capteurs/arduino_sketches/lorawan_gps_tracker/lmic_project_config.h

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define SPI_CLK 18
#define SPI_MOSI 19
#define SPI_MISO 23
#define LORA_SPI_SS 5

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { 0x9F, 0x79, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { 0xDB, 0x3D, 0x3B, 0x22, 0xDF, 0xD2, 0xBA, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x76, 0xD9, 0x64, 0x0D, 0xC6, 0xE4, 0xAD, 0x88, 0xFD, 0x76, 0x62, 0x00, 0xE2, 0x15, 0xB1, 0x10 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

//static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {21, 26, 27},
    // optional: set polarity of rxtx pin.
    .rxtx_rx_active = 0,
    // optional: set RSSI cal for listen-before-talk
    // this value is in dB, and is added to RSSI
    // measured prior to decision.
    // Must include noise guardband! Ignored in US,
    // EU, IN, other markets where LBT is not required.
    .rssi_cal = 0,
    // optional: override LMIC_SPI_FREQ if non-zero
    .spi_freq = 0,
};

class MacStore {
  public:
    bool saved = false;
    u4_t netid;
    devaddr_t devaddr;
    u1_t nwkKey[16];
    u1_t artKey[16];
    u4_t seqnoUp = 0;
    u4_t seqnoDn = 0;
};


// ##### Payload LoRaWAN #####
#include <CayenneLPP.h>
CayenneLPP lpp(51);

// ###### GPS #####
HardwareSerial esp32HWSerial(2); // ESP32 HardwareSerial number 2 : RX2 TX2 (GPIO 16 & 17)
//#define GPSport_h
#define GPS_PORT_NAME esp32HWSerial
#define gpsPort esp32HWSerial
#define GPS_RX 16
#define GPS_TX 17
#include <NMEAGPS.h>

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

// ##### Screen #####
#include <TFT_eSPI.h> // Hardware-specific library
TFT_eSPI tft = TFT_eSPI();

class ScreenData {
  bool _gpsChanged = false;
  double _gpsLatitude = 0;
  double _gpsLongitude = 0;
  double _gpsAltitude = 0;
  
  bool _macEventChanged = false;
  String _macEvent = "no data";
  String _seqnoUp = "no data";
  String _seqnoDown = "no data";
  
  bool _lorawanSessionChanged = false;
  String _lorawanAppSessionId = "no data";
  String _lorawanNetSessionId = "no data";
  
  public:
    bool isUpdated() {
      return _gpsChanged || _macEventChanged || _lorawanSessionChanged;
    }
  
    void updateGps(double latitude, double longitude, double altitude) {
      _gpsChanged = true;
      _gpsLatitude = latitude;
      _gpsLongitude = longitude;
      _gpsAltitude = altitude;
    }

    String getGpsLatitude() {
      return String(_gpsLatitude, DEC);
    }

    String getGpsLongitude() {
      return String(_gpsLongitude, DEC);
    }
    
    String getGpsAltitude() {
      return String(_gpsAltitude, DEC);
    }
    
    bool isGpsUpdated() {
      return _gpsChanged;
    }

    void updateMacEvent(String event, String seqnoUp = "", String seqnoDn = "") {
      _macEventChanged = true;
      _macEvent = event;
      _seqnoUp = seqnoUp;
      _seqnoDown = seqnoDn;
    }
    
    String getMacEvent() {
      return _macEvent;
    }
        
    String getSeqnoUp() {
      return _seqnoUp;
    }
      
    String getSeqnoDown() {
      return _seqnoDown;
    }
    
    bool isMacEventUpdated() {
      return _macEventChanged;
    }

    void updateLorawanSession(String netSessionId, String appSessionId) {
      _lorawanSessionChanged = true;
      _lorawanAppSessionId = appSessionId;
      _lorawanNetSessionId = netSessionId;
    }

    String getLorawanNetSessionId() {
      return _lorawanNetSessionId;
    }
    
    String getLorawanAppSessionId() {
      return _lorawanAppSessionId;
    }
    
    bool isLorawanSessionUpdated() {
      return _lorawanSessionChanged;
    }

    void syncChanges() {
      _gpsChanged = false;
      _macEventChanged = false;
      _lorawanSessionChanged = false;
    }
};

ScreenData *screenData = new ScreenData();

void beginLoraSpi() {
  digitalWrite(TFT_DC, HIGH);
  digitalWrite(LORA_SPI_SS, LOW);
}

void beginScreenSpi() {
  digitalWrite(LORA_SPI_SS, HIGH);
  digitalWrite(TFT_DC, LOW);
}

void endSpi() {
  digitalWrite(TFT_DC, HIGH);
  digitalWrite(LORA_SPI_SS, HIGH);
}

void initScreen() {
  Serial.println(F("Init screen."));

  //SPI.endTransaction();
  
  endSpi();
  beginScreenSpi();

  tft.init();

  //tft.startWrite();
  
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(1);
  tft.setRotation(0);
  tft.setTextWrap(false);
  tft.setTextSize(3);
  
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_CYAN);
  tft.println("GPS lat/long:");
  
  tft.setCursor(0, 80);
  tft.setTextColor(TFT_GREEN);
  tft.println("MAC:");

  tft.setCursor(0, 160);
  tft.setTextColor(TFT_YELLOW);
  tft.println("NetId/AppId:");

  tft.endWrite();

  endSpi();
  beginLoraSpi();
}

int progressBarIndex = 0;

void refreshScreen(bool force = false, bool progressBar = false) {
  //Serial.println(F("Refresh screen."));
  if (force) { Serial.println(F("Force refresh screen.")); }

  if (!screenData->isUpdated() && !force && !progressBar) {
    return;
  }

  //Serial.println(F("refreshing screen..."));

  //SPI.endTransaction();

  endSpi();
  beginScreenSpi();

  //tft.init();
  //tft.invertDisplay(true);
  
  //tft.startWrite();
  
  if (force || screenData->isGpsUpdated()) {
    tft.fillRect(0, 30, 240, 50, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextWrap(false);
    tft.setTextColor(TFT_CYAN);
    tft.setCursor(0, 30);
    tft.println(screenData->getGpsLatitude());
    tft.setCursor(0, 50);
    tft.println(screenData->getGpsLongitude());
    Serial.println(F("Refreshed gps"));
  }

  if (force || screenData->isMacEventUpdated()) {
    tft.fillRect(0, 110, 240, 50, TFT_BLACK);
    tft.setCursor(0, 110);
    tft.setTextSize(2);
    tft.setTextWrap(true);
    tft.setTextColor(TFT_GREEN);
    tft.println(screenData->getMacEvent());
    tft.setCursor(0, 130);
    tft.println("up: " + screenData->getSeqnoUp() + " | down: " + screenData->getSeqnoDown());
    Serial.println(F("Refreshed mac"));
  }

  if (force || screenData->isLorawanSessionUpdated()) {
    tft.fillRect(0, 190, 240, 50, TFT_BLACK);
    tft.setCursor(0, 190);
    tft.setTextSize(1);
    //tft.setTextWrap(true);
    tft.setTextColor(TFT_YELLOW);
    tft.println(screenData->getLorawanNetSessionId());
    tft.setCursor(0, 210);
    tft.println(screenData->getLorawanAppSessionId());
    Serial.println(F("Refreshed lorawan session"));
  }
  
  screenData->syncChanges();

  if(progressBar && millis() / 10 > progressBarIndex) {
    tft.drawPixel(progressBarIndex % 240, 239, ((1 + progressBarIndex / 240) % 2) * TFT_ORANGE );
    progressBarIndex ++;
  }
  
  tft.endWrite();

  endSpi();
  beginLoraSpi();
}

bool dataToTransmit = false;

void sensorAcquisition() {
  double latitude = 0, longitude = 0, altitude = 0;

  lpp.reset();

  // GPS acquisition
  Serial.println(F("Try to acquire GPS position ..."));
  while (gps.available( gpsPort )) {
    fix = gps.read();
    
    if (fix.valid.location) {
      latitude = fix.latitude();
      longitude = fix.longitude();
    }
    
    if (fix.valid.altitude) {
      altitude = fix.altitude();
    }
    
  }
  
  // Build the new payload
  if (latitude != 0 && longitude != 0) {
    Serial.print( F("Location: ") );
    Serial.print( fix.latitude(), 6 );
    Serial.print( ',' );
    Serial.print( fix.longitude(), 6 );
    Serial.print( F(", Altitude: ") );
    Serial.print( fix.altitude() );
    Serial.println();
      
    lpp.addGPS(1, latitude, longitude, altitude);

    screenData->updateGps(latitude, longitude, altitude);

    dataToTransmit = true;
  }

  dataToTransmit = true;
}

MacStore *macStore;
void loadMacStore() {
  EEPROM.begin(256);
  EEPROM.get(0, macStore);
  EEPROM.end();
  if (macStore) {
    Serial.println(F("A saved MacStore was loaded."));
    LMIC_setSession (macStore->netid, macStore->devaddr, macStore->nwkKey, macStore->artKey);
    LMIC.seqnoUp = macStore->seqnoUp;
    LMIC.seqnoDn = macStore->seqnoDn;
    screenData->updateLorawanSession(hex8toString(macStore->nwkKey, sizeof(macStore->nwkKey)), hex8toString(macStore->artKey, sizeof(macStore->artKey)));
  } else {
    macStore = new MacStore();
    Serial.println(F("A new MacStore was initialized."));
  }
}

void saveMacStore() {
  macStore->saved = true;
  EEPROM.begin(256);
  EEPROM.put(0, macStore);
  EEPROM.commit();
  EEPROM.end();
  Serial.println(F("The MacStore was saved."));
}

void eraseMacStore() {
  macStore = new MacStore();
  EEPROM.begin(256);
  EEPROM.put(0, macStore);
  EEPROM.commit();
  EEPROM.end();
  Serial.println(F("The MacStore was erased."));
}

void testMacStore() {
  loadMacStore();
  Serial.println(F("MacStore loaded"));
  Serial.println(String("Net session Id: ") + hex8toString(macStore->nwkKey, 16));
  Serial.println(String("seqnoUp: ") + macStore->seqnoUp);

  u1_t foo[16] = { 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB };
  memcpy(macStore->nwkKey, foo, sizeof(foo));
  macStore->seqnoUp = 42;

  saveMacStore();
  Serial.println(F("MacStore saved"));
  Serial.println(String("Net session Id: ") + hex8toString(macStore->nwkKey, 16));
  Serial.println(String("seqnoUp: ") + macStore->seqnoUp);

  // Init a new MacStore to test the EEPROM reading
  macStore = new MacStore();
  
  loadMacStore();
  Serial.println(F("MacStore loaded"));
  Serial.println(String("Net session Id: ") + hex8toString(macStore->nwkKey, 16));
  Serial.println(String("seqnoUp: ") + macStore->seqnoUp);

  eraseMacStore();
  Serial.println(F("MacStore erased"));
  Serial.println(String("Net session Id: ") + hex8toString(macStore->nwkKey, 16));
  Serial.println(String("seqnoUp: ") + macStore->seqnoUp);
}

bool lorawanComputePriority = false;

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");

    String event = "nop";
    
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            event = F("EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            event = F("EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            event = F("EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            event = F("EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            event = F("EV_JOINING");
            break;
        case EV_JOINED:
            event = F("EV_JOINED");
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              //u1_t nwkKey[16];
              //u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, macStore->nwkKey, macStore->artKey);
              
              macStore->netid = LMIC.netid;
              macStore->devaddr = LMIC.devaddr;
              //macStore->nwkKey = nwkKey;
              //macStore->artKey = artKey,
              macStore->seqnoUp = LMIC.seqnoUp;
              macStore->seqnoDn = LMIC.seqnoDn;
              saveMacStore();
              screenData->updateLorawanSession(hex8toString(macStore->nwkKey, sizeof(macStore->nwkKey)), hex8toString(macStore->artKey, sizeof(macStore->artKey)));
              
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (int i=0; i<sizeof(macStore->artKey); ++i) {
                Serial.print(macStore->artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(macStore->nwkKey); ++i) {
                Serial.print(macStore->nwkKey[i], HEX);
              }
              Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	          // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            event = F("EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            event = F("EV_REJOIN_FAILED");
            break;
        case EV_TXCOMPLETE:
            event = F("EV_TXCOMPLETE");
            //EV_TXCOMPLETE (includes waiting for RX windows)
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

            macStore->seqnoUp = LMIC.seqnoUp;
            macStore->seqnoDn = LMIC.seqnoDn;
            saveMacStore();

            lorawanComputePriority = false;
              
            break;
        case EV_LOST_TSYNC:
            event = F("EV_LOST_TSYNC");
            break;
        case EV_RESET:
            event = F("EV_RESET");
            eraseMacStore();
            lorawanComputePriority = false;
            break;
        case EV_RXCOMPLETE:
            event = F("EV_RXCOMPLETE");
            // data received in ping slot

            macStore->seqnoUp = LMIC.seqnoUp;
            macStore->seqnoDn = LMIC.seqnoDn;
            saveMacStore();
            
            break;
        case EV_LINK_DEAD:
            event = F("EV_LINK_DEAD");
            lorawanComputePriority = false;
            break;
        case EV_LINK_ALIVE:
            event = F("EV_LINK_ALIVE");
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            event = F("EV_TXSTART");
            lorawanComputePriority = true;
            break;
        default:
            event = F("unknown");
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }

    Serial.println(event);
    screenData->updateMacEvent(event, String(LMIC.seqnoUp, DEC), String(LMIC.seqnoDn, DEC));
    
    //refreshScreen();
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));

  Serial.print(F("debug level : "));
  Serial.println(LMIC_DEBUG_LEVEL);
  Serial.print(F("cfg868 : "));
  Serial.println(CFG_eu868);

  #if defined(LMIC_USE_INTERRUPTS)
  Serial.println(F("LMIC_USE_INTERRUPTS is defined"));
  #endif

  // Configure LED output
  pinMode(2, OUTPUT);
  
  // Configure SPI slave select has output
  pinMode(LORA_SPI_SS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);

  digitalWrite(LORA_SPI_SS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, -1);
  //SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, 5);

  // Begin GPS UART
  esp32HWSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  beginLoraSpi();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  //testMacStore();

  eraseMacStore();
  loadMacStore();

  initScreen();
  refreshScreen(true);

  // Perform a first sensor acquisition
  //sensorAcquisition();

  // Disable Adaptive Data Rate
  //LMIC_setAdrMode(0);
    
  // Configure LMIC Data Rate and Tx Power. Not used with OTAA.
  //LMIC_setDrTxpow(DR_SF10, 14);

  // Send a first payload (sending automatically starts OTAA too)
  //do_send(&sendjob);
}

int acquisitionCount = 0;
int acquisitionPeriod = 60000; // 1 acquisition every 10 sec
void loop() {
  unsigned long now = millis();
  if ((now & 512) != 0) {
    digitalWrite(2, HIGH);
  }
  else {
    digitalWrite(2, LOW);
  }
  
  lorawanComputePriority = false;
  if (!lorawanComputePriority) {
    if (now > acquisitionPeriod * acquisitionCount) {
      sensorAcquisition();
      acquisitionCount ++;
      //initScreen();
      //refreshScreen(false, true);
    }
    
    refreshScreen(false, true);
  }
  
  os_runloop_once();

  if (dataToTransmit) {
    do_send(&sendjob);
    dataToTransmit = false;
  }
  
}


// prints 8-bit data in hex with leading zeroes
void PrintHex8(uint8_t *data, uint8_t length) {
  char tmp[16];
  for (int i=0; i<length; i++) {
    sprintf(tmp, "0x%.2X",data[i]);
    Serial.print(tmp); Serial.print(" ");
  }
}

String hex8toString(uint8_t *data, uint8_t length) {
  char tmp[16];
  String prefix = "";
  String result = "";
  for (int i=0; i<length; i++) {
    sprintf(tmp, "%.2X",data[i]);
    result += prefix + String(tmp) + " ";
  }

  return result;
}
