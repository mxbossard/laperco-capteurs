/*
  Multiple Serial test

  Receives from the main serial port, sends to the others.
  Receives from serial port 1, sends to the main serial (Serial 0).

  This example works only with boards with more than one serial like Arduino Mega, Due, Zero etc.

  The circuit:
  - any serial device attached to Serial port 1
  - Serial Monitor open on Serial port 0

  created 30 Dec 2008
  modified 20 May 2012
  by Tom Igoe & Jed Roach
  modified 27 Nov 2015
  by Arturo Guadalupi

  This example code is in the public domain.
*/

int LORA_M0 = 16;
int LORA_M1 = 17;
int LORA_RX = 3; // ESP TX0
int LORA_TX = 21; // ESP RX0
int LORA_AUX = 21;

HardwareSerial MySerial(1);

void setup() {
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_RX, OUTPUT);
  pinMode(LORA_TX, INPUT);
  pinMode(LORA_AUX, INPUT);
  
  // initialize both serial ports:
  
  Serial.begin(9600);
  MySerial.begin(9600, SERIAL_8N1, 3, 1);
}

void loop() {
  // read from port 1, send to port 0:
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial.write(inByte);
    MySerial.write(inByte);
  }
/*
  // read from port 0, send to port 1:
  if (MySerial.available()) {
    int inByte = Serial.read();
    MySerial.write(inByte);
  }
  */
}
