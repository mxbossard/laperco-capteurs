/*
 * UART tester 1.
 * Send byte from Arduino console to Hardware UART TX then read it with RX and output it to Arduino console.
 * 
 */

int UART_RX = 16; // ESP RX2
int UART_TX = 17; // ESP TX2

int LED_BLUE = 2;

HardwareSerial MySerial(1);

void setup() {
  pinMode(UART_RX, INPUT);
  pinMode(UART_TX, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  Serial.begin(9600);
  Serial.setTimeout(1);
  MySerial.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
  MySerial.setTimeout(1);
}

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
    byte uartByte[1];
    int len = MySerial.readBytes(uartByte, 1);
    if (len > 0) {
      //Serial.print("Reading [");
      //Serial.print(uartByte[0]);
      //Serial.println("] from UART");

      Serial.write(uartByte, 1);
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
