#include <Arduino.h>

#define HEADER 0xBB
#define FRAME_TYPE_COMMAND 0x00
#define FRAME_TYPE_RESP 0x01
#define FRAME_TYPE_NOTIF 0x02
#define FRAME_TYPE_NOTIF_EMPTY 0x01
#define FRAME_CMD_SINGLE_INV 0x22
#define FRAME_CMD_SINGLE_INV_EMPTY 0xFF
#define FRAME_CMD_SINGLE_MULTI 0x27
#define FRAME_END 0x7E
#define PIN_EN 4
#define FRAME_PARAM_SINGLE_INV_EMPTY 0x15
#define TIMEOUT_MS 1000  // Maximum wait time for data in milliseconds
 
uint8_t singleRead[7] = {HEADER, FRAME_TYPE_COMMAND, FRAME_CMD_SINGLE_INV, 
                         0x00, 0x00, 0x22, FRAME_END};
uint8_t multiRead[10] = {HEADER, FRAME_TYPE_COMMAND, FRAME_CMD_SINGLE_MULTI,
                         0x00, 0x03, 0x22, 0x00, 0x10, 0x5c, FRAME_END};
 
uint8_t i;
uint8_t crc;
uint8_t header;
uint8_t frameType;
uint8_t frameCmd;
uint8_t framePl[2];
uint8_t paramL;
uint8_t tagRssi;
uint8_t tagPc[2];
uint8_t epcLength;
uint8_t tagEpc[16];
uint8_t tagCrc[2];
uint8_t frameCrc;
uint8_t frameEnd;
 
 
// Function to send the inventory request frame
void sendRequestFrame() {
  for (i = 0; i < 7; i++) {
    Serial1.write(singleRead[i]);
    delay(50);
  }
}
 
// Function to wait for data with a timeout
bool waitForSerialData(HardwareSerial& serial, unsigned long timeout) {
  unsigned long startTime = millis();
  while (!serial.available()) {
    if (millis() - startTime >= timeout) {
      return false;  // Timeout occurred
    }
    delay(1);  // Avoid tight loop
  }
  return true;
}
 
// Function to clear the Serial1 buffer
void clearSerial1Buffer() {
  if (Serial1.available()) {
//    Serial.print("Cleared buffer");
    while (Serial1.available()) {
      uint8_t data = Serial1.read();
//      if (data < 0x10) Serial.print("0");
//      Serial.print(data, HEX);
//      Serial.print(" ");
    }
//    Serial.println();
  }
}
// Function to read and validate the frame
bool readAndValidateFrame() {
  header = Serial1.read();
  if (header != HEADER) {
//    Serial.print("X Invalid header: ");
//    Serial.println(header, HEX);
    clearSerial1Buffer();
    return false;
  }
 
  frameType = Serial1.read();
  frameCmd = Serial1.read();
  crc = frameType + frameCmd;
 
  // Check if no tag found
  if (frameType == FRAME_TYPE_NOTIF_EMPTY && frameCmd == FRAME_CMD_SINGLE_INV_EMPTY) {
    uint8_t emptyParameter[5];
    for (i = 0; i < 5; ++i) {
      emptyParameter[i] = Serial1.read();
    }
    if (emptyParameter[2] == FRAME_PARAM_SINGLE_INV_EMPTY) {
//      Serial.println("Tag not found!");
    }
    return false;
  }
 
  if (frameType != FRAME_TYPE_NOTIF) {
    Serial.print("X Invalid frame type: ");
    Serial.println(frameType, HEX);
  }
 
  if (frameCmd != FRAME_CMD_SINGLE_INV) {
    Serial.print("X Invalid command code: ");
    Serial.println(frameCmd, HEX);
  }
 
  for (i = 0; i < 2; ++i) {
    crc += framePl[i] = Serial1.read();
  }
 
  paramL = (framePl[0] << 8) + framePl[1];
 
  crc += tagRssi = Serial1.read();
 
  for (i = 0; i < 2; ++i) {
    crc += tagPc[i] = Serial1.read();
  }
 
  epcLength = paramL - 5;
  for (i = 0; i < epcLength; ++i) {
    crc += tagEpc[i] = Serial1.read();
  }
 
  for (i = 0; i < 2; ++i) {
    tagCrc[i] = Serial1.read();
    crc += tagCrc[i];
  }
 
  crc &= 0xFF;
 
  frameCrc = Serial1.read();
  if (crc != frameCrc) {
//    Serial.print("X CRC mismatch: calculated ");
//    Serial.println(crc, HEX);
//    Serial.print("X Received CRC: ");
//    Serial.println(frameCrc, HEX);
  }
 
  frameEnd = Serial1.read();
  if (frameEnd != FRAME_END) {
//    Serial.print("X Invalid frame end: ");
//    Serial.println(frameEnd, HEX);
    return false;
  }
 
  return true;
}
 
// Function to print bytes in hexadecimal format
void printBytes(const uint8_t* data, uint8_t size) {
  for (uint8_t i = 0; i < size; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
// Function to print the frame data
void printFrameData() {
  Serial.println("Received frame:");
  Serial.print("- Header: ");
  Serial.println(header, HEX);
  Serial.print("- Frame type: ");
  Serial.println(frameType, HEX);
  Serial.print("- Command: ");
  Serial.println(frameCmd, HEX);
  Serial.print("- Parameter length: ");
  printBytes(framePl, 2);
  Serial.print("- RSSI: ");
  Serial.println(tagRssi, HEX);
  Serial.print("- Tag PC: ");
  printBytes(tagPc, 2);
  Serial.print("- Tag EPC: ");
  printBytes(tagEpc, epcLength);
  Serial.print("- Tag CRC: ");
  printBytes(tagCrc, 2);
  Serial.print("- CRC: ");
  Serial.println(frameCrc, HEX);
  Serial.print("- End: ");
  Serial.println(frameEnd, HEX);
}
 
 
void setup() {
  // For Arduino
  // Serial1.begin(115200);
  // For ESP32
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
 
  Serial.begin(9600);
 
  pinMode(PIN_EN, OUTPUT);
  clearSerial1Buffer();
  digitalWrite(PIN_EN, HIGH);
}
 
void loop() {
  sendRequestFrame();
 
  // Wait until data is available or timeout occurs
  if (!waitForSerialData(Serial1, TIMEOUT_MS)) {
    Serial.println("Timeout: No data received.");
    return;
  }
 
  // Read and validate the frame
  if (!readAndValidateFrame()) return;
 
  // Print the received frame data
  printFrameData();
}