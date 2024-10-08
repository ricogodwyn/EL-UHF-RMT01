#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BluetoothSerial.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3D

// Create an instance of the SSD1306 display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

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
#define TIMEOUT_MS 1000 // Maximum wait time for data in milliseconds
String qr;
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

BluetoothSerial SerialBT;

// Function to send the inventory request frame
void sendRequestFrame()
{
  for (i = 0; i < 7; i++)
  {
    Serial1.write(singleRead[i]);
    delay(50);
  }
}

// Function to wait for data with a timeout
bool waitForSerialData(HardwareSerial &serial, unsigned long timeout)
{
  unsigned long startTime = millis();
  while (!serial.available())
  {
    if (millis() - startTime >= timeout)
    {
      return false; // Timeout occurred
    }
    delay(1); // Avoid tight loop
  }
  return true;
}

// Function to clear the Serial1 buffer
void clearSerial1Buffer()
{
  if (Serial1.available())
  {
    //    Serial.print("Cleared buffer");
    while (Serial1.available())
    {
      uint8_t data = Serial1.read();
      //      if (data < 0x10) Serial.print("0");
      //      Serial.print(data, HEX);
      //      Serial.print(" ");
    }
    //    Serial.println();
  }
}
// Function to read and validate the frame
bool readAndValidateFrame()
{
  header = Serial1.read();
  if (header != HEADER)
  {
    //    Serial.print("X Invalid header: ");
    //    Serial.println(header, HEX);
    clearSerial1Buffer();
    return false;
  }

  frameType = Serial1.read();
  frameCmd = Serial1.read();
  crc = frameType + frameCmd;

  // Check if no tag found
  if (frameType == FRAME_TYPE_NOTIF_EMPTY && frameCmd == FRAME_CMD_SINGLE_INV_EMPTY)
  {
    uint8_t emptyParameter[5];
    for (i = 0; i < 5; ++i)
    {
      emptyParameter[i] = Serial1.read();
    }
    if (emptyParameter[2] == FRAME_PARAM_SINGLE_INV_EMPTY)
    {
      //      Serial.println("Tag not found!");
    }
    return false;
  }

  if (frameType != FRAME_TYPE_NOTIF)
  {
    Serial.print("X Invalid frame type: ");
    Serial.println(frameType, HEX);
  }

  if (frameCmd != FRAME_CMD_SINGLE_INV)
  {
    Serial.print("X Invalid command code: ");
    Serial.println(frameCmd, HEX);
  }

  for (i = 0; i < 2; ++i)
  {
    crc += framePl[i] = Serial1.read();
  }

  paramL = (framePl[0] << 8) + framePl[1];

  crc += tagRssi = Serial1.read();

  for (i = 0; i < 2; ++i)
  {
    crc += tagPc[i] = Serial1.read();
  }

  epcLength = paramL - 5;
  for (i = 0; i < epcLength; ++i)
  {
    crc += tagEpc[i] = Serial1.read();
  }

  for (i = 0; i < 2; ++i)
  {
    tagCrc[i] = Serial1.read();
    crc += tagCrc[i];
  }

  crc &= 0xFF;

  frameCrc = Serial1.read();
  if (crc != frameCrc)
  {
    //    Serial.print("X CRC mismatch: calculated ");
    //    Serial.println(crc, HEX);
    //    Serial.print("X Received CRC: ");
    //    Serial.println(frameCrc, HEX);
  }

  frameEnd = Serial1.read();
  if (frameEnd != FRAME_END)
  {
    //    Serial.print("X Invalid frame end: ");
    //    Serial.println(frameEnd, HEX);
    return false;
  }

  return true;
}

// Function to print bytes in hexadecimal format
void printBytes(const uint8_t *data, uint8_t size)
{
  for (uint8_t i = 0; i < size; i++)
  {
    if (data[i] < 0x10)
      Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
void displayData(const uint8_t *data, uint8_t size)
{
  display.clearDisplay();  // Clear the display buffer
  display.setCursor(0, 0); // Set cursor to top-left corner
  display.print("Tag EPC:");
  for (uint8_t i = 0; i < size; i++)
  {
    if (data[i] < 0x10)
    {
      display.print("0"); // Add leading zero for single digit hex values
    }
    display.print(data[i], HEX);
    display.print(" ");
  }
  display.display(); // Update the display with the new data
}
String tagEpcToString(const uint8_t *data, uint8_t size)
{
  String result = "";
  for (uint8_t i = 0; i < size; i++)
  {
    if (data[i] < 0x10)
    {
      result += "0"; // Add leading zero for single digit hex values
    }
    result += String(data[i], HEX);
    if (i < size - 1)
    {
      result += " "; // Add a space between bytes
    }
  }
  return result;
}
// Function to print the frame data
void printFrameData()
{
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
  Serial.println("test: ");
  SerialBT.println(tagEpcToString(tagEpc, epcLength));
  // displayData(tagEpc, epcLength);

  Serial.print("- Tag CRC: ");
  printBytes(tagCrc, 2);
  Serial.print("- CRC: ");
  Serial.println(frameCrc, HEX);
  Serial.print("- End: ");
  Serial.println(frameEnd, HEX);
}
void qrcode(){
if (Serial2.available()) {
        // Read the incoming byte
        char incomingByte = Serial2.read();
        
        // Print the incoming byte to the Serial Monitor
        Serial.print("Received: ");
        Serial.println(incomingByte);
    }
}
const int buttonPin = 2; // Pin connected to the button
bool state = true; // Initial state
bool lastButtonState = LOW; // Previous button state
void setup()
{
  // For Arduino
  // Serial1.begin(115200);
  // For ESP32
  Serial.begin(9600);
  Wire.begin(21, 22);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  Serial2.begin(9600, SERIAL_8N1, 9, 10);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  delay(2000); // Pause for 2 seconds

  if (!SerialBT.begin("ESP32_BT"))
  {
    Serial.println("An error occurred initializing Bluetooth");
  }
  else
  {
    Serial.println("Bluetooth initialized");
  }
  pinMode(buttonPin, INPUT);
  pinMode(PIN_EN, OUTPUT);
  clearSerial1Buffer();
  digitalWrite(PIN_EN, HIGH);
  delay(1000);
  display.clearDisplay(); // Clear the display buffer
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); // Set cursor to top-left corner
  display.print("scanning....");
  display.display();
}

void loop()
{
  bool buttonState = digitalRead(buttonPin);

  // Check if the button is pressed (state change from LOW to HIGH)
  if (buttonState == HIGH && lastButtonState == LOW) {
    // Toggle the state
    state = !state;
    delay(50); // Debounce delay
  }

  // Update the last button state
  lastButtonState = buttonState;

  // Perform actions based on the current state
  if (state) {
    sendRequestFrame();

  // Wait until data is available or timeout occurs
  if (!waitForSerialData(Serial1, TIMEOUT_MS))
  {
    Serial.println("Timeout: No data received.");
    display.clearDisplay();  // Clear the display buffer
    display.setCursor(0, 0); // Set cursor to top-left corner
    display.print("no data");
    display.display();
    return;
  }

  // Read and validate the frame
  if (!readAndValidateFrame())
    return;

  // Print the received frame data
  qrcode();
  printFrameData();
  } else {
    qrcode();
  }
}