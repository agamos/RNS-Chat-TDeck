#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <RadioLib.h>
#include "Framing.h"       // From RNode_Firmware_CE
#include "Utilities.h"     // From RNode_Firmware_CE

// T-Deck pins
#define BOARD_POWERON 10
#define BOARD_I2C_SDA 18
#define BOARD_I2C_SCL 8
#define LORA_SS 13
#define LORA_RST 14
#define LORA_DIO0 15
#define TFT_CS 34
#define TFT_DC 35
#define TFT_SCK 36
#define TFT_MOSI 33
#define TFT_MISO 47
#define LILYGO_KB_SLAVE_ADDRESS 0x55
#define LILYGO_KB_BRIGHTNESS_CMD 0x01
#define LILYGO_KB_ALT_B_BRIGHTNESS_CMD 0x02

// Display setup
Adafruit_ST7789 display(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);

// LoRa setup
SX127x LoRa;

// Simple message structure (inspired by LXMF)
struct Message {
  uint8_t senderHash[16];
  uint8_t content[100];
  uint16_t contentLength;
};

// Keyboard buffer
char keyboardBuffer[100]; // Reduced for memory
uint16_t keyboardIndex = 0;
bool keyboardEnterPressed = false;

// Keyboard functions (from Keyboard_T_Deck_Master.ino)
void setKeyboardBrightness(uint8_t value) {
  Wire.beginTransmission(LILYGO_KB_SLAVE_ADDRESS);
  Wire.write(LILYGO_KB_BRIGHTNESS_CMD);
  Wire.write(value);
  Wire.endTransmission();
}

void setKeyboardDefaultBrightness(uint8_t value) {
  Wire.beginTransmission(LILYGO_KB_SLAVE_ADDRESS);
  Wire.write(LILYGO_KB_ALT_B_BRIGHTNESS_CMD);
  Wire.write(value);
  Wire.endTransmission();
}

char readKeyboard() {
  char keyValue = 0;
  Wire.requestFrom(LILYGO_KB_SLAVE_ADDRESS, 1);
  while (Wire.available() > 0) {
    keyValue = Wire.read();
    if (keyValue != 0x00) {
      Serial.print("Key: ");
      Serial.println(keyValue);
      return keyValue;
    }
  }
  return 0;
}

// Display functions
void displayMessage(const char* message, const char* sender) {
  display.fillScreen(ST77XX_BLACK);
  display.setCursor(0, 0);
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(1);
  display.print("From: ");
  display.println(sender);
  display.println(message);
  display.display();
}

void displayStatus(const char* status) {
  display.fillScreen(ST77XX_BLACK);
  display.setCursor(0, 0);
  display.setTextColor(ST77XX_YELLOW);
  display.setTextSize(1);
  display.println(status);
  display.display();
}

// Simple XOR encryption (replace with Sodium for production)
void encryptMessage(uint8_t* data, uint16_t len, const uint8_t* key) {
  for (uint16_t i = 0; i < len; i++) {
    data[i] ^= key[i % 16];
  }
}

// LoRa setup
void setupLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915.0, 125000, 7, 5)) { // 915 MHz, 125 kHz BW, SF7, CR4/5
    displayStatus("LoRa init failed");
    while (1);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize power
  pinMode(BOARD_POWERON, OUTPUT);
  digitalWrite(BOARD_POWERON, HIGH);
  delay(500);
  
  // Initialize display
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  display.init(320, 240);
  display.fillScreen(ST77XX_BLACK);
  display.setCursor(0, 0);
  display.setTextColor(ST77XX_GREEN);
  display.setTextSize(1);
  display.println("T-Deck RNode Messenger");
  display.display();

  // Initialize keyboard
  Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);
  Wire.requestFrom(LILYGO_KB_SLAVE_ADDRESS, 1);
  if (Wire.read() == -1) {
    displayStatus("Keyboard not online");
    while (1);
  }
  setKeyboardDefaultBrightness(127);
  setKeyboardBrightness(127);

  // Initialize LoRa
  setupLoRa();
}

void loop() {
  // Check for incoming messages
  uint8_t buffer[256];
  int len = LoRa.receive(buffer, sizeof(buffer));
  if (len > 0) {
    Message msg;
    memcpy(msg.senderHash, buffer, 16);
    msg.contentLength = len - 16;
    memcpy(msg.content, buffer + 16, msg.contentLength);
    
    // Decrypt (simple XOR for demo)
    uint8_t key[16] = {0x1a, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f, 0x7a, 0x8b,
                       0x9c, 0xad, 0xbe, 0xcf, 0xd0, 0xe1, 0xf2, 0x03};
    encryptMessage(msg.content, msg.contentLength, key); // XOR decrypt
    
    char senderStr[33];
    bytes_to_hex(msg.senderHash, 16, senderStr);
    msg.content[msg.contentLength] = '\0';
    displayMessage((const char*)msg.content, senderStr);
    delay(2000);
  }

  // Check for keyboard input
  char c = readKeyboard();
  if (c == '\n') {
    keyboardEnterPressed = true;
    keyboardBuffer[keyboardIndex] = '\0';
  } else if (c != 0 && keyboardIndex < sizeof(keyboardBuffer) - 1) {
    keyboardBuffer[keyboardIndex++] = c;
  }

  if (keyboardEnterPressed && keyboardIndex > 0) {
    // Prepare message
    Message msg;
    uint8_t senderHash[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                             0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10};
    memcpy(msg.senderHash, senderHash, 16);
    msg.contentLength = keyboardIndex;
    memcpy(msg.content, keyboardBuffer, msg.contentLength);
    
    // Encrypt (simple XOR for demo)
    uint8_t key[16] = {0x1a, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f, 0x7a, 0x8b,
                       0x9c, 0xad, 0xbe, 0xcf, 0xd0, 0xe1, 0xf2, 0x03};
    encryptMessage(msg.content, msg.contentLength, key);
    
    // Send via LoRa
    uint8_t packet[256];
    memcpy(packet, msg.senderHash, 16);
    memcpy(packet + 16, msg.content, msg.contentLength);
    LoRa.transmit(packet, 16 + msg.contentLength);
    
    displayStatus("Sent message");
    keyboardIndex = 0;
    keyboardEnterPressed = false;
  }

  delay(10);
}
```