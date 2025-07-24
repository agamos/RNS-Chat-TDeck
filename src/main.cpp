#include <globals.h>

#include <EEPROM.h>

#include <tft.h>
#include <SD.h>
#include <SPIFFS.h>

#include "powerSave.h"
#include <functional>
#include <iostream>
#include <string>
#include <vector>


// T-Deck Plus pins
#define BOARD_POWERON 10
#define TFT_CS 34
#define TFT_DC 35
#define TFT_SCK 36
#define TFT_MOSI 33
#define TFT_MISO 47
#define RADIO_CS 13 // LoRa CS
#define SD_CS 12    // MicroSD CS

// Public Globals
uint32_t MAX_SPIFFS = 0;
uint32_t MAX_APP = 0;
uint32_t MAX_FAT_vfs = 0;
uint32_t MAX_FAT_sys = 0;
uint16_t FGCOLOR = GREEN;
uint16_t ALCOLOR = RED;
uint16_t BGCOLOR = BLACK;
uint16_t odd_color = 0x30c5;
uint16_t even_color = 0x32e5;
// Navigation Variables
long LongPressTmp = 0;
volatile bool LongPress = false;
volatile bool NextPress = false;
volatile bool PrevPress = false;
volatile bool UpPress = false;
volatile bool DownPress = false;
volatile bool SelPress = false;
volatile bool EscPress = false;
volatile bool AnyKeyPress = false;
TouchPoint touchPoint;
keyStroke KeyStroke;

#if defined(HAS_TOUCH)
volatile uint16_t tftHeight = TFT_WIDTH - 20;
#else
volatile uint16_t tftHeight = TFT_WIDTH;
#endif
volatile uint16_t tftWidth = TFT_HEIGHT;
TaskHandle_t xHandle;
void __attribute__((weak)) taskInputHandler(void *parameter) {
    auto timer = millis();
    while (true) {
        checkPowerSaveTime();
        if (!AnyKeyPress || millis() - timer > 75) {
            resetGlobals();
#ifndef DONT_USE_INPUT_TASK
            InputHandler();
#endif
            timer = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// More 2nd grade global Variables
int dimmerSet = 20;
unsigned long previousMillis;
bool isSleeping;
bool isScreenOff;
bool dev_mode = false;
int bright = 100;
bool dimmer = false;
int prog_handler; // 0 - Flash, 1 - SPIFFS
int currentIndex;
int rotation = ROTATION;
bool sdcardMounted;
bool onlyBins;
bool returnToMenu;
bool update;
bool askSpiffs;
#ifdef DISABLE_OTA
bool stopOta = true;
#else
bool stopOta;
#endif

#include "display.h"
#include "massStorage.h"
#include "mykeyboard.h"
#include "sd_functions.h"
#include "settings.h"


// Display setup
// TFT_eSPI display = TFT_eSPI();

/*********************************************************************
**  Function: _setup_gpio()
**  Sets up a weak (empty) function to be replaced by /ports/* /interface.h
*********************************************************************/
void _setup_gpio() __attribute__((weak));
void _setup_gpio() {}

/*********************************************************************
**  Function: _post_setup_gpio()
**  Sets up a weak (empty) function to be replaced by /ports/* /interface.h
*********************************************************************/
void _post_setup_gpio() __attribute__((weak));
void _post_setup_gpio() {}

void prepareEEPROM() {
    EEPROM.begin(EEPROMSIZE + 32); // open eeprom.... 32 is the size of the SSID string stored at the end of
                                   // the memory, using this trick to not change all the addresses
    if (EEPROM.read(EEPROMSIZE - 13) > 3 || EEPROM.read(EEPROMSIZE - 14) > 240 ||
        EEPROM.read(EEPROMSIZE - 15) > 100 || EEPROM.read(EEPROMSIZE - 1) > 1 ||
        EEPROM.read(EEPROMSIZE - 2) > 1 ||
        (EEPROM.read(EEPROMSIZE - 3) == 0xFF && EEPROM.read(EEPROMSIZE - 4) == 0xFF &&
         EEPROM.read(EEPROMSIZE - 5) == 0xFF && EEPROM.read(EEPROMSIZE - 6) == 0xFF)) {
        log_i(
            "EEPROM back to default\n0=%d\n1=%d\n2=%d\n9=%d\nES-1=%d",
            EEPROM.read(EEPROMSIZE - 13),
            EEPROM.read(EEPROMSIZE - 14),
            EEPROM.read(EEPROMSIZE - 15),
            EEPROM.read(EEPROMSIZE - 1),
            EEPROM.read(EEPROMSIZE - 2)
        );
        EEPROM.write(EEPROMSIZE - 13, rotation); // Left rotation
        EEPROM.write(EEPROMSIZE - 14, 20);       // 20s Dimm time
        EEPROM.write(EEPROMSIZE - 15, 100);      // 100% brightness
        EEPROM.write(EEPROMSIZE - 1, 1);         // OnlyBins
        EEPROM.writeString(20, "");
        EEPROM.writeString(EEPROMSIZE, ""); // resets ssid at the end of the EEPROM
        EEPROM.write(EEPROMSIZE - 2, 1);    // AskSpiffs

        // FGCOLOR
        EEPROM.write(EEPROMSIZE - 3, 0x07);
        EEPROM.write(EEPROMSIZE - 4, 0xE0);
        // BGCOLOR
        EEPROM.write(EEPROMSIZE - 5, 0);
        EEPROM.write(EEPROMSIZE - 6, 0);
        // ALCOLOR
        EEPROM.write(EEPROMSIZE - 7, 0xF8);
        EEPROM.write(EEPROMSIZE - 8, 0x00);
        // odd
        EEPROM.write(EEPROMSIZE - 9, 0x30);
        EEPROM.write(EEPROMSIZE - 10, 0xC5);
        // even
        EEPROM.write(EEPROMSIZE - 11, 0x32);
        EEPROM.write(EEPROMSIZE - 12, 0xe5);

#if defined(HEADLESS)
        // SD Pins
        EEPROM.write(90, 0);
        EEPROM.write(91, 0);
        EEPROM.write(92, 0);
        EEPROM.write(93, 0);
#endif
        EEPROM.commit(); // Store data to EEPROM
    }

    rotation = EEPROM.read(EEPROMSIZE - 13);
    dimmerSet = EEPROM.read(EEPROMSIZE - 14);
    bright = EEPROM.read(EEPROMSIZE - 15);
    onlyBins = EEPROM.read(EEPROMSIZE - 1);
    askSpiffs = EEPROM.read(EEPROMSIZE - 2);
    FGCOLOR = (EEPROM.read(EEPROMSIZE - 3) << 8) | EEPROM.read(EEPROMSIZE - 4);
    BGCOLOR = (EEPROM.read(EEPROMSIZE - 5) << 8) | EEPROM.read(EEPROMSIZE - 6);
    ALCOLOR = (EEPROM.read(EEPROMSIZE - 7) << 8) | EEPROM.read(EEPROMSIZE - 8);
    odd_color = (EEPROM.read(EEPROMSIZE - 9) << 8) | EEPROM.read(EEPROMSIZE - 10);
    even_color = (EEPROM.read(EEPROMSIZE - 11) << 8) | EEPROM.read(EEPROMSIZE - 12);
    // pwd = EEPROM.readString(20);          // read what is on EEPROM here for headless environment
    // ssid = EEPROM.readString(EEPROMSIZE); // read what is on EEPROM here for headless environment
    EEPROM.end();
}

/*********************************************************************
**  Function: setup
**  Where the devices are started and variables set
*********************************************************************/
void setup()
{
    Serial.begin(115200);
    delay(2000); // Stabilize USB CDC
    Serial.println("Starting T-Deck Plus display test...");

#if defined(BACKLIGHT)
    pinMode(BACKLIGHT, OUTPUT);
#endif

    _setup_gpio();

    prepareEEPROM(); // Prepare EEPROM

    // Init Display
    tft->begin();

    tft->setRotation(rotation);
    if (rotation & 0b1)
    {
#if defined(HAS_TOUCH)
        tftHeight = TFT_WIDTH - 20;
#else
        tftHeight = TFT_WIDTH;
#endif
        tftWidth = TFT_HEIGHT;
    }
    else
    {
#if defined(HAS_TOUCH)
        tftHeight = TFT_HEIGHT - 20;
#else
        tftHeight = TFT_HEIGHT;
#endif
        tftWidth = TFT_WIDTH;
    }
    tft->fillScreen(BGCOLOR);
    setBrightness(bright, false);
    initDisplay(true);

    _post_setup_gpio();

    // This task keeps running all the time, will never stop
    xTaskCreate(
        taskInputHandler, // Task function
        "InputHandler",   // Task Name
        3500,             // Stack size
        NULL,             // Task parameters
        2,                // Task priority (0 to 3), loopTask has priority 2.
        &xHandle          // Task handle (not used)
    );

    delay(2500);
    resetTftDisplay();
    testDisplay();
}

void loop() {
  Serial.println("Loop running...");
  delay(1000);
}