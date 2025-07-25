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

// EEPROM settings
#define EEPROM_SIZE 512
#define EEPROM_PRODUCT 0xB0 // LilyGO LoRa32
#define EEPROM_MODEL 0xB8   // v2.0, 850–950 MHz
#define EEPROM_HWREV 1
#define EEPROM_SERIAL 0x00000002
#define EEPROM_FREQ_MIN 850000000
#define EEPROM_FREQ_MAX 950000000
#define EEPROM_MAX_TX 22
#define EEPROM_MODE 0x00    // Normal mode
#define EEPROM_SIGNATURE 0xDEADBEEF // Dummy signature
#define EEPROM_HASH_OFFSET 22
#define FIRMWARE_HASH_LENGTH 32

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
uint8_t margin = 10;
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
#include "Utilities.h"
// #include "lora.hpp"

#ifndef INTERFACE_SPI
// INTERFACE_SPI is only required on NRF52 platforms, as the SPI pins are set in the class constructor and not by a setter method.
// Even if custom SPI interfaces are not needed, the array must exist to prevent compilation errors.
#define INTERFACE_SPI
SPIClass interface_spi[1];
#endif


FIFOBuffer serialFIFO;
uint8_t serialBuffer[CONFIG_UART_BUFFER_SIZE+1];

uint16_t packet_starts_buf[(CONFIG_QUEUE_MAX_LENGTH)+1];

uint16_t packet_lengths_buf[(CONFIG_QUEUE_MAX_LENGTH)+1];

FIFOBuffer16 packet_starts[INTERFACE_COUNT];
FIFOBuffer16 packet_lengths[INTERFACE_COUNT];

volatile uint8_t queue_height[INTERFACE_COUNT] = {0};
volatile uint16_t queued_bytes[INTERFACE_COUNT] = {0};

volatile uint16_t queue_cursor[INTERFACE_COUNT] = {0};
volatile uint16_t current_packet_start[INTERFACE_COUNT] = {0};
volatile bool serial_buffering = false;

extern void setup_interfaces(); // from /src/misc/ModemISR.h



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

#define MODEM_QUEUE_SIZE 4*INTERFACE_COUNT
typedef struct {
      size_t len;
      int rssi;
      int snr_raw;
      uint8_t interface;
      uint8_t data[];
} modem_packet_t;
static xQueueHandle modem_packet_queue = NULL;

char sbuf[128];

uint8_t *packet_queue[INTERFACE_COUNT];

void prepareRadio() {
    int seed_val = (int)esp_random();
    randomSeed(seed_val);

    // Initialise serial communication
    Serial.setRxBufferSize(CONFIG_UART_BUFFER_SIZE);
    memset(serialBuffer, 0, sizeof(serialBuffer));
    fifo_init(&serialFIFO, serialBuffer, CONFIG_UART_BUFFER_SIZE);

    Serial.begin(serial_baudrate);

      for (int i = 0; i < INTERFACE_COUNT; i++) {
    if (interface_pins[i][9] != -1) {
        pinMode(interface_pins[i][9], OUTPUT);
        digitalWrite(interface_pins[i][9], HIGH);
    }
  }

  // Initialise buffers
  memset(pbuf, 0, sizeof(pbuf));
  memset(cmdbuf, 0, sizeof(cmdbuf));
  
  memset(packet_starts_buf, 0, sizeof(packet_starts_buf));
  memset(packet_lengths_buf, 0, sizeof(packet_starts_buf));

  memset(seq, 0xFF, sizeof(seq));
  memset(read_len, 0, sizeof(read_len));

  setup_interfaces();

  modem_packet_queue = xQueueCreate(MODEM_QUEUE_SIZE, sizeof(modem_packet_t*));

  for (int i = 0; i < INTERFACE_COUNT; i++) {
      fifo16_init(&packet_starts[i], packet_starts_buf, CONFIG_QUEUE_MAX_LENGTH);
      fifo16_init(&packet_lengths[i], packet_lengths_buf, CONFIG_QUEUE_MAX_LENGTH);
      packet_queue[i] = (uint8_t*)malloc(getQueueSize(i)+1);
  }

  memset(packet_rdy_interfaces_buf, 0, sizeof(packet_rdy_interfaces_buf));

  fifo_init(&packet_rdy_interfaces, packet_rdy_interfaces_buf, MAX_INTERFACES);

  // Create and configure interface objects
  for (uint8_t i = 0; i < INTERFACE_COUNT; i++) {
      switch (interfaces[i]) {
          case SX1262:
          {
              sx126x* obj;
              // if default spi enabled
              if (interface_cfg[i][0]) {
                obj = new sx126x(i, &SPI, interface_cfg[i][1],
                interface_cfg[i][2], interface_pins[i][0], interface_pins[i][1],
                interface_pins[i][2], interface_pins[i][3], interface_pins[i][6],
                interface_pins[i][5], interface_pins[i][4], interface_pins[i][8]);
              }
              else {
            obj = new sx126x(i, &interface_spi[i], interface_cfg[i][1],
            interface_cfg[i][2], interface_pins[i][0], interface_pins[i][1],
            interface_pins[i][2], interface_pins[i][3], interface_pins[i][6],
            interface_pins[i][5], interface_pins[i][4], interface_pins[i][8]);
              }
            interface_obj[i] = obj;
            interface_obj_sorted[i] = obj;
            break;
          }

          case SX1276:
          case SX1278:
          {
              sx127x* obj;
              // if default spi enabled
              if (interface_cfg[i][0]) {
            obj = new sx127x(i, &SPI, interface_pins[i][0],
            interface_pins[i][1], interface_pins[i][2], interface_pins[i][3],
            interface_pins[i][6], interface_pins[i][5], interface_pins[i][4]);
              }
              else {
            obj = new sx127x(i, &interface_spi[i], interface_pins[i][0],
            interface_pins[i][1], interface_pins[i][2], interface_pins[i][3],
            interface_pins[i][6], interface_pins[i][5], interface_pins[i][4]);
              }
            interface_obj[i] = obj;
            interface_obj_sorted[i] = obj;
            break;
          }

          case SX1280:
          {
              sx128x* obj;
              // if default spi enabled
              if (interface_cfg[i][0]) {
            obj = new sx128x(i, &SPI, interface_cfg[i][1],
            interface_pins[i][0], interface_pins[i][1], interface_pins[i][2],
            interface_pins[i][3], interface_pins[i][6], interface_pins[i][5],
            interface_pins[i][4], interface_pins[i][8], interface_pins[i][7]);
            }
            else {
            obj = new sx128x(i, &interface_spi[i], interface_cfg[i][1],
            interface_pins[i][0], interface_pins[i][1], interface_pins[i][2],
            interface_pins[i][3], interface_pins[i][6], interface_pins[i][5],
            interface_pins[i][4], interface_pins[i][8], interface_pins[i][7]);
            }
            interface_obj[i] = obj;
            interface_obj_sorted[i] = obj;
            break;
          }
          
          default:
            break;
      }
  }

    // Check installed transceiver chip(s) and probe boot parameters. If any of
    // the configured modems cannot be initialised, do not boot
    for (int i = 0; i < INTERFACE_COUNT; i++) {
        switch (interfaces[i]) {
            case SX1262:
            case SX1276:
            case SX1278:
            case SX1280:
                selected_radio = interface_obj[i];
                break;

            default:
                modems_installed = false;
                break;
        }
        if (selected_radio->preInit()) {
          modems_installed = true;
          #if HAS_INPUT
            // Skip quick-reset console activation
          #else
              uint32_t lfr = selected_radio->getFrequency();
              if (lfr == 0) {
                // Normal boot
              } else if (lfr == M_FRQ_R) {
                // Quick reboot
                #if HAS_CONSOLE
                  if (rtc_get_reset_reason(0) == POWERON_RESET) {
                    console_active = true;
                  }
                #endif
              } else {
                // Unknown boot
              }
              selected_radio->setFrequency(M_FRQ_S);
          #endif
        } else {
          modems_installed = false;
        }
        if (!modems_installed) {
            break;
        }
    }

    for (int i = 0; i < INTERFACE_COUNT; i++) {
        selected_radio = interface_obj[i];
        if (interfaces[i] == SX1280) {
            selected_radio->setAvdInterference(false);
        }
        if (selected_radio->getAvdInterference()) {
          #if HAS_EEPROM
            uint8_t ia_conf = EEPROM.read(eeprom_addr(ADDR_CONF_DIA));
            if (ia_conf == 0x00) { selected_radio->setAvdInterference(true); }
            else                 { selected_radio->setAvdInterference(false); }
          #elif MCU_VARIANT == MCU_NRF52
            uint8_t ia_conf = eeprom_read(eeprom_addr(ADDR_CONF_DIA));
            if (ia_conf == 0x00) { selected_radio->setAvdInterference(true); }
            else                 { selected_radio->setAvdInterference(false); }
          #endif
        }
    }


}

void validate_status() {
  #if MCU_VARIANT == MCU_ESP32
      // TODO: Get ESP32 boot flags
      uint8_t boot_flags = 0x02;
      uint8_t F_POR = 0x00;
      uint8_t F_BOR = 0x00;
      uint8_t F_WDR = 0x01;
  #elif MCU_VARIANT == MCU_NRF52
      // TODO: Get NRF52 boot flags
      uint8_t boot_flags = 0x02;
      uint8_t F_POR = 0x00;
      uint8_t F_BOR = 0x00;
      uint8_t F_WDR = 0x01;
  #endif

  if (hw_ready || device_init_done) {
    hw_ready = false;
    Serial.write("Error, invalid hardware check state\r\n");
    #if HAS_DISPLAY
      if (disp_ready) {
        device_init_done = true;
        update_display();
      }
    #endif
    led_indicate_boot_error();
  }

  if (boot_flags & (1<<F_POR)) {
    boot_vector = START_FROM_POWERON;
  } else if (boot_flags & (1<<F_BOR)) {
    boot_vector = START_FROM_BROWNOUT;
  } else if (boot_flags & (1<<F_WDR)) {
    boot_vector = START_FROM_BOOTLOADER;
  } else {
      Serial.write("Error, indeterminate boot vector\r\n");
      #if HAS_DISPLAY
        if (disp_ready) {
          device_init_done = true;
          update_display();
        }
      #endif
      led_indicate_boot_error();
  }

  if (boot_vector == START_FROM_BOOTLOADER || boot_vector == START_FROM_POWERON) {
    if (eeprom_lock_set()) {
      if (eeprom_product_valid() && eeprom_model_valid() && eeprom_hwrev_valid()) {
        if (eeprom_checksum_valid()) {
          eeprom_ok = true;
          if (modems_installed) {
            if (device_init()) {
              hw_ready = true;
            } else {
              hw_ready = false;
            }
          } else {
            hw_ready = false;
            Serial.write("No radio module found\r\n");
            #if HAS_DISPLAY
              if (disp_ready) {
                device_init_done = true;
                update_display();
              }
            #endif
          }
        } else {
          hw_ready = false;
          Serial.write("Invalid EEPROM checksum\r\n");
          #if HAS_DISPLAY
            if (disp_ready) {
              device_init_done = true;
              update_display();
            }
          #endif
        }
      } else {
        hw_ready = false;
        Serial.write("Invalid EEPROM configuration\r\n");
        #if HAS_DISPLAY
          if (disp_ready) {
            device_init_done = true;
            update_display();
          }
        #endif
      }
    } else {
      hw_ready = false;
      Serial.write("Device unprovisioned, no device configuration found in EEPROM\r\n");
      #if HAS_DISPLAY
        if (disp_ready) {
          device_init_done = true;
          update_display();
        }
      #endif
    }
  } else {
    hw_ready = false;
    Serial.write("Error, incorrect boot vector\r\n");
    #if HAS_DISPLAY
      if (disp_ready) {
        device_init_done = true;
        update_display();
      }
    #endif
    led_indicate_boot_error();
  }
}

#define EEPROM_HASH_OFFSET 22
#define FIRMWARE_HASH_LENGTH 32

void provision_eeprom() {
    EEPROM.begin(EEPROM_SIZE);
    
    EEPROM.write(0, EEPROM_PRODUCT);
    EEPROM.write(1, EEPROM_MODEL);
    EEPROM.write(2, EEPROM_HWREV);
    EEPROM.put(3, EEPROM_SERIAL);
    EEPROM.put(7, EEPROM_FREQ_MIN);
    EEPROM.put(11, EEPROM_FREQ_MAX);
    EEPROM.write(15, EEPROM_MAX_TX);
    EEPROM.write(16, EEPROM_MODE);
    EEPROM.put(17, EEPROM_SIGNATURE);
    
    uint8_t checksum = 0;
    for (int i = 0; i < 21; i++) {
        uint8_t byte = EEPROM.read(i);
        checksum = (checksum + byte) & 0xFF; // Match RNode_Firmware_CE
        Serial.printf("EEPROM write: Offset=%d, Value=0x%X, Checksum=%d\n", i, byte, checksum);
    }
    EEPROM.write(21, checksum);
    
    bool commit_ok = EEPROM.commit();
    EEPROM.end();
    
    Serial.printf("EEPROM provisioned: Product=0x%X, Model=0x%X, Checksum=0x%X, Commit=%s\n", 
                  EEPROM.read(0), EEPROM.read(1), checksum, commit_ok ? "OK" : "Failed");
    tftprintln("EEPROM provisioned", margin);
}



void set_firmware_hash() {
    uint8_t default_hash[FIRMWARE_HASH_LENGTH] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
        0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20
    };

    EEPROM.begin(EEPROM_SIZE);
    bool hash_missing = false;
    for (int i = 0; i < FIRMWARE_HASH_LENGTH; i++) {
        if (EEPROM.read(EEPROM_HASH_OFFSET + i) == 0xFF) {
            hash_missing = true;
            break;
        }
    }

    if (hash_missing) {
        Serial.println("Firmware hash missing, writing default hash...");
        tftprintln("Writing hash...", margin);
        for (int i = 0; i < FIRMWARE_HASH_LENGTH; i++) {
            EEPROM.write(EEPROM_HASH_OFFSET + i, default_hash[i]);
            Serial.printf("EEPROM hash write: Offset=%d, Value=0x%X\n", 
                         EEPROM_HASH_OFFSET + i, default_hash[i]);
        }
        uint8_t checksum = 0;
        for (int i = 0; i < 21; i++) {
            uint8_t byte = EEPROM.read(i);
            checksum = (checksum + byte) & 0xFF;
        }
        EEPROM.write(21, checksum);
        bool commit_ok = EEPROM.commit();
        EEPROM.end();
        Serial.printf("Firmware hash set: Hash[0]=0x%X, Checksum=0x%X, Commit=%s\n", 
                      EEPROM.read(EEPROM_HASH_OFFSET), checksum, commit_ok ? "OK" : "Failed");
        tftprintln("Hash set", margin);
    } else {
        Serial.println("Firmware hash already set");
        tftprintln("Hash OK", margin);
    }
}

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

bool verify_eeprom() {
    EEPROM.begin(EEPROM_SIZE);
    uint8_t product = EEPROM.read(0);
    uint8_t model = EEPROM.read(1);
    uint32_t serial = 0; EEPROM.get(3, serial);
    uint32_t freq_min = 0; EEPROM.get(7, freq_min);
    uint32_t freq_max = 0; EEPROM.get(11, freq_max);
    uint8_t max_tx = EEPROM.read(15);
    uint8_t mode = EEPROM.read(16);
    uint32_t signature = 0; EEPROM.get(17, signature);
    uint8_t checksum = 0;
    for (int i = 0; i < 21; i++) {
        uint8_t byte = EEPROM.read(i);
        checksum = (checksum + byte) & 0xFF;
        Serial.printf("EEPROM read: Offset=%d, Value=0x%X, Checksum=%d\n", i, byte, checksum);
    }
    uint8_t stored_checksum = EEPROM.read(21);
    uint8_t hash_check = EEPROM.read(EEPROM_HASH_OFFSET);
    EEPROM.end();
    
    bool valid = (product == EEPROM_PRODUCT && model == EEPROM_MODEL &&
                  serial == EEPROM_SERIAL && freq_min == EEPROM_FREQ_MIN &&
                  freq_max == EEPROM_FREQ_MAX && max_tx == EEPROM_MAX_TX &&
                  mode == EEPROM_MODE && signature == EEPROM_SIGNATURE &&
                  checksum == stored_checksum && hash_check != 0xFF);
    
    Serial.printf("EEPROM verify: Product=0x%X, Model=0x%X, Serial=0x%X, Freq=%lu-%lu, "
                  "TX=%d, Mode=0x%X, Sig=0x%X, Checksum=0x%X, Stored=0x%X, Hash[0]=0x%X, Valid=%d\n",
                  product, model, serial, freq_min, freq_max, max_tx, mode, signature,
                  checksum, stored_checksum, hash_check, valid);
    tftprint("EEPROM:", margin);
    tftprintln((String)(valid ? "Valid" : "Invalid"), margin);
    return valid;
}

/*********************************************************************
**  Function: setup
**  Where the devices are started and variables set
*********************************************************************/
void setup()
{
#if HAS_CONSOLE
    Serial.begin(115200);
    delay(2000); // Stabilize USB CDC
    Serial.println("Starting T-Deck Plus display test...");
#endif

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

    delay(2000);  // Wait for the display to initialize
    resetTftDisplay();    
    testDisplay();

    int prepCount= 1;
    verify_eeprom();
    prepareRadio();
    validate_status();
    while(!hw_ready && prepCount < 2) {
        Serial.println("Radio not ready, retrying...");
        tftprintln("Radio not ready, retrying...", margin);
        delay(1000);
        prepCount++;
        Serial.println("Radio unprovisioned, provisioning EEPROM...");
        tftprintln("Provisioning...", margin);
        provision_eeprom();
        Serial.println("Setting firmware hash...");
        tftprintln("Firmware hash...", margin);
        set_firmware_hash();
        if(verify_eeprom()) {
            Serial.println("EEPROM verified, radio ready.");
            tft->println("EEPROM OK, radio ready.");
        } else {
            Serial.println("EEPROM verification failed, retrying...");
            tftprintln("EEPROM failed, retrying...", margin);
        }
        prepareRadio();
        validate_status();
    }
    if (hw_ready) {
        Serial.println("Radio initialised");
        tftprintln("Radio initialised", margin);
    }

}

int first_time = 1;
void loop() {
#if HAS_CONSOLE
    if (first_time) {
        Serial.println("");
        Serial.print("Loop running");
        first_time = 0;
    } else {
        Serial.print(".");
    }
  delay(1000);
#endif
}