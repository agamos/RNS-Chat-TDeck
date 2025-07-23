#include <Wire.h>

#define LILYGO_KB_SLAVE_ADDRESS             0x55
#define LILYGO_KB_BRIGHTNESS_CMD            0x01
#define LILYGO_KB_ALT_B_BRIGHTNESS_CMD      0x02

#define BOARD_POWERON       10
#define BOARD_I2C_SDA       18
#define BOARD_I2C_SCL       8


/*
* Dynamically modify backlight brightness at runtime
* Brightness Range: 0 ~ 255
* */
void setKeyboardBrightness(uint8_t value)
{
    Wire.beginTransmission(LILYGO_KB_SLAVE_ADDRESS);
    Wire.write(LILYGO_KB_BRIGHTNESS_CMD);
    Wire.write(value);
    Wire.endTransmission();
}

/*
* Set the default backlight brightness level. If the user sets the backlight to 0
* via setKeyboardBrightness, the default brightness is used when pressing ALT+B,
* rather than the backlight brightness level set by the user. This ensures that
* pressing ALT+B can respond to the backlight being turned on and off normally.
* Brightness Range: 30 ~ 255
* */
void setKeyboardDefaultBrightness(uint8_t value)
{
    Wire.beginTransmission(LILYGO_KB_SLAVE_ADDRESS);
    Wire.write(LILYGO_KB_ALT_B_BRIGHTNESS_CMD);
    Wire.write(value);
    Wire.endTransmission();
}
