/**
 * Author: Vivien Debuchy
 * Simple library for LCD-1602 I2C
 * Based on https://github.com/sterlingbeason/LCD-1602-I2C
 */

#ifndef INC_LDC_SCREEN_I2C_H_
#define INC_LDC_SCREEN_I2C_H_

#include <zephyr/types.h>

struct i2c_dt_spec;

#define LCD_BACKLIGHT 0x08
#define LCD_CMD 0x00
#define LCD_CHR 0x01
#define LCD_ENABLE 0b00000100
#define LCD_LINE_1 0x80 // Select line 1
#define LCD_LINE_2 0xc0 // Select line 2
#define LCD_WIDTH 16    // Max number of carac per lines

#define HELLO_MSG "Bienvenue chez  "
#define ZEPHYR_MSG "Baptiste !      "

#define START_ALERT_MONITORING_MSG_1 "Alarm"
#define START_ALERT_MONITORING_MSG_2 "Protection"

#define INTRUDER_MSG_1 "INTRUDER"
#define INTRUDER_MSG_2 "LEAVE !"

void init_lcd(const struct i2c_dt_spec *_dev_lcd_screen);
void write_lcd(const struct i2c_dt_spec *dev_lcd_screen, const char *msg, uint8_t line);
void write_lcd_clear(const struct i2c_dt_spec *dev_lcd_screen, const char *msg, uint8_t line);

#endif // INC_LDC_SCREEN_I2C_H_