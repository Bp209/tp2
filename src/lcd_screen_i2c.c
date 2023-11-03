#include "../inc/lcd_screen_i2c.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

static void lcd_toggle_enable(const struct i2c_dt_spec *dev_lcd_screen, uint8_t bits);

static void lcd_byte(const struct i2c_dt_spec *dev_lcd_screen, uint8_t bits, uint8_t mode);

void init_lcd(const struct i2c_dt_spec *dev_lcd_screen)
{
    lcd_byte(dev_lcd_screen, 0x33, LCD_CMD);
    lcd_byte(dev_lcd_screen, 0x32, LCD_CMD);
    lcd_byte(dev_lcd_screen, 0x06, LCD_CMD);
    lcd_byte(dev_lcd_screen, 0x0c, LCD_CMD);
    lcd_byte(dev_lcd_screen, 0x28, LCD_CMD);
    lcd_byte(dev_lcd_screen, 0x01, LCD_CMD);
    write_lcd(dev_lcd_screen, HELLO_MSG, LCD_LINE_1);
    write_lcd(dev_lcd_screen, ZEPHYR_MSG, LCD_LINE_2);
}

static void lcd_byte(const struct i2c_dt_spec *dev_lcd_screen, uint8_t bits, uint8_t mode)
{
    int ret = 0;
    uint8_t bits_high = mode | (bits & 0xf0) | LCD_BACKLIGHT;
    uint8_t bits_low = mode | ((bits << 4) & 0xf0) | LCD_BACKLIGHT;
    ret = i2c_write_dt(dev_lcd_screen, &bits_high, sizeof(bits_high));
    if (ret != 0)
    {
        printk("ERROR while writing to I2C");
    }
    lcd_toggle_enable(dev_lcd_screen, bits_high);

    ret = i2c_write_dt(dev_lcd_screen, &bits_low, sizeof(bits_low));
    if (ret != 0)
    {
        printk("ERROR while writing to I2C");
    }
    lcd_toggle_enable(dev_lcd_screen, bits_low);
}

void write_lcd(const struct i2c_dt_spec *dev_lcd_screen, const char *msg, uint8_t line)
{
    lcd_byte(dev_lcd_screen, line, LCD_CMD);

    // Todo: fill the array with space at the end to complete the line

    if (strlen(msg) > LCD_WIDTH)
    {
        return;
    }
    for (uint8_t i = 0; i < strlen(msg); i++)
    {
        lcd_byte(dev_lcd_screen, msg[i], LCD_CHR);
    }
}

static void lcd_toggle_enable(const struct i2c_dt_spec *dev_lcd_screen, uint8_t bits)
{
    int ret = 0;
    k_sleep(K_MSEC(5));
    uint8_t bits1 = bits | LCD_ENABLE;
    ret = i2c_write_dt(dev_lcd_screen, &bits1, sizeof(bits1));
    if (ret != 0)
    {
        printk("ERROR while writing to I2C");
    }
    k_sleep(K_MSEC(5));
    uint8_t bits2 = bits & ~LCD_ENABLE;
    ret = i2c_write_dt(dev_lcd_screen, &bits2, sizeof(bits2));
    if (ret != 0)
    {
        printk("ERROR while writing to I2C");
    }
    k_sleep(K_MSEC(5));
}
