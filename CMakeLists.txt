cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS
$ENV{ZEPHYR_BASE})
project(tp2)

target_sources(app PRIVATE
    src/main.c
    src/lcd_screen_i2c.c
)