#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "../inc/lcd_screen_i2c.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>

#define LED_YELLOW_NODE DT_ALIAS(led_yellow)
#define AFFICHEUR_NODE DT_ALIAS(afficheur_print)
#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
    ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

#define BUZZER_PIN DT_ALIAS(buzz)
#define BUZZER_TOGGLE_PERIOD K_MSEC(100)
#define ALARM_NODE DT_ALIAS(alar)

static bool ALARME = false;
static bool PAS_ALARME = false;

const struct gpio_dt_spec led_yellow_gpio = GPIO_DT_SPEC_GET_OR(LED_YELLOW_NODE, gpios, {0});
const struct i2c_dt_spec afficheur = I2C_DT_SPEC_GET(AFFICHEUR_NODE);
const struct device *const dht11 = DEVICE_DT_GET_ONE(aosong_dht);
const struct gpio_dt_spec buzzer_gpio = GPIO_DT_SPEC_GET_OR(BUZZER_PIN, gpios, {0});
const struct gpio_dt_spec presence_sensor_gpio = GPIO_DT_SPEC_GET_OR(ALARM_NODE, gpios, {0});

static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)};

static struct gpio_callback button_callback_data;
static struct gpio_callback button_callback_data2;

void button_pressed()
{
    printf("bouton 16 pressé !\n");
    ALARME = true;
}

void button_pressed2()
{
    printf("bouton 27 pressé !\n");
    PAS_ALARME = true;
}

int main(void)
{
    // LED ORANGE :
    gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_HIGH);
    k_sleep(K_SECONDS(1));
    gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_LOW);
    k_sleep(K_SECONDS(1));
    gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_HIGH);
    k_sleep(K_SECONDS(1));
    gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_LOW);
    k_sleep(K_SECONDS(1));
    gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_HIGH);
    k_sleep(K_SECONDS(1));

    // CAPTEUR TEMPÉRATURE / STEAM
    struct sensor_value temp, humidity;
    int err;
    uint16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        /* buffer size in bytes, not number of samples */
        .buffer_size = sizeof(buf),
    };

    err = adc_channel_setup_dt(&adc_channels[0]);

    // Configurer La broche GPIO du bouton pour les interruptions
    const struct gpio_dt_spec button_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw16), gpios, {0});

    gpio_pin_configure(button_gpio.port, button_gpio.pin, GPIO_INPUT | button_gpio.dt_flags);
    gpio_init_callback(&button_callback_data, button_pressed, BIT(button_gpio.pin));
    gpio_add_callback(button_gpio.port, &button_callback_data);
    gpio_pin_interrupt_configure(button_gpio.port, button_gpio.pin, GPIO_INT_EDGE_TO_ACTIVE);

    const struct gpio_dt_spec button_gpio2 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw27), gpios, {0});

    gpio_pin_configure(button_gpio2.port, button_gpio2.pin, GPIO_INPUT | button_gpio2.dt_flags);
    gpio_init_callback(&button_callback_data2, button_pressed2, BIT(button_gpio2.pin));
    gpio_add_callback(button_gpio2.port, &button_callback_data2);
    gpio_pin_interrupt_configure(button_gpio2.port, button_gpio2.pin, GPIO_INT_EDGE_TO_ACTIVE);

    while (1)
    {

        sensor_sample_fetch(dht11);
        sensor_channel_get(dht11, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        sensor_channel_get(dht11, SENSOR_CHAN_HUMIDITY, &humidity);

        printk("temp: %d.%06d; humidity: %d.%06d\n", temp.val1, temp.val2, humidity.val1, humidity.val2);

        (void)adc_sequence_init_dt(&adc_channels[0], &sequence);

        int16_t adc_value;
        err = adc_read_dt(&adc_channels[0], &sequence);
        if (err == 0)
        {
            uint32_t val_mv;
            val_mv = (int32_t)buf;
            err = adc_raw_to_millivolts_dt(&adc_channels[0], &val_mv);
            if (err == 0)
            {
                double volt = val_mv / 1000.0;
                printf("Tension analogique de l'humidité : %f V\n", volt);
            }
            else
            {
                printf("Erreur lors de la conversion en V");
            }
        }

        else
        {
            printf("Erreur lors de la lecture analogique\n");
        }

        k_sleep(K_SECONDS(5));
    }
}

void alarm_button_thread()
{
    int count = 0;
    init_lcd(&afficheur);
    while (1)
    {
        gpio_pin_configure_dt(&presence_sensor_gpio, GPIO_INPUT);
        // Lire la valeur du capteur de présence
        int presence = gpio_pin_get(presence_sensor_gpio.port, presence_sensor_gpio.pin);
        if (ALARME)
        {
            printf("Je suis passé par là 2\n");
            // write_lcd(&afficheur, MODE_ALARME, LCD_LINE_1);

            if (presence)
            {
                printk("Présence détectée\n");
                k_sleep(BUZZER_TOGGLE_PERIOD);
                gpio_pin_configure_dt(&buzzer_gpio, GPIO_OUTPUT_LOW);
                k_sleep(BUZZER_TOGGLE_PERIOD);
                gpio_pin_configure_dt(&buzzer_gpio, GPIO_OUTPUT_HIGH);
                k_sleep(BUZZER_TOGGLE_PERIOD);

                count++;

                if (count >= 5)
                {
                    write_lcd(&afficheur, " ATTENTION !!!! ", LCD_LINE_1);
                    write_lcd(&afficheur, "DANGER INTRU !!!", LCD_LINE_2);
                }
            }
            else
            {
                printk("Pas de présence\n");
                write_lcd(&afficheur, "Mode Alarme     ", LCD_LINE_1);
                write_lcd(&afficheur, "Active          ", LCD_LINE_2);
                count = 0;
            }
        }

        if (PAS_ALARME)
        {
            write_lcd(&afficheur, HELLO_MSG, LCD_LINE_1);
            write_lcd(&afficheur, ZEPHYR_MSG, LCD_LINE_2);
            ALARME = false;
            PAS_ALARME = false;
        }

        k_sleep(BUZZER_TOGGLE_PERIOD);
        // k_sleep(BUZZER_TOGGLE_PERIOD);
    }
}

K_THREAD_DEFINE(alarm_button_id, 521, alarm_button_thread, NULL, NULL, NULL, 9, 0, 0);
