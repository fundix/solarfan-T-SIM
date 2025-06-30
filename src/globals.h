#ifndef GLOBALS_H
#define GLOBALS_H

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
// #define TINY_GSM_YIELD_MS 2

#define SerialAT Serial1
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

#define TINY_GSM_DEBUG Serial
#define isNBIOT false

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// LilyGO T-SIM7000G Pinout
#define UART_BAUD 115200
#define PIN_DTR GPIO_NUM_25
#define PIN_TX GPIO_NUM_27
#define PIN_RX GPIO_NUM_26
#define PWR_PIN GPIO_NUM_4

#define SD_MISO GPIO_NUM_2
#define SD_MOSI GPIO_NUM_15
#define SD_SCLK GPIO_NUM_14
#define SD_CS GPIO_NUM_13
#define LED_PIN GPIO_NUM_12

#define PIN_ADC_BAT GPIO_NUM_35
#define PIN_ADC_SOLAR GPIO_NUM_36
#define ADC_BATTERY_LEVEL_SAMPLES 100

#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22

#define PWM_GPIO GPIO_NUM_32                // GPIO pro PWM výstup
#define PWM_CHANNEL 1                       // PWM kanál
#define PWM_FREQUENCY 25000                 // Frekvence PWM (25 kHz)
#define PWM_RESOLUTION 10                   // Rozlišení PWM (10 bit, 0‑1023)
#define PWM_MAX ((1 << PWM_RESOLUTION) - 1) // Maximální hodnota duty‑cycle (1023 pro 10‑bit)
#define PWM_MIN 100                         // Minimální hodnota duty‑cycle (0)

#endif // GLOBALS_H