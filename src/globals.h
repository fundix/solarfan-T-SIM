#ifndef GLOBALS_H
#define GLOBALS_H

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define TINY_GSM_YIELD_MS 2

#define SerialAT Serial1
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

#define TINY_GSM_DEBUG Serial
#define isNBIOT false

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// LilyGO T-SIM7000G Pinout
#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCLK 14
#define SD_CS 13
#define LED_PIN 12

#define PIN_ADC_BAT 35
#define PIN_ADC_SOLAR 36
#define ADC_BATTERY_LEVEL_SAMPLES 100

#define I2C_SDA 21
#define I2C_SCL 22

#endif // GLOBALS_H