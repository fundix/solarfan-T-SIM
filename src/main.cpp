#include <Arduino.h>
#include <sys/time.h>
#include <time.h>
#include "esp_log.h"
#include "Wire.h"        // Pro I2C komunikaci
#include <driver/ledc.h> // ESP‑IDF LEDC driver (ledc_timer_config, ledc_channel_config, ledc_set_duty)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define TAG "MAIN"

#define TINY_GSM_DEBUG Serial

// Check if PSRAM is available and initialize canvas with PSRAM
#include "globals.h"
#include "credentials.h"
#include <TinyGsmClient.h>
#include <PubSubClient.h>

#define BTN1 41 // Definice pinu pro tlačítko (GPIO 41)

#include <Preferences.h>   // Pro ukládání dat do NVS (Non-Volatile Storage)
#include <WiFi.h>          // Kvůli WiFi.mode()
#include "esp_wifi.h"      // pro esp_wifi_stop() a esp_wifi_deinit()
#include "esp32-hal-cpu.h" // pro setCpuFrequencyMhz()

// #include <WiFi.h>                     // Pro WiFi připojení
// #include <AsyncTCP.h>                 // Asynchronní TCP knihovna
// #include <ESPAsyncWebServer.h>        // Asynchronní webový server
// #include <ESPAsyncHTTPUpdateServer.h> // Pro OTA aktualizace firmwaru
// ESPAsyncHTTPUpdateServer updateServer;
// AsyncWebServer server(80); // Webový server na portu 80

#include <Adafruit_INA228.h>
const uint8_t bat_addr = 0x40;   // Adresa INA228 pro baterii
const uint8_t solar_addr = 0x41; // Adresa INA228 pro solární panel
Adafruit_INA228 ina228_bat = Adafruit_INA228();
Adafruit_INA228 ina228_solar = Adafruit_INA228();
bool ina_inicialized = false;

float bat_shuntVoltage = 0.0;
float bat_busVoltage = 0.0;
float bat_current = 0.0;
float bat_power = 0.0;

float solar_shuntVoltage = 0.0;
float solar_busVoltage = 0.0;
float solar_current = 0.0;
float solar_power = 0.0;

#include "NimBLEDevice.h"
#include "NimBLEClient.h"
float BLE_temperature = 0.0f; // Teplota z BLE senzoru
float BLE_humidity = 0.0f;    // Vlhkost z BLE senzoru
float BLE_voltage = 0.0f;     // Napětí z BLE senzoru (pokud je potřeba)
static constexpr uint32_t scanTimeMs = 15 * 1000;
static bool doConnect = false;
static bool BLEConnected = false;
static uint8_t BLERetries = 0;
static const NimBLEAdvertisedDevice *advDevice;
#define BLE_DEVICE_NAME "LYWSD03MMC"
#define BLE_SCAN_MAX_RESULTS 80
// The remote service we wish to connect to.
#define SERVICE_UUID "ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6"
// The characteristic of the remote service we are interested in.
#define CHAR_UUID "ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6"
void ble_setup();

static unsigned long lastActivityTime = 0; // Timestamp of last user action
static bool displaySleeping = false;

inline void resetInactivityTimer()
{
    lastActivityTime = millis();
}

uint32_t lastReconnectAttempt = 0;
void gsmLoop();
void gsmSetup();
void gsmTask(void *pvParameters);
void gsmConnect(void);
void disableGPS(void);
void enableGPS(void);

bool mqttConnected = false;
const char *topicStatus = "solarfan/status";
const char *topicControl = "solarfan/control";

#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

// MQTT details

void measure();
// Funkce pro obsluhu nenalezených stránek webového serveru

// void setupWiFiClient();
// void buttonLoop();
// void shortPressed();
// void longPressed();
// void drawGUI(); // Deklarace funkce pro kreslení GUI
void setupPWM();
void read_adc_bat(uint16_t *voltage);
void read_adc_solar(uint16_t *voltage);

// PWM definice

uint16_t pwm = 0;

Preferences preferences; // Instance pro NVS Preferences

// --- LED heartbeat definitions ---
static TimerHandle_t ledTimer = NULL; // FreeRTOS timer for heartbeat LED
static void ledTimerCallback(TimerHandle_t xTimer);

// Globální proměnné
static unsigned long buttonPressStartTime = 0;
static bool buttonPressed = false;
static bool longButtonPressed = false;
static bool updateStarted = false;
static float rotationAngle = 0.0f; // Úhel rotace pro displej

// #define DUMP_AT_COMMANDS // If you need to debug, you can open this macro definition and TinyGsmClientSIM7028.h line 13 //#define TINY_GSM_DEBUG Serial
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);
bool mqttConnect();
void publishMeasurements();

/** Define a class to handle the callbacks for client connection events */
class ClientCallbacks : public NimBLEClientCallbacks
{
    void onConnect(NimBLEClient *pClient) override
    {
        BLERetries = 0;
        ESP_LOGI(TAG, "Connected");
    }

    void onDisconnect(NimBLEClient *pClient, int reason) override
    {
        ESP_LOGI(TAG, "%s Disconnected, reason = %d - Starting scan", pClient->getPeerAddress().toString().c_str(), reason);
        NimBLEDevice::getScan()->start(scanTimeMs, true, true);
        BLEConnected = false;
    }

} clientCallbacks;

/** Define a class to handle the callbacks when advertisements are received */
class ScanCallbacks : public NimBLEScanCallbacks
{
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
    {
        if (advertisedDevice->getRSSI() < -82 || !advertisedDevice->haveName())
        {
            NimBLEDevice::getScan()->erase(advertisedDevice);
            return;
        }

        std::string deviceName = advertisedDevice->getName();

        // Hledáme zařízení s přesným názvem BLEName
        if (deviceName == BLE_DEVICE_NAME)
        {
            ESP_LOGI(TAG, "Found exact matching device: %s", BLE_DEVICE_NAME);
            NimBLEDevice::getScan()->stop();
            advDevice = advertisedDevice;
            doConnect = true;
        }

        ESP_LOGI(TAG, "Advertised Device found: %s, heap: %d", deviceName.c_str(), ESP.getFreeHeap());
    }

    /** Callback to process the results of the completed scan or restart it */
    void onScanEnd(const NimBLEScanResults &results, int reason) override
    {
        ESP_LOGI(TAG, "Scan Ended, reason: %d, device count: %d; Restarting scan", reason, results.getCount());

        // if (BLERetries < BLEMaxRetries)
        // {

        if (NimBLEDevice::getConnectedClients().empty())
        {
            // NimBLEClient *pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());

            // after end of scan, we look for the table in the current list
            ESP_LOGI(TAG, "Scan End results.count %d", results.getCount());

            for (int i = 0; i < results.getCount(); i++)
            {
                const NimBLEAdvertisedDevice *device = results.getDevice(i);
                String deviceName = device->getName().c_str();
                ESP_LOGI(TAG, "Device name: %s", deviceName.c_str());
                if (deviceName == BLE_DEVICE_NAME)
                {
                    ESP_LOGI(TAG, "Found exact matching device: %s, rssi: %d", BLE_DEVICE_NAME, device->getRSSI());
                    NimBLEDevice::getScan()->stop();

                    advDevice = results.getDevice(i);
                    doConnect = true;
                }
            }

            NimBLEScan *pScan = NimBLEDevice::getScan();

            // NimBLEDevice::getScan()->stop();
            pScan->start(scanTimeMs, false, true);
            pScan->setDuplicateFilter(false); // if is false then onResult will report all new results
        }
        else
        {
            ESP_LOGI(TAG, "getConnectedClients not empty");
            if (BLEConnected)
            {
                std::vector<NimBLEClient *> connectedClients = NimBLEDevice::getConnectedClients();
                for (auto *client : connectedClients)
                {
                    if (client && client->isConnected())
                    {
                        client->disconnect();
                        ESP_LOGI("BLE", "Disconnected client: %s", client->getPeerAddress().toString().c_str());
                    }
                }
            }
        }
        // }
    }
} scanCallbacks;

bool connectToBTServer()
{
    ESP_LOGI(TAG, "heap %d", ESP.getFreeHeap());

    NimBLEClient *pClient = nullptr;

    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getCreatedClientCount())
    {
        /**
         *  Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if (pClient)
        {
            if (!pClient->connect(advDevice, false))
            {
                ESP_LOGI(TAG, "Reconnect failed");
                return false;
            }
            ESP_LOGI(TAG, "Reconnected client");
        }
        else
        {
            /**
             *  We don't already have a client that knows this device,
             *  check for a client that is disconnected that we can use.
             */
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if (!pClient)
    {
        if (NimBLEDevice::getCreatedClientCount() >= NIMBLE_MAX_CONNECTIONS)
        {
            ESP_LOGI(TAG, "Max clients reached - no more connections available");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        ESP_LOGI(TAG, "New client created");

        pClient->setClientCallbacks(&clientCallbacks, false);
        /**
         *  Set initial connection parameters:
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 150 * 10ms = 1500ms timeout
         */
        pClient->setConnectionParams(24, 40, 0, 200, 16, 16);

        /** Set how long we are willing to wait for the connection to complete (milliseconds), default is 30000. */
        pClient->setConnectTimeout(5 * 1000);

        if (!pClient->connect(advDevice))
        {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            ESP_LOGI(TAG, "Failed to connect, deleted client");
            return false;
        }
    }

    if (!pClient->isConnected())
    {
        if (!pClient->connect(advDevice))
        {
            ESP_LOGI(TAG, "Failed to connect");
            return false;
        }
    }

    ESP_LOGI(TAG, "Connected to: %s RSSI: %d", pClient->getPeerAddress().toString().c_str(), pClient->getRssi());

    /** Now we can read/write/subscribe the characteristics of the services we are interested in */
    NimBLERemoteService *pSvc = nullptr;
    NimBLERemoteCharacteristic *pChrTable = nullptr;

    // NimBLERemoteDescriptor *pDsc = nullptr;

    pSvc = pClient->getService(SERVICE_UUID);
    if (pSvc)
    {
        pChrTable = pSvc->getCharacteristic(CHAR_UUID);
        if (pChrTable)
        {
            // if (pChrTable->canRead())
            // {
            //   std::string value = pChrTable->readValue();
            //   if (value.length() >= sizeof(uint16_t))
            //   {
            //     uint16_t intValue = *(uint16_t *)value.data();
            //     tableController->set_height(intValue / 10.0f);

            //     ESP_LOGI(TAG, "Characteristic value: %u", intValue);
            //   }
            //   else
            //   {
            //     ESP_LOGI(TAG, "Characteristic value is smaller than expected (less than 2 bytes)!");
            //   }
            // }
            // else
            // {
            //   ESP_LOGI(TAG, "Characteristic does not support reading.");
            // }

            if (pChrTable->canNotify())
            {
                pChrTable->subscribe(true, [](NimBLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify)
                                     {
          ESP_LOGV(TAG, "Notify callback for characteristic %s, length=%d",
                   pCharacteristic->getUUID().toString().c_str(), length);

          if (length >= 3) {
            // Temperature: signed int16 little‑endian, value is °C × 100
            int16_t rawTemp = (int16_t)(pData[0] | (pData[1] << 8));
            BLE_temperature = rawTemp / 100.0f;

            // Humidity: single byte, % RH
            BLE_humidity = pData[2];

            // Optional battery voltage: unsigned int16 little‑endian, value is mV
            if (length >= 5) {
              uint16_t rawVolt = (uint16_t)(pData[3] | (pData[4] << 8));
              BLE_voltage = rawVolt / 1000.0f;
            }

            ESP_LOGV(TAG,
                     "temp = %.2f °C ; humidity = %.1f %% ; voltage = %.3f V",
                     BLE_temperature, BLE_humidity, BLE_voltage);
          } else {
            ESP_LOGI(TAG, "Notification data size unexpected: %d bytes", length);
          } });

                ESP_LOGI(TAG, "Subscribed to notifications!");
            }
            else
            {
                ESP_LOGI(TAG, "Characteristic does not support notifications.");
            }
        }
        else
        {
            ESP_LOGI(TAG, "Failed to find characteristic for table height.");
        }
    }

    // BLESaveToWhiteList();
    // Počkej krátkou dobu na dokončení párování
    ESP_LOGI(TAG, "Connection completed (no secureConnection check needed)");
    ESP_LOGI(TAG, "Done with this device!");

    return true;
}

/* ------------------------------------------------------------------
 *  NetworkTest‑derived modem helpers
 * ------------------------------------------------------------------*/
static void modemPowerOn()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);  // active LOW pulse on T‑SIM7000 PWR
    delay(1500);                 // ≥1.5 s
    digitalWrite(PWR_PIN, HIGH); // back to idle
}

static void modemPowerOff()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(1500);
    digitalWrite(PWR_PIN, HIGH);
}

// Full attach sequence taken from Arduino_NetworkTest.ino
bool modemConnect()
{
    modemPowerOn();
    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    // 1) Wait until AT replies, try hard‑reset every 30 s if silent
    uint32_t t0 = millis();
    while (!modem.testAT())
    {
        if (millis() - t0 > 30000)
        {
            modemPowerOff();
            delay(3000);
            modemPowerOn();
            t0 = millis();
        }
        delay(500);
    }

    // 2) SIM ready?
    if (modem.getSimStatus() != SIM_READY)
        return false;

    // 3) Radio setup – GSM‑only (2 G) – faster attach, no LTE
    // Allow just GSM bands; you can narrow the list further if desired
    modem.sendAT("+CBAND=GSM850P,GSM900P,DCS1800P,PCS1900P");
    modem.setPreferredMode(1); // 1 = GSM
    modem.setNetworkMode(1);   // 1 = GSM only

    // modem.sendAT("+CBAND=ALL_MODE"); // let FW scan entire band map
    // modem.setPreferredMode(3);       // 3 = Cat‑M + NB‑IoT
    // modem.setNetworkMode(2);         // 2 = Automatic (LTE+GSM)

    // 4) Wait for network registration (max 180 s)
    SIM70xxRegStatus reg;
    uint32_t limit = millis() + 180000;
    do
    {
        reg = modem.getRegistrationStatus();
        if (reg == REG_DENIED)
            return false;
        delay(500);
    } while (reg != REG_OK_HOME && reg != REG_OK_ROAMING && millis() < limit);
    if (reg != REG_OK_HOME && reg != REG_OK_ROAMING)
        return false;

    // 5) PDP attach with Hologram APN
    if (!modem.gprsConnect(apn, gprsUser, gprsPass))
        return false;

    return true; // modem has IP, ready for MQTT
}

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
    ESP_LOGI(TAG, "Message arrived [%s]: %.*s", topic, len, payload);

    // Only proceed if incoming message's topic matches
    if (String(topic) == topicControl)
    {
        // ledStatus = !ledStatus;
        // Convert payload to string and then to integer
        String msg = String((char *)payload, len);
        if (msg == "ON")
        {
            pwm = PWM_MAX; // Set to maximum duty cycle
        }
        else if (msg == "OFF")
        {
            pwm = 0;
        }
        else
        {
            pwm = msg.toInt();
            // Ensure pwm value is within valid range (0‑PWM_MAX for 10‑bit resolution)
            pwm = constrain(pwm, 0, PWM_MAX); // Udrž hodnotu v rozsahu rozlišení
            // Set PWM value
            if (pwm < PWM_MIN)
            {
                pwm = 0; // Ensure minimum duty cycle
            }
        }

        ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), pwm);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL));
        ESP_LOGI(TAG, "Setting PWM to %d", pwm);

        publishMeasurements();
        // digitalWrite(LED_PIN, ledStatus);
        // mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
    }
}

// void modem_reset()
// {
//     Serial.println("Modem hardware reset");
//     pinMode(MODEM_RST, OUTPUT);
//     digitalWrite(MODEM_RST, LOW);
//     delay(260); // Treset 252ms
//     digitalWrite(MODEM_RST, HIGH);
//     delay(4000); // Modem takes longer to get ready and reply after this kind of reset vs power on

//     // modem.factoryDefault();
//     // modem.restart(); //this results in +CGREG: 0,0
// }

void modem_on()
{
    // Ensure UART is ready so we can probe the modem
    Serial.println("Probing modem…");
    if (modem.testAT(500))
    { // quick "AT" probe (wait 500 ms)
        Serial.println("Modem is already ON → skip PWRKEY");
        return;
    }

    // Modem is OFF → toggle PWRKEY (LOW pulse ≥ 1.5 s on LilyGO T‑SIM7000G)
    Serial.println("Toggling PWRKEY to power modem ON");
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH); // idle level
    delay(10);
    digitalWrite(PWR_PIN, LOW);  // active LOW
    delay(1600);                 // hold ≥1.5 s
    digitalWrite(PWR_PIN, HIGH); // back to idle

    Serial.println("Pulse sent, waiting 6 s for modem boot…");
    delay(6000); // boot‑up time until 'RDY'
}

void modem_off()
{
    // if you turn modem off while activating the fancy sleep modes it takes ~20sec, else its immediate
    Serial.println("Going to sleep now with modem turned off");
    // modem.gprsDisconnect();
    // modem.radioOff();
    modem.sleepEnable(false); // required in case sleep was activated and will apply after reboot
    modem.poweroff();
}

// fancy low power mode - while connected
// void modem_sleep() // will have an effect after reboot and will replace normal power down
// {
//     Serial.println("Going to sleep now with modem in power save mode");
//     // needs reboot to activa and takes ~20sec to sleep
//     modem.PSM_mode();    // if network supports will enter a low power sleep PCM (9uA)
//     modem.eDRX_mode14(); // https://github.com/botletics/SIM7000-LTE-Shield/wiki/Current-Consumption#e-drx-mode
//     modem.sleepEnable(); // will sleep (1.7mA), needs DTR or PWRKEY to wake
//     pinMode(PIN_DTR, OUTPUT);
//     digitalWrite(PIN_DTR, HIGH);
// }

void modem_wake()
{
    Serial.println("Wake up modem from sleep");
    // DTR low to wake serial
    pinMode(PIN_DTR, OUTPUT);
    digitalWrite(PIN_DTR, LOW);
    delay(50);
    // wait_till_ready();
}

void shutdown()
{

    // modem_sleep();
    modem_off();

    delay(1000);
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("Going to sleep now");
    delay(1000);
    Serial.flush();
    esp_deep_sleep_start();
}

// Initialize PWM
void setupPWM()
{

    // --- Configure LEDC timer ---
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

    // --- Configure LEDC channel ---
    ledc_channel_config_t channel_conf = {
        .gpio_num = PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = static_cast<ledc_channel_t>(PWM_CHANNEL),
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&channel_conf);

    // Ensure initial duty is applied
    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL));

    ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL, 0);
}

bool mqttConnect()
{
    ESP_LOGI(TAG, "Connecting to MQTT broker %s:%d", broker, mqttPort);

    // Connect to MQTT Broker
    bool status = mqtt.connect("SolarFan", MQTT_USER, MQTT_PASS);

    // Or, if you want to authenticate MQTT:
    // bool status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

    if (!status)
    {
        ESP_LOGI(TAG, "fail (state = %d)", mqtt.state());
        return false;
    }
    ESP_LOGI(TAG, "success");
    // mqtt.publish(topicInit, "GsmClientTest started");
    mqtt.subscribe(topicControl);
    publishMeasurements();

    mqttConnected = mqtt.connected();
    return mqttConnected;
}

void publishMeasurements()
{
    // Build a JSON string with all fields
    String payload = "{";
    payload += "\"tmp\":" + String(BLE_temperature, 2);
    payload += ",\"hum\":" + String(BLE_humidity, 1);
    payload += ",\"bv\":" + String(BLE_voltage, 3);
    payload += ",\"batv\":" + String(bat_busVoltage, 2);
    payload += ",\"bp\":" + String(bat_power, 0);
    payload += ",\"bc\":" + String(bat_current, 0);
    payload += ",\"sv\":" + String(solar_busVoltage, 2);
    payload += ",\"sp\":" + String(solar_power, 0);
    payload += ",\"pwm\":" + String(pwm);
    payload += "}";

    ESP_LOGI(TAG, "Publishing measurements: %s", payload.c_str());

    if (mqtt.connected())
    {
        mqtt.publish(topicStatus, payload.c_str());
    }
}

/**
 * Heart‑beat LED pattern (non‑blocking)
 * - If mqttConnected == true  → single 100 ms flash every 5 s
 * - If mqttConnected == false → triple 100 ms flashes every 5 s
 * Executed by a 100 ms FreeRTOS software timer.
 */
static void ledTimerCallback(TimerHandle_t /*xTimer*/)
{
    static uint32_t tick = 0;        // 100 ms ticks
    const uint32_t periodTicks = 50; // 5 s / 100 ms
    uint32_t idx = tick % periodTicks;

    if (mqttConnected)
    {
        // One short flash at the start of each 5‑second window
        digitalWrite(LED_PIN, (idx == 0) ? LOW : HIGH);
    }
    else
    {
        // Three flashes (idx 0,2,4) at the start of each window
        bool on = (idx == 0 || idx == 2 || idx == 4);
        digitalWrite(LED_PIN, on ? LOW : HIGH);
    }

    tick++;
}

void setup()
{
    // Initialize ESP log system
    esp_log_level_set("*", ESP_LOG_INFO); // Set default log level to INFO for all tags

    setupPWM();
    // Inicializace NVS pro ukládání nastavení displeje
    // preferences.begin("display", false);
    lastActivityTime = millis();

    // Inicializace M5Stack AtomS3 a displeje
    // --- Deaktivace Wi‑Fi rozhraní, šetří paměť i energii ---
    esp_wifi_set_mode(WIFI_MODE_NULL); // Vypne WiFi (všechny režimy)
    esp_wifi_stop();                   // Zastaví Wi‑Fi driver
    esp_wifi_deinit();                 // Uvolní RAM alokovanou driverem
    // --- Snížení taktu CPU kvůli úspoře energie ---
    // setCpuFrequencyMhz(80); // 80 MHz místo výchozích 240 MHz

    Serial.begin(115200); // Inicializace sériové komunikace

    // --- LED heartbeat setup ---
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    ledTimer = xTimerCreate(
        "LedTimer",
        pdMS_TO_TICKS(100), // 100 ms period
        pdTRUE,             // auto‑reload
        nullptr,
        ledTimerCallback);

    if (ledTimer != NULL)
    {
        xTimerStart(ledTimer, 0);
    }

    vTaskDelay(4000 / portTICK_PERIOD_MS); // Zpoždění pro připojení sériového monitoru

    pinMode(PIN_ADC_BAT, INPUT);
    pinMode(PIN_ADC_SOLAR, INPUT);

    uint16_t v_bat = 0;
    uint16_t v_solar = 0;
    read_adc_bat(&v_bat);
    Serial.print("BAT: ");
    Serial.print(v_bat);

    read_adc_solar(&v_solar);
    Serial.print(" SOLAR: ");
    Serial.println(v_solar);

    // Create FreeRTOS timer for publishing measurements every 5 minutes
    static TimerHandle_t publishTimer = NULL;
    if (publishTimer == NULL)
    {
        publishTimer = xTimerCreate(
            "PublishTimer",
            pdMS_TO_TICKS(300000), // 5 minutes = 300000 ms
            pdTRUE,                // Auto reload
            nullptr,
            [](TimerHandle_t xTimer)
            {
                publishMeasurements();
            });
        // xTimerStart(publishTimer, 0);
    }

    Wire.begin(I2C_SDA, I2C_SCL); // Inicializace I2C sběrnice

    if (ina228_bat.begin(bat_addr) && ina228_solar.begin(solar_addr))
    {
        ina_inicialized = true;
        ESP_LOGI(TAG, "INA228 devices initialized successfully");

        ina228_bat.setShunt(0.015, 10.0);
        ina228_bat.setAveragingCount(INA228_COUNT_64);
        ina228_bat.setVoltageConversionTime(INA228_TIME_540_us);
        ina228_bat.setCurrentConversionTime(INA228_TIME_280_us);

        ina228_solar.setShunt(0.015, 10.0);
        ina228_solar.setAveragingCount(INA228_COUNT_64);
        ina228_solar.setVoltageConversionTime(INA228_TIME_540_us);
        ina228_solar.setCurrentConversionTime(INA228_TIME_280_us);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize INA228 devices");
    }

    ble_setup();
    xTaskCreatePinnedToCore(gsmTask, "GSM", 8192, NULL, 1, NULL, 1);
}

void ble_setup()
{
    ESP_LOGI(TAG, "Starting NimBLE Client");

    /** Initialize NimBLE and set the device name */
#if !CONFIG_BT_BLUEDROID_ENABLED // Arduino vždy vypíná Bluedroid
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
#endif
    ESP_LOGI(TAG, "Free DRAM after NimBLE: %u", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    NimBLEDevice::init("SolarFan");
    NimBLEDevice::setPower(8);

    // NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND);   // Jen bonding, bez MITM
    // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // Just Works režim

    // NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    // NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

    /** Set the callbacks to call when scan events occur, no duplicates */
    NimBLEScan *pScan = NimBLEDevice::getScan();

    /** Set the callbacks to call when scan events occur, no duplicates */
    pScan->setScanCallbacks(&scanCallbacks, false);

    /**
     * Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    pScan->setActiveScan(true);
    pScan->setMaxResults(BLE_SCAN_MAX_RESULTS);

    /** Start scanning for advertisers */
    pScan->start(scanTimeMs, true, true);
    ESP_LOGI(TAG, "Scanning for peripherals");
}

void loop()
{
    delay(20); // Krátké zpoždění pro stabilitu

    // Volání buttonLoop() pro zpracování stisku tlačítka
    // buttonLoop();

    // BLE scanning and connection to sensors named "LYWSD03MMC"
    // (Non-blocking scan and connection handled by MyAdvertisedDeviceCallbacks)

    static unsigned long lastDrawTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastDrawTime >= 5000)
    {
        measure();
        // drawGUI();
        lastDrawTime = currentTime;

        // if (!displaySleeping && (millis() - lastActivityTime > 120000UL))
        // {
        //     M5.Display.sleep(); // turns the panel off (M5Unified)
        //     displaySleeping = true;
        // }
    }

    if (doConnect)
    {
        doConnect = false;
        /** Found a device we want to connect to, do it now */
        if (connectToBTServer())
        {
            BLEConnected = true;

            ESP_LOGI(TAG, "Success! we should now be getting notifications!");
        }
        else
        {
            BLEConnected = false;

            ESP_LOGI(TAG, "Failed to connect, starting scan");
            NimBLEDevice::getScan()->start(scanTimeMs, true, true);
        }
    }
}

void measure()
{
    // Battery measurements
    if (!ina_inicialized)
    {
        // ESP_LOGE(TAG, "INA228 devices not initialized");
        return; // Pokud INA228 není inicializován, ukončíme měření
    }

    bat_shuntVoltage = ina228_bat.readShuntVoltage();
    bat_busVoltage = ina228_bat.readBusVoltage() / 1000000.0;
    bat_current = ina228_bat.readCurrent();
    bat_power = ina228_bat.readPower();

    solar_shuntVoltage = ina228_solar.readShuntVoltage();
    solar_busVoltage = ina228_solar.readBusVoltage() / 1000000.0;
    solar_current = ina228_solar.readCurrent();
    solar_power = ina228_solar.readPower();

    // Table header
    ESP_LOGI(TAG, "Battery & Solar Measurements Table");
    ESP_LOGI(TAG, "+---------------+-------------+-------------+");
    ESP_LOGI(TAG, "| Parameter     | Battery     | Solar       |");
    ESP_LOGI(TAG, "+---------------+-------------+-------------+");
    // ESP_LOGI(TAG, "| Shunt Voltage | %8.2f mV | %8.2f mV |", bat_shuntVoltage, solar_shuntVoltage);
    ESP_LOGI(TAG, "| Bus Voltage   | %8.2f V  | %8.2f V  |", bat_busVoltage, solar_busVoltage);
    ESP_LOGI(TAG, "| Current       | %8.2f mA | %8.2f mA |", bat_current, solar_current);
    ESP_LOGI(TAG, "| Power         | %8.1f mW | %8.1f mW |", bat_power, solar_power);
    ESP_LOGI(TAG, "+---------------+-------------+-------------+");
}

// Funkce pro zpracování stisku tlačítka
// void buttonLoop()
// {
//     if (digitalRead(BTN1) == LOW)
//     { // Tlačítko stisknuto (aktivní LOW)
//         if (!buttonPressed)
//         {
//             buttonPressed = true;
//             buttonPressStartTime = millis();
//         }
//         // Pokud držíte tlačítko déle než 2 sekundy a ještě nebyl detekován dlouhý stisk
//         else if (!longButtonPressed && (millis() - buttonPressStartTime > 2000))
//         {
//             longButtonPressed = true;
//             longPressed(); // Zavoláme funkci pro dlouhý stisk
//         }

//         if (displaySleeping)
//         {
//             M5.Display.wakeup();
//             M5.Display.setBrightness(20); // your normal brightness
//             displaySleeping = false;
//         }
//         resetInactivityTimer();
//     }
//     else
//     { // Tlačítko uvolněno
//         if (buttonPressed)
//         {
//             if (!longButtonPressed)
//             {
//                 shortPressed(); // Zavoláme funkci pro krátký stisk
//             }
//             // Resetujeme stav tlačítka
//             buttonPressed = false;
//             longButtonPressed = false;
//         }
//     }
// }

// Funkce pro krátký stisk tlačítka
// void shortPressed()
// {
//     ESP_LOGI(TAG, "Short button press");

//     // Změna úhlu rotace displeje
// }

// // Funkce pro dlouhý stisk tlačítka
// void longPressed()
// {
//     ESP_LOGI(TAG, "Long button press");
//     rotationAngle = fmod(rotationAngle + 15, 360.0f);
//     preferences.putFloat("rotation", rotationAngle); // Uložení nového úhlu
//                                                      // if (updateStarted)
//                                                      // {
//                                                      //   ESP.restart(); // Restart zařízení, pokud je aktualizace spuštěna
//                                                      //   return;
//                                                      // }
//                                                      // preferences.end(); // Uzavření NVS Preferences

//     // setupWiFiClient(); // Spuštění WiFi klienta a OTA serveru

//     // updateStarted = true; // Nastavení příznaku, že aktualizace začala
// }

// Funkce pro kreslení grafického uživatelského rozhraní
// void drawGUI()
// {
//     if (displaySleeping)
//         return;

//     canvas.fillSprite(BLACK); // Vyplnění canvasu černou barvou
//     // Vykreslení kruhu pro ukazatel rychlosti
//     int centerX = canvas.width() / 2;
//     int centerY = canvas.height() / 2;

//     if (updateStarted)
//     {
//         // Update mode display
//         canvas.setTextSize(0.6);
//         canvas.setTextDatum(middle_center);
//         canvas.drawString("Updating", centerX, centerY - 11);

//         // if (WiFi.status() == WL_CONNECTED)
//         // {
//         //   IPAddress IP = WiFi.localIP();
//         //   canvas.setTextSize(0.4);
//         //   canvas.drawString(IP.toString().c_str(), centerX, centerY + 11);
//         // }
//     }
//     else
//     {
//         // Nastavení pro canvas (jak bylo poskytnuto)
//         // --- Nastavení pro canvas ---
//         canvas.setFont(&fonts::Font4); // Nastavení písma (Font4 je dobrá volba pro čitelnost)
//         canvas.setTextSize(0.6);       // Nastavení velikosti textu pro všechny výpisy
//         // canvas.setTextDatum(middle_center); // Zarovnání textu na střed (horizontálně i vertikálně)

//         int x = 10;           // Středová osa X displeje
//         int y = 10;           // Počáteční pozice Y pro střed prvního řádku textu
//         int lineSpacing = 17; // Mezera mezi středy řádků textu (upraveno pro velikost písma 0.6)

//         // --- Vykreslení informací na displej ---
//         // 2. Sekce Solárního panelu - zkrácené popisky
//         canvas.drawString("Solar", x, y); // Zkrácený název
//         if (mqttConnected)
//         {
//             canvas.drawString("Mqtt 1", x + 60, y);
//         }
//         else
//         {
//             canvas.drawString("Mqtt 0", x + 60, y);
//         }
//         y += lineSpacing; // Mezera před další sekcí

//         canvas.drawString(String(solar_busVoltage, 2) + "  V", x, y);
//         canvas.drawString(String(solar_power, 0) + "  mW", x + 50, y);
//         y += lineSpacing; // Mezera před další sekcí

//         // 3. Sekce Baterie - zkrácené popisky
//         canvas.drawString("Bat", x, y); // Zkrácený název
//         y += lineSpacing;               // Mezera před další sekcí

//         canvas.drawString(String(bat_busVoltage, 2) + "  V", x, y);
//         canvas.drawString(String(bat_current * bat_busVoltage, 0) + "  mW", x + 50, y);

//         y += lineSpacing; // Mezera před další sekcí
//         // 4. Sekce BLE Senzor - zkrácené popisky
//         canvas.drawString("BLE", x, y); // Zkrácený název
//         y += lineSpacing;               // Mezera před další sekcí
//         canvas.drawString(String(BLE_temperature, 2) + "  °C", x, y);
//         canvas.drawString(String(BLE_humidity, 1) + "  %", x + 60, y);
//         y += lineSpacing; // Mezera před další sekcí
//         canvas.drawString(String(BLE_voltage, 3) + "  V", x, y);
//     }

//     // Otočení a zobrazení canvasu na displeji
//     canvas.pushRotated(rotationAngle);
// }

// void notFound(AsyncWebServerRequest *request)
// {
//   request->send(404, "text/plain", "Not found");
// }

// Funkce pro nastavení WiFi klienta a OTA serveru
// void setupWiFiClient()
// {
// Připojení k WiFi síti
// WiFi.mode(WIFI_STA);
// // Nahraď SSID a heslo správnými údaji!
// WiFi.begin("Vivien", "Bionicman123");

// ESP_LOGI(TAG, "Connecting to WiFi...");

// // Čekání na připojení
// int timeout = 20; // Maximální čas připojení (10 sekund)
// while (WiFi.status() != WL_CONNECTED && timeout > 0)
// {
//   delay(500);
//   ESP_LOGI(TAG, ".");
//   timeout--;
// }

// if (WiFi.status() == WL_CONNECTED)
// {
//   ESP_LOGI(TAG, "Connected to WiFi!");
//   IPAddress IP = WiFi.localIP();
//   ESP_LOGI(TAG, "Client IP address: %s", IP.toString().c_str());

//   // Nastavení webového serveru
//   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
//             {
//     String html = "<html><body>";
//     html += "<h1>FanSpeed</h1>";
//     html += "<p><a href='/update'>Update firmware</a> (login: admin, password: admin)</p>";
//     html += "</body></html>";
//     request->send(200, "text/html", html); });

//   server.onNotFound(notFound);

//   // Nastavení OTA aktualizačního serveru s přihlašovacími údaji
//   updateServer.setup(&server, "admin", "admin");
//   server.begin(); // Spuštění webového serveru
// }
// else
// {
//   ESP_LOGI(TAG, "Failed to connect to WiFi!");
// }
// }

void gsmSetup()
{
    ESP_LOGI(TAG, "Starting GSM modem Serial...");

    // First bring up the UART so we can probe the modem state
    SerialAT.end();
    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    // Now power the modem on – the routine will skip the pulse if it already responds
    if (!modemConnect())
    {
        ESP_LOGE(TAG, "Modem failed to attach – rebooting");
        ESP.restart();
    }
    disableGPS();

    // SerialAT.setPins(ATOM_DTU_SIM7028_RX, ATOM_DTU_SIM7028_TX, -1); // Nastavení pinů pro RX, TX, RST (pokud není potřeba, použijte -1)
    delay(100);

    String modemInfo = modem.getModemInfo();
    Serial.print(F("Modem: "));
    Serial.println(modemInfo);

    // gsmConnect();

    mqtt.setServer(broker, mqttPort);
    mqtt.setCallback(mqttCallback);
    if (!mqttConnect())
    {
        ESP_LOGE(TAG, "MQTT connect failed");
    }
    if (modem.isNetworkConnected())
    {
        ESP_LOGI(TAG, "Network connected");
    }
}

void gsmLoop()
{
    static uint32_t lastNetCheck = 0;
    static uint32_t lastMqttServe = 0;

    const uint32_t MQTT_LOOP_PERIOD = 500;   // serve MQTT every 0.5 s
    const uint32_t NET_CHECK_PERIOD = 30000; // check network every 30 s

    uint32_t now = millis();

    /* ---- MQTT keep‑alive ---- */
    if (now - lastMqttServe >= MQTT_LOOP_PERIOD)
    {
        lastMqttServe = now;

        if (mqtt.connected())
        {
            mqtt.loop(); // fast, non‑blocking
        }
        else
        {
            mqttConnected = false; // reflect status for GUI
            mqttConnect();         // quick reconnect attempt
        }
    }

    /* ---- Cellular link health ---- */
    if (now - lastNetCheck >= NET_CHECK_PERIOD)
    {
        lastNetCheck = now;

        if (!modem.isNetworkConnected())
        {
            ESP_LOGI(TAG, "Network lost → reconnecting");
            if (!modem.waitForNetwork(180000L, true))
            {
                ESP_LOGW(TAG, "Re‑attach failed, will retry later");
            }
            else
            {
                ESP_LOGI(TAG, "Network re‑attached");
            }
        }
    }
}

void gsmTask(void *pvParameters)
{
    gsmSetup(); // one‑time init
    for (;;)
    {
        gsmLoop();                           // keep GSM & MQTT alive
        vTaskDelay(50 / portTICK_PERIOD_MS); // run every 50 ms → plenty for timers above
    }
}

void gsmConnect(void)
{
    unsigned long start = millis();
    ESP_LOGI(TAG, "Initializing modem...");
    while (!modem.init())
    {
        ESP_LOGI(TAG, "waiting....%ds", (millis() - start) / 1000);
    };

    // --- Hologram SIM7000 configuration sequence (per official guide) ---
    // 1) Ensure full functionality (RF on)
    modem.sendAT("+COPS=0");

    // 2) Restrict the radio to LTE only
    // modem.sendAT("+CNMP=38");

    // 3) Prefer LTE Cat‑M1 over NB‑IoT
    // modem.sendAT("+CMNB=1");

    // 4) Define PDP context 1 with the Hologram APN
    modem.sendAT("+CGDCONT=1,\"IP\",\"hologram\"");

    modem.sendAT("+CFUN=1");

    vTaskDelay(200 / portTICK_PERIOD_MS); // give the modem a moment to apply settings

    // --- wait until the SIM is reported READY (max 15 s) ---
    uint32_t t0 = millis();
    while (modem.getSimStatus() != SIM_READY && millis() - t0 < 15000)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "SIM status after CFUN=1: %d", modem.getSimStatus());

    start = millis();
    ESP_LOGI(TAG, "Waiting for network...");
    while (!modem.waitForNetwork())
    {
        ESP_LOGI(TAG, "waiting....%ds", (millis() - start) / 1000);
    }
    ESP_LOGI(TAG, "success");
    // Get card number
    String ccid = modem.getSimCCID();
    ESP_LOGI(TAG, "CCID: %s", ccid.c_str());
    // Acquire signal strength
    int csq = modem.getSignalQuality();
    ESP_LOGI(TAG, "Signal quality: %d", csq);
    ESP_LOGI(TAG, "Waiting for GPRS connect...");
    if (!modem.gprsConnect(apn, gprsUser, gprsPass))
    {
        ESP_LOGI(TAG, "waiting....%ds", (millis() - start) / 1000);
    }
    ESP_LOGI(TAG, "success");

    // Example Query the IP address of a device
    String ip = modem.getLocalIP();

    ESP_LOGI(TAG, "Device IP address: %s", ip.c_str());

    ESP_LOGI(TAG, "success");
}

void read_adc_bat(uint16_t *voltage)
{
    uint32_t in = 0;
    for (int i = 0; i < ADC_BATTERY_LEVEL_SAMPLES; i++)
    {
        in += (uint32_t)analogRead(PIN_ADC_BAT);
    }
    in = (int)in / ADC_BATTERY_LEVEL_SAMPLES;

    uint16_t bat_mv = ((float)in / 4096) * 3600 * 2;

    *voltage = bat_mv;
}

void read_adc_solar(uint16_t *voltage)
{
    uint32_t in = 0;
    for (int i = 0; i < ADC_BATTERY_LEVEL_SAMPLES; i++)
    {
        in += (uint32_t)analogRead(PIN_ADC_SOLAR);
    }
    in = (int)in / ADC_BATTERY_LEVEL_SAMPLES;

    uint16_t bat_mv = ((float)in / 4096) * 3600 * 2;

    *voltage = bat_mv;
}

void enableGPS(void)
{
    // Set Modem GPS Power Control Pin to HIGH ,turn on GPS power
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+CGPIO=0,48,1,1");
    if (modem.waitResponse(10000L) != 1)
    {
        DBG("Set GPS Power HIGH Failed");
    }
    modem.enableGPS();
}

void disableGPS(void)
{
    // Set Modem GPS Power Control Pin to LOW ,turn off GPS power
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+CGPIO=0,48,1,0");
    if (modem.waitResponse(10000L) != 1)
    {
        DBG("Set GPS Power LOW Failed");
    }
    modem.disableGPS();
}