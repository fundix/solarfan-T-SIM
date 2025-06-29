#include "globals.h"
#include "credentials.h"

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SPI.h>
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_MODEM_SIM7000

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

const char *topicStatus = "solarfan/status";
const char *topicControl = "solarfan/control";

int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
    SerialMon.print("Message arrived [");
    SerialMon.print(topic);
    SerialMon.print("]: ");
    SerialMon.write(payload, len);
    SerialMon.println();

    if (String(topic) == topicControl)
    {
        ledStatus = !ledStatus;
        digitalWrite(LED_PIN, ledStatus);
        mqtt.publish(topicStatus, ledStatus ? "1" : "0");
    }
}

bool mqttConnect()
{
    SerialMon.print("Connecting to MQTT... ");
    bool status = mqtt.connect("solarfan-client", MQTT_USER, MQTT_PASS);

    if (!status)
    {
        SerialMon.println("MQTT connection failed");
        return false;
    }

    SerialMon.println("MQTT connected");
    mqtt.publish(topicStatus, "solarfan online");
    mqtt.subscribe(topicControl);
    return true;
}

void modemPowerOn()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(1000);
    digitalWrite(PWR_PIN, HIGH);
}

void reportError(const char *message)
{
    SerialMon.println(message);
    digitalWrite(LED_PIN, LOW); // zapni LED jako indikátor chyby
}

void setup()
{
    SerialMon.begin(115200);
    delay(10);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // LED off

    modemPowerOn();
    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    delay(5000);
    SerialMon.println("Initializing modem...");
    if (!modem.restart())
    {
        reportError("Modem restart failed");
        return;
    }

    if (GSM_PIN && modem.getSimStatus() != 3)
    {
        modem.simUnlock(GSM_PIN);
    }

    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork(30000L))
    {
        reportError("Network connection failed");
        return;
    }
    SerialMon.println("Network connected");

    SerialMon.print("Connecting to GPRS...");
    if (!modem.gprsConnect(apn, gprsUser, gprsPass))
    {
        reportError("GPRS connection failed");
        return;
    }
    SerialMon.println("GPRS connected");

    mqtt.setServer(broker, mqttPort);
    mqtt.setCallback(mqttCallback);
}
void loop()
{
    if (!modem.isNetworkConnected())
    {
        SerialMon.println("Network disconnected");
        if (modem.waitForNetwork(10000L))
        {
            SerialMon.println("Network reconnected");
        }
        else
        {
            reportError("Network reconnection failed");
        }
    }

    if (!modem.isGprsConnected())
    {
        SerialMon.println("GPRS disconnected");
        if (modem.gprsConnect(apn, gprsUser, gprsPass))
        {
            SerialMon.println("GPRS reconnected");
        }
        else
        {
            reportError("GPRS reconnection failed");
        }
    }

    if (!mqtt.connected())
    {
        SerialMon.println("MQTT not connected");
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L)
        {
            lastReconnectAttempt = t;
            SerialMon.print("Connecting to MQTT... ");
            if (mqttConnect())
            {
                SerialMon.println("MQTT reconnected");
                lastReconnectAttempt = 0;
            }
            else
            {
                reportError("MQTT connection failed");
            }
        }
    }
    else
    {
        mqtt.loop();
    }

    delay(100);
}