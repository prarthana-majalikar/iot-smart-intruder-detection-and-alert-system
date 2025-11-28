/***************************************************
 * Smart Intruder Detection System
 * ESP32 + SparkFun LSM6DSO (SEN-18020, Qwiic)
 * + PHOTO SENSOR + LED + Buzzer + Blynk IoT
 ***************************************************/

// ---- BLYNK TEMPLATE INFO (from Blynk web) ----
#define BLYNK_TEMPLATE_ID "TMPL2Aga6AILl"
#define BLYNK_TEMPLATE_NAME "Smart Intruder Detection and Alert System"
#define BLYNK_AUTH_TOKEN "TFtfgkt7bC9tOYLHNXeBTEyPEcQtleN2"

#include <BlynkSimpleEsp32.h>
// ----------------------------------------------

#include <WiFi.h>
#include <Wire.h>
#include <SparkFunLSM6DSO.h>
#include <Adafruit_Sensor.h>

// ---- USER WIFI + AUTH ----
#include "nvs.h" // ESP NVS (non-volatile storage) APIs
#include "nvs_flash.h"

// ------------ USER CONFIG--------------
char ssid[50];
char pass[50];
// ---------------------------

// Pin assignments (match wiring!)
#define LED_PIN 2     // LED + resistor
#define LIGHT_PIN 34  // Junction node of 10k + TEMT6000
#define BUZZER_PIN 25 // Buzzer +
#define SDA_PIN 21    // I2C SDA to LSM6DSO
#define SCL_PIN 22    // I2C SCL to LSM6DSO

// Blynk Virtual Pins
#define VPIN_ALERT V0  // String alerts
#define VPIN_STATUS V1 // 0 = offline, 1 = online

LSM6DSO imu;

// Motion thresholds in g (SparkFun LSM6DSO returns g units)
float ACCEL_LOW = 0.05;  // <0.05 g: tiny noise / pet
float ACCEL_MED = 0.15;  // 0.05–0.15 g: medium motion
float ACCEL_HIGH = 0.30; // >0.15–0.30 g: strong motion

// Light detection thresholds
int LIGHT_BASE = 0;    // measured at startup
int LIGHT_DELTA = 400; // significant change

// Online/offline + event buffering
bool online = false;
String eventBuffer[20];
int bufferCount = 0;

// Periodic status update to Blynk (even in standby)
const unsigned long STATUS_INTERVAL_MS = 1000; // 1 second
unsigned long lastStatusSend = 0;

// ---------------------- Helper Functions ------------------------

void read_wifi_from_nvs()
{
    // Initialize or repair NVS partition if required
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // Erase NVS and re-init if layout changed or no free pages
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t handle;
    Serial.print("Opening NVS… ");
    // Open the NVS namespace "storage" for read/write
    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        Serial.println("Failed!");
        return; // early return if cannot open NVS
    }
    Serial.println("Done");

    // Prepare lengths for nvs_get_str (it writes actual length back)
    size_t ssid_len = sizeof(ssid);
    size_t pass_len = sizeof(pass);

    // Read the stored strings into the buffers
    err = nvs_get_str(handle, "ssid", ssid, &ssid_len);
    err |= nvs_get_str(handle, "pass", pass, &pass_len);

    if (err == ESP_OK)
    {
        Serial.println("Retrieved SSID & PASS from NVS");
    }
    else
    {
        Serial.println("Failed to read WiFi credentials!");
    }
    nvs_close(handle);
}

void connect_to_wifi()
{
    Serial.printf("Connecting to %s\n", ssid);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\nWiFi connected.");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
}

void bufferEvent(const String &msg)
{
    if (bufferCount < 20)
    {
        eventBuffer[bufferCount++] = msg;
    }
}

void flushBuffer()
{
    for (int i = 0; i < bufferCount; i++)
    {
        Blynk.virtualWrite(VPIN_ALERT, "[OFFLINE] " + eventBuffer[i]);
        delay(100);
    }
    bufferCount = 0;
}

void sendAlert(const String &msg)
{
    Serial.println(msg);

    // Simple visual + audible alert: 3 quick beeps/blinks
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        tone(BUZZER_PIN, 1500, 150); // 1.5 kHz tone, 150 ms
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }

    if (online && Blynk.connected())
    {
        Blynk.virtualWrite(VPIN_ALERT, msg);
    }
    else
    {
        bufferEvent(msg);
    }
}

void reconnectCheck()
{
    // Check WiFi
    if (WiFi.status() != WL_CONNECTED)
    {
        online = false;
        Blynk.virtualWrite(VPIN_STATUS, 0);
        return;
    }

    // Check Blynk
    if (!Blynk.connected())
    {
        Blynk.connect(1000); // try connecting for 1 sec
    }

    if (Blynk.connected())
    {
        if (!online)
        {
            // We just came back online
            flushBuffer();
        }
        online = true;
        Blynk.virtualWrite(VPIN_STATUS, 1);
    }
    else
    {
        online = false;
        Blynk.virtualWrite(VPIN_STATUS, 0);
    }
}

// --------------------------- Setup ------------------------------

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // I2C to LSM6DSO
    Wire.begin(SDA_PIN, SCL_PIN);

    if (!imu.begin())
    { // uses I2C, default addr 0x6B
        Serial.println("Failed to find LSM6DSO chip");
        while (1)
        {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(300);
        }
    }

    // Load basic accel/gyro settings
    if (!imu.initialize(BASIC_SETTINGS))
    {
        Serial.println("Failed to initialize IMU settings");
    }

    // Calibrate light baseline
    long sum = 0;
    const int samples = 50;
    for (int i = 0; i < samples; i++)
    {
        sum += analogRead(LIGHT_PIN);
        delay(10);
    }
    LIGHT_BASE = sum / samples;
    Serial.print("Light Baseline: ");
    Serial.println(LIGHT_BASE);

    // Start WiFi
    read_wifi_from_nvs();
    connect_to_wifi();

    // Blynk config (non-blocking)
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect(5000); // try for 3 seconds
}

// ---------------------------- Loop ------------------------------

void loop()
{
    if (Blynk.connected())
    {
        Blynk.run();
    }

    reconnectCheck();

    // ---- Read IMU ----
    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();

    // Total acceleration magnitude and delta from 1g
    float mag = sqrt(ax * ax + ay * ay + az * az); // in g
    float delta = fabs(mag - 1.0f);                // 1.0 g baseline
    // ---- Read light sensor ----
    int light = analogRead(LIGHT_PIN);
    int Ldelta = abs(light - LIGHT_BASE);
    bool lightChange = (Ldelta > LIGHT_DELTA);

    // ---- Classify motion ----
    bool pet = false;
    bool intruder = false;

    Serial.print("mag=");
    Serial.print(mag, 3);
    Serial.print("  delta=");
    Serial.print(delta, 3);
    Serial.print("  light=");
    Serial.print(light);
    Serial.print("  dL=");
    Serial.println(Ldelta);

    if (delta < ACCEL_LOW)
    {
        // Tiny motion -> ignore (pet/noise)
        pet = true;
    }
    else if (delta < ACCEL_MED)
    {
        // Medium motion -> might be pet or human
        pet = true;
        if (lightChange)
        {
            // Medium motion + big light change = likely human
            intruder = true;
        }
    }
    else if (delta < ACCEL_HIGH)
    {
        // Strong motion
        intruder = true;
    }
    else
    {
        // Very strong motion
        intruder = true;
    }

    // ---- Periodic status updates in standby ----
    if (!intruder && online && Blynk.connected())
    {
        unsigned long now = millis();
        if (now - lastStatusSend >= STATUS_INTERVAL_MS)
        {
            lastStatusSend = now;

            String statusMsg = "Standby | dAccel=" + String(delta, 2) +
                               " | light=" + String(light) +
                               " | dLight=" + String(Ldelta);

            // Reuse the same string widget (V0) so it always shows "last known state"
            Blynk.virtualWrite(VPIN_ALERT, statusMsg);
        }
    }

    if (intruder)
    {
        String msg = "INTRUSION DETECTED | dAccel=" + String(delta, 2) + " | light=" + String(light) + " | dLight=" + String(Ldelta);
        sendAlert(msg);
    }

    delay(50); // ~20 Hz loop
}