// **********************************
// * Project 9
// * Connect a Red LED on MCU (You decide the pin order and declare it in platformio.ini)
// **********************************
// * Import Libraries
#include <Arduino.h>          // Includes the core Arduino functions for the ESP32
#include <WiFi.h>             // Includes the WiFi library to connect to WiFi networks
#include <ESP32Ping.h>        // Includes the ESP32 Ping library for ping functionality
#include <Wire.h>             // Includes the Wire library for I2C communication
#include <Adafruit_GFX.h>     // Includes the Adafruit GFX library for graphics functions
#include <Adafruit_SSD1306.h> // Includes the Adafruit SSD1306 library for the OLED display
#include "music.h"            // Includes the music header file

// * Constants and Variable Declaration
const char *ssid = "Brown-Guest";                       // ! The name of the WiFi network (SSID), change it to a fake to test the alarm
const char *password = "";                              // ! The password for the WiFi network, Brown-Guest SSID does not have a password
const char *host = PING_HOST;                           // ! Host to ping www.google.com which is defined in platformio.ini
const char *ntpServer = NTP_SERVER;                     // ! Fetch current data and time from a NTP server which is defined in platformio.ini
constexpr unsigned long NTP_SYNC_DELAY_MS = 1000;       // ! Delay between NTP time sync attempts
constexpr int maxRetries = 3;                           // ! Maximum number of retries for WiFi, Ping, and NTP
constexpr unsigned long RESTART_DELAY_MS = 3000;        // ! Delay before restarting the ESP32 in milliseconds
constexpr unsigned long WIFI_RETRY_DELAY_MS = 500;      // ! Delay between WiFi connection attempts
constexpr unsigned long WIFI_RETRY_WAIT_MS = 1000;      // ! Wait time before next WiFi connection attempt
constexpr unsigned long PING_RETRY_DELAY_MS = 500;      // ! Delay between Ping attempts
constexpr unsigned long OLED_INIT_DELAY_MS = 2000;      // ! Delay to ensure OLED starts correctly
constexpr unsigned long LOOP_DELAY_MS = 30000;          // ! Delay for the main loop
constexpr unsigned long ALARM_HIGH_FREQUENCY = 2000;    // ! High frequency for the alarm
constexpr unsigned long ALARM_LOW_FREQUENCY = 500;      // ! Low frequency for the alarm
constexpr unsigned long ALARM_TONE_DURATION_MS = 100;   // ! Duration of each alarm tone
constexpr unsigned long ALARM_TOTAL_DURATION_MS = 3000; // ! Total duration of the alarm

const int greenLedChannel = 0; // PWM channel for Green LED
const int redLedChannel = 1;   // PWM channel for Red LED
const int buzzerChannel = 2;   // PWM channel for Buzzer
const int pwmFrequency = 5000; // Frequency for PWM
const int pwmResolution = 8;   // 8-bit resolution (0-255)

#define SCREEN_WIDTH 128                                                  // OLED display width, in pixels
#define SCREEN_HEIGHT 64                                                  // OLED display height, in pixels
#define OLED_RESET -1                                                     // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Initialize the OLED display object

// * Function Declarations
bool connectToWiFi();       // Connects the ESP32 to the specified WiFi network
bool pingHost();            // Pings the specified host to check network connectivity
bool syncNTP();             // Synchronizes the ESP32's time with an NTP (Network Time Protocol) server
void initPWM();            // Sets up PWM (Pulse Width Modulation) channels for controlling LEDs and the buzzer
void buzzerAndBlinkAlarm(); // Activates the buzzer and blinks the red LED to signal an alarm
void initOLED();            // Function to initialize OLED display

// * setup() function
void setup()
{
    // Initialize serial communication
    Serial.begin(115200); // Sets up serial communication at a baud rate of 115200 bits per second
    delay(100);           // Small delay to ensure the serial monitor is ready

    // Initialize PWM for LED and Buzzer
    initPWM(); // Calls the function to set up PWM channels

    // Initialize OLED display with SCL and SDA pins
    initOLED(); // Initialize OLED display

    // Connect to WiFi
    if (!connectToWiFi())
    {
        Serial.println("Failed to connect to WiFi: " + String(ssid));
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Failed to connect to WiFi: " + String(ssid));
        display.print("ESP32 is rebooting in ");
        display.print(RESTART_DELAY_MS / 1000);
        display.println(" seconds.");
        display.display();
        ledcWrite(redLedChannel, 255); // Turn on Red LED
        delay(RESTART_DELAY_MS);       // Wait before restarting
        ESP.restart();                 // Reboot the MCU
    }

    // Print the current WiFi SSID, IP address, and RSSI value
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("SSID: ");
    display.println(WiFi.SSID());
    display.print("IP: ");
    display.println(WiFi.localIP());
    display.print("RSSI: ");
    display.println(WiFi.RSSI());
    display.display();

    // Ping Google's DNS server
    if (!pingHost())
    {
        Serial.println("Failed to ping host: " + String(host));
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Failed to ping host: " + String(host));
        display.print("ESP32 is rebooting in ");
        display.print(RESTART_DELAY_MS / 1000);
        display.println(" seconds.");
        display.display();
        ledcWrite(redLedChannel, 255); // Turn on Red LED
        delay(RESTART_DELAY_MS);       // Wait before restarting
        ESP.restart();                 // Reboot the MCU
    }

    // Synchronize NTP time
    if (!syncNTP())
    {
        Serial.println("Failed to sync NTP: " + String(ntpServer));
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Failed to sync NTP: " + String(ntpServer));
        display.print("ESP32 is rebooting in ");
        display.print(RESTART_DELAY_MS / 1000);
        display.println(" seconds.");
        display.display();
        ledcWrite(redLedChannel, 255); // Turn on Red LED
        delay(RESTART_DELAY_MS);       // Wait before restarting
        ESP.restart();                 // Reboot the MCU
    }

    // Play music when NTP sync is successful
    playMusic(buzzerChannel);
}

// * loop() function
void loop()
{
    // Check if WiFi is connected
    if (WiFi.status() == WL_CONNECTED)
    {
        ledcWrite(greenLedChannel, 255); // Set Green LED brightness to maximum
        ledcWrite(redLedChannel, 0);     // Turn off Red LED
        Serial.println("WiFi is connected.");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("WiFi is connected.");
        display.display();
    }
    else
    {
        Serial.println("WiFi connection lost. Attempting to reconnect...");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("WiFi connection lost.");
        display.println("Attempting to reconnect...");
        display.display();
        ledcWrite(greenLedChannel, 0); // Turn off Green LED
        ledcWrite(redLedChannel, 255); // Turn on Red LED
        int retryCount = 0;            // Initialize retry count
        bool reconnected = false;      // Initialize reconnection status

        // Try to reconnect to WiFi
        while (retryCount < maxRetries && !reconnected)
        {
            retryCount++; // Increment retry count
            Serial.print("Retrying ");
            Serial.print(retryCount);
            Serial.print("/");
            Serial.println(maxRetries);
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("Retrying ");
            display.print(retryCount);
            display.print("/");
            display.println(maxRetries);
            display.display();
            if (connectToWiFi())
            {
                reconnected = true;              // Set reconnection status to true
                ledcWrite(greenLedChannel, 255); // Set Green LED brightness to maximum
                ledcWrite(redLedChannel, 0);     // Turn off Red LED
                Serial.println("Reconnected to WiFi successfully!");
                display.clearDisplay();
                display.setCursor(0, 0);
                display.println("Reconnected to WiFi");
                display.println("successfully!");
                display.display();
            }
            else
            {
                delay(WIFI_RETRY_WAIT_MS); // Wait before next attempt
            }
        }

        if (!reconnected)
        {
            Serial.println("Failed to reconnect to WiFi after 3 attempts.");
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Failed to reconnect to WiFi.");
            display.print("ESP32 is rebooting in ");
            display.print(RESTART_DELAY_MS / 1000);
            display.println(" seconds.");
            display.display();
            ledcWrite(redLedChannel, 255); // Turn on Red LED
            delay(RESTART_DELAY_MS);       // Wait before restarting
            ESP.restart();                 // Reboot the MCU
        }
    }

    // Display default information
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("SSID: ");
    display.println(WiFi.SSID());
    display.print("IP: ");
    display.println(WiFi.localIP());
    display.print("RSSI: ");
    display.println(WiFi.RSSI());

    // Print Date and Time
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
        char timeStr[64];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
        display.println(timeStr);
    }
    else
    {
        display.println("Time: N/A");
    }

    // Print LED status
    display.print("Green LED: ");
    display.println(ledcRead(greenLedChannel) ? "On" : "Off");
    display.print("Red LED: ");
    display.println(ledcRead(redLedChannel) ? "On" : "Off");

    display.display();

    delay(LOOP_DELAY_MS); // Delay before next loop iteration
}

// * Functions Definition

// ! Function to connect to WiFi
bool connectToWiFi()
{
    Serial.println("Connecting to WiFi: " + String(ssid));
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting to WiFi: " + String(ssid));
    display.display();

    int attempt = 0; // Initialize attempt count
    while (attempt < maxRetries)
    {
        attempt++;                  // Increment attempt count
        WiFi.begin(ssid, password); // Start WiFi connection with given SSID and password
        int retries = 0;            // Initialize retries count
        while (WiFi.status() != WL_CONNECTED && retries < maxRetries)
        {                               // Check if WiFi is not connected
            delay(WIFI_RETRY_DELAY_MS); // Wait before next check
            Serial.print(".");
            display.print(".");
            display.display();
            retries++; // Increment retries count
        }

        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.println(""); // Print new line for readability
            Serial.println("WiFi connected.");
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("WiFi connected.");
            display.display();
            return true; // Return true if connected
        }
        else
        {
            Serial.print("WiFi connection attempt failed. Retrying ");
            Serial.print(attempt);
            Serial.print("/");
            Serial.println(maxRetries);
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("WiFi connection attempt failed. Retrying ");
            display.print(attempt);
            display.print("/");
            display.println(maxRetries);
            display.display();
            WiFi.disconnect();         // Disconnect from WiFi
            delay(WIFI_RETRY_WAIT_MS); // Wait before next attempt
        }
    }

    return false; // If all attempts fail, return false
}

// ! Function to ping host
bool pingHost()
{
    Serial.print("Pinging host: " + String(host));
    Serial.println("...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Pinging host: " + String(host));
    display.println("...");
    display.display();

    int retries = 0;
    while (!Ping.ping(host) && retries < maxRetries)
    {
        retries++; // Increment retries count
        Serial.print("Ping failed, retrying ");
        Serial.print(retries);
        Serial.print("/");
        Serial.println(maxRetries);
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Ping failed, retrying ");
        display.print(retries);
        display.print("/");
        display.println(maxRetries);
        display.display();
    }
    if (Ping.ping(host))
    {
        Serial.println("Ping successful.");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Ping successful.");
        display.display();
        return true;
    }
    else
    {
        return false;
    }
}

// ! Function to synchronize NTP time
bool syncNTP()
{
    Serial.println("Synchronizing NTP: " + String(ntpServer));
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Synchronizing NTP: " + String(ntpServer));
    display.display();

    String ntpMessage = "Fetching date and time from NTP server: " + String(ntpServer);
    Serial.println(ntpMessage);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(ntpMessage);
    display.display();

    struct timeval tv = {0};     // Clear system time
    settimeofday(&tv, NULL);     // Set system time to zero
    configTime(0, 0, ntpServer); // Configure time with NTP server
    struct tm timeinfo;          // Structure to hold time information
    int retries = 0;             // Initialize retries count
    while (!getLocalTime(&timeinfo) && retries < maxRetries)
    {
        retries++; // Increment retries count
        Serial.print("Waiting for NTP time sync, retrying ");
        Serial.print(retries);
        Serial.print("/");
        Serial.println(maxRetries);
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Waiting for NTP time sync, retrying ");
        display.print(retries);
        display.print("/");
        display.println(maxRetries);
        display.display();
        delay(NTP_SYNC_DELAY_MS); // Wait before next attempt
    }
    if (timeinfo.tm_year > (2020 - 1900))
    {                                                                       // Check if the year is valid
        char timeStr[64];                                                   // Buffer to hold formatted time string
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo); // Format time string
        Serial.println(timeStr);                                            // Print formatted time to serial
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(timeStr);
        display.display();
        return true; // Return true if NTP sync was successful
    }
    else
    {
        return false;
    }
}

// ! Function to initialize OLED display
void initOLED()
{
    Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C communication with given SDA and SCL pins
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { // Address 0x3C for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    delay(OLED_INIT_DELAY_MS); // Small delay to ensure OLED starts correctly
    display.clearDisplay();
    display.setTextSize(1);              // Set text size
    display.setTextColor(SSD1306_WHITE); // Set text color
    display.setCursor(0, 0);             // Set cursor position
    display.println(F("Initializing..."));
    display.display();
}

// ! Function to configure PWM functionalities
void initPWM()
{
    // Configure LED PWM functionalities
    ledcSetup(greenLedChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(GREEN_LED_PIN, greenLedChannel);

    ledcSetup(redLedChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(RED_LED_PIN, redLedChannel);

    // Configure Buzzer PWM functionalities
    ledcSetup(buzzerChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(BUZZER_PIN, buzzerChannel);
}

// ! Function to trigger alarm
void buzzerAndBlinkAlarm()
{
    unsigned long startTime = millis(); // Record the start time

    while (millis() - startTime < ALARM_TOTAL_DURATION_MS)
    {
        // Play high frequency and turn on Red LED
        ledcWriteTone(buzzerChannel, ALARM_HIGH_FREQUENCY);
        ledcWrite(redLedChannel, 255); // Turn on Red LED
        delay(ALARM_TONE_DURATION_MS);

        // Play low frequency and turn off Red LED
        ledcWriteTone(buzzerChannel, ALARM_LOW_FREQUENCY);
        ledcWrite(redLedChannel, 0); // Turn off Red LED
        delay(ALARM_TONE_DURATION_MS);
    }

    // Turn off the Buzzer
    ledcWriteTone(buzzerChannel, 0);
}
