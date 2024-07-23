#include <Arduino.h>           // Core library for Arduino programming
#include <Wire.h>              // Library for I2C communication
#include <Adafruit_GFX.h>      // Graphics library for OLED display
#include <Adafruit_SSD1306.h>  // Driver library for OLED display
#include <DHT.h>               // Library for DHT sensor
#include <WiFi.h>              // Library for WiFi functionality
#include <ESPAsyncWebServer.h> // Library for running asynchronous web server
#include <ESP32Ping.h>         // Library for pinging a host
#include <HTTPClient.h>        // Library for HTTP client functionality
#include <ArduinoJson.h>       // Library for parsing JSON data
#include <time.h>              // Library for time-related functions
#include "music.h"             // Custom library for playing music
#include "webPage.h"           // Custom library for web page content
#include <WiFiClientSecure.h>  // Library for secure WiFi client
#include "SecureCredentials.h" // Header file containing secure credentials
#include <PubSubClient.h>      // Library for MQTT client functionality

// Constants and Variable Declarations
constexpr int SCREEN_WIDTH = 128;      // OLED display width, in pixels
constexpr int SCREEN_HEIGHT = 64;      // OLED display height, in pixels
constexpr int OLED_RESET = -1;         // Reset pin for OLED display, -1 if not used
constexpr int OLED_I2C_ADDRESS = 0x3C; // I2C address for OLED display

const int greenLedChannel = 0;  // PWM channel for green LED
const int redLedChannel = 1;    // PWM channel for red LED
const int buzzerChannel = 2;    // PWM channel for buzzer
const int pwmFrequency = 5000;  // PWM frequency for LEDs and buzzer
const int pwmResolution = 8;    // PWM resolution
const int buzzerDutyCycle = 64; // Duty cycle for buzzer PWM

constexpr float TEMP_HIGH_THRESHOLD_F = 90.0;   // High temperature threshold in Fahrenheit
constexpr float TEMP_LOW_THRESHOLD_F = 60.0;    // Low temperature threshold in Fahrenheit
constexpr float HUMIDITY_HIGH_THRESHOLD = 75.0; // High humidity threshold
constexpr float HUMIDITY_LOW_THRESHOLD = 15.0;  // Low humidity threshold

constexpr unsigned long ALARM_HIGH_FREQUENCY = 2000;    // High frequency for alarm tone
constexpr unsigned long ALARM_LOW_FREQUENCY = 500;      // Low frequency for alarm tone
constexpr unsigned long ALARM_TONE_DURATION_MS = 100;   // Duration of each alarm tone in milliseconds
constexpr unsigned long ALARM_TOTAL_DURATION_MS = 3000; // Total duration of alarm in milliseconds

constexpr int REBOOT_DELAY_SECONDS = 10;    // Delay before rebooting in seconds
constexpr int DISPLAY_INIT_DELAY_MS = 2000; // Delay for initializing display in milliseconds
constexpr int RETRY_DELAY_MS = 1000;        // Delay between retries in milliseconds
unsigned long lastUpdate = 0;               // Timestamp of the last update
const unsigned long updateInterval = 10000; // Interval between updates in milliseconds

constexpr int MAX_RETRIES = 3;                     // Maximum number of retries for certain operations
constexpr unsigned long PING_RETRY_DELAY_MS = 500; // Delay between ping retries in milliseconds

const char *ssid = WIFI_SSID;         // WiFi SSID (network name)
const char *password = WIFI_PASSWORD; // WiFi password
const char *hostName = PING_HOST;     // Hostname to ping
const char *ntpServer = NTP_SERVER;   // NTP server for time synchronization

// Weather API related constants
const char *weatherApiKey = WEATHER_API_KEY; // API key for weather data
const char *city = CITY;                     // City for weather data
const char *city2 = CITY2;                   // Second city for weather data

unsigned long lastWeatherUpdate = 0;                            // Timestamp of the last weather update
unsigned long nextWeatherUpdate = 0;                            // Timestamp of the next weather update
String weatherDescription, weatherDescription2;                 // Weather descriptions for two cities
float weatherTempC, weatherTempF, weatherTempC2, weatherTempF2; // Weather temperatures for two cities

// DHT sensor related constants
constexpr int DHT_TYPE = DHT11;             // Type of DHT sensor
DHT dht(DHT_PIN, DHT_TYPE);                 // DHT sensor object
float temperatureC, temperatureF, humidity; // Variables to store sensor data

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // OLED display object

AsyncWebServer server(80); // Web server object

bool greenLedState = false; // State of the green LED
bool redLedState = false;   // State of the red LED

WiFiClientSecure net;                                   // Secure WiFi client object
PubSubClient mqttClient(net);                           // MQTT client object
String deviceID;                                        // Device ID
String AWS_IOT_PUBLISH_TOPIC;                           // Topic to publish messages to
constexpr unsigned long MQTT_RECONNECT_DELAY_MS = 3000; // Delay between MQTT reconnection attempts
bool awsConnected = false;                              // Flag to indicate AWS connection status

// Function prototypes
void initPWM();                                                                                                                             // Function to initialize PWM
bool connectToWiFi();                                                                                                                       // Function to connect to WiFi
bool pingHost();                                                                                                                            // Function to ping a host
bool syncNTP();                                                                                                                             // Function to synchronize time with NTP server
void readDHT(float &temperatureC, float &temperatureF, float &humidity);                                                                    // Function to read data from DHT sensor
bool isMotionOn();                                                                                                                          // Function to check motion sensor status
bool isTiltOn();                                                                                                                            // Function to check tilt sensor status
bool isFlameOn();                                                                                                                           // Function to check flame sensor status
void triggerAlarm();                                                                                                                        // Function to trigger alarm
void showDisplay(float temperatureC, float temperatureF, float humidity, bool motionStatus, bool tiltStatus, bool flameStatus, bool alarm); // Function to show data on OLED display
void showSerial(float temperatureC, float temperatureF, float humidity, bool motionStatus, bool tiltStatus, bool flameStatus, bool alarm);  // Function to show data on serial monitor
bool validateServer();                                                                                                                      // Function to validate server
void handleRootRequest(AsyncWebServerRequest *request);                                                                                     // Function to handle root request
void handleDataRequest(AsyncWebServerRequest *request);                                                                                     // Function to handle data request
void handleGreenLedRequest(AsyncWebServerRequest *request);                                                                                 // Function to handle green LED request
void handleRedLedRequest(AsyncWebServerRequest *request);                                                                                   // Function to handle red LED request
void handleTimeRequest(AsyncWebServerRequest *request);                                                                                     // Function to handle time request
void fetchWeatherData(const char *city, String &weatherDescription, float &weatherTempC, float &weatherTempF);                              // Function to fetch weather data
bool connectAWS();                                                                                                                          // Function to connect to AWS
void mqttPublishMessage(float humidity, float temperature_c, float temperature_f, bool motionStatus, bool tiltStatus, bool flameStatus);

void setup()
{
    Serial.begin(115200);                              // Initialize serial communication at 115200 baud rate
    Serial.println("*******************************"); // Print a separator for clarity

    Wire.begin(SDA_PIN, SCL_PIN);                               // Initialize I2C communication with specified SDA and SCL pins
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) // Initialize the OLED display with the I2C address
    {
        Serial.println(F("SSD1306 allocation failed")); // Print error message if OLED initialization fails
        for (;;)
            ; // Infinite loop to halt execution if display initialization fails
    }
    display.display();            // Display initial buffer contents
    delay(DISPLAY_INIT_DELAY_MS); // Delay to let the display initialize
    display.clearDisplay();       // Clear the display buffer

    display.setCursor(0, 0);             // Set cursor to the top-left corner
    display.setTextSize(1);              // Set text size to 1
    display.setTextColor(SSD1306_WHITE); // Set text color to white
    display.println("Initializing...");  // Print "Initializing..." message
    display.display();                   // Update the display with the initialization message

    initPWM(); // Initialize PWM for LEDs and buzzer

    dht.begin(); // Initialize the DHT sensor

    deviceID = String(ESP.getEfuseMac(), HEX); // Get the device ID from ESP32's MAC address
    Serial.println("Device ID: " + deviceID);  // Print the device ID to the serial monitor
    display.println("Device ID: " + deviceID); // Display the device ID on the OLED
    display.display();                         // Update the display with the device ID

    // Define the MQTT topics
    AWS_IOT_PUBLISH_TOPIC = deviceID + "/pub"; // Publish topic for MQTT messages

    if (!connectToWiFi()) // Attempt to connect to WiFi
    {
        Serial.println("Failed to connect to Wi-Fi!");  // Print failure message to serial monitor
        display.clearDisplay();                         // Clear the display buffer
        display.setCursor(0, 0);                        // Set cursor to the top-left corner
        display.println("Failed to connect to Wi-Fi!"); // Display failure message on the OLED
        display.println("Rebooting in 10 seconds...");  // Display reboot message on the OLED
        display.display();                              // Update the display with the failure message
        ledcWrite(redLedChannel, 255);                  // Turn on the red LED to indicate failure
        delay(REBOOT_DELAY_SECONDS * 1000);             // Wait for 10 seconds
        ESP.restart();                                  // Restart the ESP32
    }

    display.clearDisplay();             // Clear the display buffer
    display.setCursor(0, 0);            // Set cursor to the top-left corner
    display.println("Wi-Fi Connected"); // Display Wi-Fi connected message
    display.print("SSID: ");            // Display SSID label
    display.println(ssid);              // Display the SSID
    display.print("IP: ");              // Display IP address label
    display.println(WiFi.localIP());    // Display the local IP address
    display.display();                  // Update the display with Wi-Fi connection details
    delay(DISPLAY_INIT_DELAY_MS);       // Delay to let the user read the display

    if (!pingHost()) // Attempt to ping the host
    {
        Serial.println("Failed to ping host!");        // Print failure message to serial monitor
        display.clearDisplay();                        // Clear the display buffer
        display.setCursor(0, 0);                       // Set cursor to the top-left corner
        display.println("Failed to ping host!");       // Display failure message on the OLED
        display.println("Rebooting in 10 seconds..."); // Display reboot message on the OLED
        display.display();                             // Update the display with the failure message
        ledcWrite(redLedChannel, 255);                 // Turn on the red LED to indicate failure
        delay(REBOOT_DELAY_SECONDS * 1000);            // Wait for 10 seconds
        ESP.restart();                                 // Restart the ESP32
    }

    if (!syncNTP()) // Attempt to synchronize time with NTP server
    {
        Serial.println("Failed to sync NTP!");         // Print failure message to serial monitor
        display.clearDisplay();                        // Clear the display buffer
        display.setCursor(0, 0);                       // Set cursor to the top-left corner
        display.println("Failed to sync NTP!");        // Display failure message on the OLED
        display.println("Rebooting in 10 seconds..."); // Display reboot message on the OLED
        display.display();                             // Update the display with the failure message
        ledcWrite(redLedChannel, 255);                 // Turn on the red LED to indicate failure
        delay(REBOOT_DELAY_SECONDS * 1000);            // Wait for 10 seconds
        ESP.restart();                                 // Restart the ESP32
    }

    server.on("/", HTTP_GET, handleRootRequest);             // Define route for root URL
    server.on("/data", HTTP_GET, handleDataRequest);         // Define route for data request
    server.on("/greenLed", HTTP_GET, handleGreenLedRequest); // Define route for green LED control
    server.on("/redLed", HTTP_GET, handleRedLedRequest);     // Define route for red LED control
    server.on("/time", HTTP_GET, handleTimeRequest);         // Define route for time request
    server.begin();                                          // Start the web server

    if (validateServer()) // Validate if the server started successfully
    {
        Serial.println("Web server started successfully."); // Print success message to serial monitor
        display.clearDisplay();                             // Clear the display buffer
        display.setCursor(0, 0);                            // Set cursor to the top-left corner
        display.println("Web server started");              // Display server started message on the OLED
        display.display();                                  // Update the display with the success message
    }
    else
    {
        Serial.println("Web server failed to start."); // Print failure message to serial monitor
        display.clearDisplay();                        // Clear the display buffer
        display.setCursor(0, 0);                       // Set cursor to the top-left corner
        display.println("Web server failed");          // Display server failure message on the OLED
        display.display();                             // Update the display with the failure message
    }

    fetchWeatherData(city, weatherDescription, weatherTempC, weatherTempF);     // Fetch initial weather data for the first city
    fetchWeatherData(city2, weatherDescription2, weatherTempC2, weatherTempF2); // Fetch initial weather data for the second city

    if (!awsConnected) // Check if AWS is connected
    {
        if (connectAWS()) // Attempt to connect to AWS
        {
            awsConnected = true; // Set AWS connection flag to true if successful
        }
        else
        {
            Serial.println("Failed to connect to AWS Cloud: " + String(AWS_IOT_MQTT_SERVER)); // Print failure message to serial monitor
            ledcWrite(redLedChannel, 255);                                                    // Turn on the red LED to indicate failure
            delay(REBOOT_DELAY_SECONDS * 1000);                                               // Wait for 10 seconds
            ESP.restart();                                                                    // Restart the ESP32
        }
    }

    display.clearDisplay();            // Clear the display buffer
    display.setCursor(0, 0);           // Set cursor to the top-left corner
    display.println("Initialization"); // Display initialization complete message
    display.println("completed");
    display.display();            // Update the display with the completion message
    delay(DISPLAY_INIT_DELAY_MS); // Delay to let the user read the display

    playMusic(buzzerChannel); // Play a sound to indicate initialization is complete
}

void loop()
{
    unsigned long currentTime = millis(); // Get the current time in milliseconds

    unsigned long nextWeatherUpdate = lastWeatherUpdate + (60 * 60 * 1000); // Calculate the next weather update time (1 hour later)

    if (currentTime - lastUpdate >= updateInterval) // Check if it's time for the next update
    {
        Serial.println("*******************************"); // Print a separator for clarity

        if (WiFi.status() != WL_CONNECTED) // Check if WiFi is still connected
        {
            for (int i = 0; i < 3; i++)
            {
                if (connectToWiFi())
                    break;             // Exit loop if connection is successful
                delay(RETRY_DELAY_MS); // Delay between retries
            }

            if (WiFi.status() != WL_CONNECTED) // If still not connected
            {
                Serial.println("Failed to reconnect to Wi-Fi!");  // Print failure message
                display.clearDisplay();                           // Clear the display buffer
                display.setCursor(0, 0);                          // Set cursor to the top-left corner
                display.println("Failed to reconnect to Wi-Fi!"); // Display failure message on the OLED
                display.println("Rebooting in 10 seconds...");    // Display reboot message on the OLED
                display.display();                                // Update the display with the failure message
                ledcWrite(redLedChannel, 255);                    // Turn on the red LED to indicate failure
                delay(REBOOT_DELAY_SECONDS * 1000);               // Wait for 10 seconds
                ESP.restart();                                    // Restart the ESP32
            }
        }

        for (int i = 0; i < 3; i++)
        {
            readDHT(temperatureC, temperatureF, humidity);                        // Read data from the DHT sensor
            if (!isnan(temperatureC) && !isnan(temperatureF) && !isnan(humidity)) // Check if the readings are valid
            {
                break; // Exit loop if valid readings are obtained
            }
            delay(RETRY_DELAY_MS); // Delay between retries
        }

        if (isnan(temperatureC) || isnan(temperatureF) || isnan(humidity)) // If any reading is invalid
        {
            Serial.println("Failed to read from DHT sensor!");  // Print failure message
            display.clearDisplay();                             // Clear the display buffer
            display.setCursor(0, 0);                            // Set cursor to the top-left corner
            display.println("Failed to read from DHT sensor!"); // Display failure message on the OLED
            display.println("Rebooting in 10 seconds...");      // Display reboot message on the OLED
            display.display();                                  // Update the display with the failure message
            ledcWrite(redLedChannel, 255);                      // Turn on the red LED to indicate failure
            delay(REBOOT_DELAY_SECONDS * 1000);                 // Wait for 10 seconds
            ESP.restart();                                      // Restart the ESP32
        }

        bool motionStatus = isMotionOn(); // Check motion sensor status
        bool tiltStatus = isTiltOn();     // Check tilt sensor status
        bool flameStatus = isFlameOn();   // Check flame sensor status
        bool alarm = false;               // Initialize alarm status to false

        if (temperatureF >= TEMP_LOW_THRESHOLD_F && temperatureF <= TEMP_HIGH_THRESHOLD_F &&
            humidity >= HUMIDITY_LOW_THRESHOLD && humidity <= HUMIDITY_HIGH_THRESHOLD &&
            !motionStatus && !tiltStatus && !flameStatus) // Check if conditions are normal
        {
            ledcWrite(greenLedChannel, 255); // Turn on the green LED
            ledcWrite(redLedChannel, 0);     // Turn off the red LED
            ledcWrite(buzzerChannel, 0);     // Turn off the buzzer
        }
        else
        {
            ledcWrite(greenLedChannel, 0); // Turn off the green LED
            triggerAlarm();                // Trigger the alarm
            alarm = true;                  // Set alarm status to true
        }

        Serial.println("Device ID: " + deviceID);                                                        // Print the device ID
        Serial.println("AWS Connection Status: " + String(awsConnected ? "Connected" : "Disconnected")); // Print AWS connection status
        display.println("Device ID: " + deviceID);                                                       // Display the device ID on the OLED

        showSerial(temperatureC, temperatureF, humidity, motionStatus, tiltStatus, flameStatus, alarm);  // Show sensor data on the serial monitor
        showDisplay(temperatureC, temperatureF, humidity, motionStatus, tiltStatus, flameStatus, alarm); // Show sensor data on the OLED display

        if (currentTime - lastWeatherUpdate >= 3600000) // If it's time to update the weather data
        {
            fetchWeatherData(city, weatherDescription, weatherTempC, weatherTempF);     // Fetch weather data for the first city
            fetchWeatherData(city2, weatherDescription2, weatherTempC2, weatherTempF2); // Fetch weather data for the second city
            lastWeatherUpdate = currentTime;                                            // Update the last weather update time
        }

        if (!mqttClient.connected()) // Check if the MQTT client is connected
        {
            awsConnected = false; // Set AWS connection flag to false
            if (!connectAWS())    // Attempt to reconnect to AWS
            {
                Serial.println("Rebooting in 10 seconds due to AWS connection failure..."); // Print failure message
                display.clearDisplay();                                                     // Clear the display buffer
                display.setCursor(0, 0);                                                    // Set cursor to the top-left corner
                display.println("AWS connection failed");                                   // Display AWS failure message on the OLED
                display.println("Rebooting in 10 seconds...");                              // Display reboot message on the OLED
                display.display();                                                          // Update the display with the failure message
                ledcWrite(redLedChannel, 255);                                              // Turn on the red LED to indicate failure
                delay(REBOOT_DELAY_SECONDS * 1000);                                         // Wait for 10 seconds
                ESP.restart();                                                              // Restart the ESP32
            }
            else
            {
                awsConnected = true; // Set AWS connection flag to true if successful
            }
        }

        // Publish sensor data to AWS IoT
        mqttPublishMessage(humidity, temperatureC, temperatureF, motionStatus, tiltStatus, flameStatus);

        lastUpdate = currentTime; // Update the last update time
    }
}

void initPWM()
{
    ledcSetup(redLedChannel, pwmFrequency, pwmResolution); // Initialize PWM for red LED
    ledcAttachPin(RED_LED_PIN, redLedChannel);             // Attach red LED to PWM channel

    ledcSetup(greenLedChannel, pwmFrequency, pwmResolution); // Initialize PWM for green LED
    ledcAttachPin(GREEN_LED_PIN, greenLedChannel);           // Attach green LED to PWM channel

    ledcSetup(buzzerChannel, pwmFrequency, pwmResolution); // Initialize PWM for buzzer
    ledcAttachPin(BUZZER_PIN, buzzerChannel);              // Attach buzzer to PWM channel
}

bool connectToWiFi()
{
    Serial.print("Connecting to Wi-Fi: ");  // Print connecting message
    Serial.println(ssid);                   // Print the SSID
    display.clearDisplay();                 // Clear the display buffer
    display.setCursor(0, 0);                // Set cursor to the top-left corner
    display.setTextSize(1);                 // Set text size to 1
    display.setTextColor(SSD1306_WHITE);    // Set text color to white
    display.print("Connecting to Wi-Fi: "); // Display connecting message
    display.println(ssid);                  // Display the SSID
    display.display();                      // Update the display with the connecting message

    WiFi.begin(ssid, password);                           // Begin WiFi connection with SSID and password
    int retries = 0;                                      // Initialize retry counter
    while (WiFi.status() != WL_CONNECTED && retries < 10) // Retry until connected or max retries reached
    {
        delay(RETRY_DELAY_MS); // Delay between retries
        Serial.print(".");     // Print dot for each retry
        display.print(".");    // Display dot for each retry
        display.display();     // Update the display with the dots
        retries++;             // Increment retry counter
    }

    if (WiFi.status() == WL_CONNECTED) // If connected to WiFi
    {
        Serial.println("\nWi-Fi connected."); // Print success message
        display.clearDisplay();               // Clear the display buffer
        display.setCursor(0, 0);              // Set cursor to the top-left corner
        display.println("Wi-Fi connected.");  // Display success message
        display.display();                    // Update the display with the success message
        return true;                          // Return true to indicate success
    }

    Serial.println("\nWi-Fi connection failed."); // Print failure message
    display.clearDisplay();                       // Clear the display buffer
    display.setCursor(0, 0);                      // Set cursor to the top-left corner
    display.println("Wi-Fi connection failed.");  // Display failure message
    display.display();                            // Update the display with the failure message
    return false;                                 // Return false to indicate failure
}

bool pingHost()
{
    Serial.print("Pinging host: ");      // Print pinging message
    Serial.println(hostName);            // Print the host name
    display.clearDisplay();              // Clear the display buffer
    display.setCursor(0, 0);             // Set cursor to the top-left corner
    display.setTextSize(1);              // Set text size to 1
    display.setTextColor(SSD1306_WHITE); // Set text color to white
    display.print("Pinging host: ");     // Display pinging message
    display.println(hostName);           // Display the host name
    display.display();                   // Update the display with the pinging message

    int retries = 0;                                      // Initialize retry counter
    while (!Ping.ping(hostName) && retries < MAX_RETRIES) // Retry until ping successful or max retries reached
    {
        delay(PING_RETRY_DELAY_MS); // Delay between retries
        Serial.print(".");          // Print dot for each retry
        display.print(".");         // Display dot for each retry
        display.display();          // Update the display with the dots
        retries++;                  // Increment retry counter
    }

    if (Ping.ping(hostName)) // If ping successful
    {
        Serial.println("\nPing successful."); // Print success message
        display.clearDisplay();               // Clear the display buffer
        display.setCursor(0, 0);              // Set cursor to the top-left corner
        display.print("Ping successful:\n");  // Display success message
        display.println(hostName);            // Display the host name
        display.display();                    // Update the display with the success message
        return true;                          // Return true to indicate success
    }

    Serial.println("\nPing failed.");   // Print failure message
    display.clearDisplay();             // Clear the display buffer
    display.setCursor(0, 0);            // Set cursor to the top-left corner
    display.print("Ping failed to:\n"); // Display failure message
    display.println(hostName);          // Display the host name
    display.display();                  // Update the display with the failure message
    return false;                       // Return false to indicate failure
}

bool syncNTP()
{
    Serial.print("Syncing NTP with server: "); // Print syncing message
    Serial.println(ntpServer);                 // Print the NTP server
    display.clearDisplay();                    // Clear the display buffer
    display.setCursor(0, 0);                   // Set cursor to the top-left corner
    display.setTextSize(1);                    // Set text size to 1
    display.setTextColor(SSD1306_WHITE);       // Set text color to white
    display.print("Syncing NTP with:\n");      // Display syncing message
    display.println(ntpServer);                // Display the NTP server
    display.display();                         // Update the display with the syncing message

    configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, ntpServer); // Configure time settings with NTP server
    struct tm timeinfo;                                    // Structure to hold time information
    int retries = 0;                                       // Initialize retry counter

    while (!getLocalTime(&timeinfo) && retries < 10) // Retry until time synced or max retries reached
    {
        delay(RETRY_DELAY_MS); // Delay between retries
        Serial.print(".");     // Print dot for each retry
        display.print(".");    // Display dot for each retry
        display.display();     // Update the display with the dots
        retries++;             // Increment retry counter
    }

    if (getLocalTime(&timeinfo)) // If time synced successfully
    {
        Serial.println("\nNTP sync successful.");     // Print success message
        display.clearDisplay();                       // Clear the display buffer
        display.setCursor(0, 0);                      // Set cursor to the top-left corner
        display.print("NTP sync successful\nwith: "); // Display success message
        display.println(ntpServer);                   // Display the NTP server
        display.display();                            // Update the display with the success message
        return true;                                  // Return true to indicate success
    }

    Serial.println("\nNTP sync failed.");     // Print failure message
    display.clearDisplay();                   // Clear the display buffer
    display.setCursor(0, 0);                  // Set cursor to the top-left corner
    display.print("NTP sync failed with:\n"); // Display failure message
    display.println(ntpServer);               // Display the NTP server
    display.display();                        // Update the display with the failure message
    return false;                             // Return false to indicate failure
}

void readDHT(float &temperatureC, float &temperatureF, float &humidity)
{
    humidity = dht.readHumidity();            // Read humidity from DHT sensor
    temperatureC = dht.readTemperature();     // Read temperature in Celsius from DHT sensor
    temperatureF = dht.readTemperature(true); // Read temperature in Fahrenheit from DHT sensor
}

bool isMotionOn()
{
    return digitalRead(MOTION_PIN) == HIGH; // Return true if motion sensor detects motion
}

bool isTiltOn()
{
    return digitalRead(TILT_PIN) == HIGH; // Return true if tilt sensor detects tilt
}

bool isFlameOn()
{
    return digitalRead(FLAME_PIN) == LOW; // Return true if flame sensor detects flame
}

void triggerAlarm()
{
    unsigned long startTime = millis(); // Get the current time

    while (millis() - startTime < ALARM_TOTAL_DURATION_MS) // Loop until alarm duration is reached
    {
        ledcWrite(redLedChannel, 255);                      // Turn on the red LED
        ledcWrite(buzzerChannel, buzzerDutyCycle);          // Turn on the buzzer
        ledcWriteTone(buzzerChannel, ALARM_HIGH_FREQUENCY); // Set buzzer tone to high frequency
        delay(ALARM_TONE_DURATION_MS);                      // Delay for tone duration
        ledcWrite(redLedChannel, 0);                        // Turn off the red LED
        ledcWrite(buzzerChannel, buzzerDutyCycle);          // Keep buzzer on
        ledcWriteTone(buzzerChannel, ALARM_LOW_FREQUENCY);  // Set buzzer tone to low frequency
        delay(ALARM_TONE_DURATION_MS);                      // Delay for tone duration
    }
    ledcWriteTone(buzzerChannel, 0); // Turn off the buzzer tone
}

void showDisplay(float temperatureC, float temperatureF, float humidity, bool motionStatus, bool tiltStatus, bool flameStatus, bool alarm)
{
    display.clearDisplay();              // Clear the display buffer
    display.setCursor(0, 0);             // Set cursor to the top-left corner
    display.setTextSize(1);              // Set text size to 1
    display.setTextColor(SSD1306_WHITE); // Set text color to white

    struct tm timeinfo;          // Structure to hold time information
    if (getLocalTime(&timeinfo)) // Get local time
    {
        char timeString[30];                                                      // Buffer to hold formatted time
        strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo); // Format time as string
        display.println(timeString);                                              // Display the time
    }
    else
    {
        display.println("Time not set"); // Display message if time not set
    }

    display.printf("Temp_C: %.2f C\nTemp_F: %.2f F\nHumidity: %.2f %%", temperatureC, temperatureF, humidity); // Display temperature and humidity
    display.printf("\nMotion: %s\nTilt: %s\nFlame: %s",
                   motionStatus ? "ON" : "OFF", // Display motion status
                   tiltStatus ? "ON" : "OFF",   // Display tilt status
                   flameStatus ? "ON" : "OFF"); // Display flame status
    if (alarm)
    {
        display.println("\nWARNING: Alarm !!!"); // Display alarm warning if alarm is triggered
    }

    display.printf("\nWeather: %s\nTemp_C: %.2f C\nTemp_F: %.2f F",
                   weatherDescription.c_str(), weatherTempC, weatherTempF); // Display weather data for the first city

    display.printf("\nWeather2: %s\nTemp_C: %.2f C\nTemp_F: %.2f F",
                   weatherDescription2.c_str(), weatherTempC2, weatherTempF2); // Display weather data for the second city

    display.display(); // Update the OLED display with the new data
}

void showSerial(float temperatureC, float temperatureF, float humidity, bool motionStatus, bool tiltStatus, bool flameStatus, bool alarm)
{
    struct tm timeinfo;          // Structure to hold time information
    if (getLocalTime(&timeinfo)) // Get local time
    {
        char timeString[30];                                                      // Buffer to hold formatted time
        strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo); // Format time as string
        Serial.printf("Date and time: %s\n", timeString);                         // Print the time to the serial monitor
    }
    else
    {
        Serial.println("Date and time: Time not set"); // Print message if time not set
    }

    Serial.printf("Local Weather: %s\n", weatherDescription.c_str()); // Print weather description for the first city
    Serial.printf("Weather Temp_C: %.2f C\n", weatherTempC);          // Print weather temperature in Celsius for the first city
    Serial.printf("Weather Temp_F: %.2f F\n", weatherTempF);          // Print weather temperature in Fahrenheit for the first city

    Serial.printf("Weather2: %s\n", weatherDescription2.c_str()); // Print weather description for the second city
    Serial.printf("Weather Temp2_C: %.2f C\n", weatherTempC2);    // Print weather temperature in Celsius for the second city
    Serial.printf("Weather Temp2_F: %.2f F\n", weatherTempF2);    // Print weather temperature in Fahrenheit for the second city

    Serial.printf("Sensor Temp_C: %.2f C\n", temperatureC);               // Print DHT sensor temperature in Celsius
    Serial.printf("Sensor Temp_F: %.2f F\n", temperatureF);               // Print DHT sensor temperature in Fahrenheit
    Serial.printf("Sensor Humidity: %.2f %%\n", humidity);                // Print DHT sensor humidity
    Serial.printf("Motion detection: %s\n", motionStatus ? "ON" : "OFF"); // Print motion sensor status
    Serial.printf("Tilt detection: %s\n", tiltStatus ? "ON" : "OFF");     // Print tilt sensor status
    Serial.printf("Flame detection: %s\n", flameStatus ? "ON" : "OFF");   // Print flame sensor status

    Serial.printf("SSID: %s\n", ssid);                                    // Print WiFi SSID
    Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str()); // Print local IP address

    if (alarm)
    {
        Serial.println("WARNING: Alarm !!!"); // Print alarm warning if alarm is triggered
    }
}

bool validateServer()
{
    HTTPClient http;                                          // Create HTTP client object
    String url = "http://" + WiFi.localIP().toString() + "/"; // Construct URL for local server
    http.begin(url);                                          // Initialize HTTP client with the URL
    int httpCode = http.GET();                                // Send GET request

    if (httpCode > 0) // Check if the request was successful
    {
        http.end();  // End HTTP connection
        return true; // Return true to indicate success
    }
    else
    {
        http.end();   // End HTTP connection
        return false; // Return false to indicate failure
    }
}

void handleRootRequest(AsyncWebServerRequest *request)
{
    request->send_P(200, "text/html", dashboardHTML); // Send HTML content for the root URL
}

void handleDataRequest(AsyncWebServerRequest *request)
{
    struct tm timeinfo;                               // Structure to hold time information
    char timeString[30] = "Time not set";             // Buffer to hold formatted time
    char nextWeatherUpdateTimeString[30] = "Not set"; // Buffer to hold next weather update time

    if (getLocalTime(&timeinfo)) // Get local time
    {
        strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo); // Format time as string
    }

    if (nextWeatherUpdate != 0) // Check if next weather update time is set
    {
        struct tm nextWeatherTimeInfo;                                                                                         // Structure to hold next weather update time information
        time_t nextUpdateTime = (time_t)nextWeatherUpdate;                                                                     // Convert next weather update time to time_t
        localtime_r(&nextUpdateTime, &nextWeatherTimeInfo);                                                                    // Convert time_t to struct tm
        strftime(nextWeatherUpdateTimeString, sizeof(nextWeatherUpdateTimeString), "%Y-%m-%d %H:%M:%S", &nextWeatherTimeInfo); // Format next weather update time as string
    }

    String json = "{";                                                                // Initialize JSON string
    json += "\"currentDateTime\":\"" + String(timeString) + "\",";                    // Add current date and time to JSON
    json += "\"wifiSSID\":\"" + String(WiFi.SSID()) + "\",";                          // Add WiFi SSID to JSON
    json += "\"ipAddress\":\"" + String(WiFi.localIP().toString()) + "\",";           // Add IP address to JSON
    json += "\"city\":\"" + String(city) + "\",";                                     // Add city to JSON
    json += "\"temperatureC\":" + String(temperatureC, 2) + ",";                      // Add temperature in Celsius to JSON
    json += "\"temperatureF\":" + String(temperatureF, 2) + ",";                      // Add temperature in Fahrenheit to JSON
    json += "\"humidity\":" + String(humidity, 2) + ",";                              // Add humidity to JSON
    json += "\"motion\":" + String(isMotionOn() ? "true" : "false") + ",";            // Add motion sensor status to JSON
    json += "\"tilt\":" + String(isTiltOn() ? "true" : "false") + ",";                // Add tilt sensor status to JSON
    json += "\"flame\":" + String(isFlameOn() ? "true" : "false") + ",";              // Add flame sensor status to JSON
    json += "\"weatherDescription\":\"" + weatherDescription + "\",";                 // Add weather description for the first city to JSON
    json += "\"weatherTempC\":" + String(weatherTempC, 2) + ",";                      // Add weather temperature in Celsius for the first city to JSON
    json += "\"weatherTempF\":" + String(weatherTempF, 2) + ",";                      // Add weather temperature in Fahrenheit for the first city to JSON
    json += "\"city2\":\"" + String(city2) + "\",";                                   // Add second city to JSON
    json += "\"weatherDescription2\":\"" + weatherDescription2 + "\",";               // Add weather description for the second city to JSON
    json += "\"weatherTempC2\":" + String(weatherTempC2, 2) + ",";                    // Add weather temperature in Celsius for the second city to JSON
    json += "\"weatherTempF2\":" + String(weatherTempF2, 2) + ",";                    // Add weather temperature in Fahrenheit for the second city to JSON
    json += "\"nextWeatherUpdate\":\"" + String(nextWeatherUpdateTimeString) + "\","; // Add next weather update time to JSON
    json += "\"deviceID\":\"" + deviceID + "\",";                                     // Add device ID to JSON
    json += "\"awsStatus\":" + String(awsConnected ? "true" : "false") + ",";         // Add AWS connection status to JSON
    json += "\"mqttStatus\":" + String(mqttClient.connected() ? "true" : "false");    // Add MQTT connection status to JSON
    json += "}";                                                                      // Close JSON string
    request->send(200, "application/json", json);                                     // Send JSON response
}

void handleGreenLedRequest(AsyncWebServerRequest *request)
{
    if (request->hasParam("state")) // Check if state parameter is present
    {
        String state = request->getParam("state")->value(); // Get the state parameter value
        if (state == "1")                                   // If state is "1"
        {
            ledcWrite(greenLedChannel, 255); // Turn on the green LED
            greenLedState = true;            // Set green LED state to true
        }
        else // If state is not "1"
        {
            ledcWrite(greenLedChannel, 0); // Turn off the green LED
            greenLedState = false;         // Set green LED state to false
        }
    }
    request->send(200, "text/plain", greenLedState ? "Green LED is ON" : "Green LED is OFF"); // Send response with green LED state
}

void handleRedLedRequest(AsyncWebServerRequest *request)
{
    if (request->hasParam("state")) // Check if state parameter is present
    {
        String state = request->getParam("state")->value(); // Get the state parameter value
        if (state == "1")                                   // If state is "1"
        {
            ledcWrite(redLedChannel, 255); // Turn on the red LED
            redLedState = true;            // Set red LED state to true
        }
        else // If state is not "1"
        {
            ledcWrite(redLedChannel, 0); // Turn off the red LED
            redLedState = false;         // Set red LED state to false
        }
    }
    request->send(200, "text/plain", redLedState ? "Red LED is ON" : "Red LED is OFF"); // Send response with red LED state
}

void handleTimeRequest(AsyncWebServerRequest *request)
{
    struct tm timeinfo;                   // Structure to hold time information
    char timeString[30] = "Time not set"; // Buffer to hold formatted time
    if (getLocalTime(&timeinfo))          // Get local time
    {
        strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo); // Format time as string
    }
    String json = "{\"currentDateTime\":\"" + String(timeString) + "\"}"; // Create JSON response with current date and time
    request->send(200, "application/json", json);                         // Send JSON response
}

void fetchWeatherData(const char *city, String &weatherDescription, float &weatherTempC, float &weatherTempF)
{
    bool weatherFetchSuccess = false; // Initialize weather fetch success flag

    if (WiFi.status() == WL_CONNECTED) // Check if WiFi is connected
    {
        HTTPClient http;                                                                                                                         // Create HTTP client object
        String weatherUrl = "http://api.openweathermap.org/data/2.5/weather?q=" + String(city) + "&units=metric&appid=" + String(weatherApiKey); // Construct URL for weather API
        http.begin(weatherUrl);                                                                                                                  // Initialize HTTP client with the URL

        int httpCode = http.GET(); // Send GET request
        if (httpCode > 0)          // Check if the request was successful
        {
            String payload = http.getString(); // Get the response payload
            DynamicJsonDocument doc(1024);     // Create JSON document
            deserializeJson(doc, payload);     // Parse JSON payload

            if (doc["weather"].isNull() || doc["main"].isNull()) // Check if JSON data is valid
            {
                Serial.println("Invalid weather data received."); // Print error message
                weatherDescription = "null";                      // Set weather description to null
                weatherTempC = 0.0;                               // Set weather temperature in Celsius to 0
                weatherTempF = 0.0;                               // Set weather temperature in Fahrenheit to 0
            }
            else
            {
                weatherDescription = doc["weather"][0]["description"].as<String>(); // Get weather description
                weatherTempC = doc["main"]["temp"].as<float>();                     // Get weather temperature in Celsius
                weatherTempF = weatherTempC * 9.0 / 5.0 + 32.0;                     // Convert Celsius to Fahrenheit
                weatherFetchSuccess = true;                                         // Set weather fetch success flag to true

                time_t now;                                   // Variable to hold current time
                time(&now);                                   // Get current time
                lastWeatherUpdate = now;                      // Update the last weather update time
                nextWeatherUpdate = lastWeatherUpdate + 3600; // Set the next weather update time (1 hour later)
            }
        }
        else
        {
            Serial.println("Error fetching weather data"); // Print error message
            weatherDescription = "null";                   // Set weather description to null
            weatherTempC = 0.0;                            // Set weather temperature in Celsius to 0
            weatherTempF = 0.0;                            // Set weather temperature in Fahrenheit to 0
        }
        http.end(); // End HTTP connection
    }
    else
    {
        weatherDescription = "null"; // Set weather description to null
        weatherTempC = 0.0;          // Set weather temperature in Celsius to 0
        weatherTempF = 0.0;          // Set weather temperature in Fahrenheit to 0
    }

    if (weatherFetchSuccess) // If weather data was fetched successfully
    {
        Serial.println("Weather data fetched successfully."); // Print success message
        Serial.print("Next weather update at: ");             // Print next weather update time
        Serial.println(ctime((time_t *)&nextWeatherUpdate));  // Print formatted next weather update time
    }
    else
    {
        Serial.println("Failed to fetch weather data."); // Print failure message
    }
}

bool connectAWS()
{
    net.setCACert(AWS_ROOT_CA);                                   // Set CA certificate for secure connection
    net.setCertificate(AWS_CERT_CRT);                             // Set client certificate
    net.setPrivateKey(AWS_PRIVATE_KEY);                           // Set private key
    mqttClient.setServer(AWS_IOT_MQTT_SERVER, AWS_IOT_MQTT_PORT); // Set MQTT server and port
    Serial.println("Connecting to AWS IoT Core...");              // Print connecting message
    display.println("Connecting to AWS IoT Core...");             // Display connecting message
    int attempt = 0;                                              // Initialize attempt counter
    while (!mqttClient.connect(deviceID.c_str()) && attempt < 3)  // Retry until connected or max attempts reached
    {
        attempt++;                                // Increment attempt counter
        Serial.print("Attempt ");                 // Print attempt message
        display.print("Attempt ");                // Display attempt message
        Serial.print(attempt);                    // Print attempt number
        Serial.println("/3 failed, retrying..."); // Print retry message
        display.print("/3 failed, retrying...");  // Display retry message
        delay(MQTT_RECONNECT_DELAY_MS);           // Delay between attempts
    }
    if (mqttClient.connected()) // If connected to AWS
    {
        Serial.println("Connected to AWS Cloud successfully!");  // Print success message
        display.println("Connected to AWS Cloud successfully!"); // Display success message
        awsConnected = true;                                     // Set AWS connection flag to true
        return true;                                             // Return true to indicate success
    }
    else
    {
        Serial.println("Failed to connect to AWS Cloud after 3 attempts."); // Print failure message
        awsConnected = false;                                               // Set AWS connection flag to false
        return false;                                                       // Return false to indicate failure
    }
}

void mqttPublishMessage(float humidity, float temperature_c, float temperature_f, bool motionStatus, bool tiltStatus, bool flameStatus)
{
    if (mqttClient.connected())
    {
        // Fetch the current time
        time_t unixTime = time(nullptr); // Get the current time as Unix time
        // Check if time is valid
        if (unixTime == -1)
        {
            Serial.println("Failed to obtain time");
            return;
        }
        // Create a JSON document
        StaticJsonDocument<512> doc;

        // Populate document
        doc["timeStamp"] = unixTime;                      // Add timestamp to JSON document
        doc["deviceModel"] = "ESP32";                     // Add device model to JSON document
        doc["owner"] = "Jun";                             // Add owner to JSON document
        doc["group"] = "Instructor";                      // Add group to JSON document
        doc["deviceID"] = String(ESP.getEfuseMac(), HEX); // Add device ID to JSON document
        JsonObject data = doc.createNestedObject("data"); // Create a nested object for data
        data["temp_c"] = round(temperature_c);            // Add temperature in Celsius to JSON document
        data["temp_f"] = round(temperature_f);            // Add temperature in Fahrenheit to JSON document
        data["humidity"] = humidity;                      // Add humidity to JSON document
        data["motion"] = motionStatus ? "ON" : "OFF";     // Add motion status to JSON document
        data["tilt"] = tiltStatus ? "ON" : "OFF";         // Add tilt status to JSON document
        data["flame"] = flameStatus ? "ON" : "OFF";       // Add flame status to JSON document
        data["lat"] = "42.056204";                        // Add latitude to JSON document
        data["lon"] = "-87.674700";                       // Add longitude to JSON document

        String jsonString;                    // Create a string to hold the JSON data
        serializeJson(doc, jsonString);       // Serialize the JSON document to a string
        Serial.print("Publishing message: "); // Print the message
        Serial.println(jsonString);           // Print the JSON data

        // Determine buffer size
        size_t jsonSize = measureJson(doc) + 1; // +1 for null terminator
        Serial.print("Calculated JSON buffer size: ");
        Serial.println(jsonSize); // Print the buffer size
        if (!mqttClient.publish(AWS_IOT_PUBLISH_TOPIC.c_str(), jsonString.c_str()))
        {
            Serial.println("Publish failed"); // Print publish failure message
        }
        else
        {
            Serial.println("Publish succeeded"); // Print publish success message
        }

        doc.clear(); // Clear the JSON document
        Serial.println("JSON document cleared");
        jsonString = ""; // Clear the JSON string
        Serial.println("JSON string cleared");
    }
}
