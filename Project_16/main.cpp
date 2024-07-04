// **********************************
// * Project 16
// * based from project 15, publish sensor data to AWS Cloud
// **********************************
// * Import libraries
#include <Wire.h>              // Library for I2C communication
#include <Adafruit_GFX.h>      // Core graphics library
#include <Adafruit_SSD1306.h>  // Library for SSD1306 OLED display
#include <DHT.h>               // Library for DHT sensor
#include <WiFi.h>              // Library for WiFi
#include <ESP32Ping.h>         // Library for ping functionality
#include <time.h>              // Library for NTP synchronization
#include "music.h"             // Library for music notes
#include <WiFiClientSecure.h>  // Include the WiFiClientSecure library
#include "SecureCredentials.h" // Include the secrets file
#include <PubSubClient.h>      // Include the MQTT client library
#include <ArduinoJson.h>       // Include the ArduinoJson library

// * Constants and variable declaration
#define SCREEN_WIDTH 128                                                  // OLED display width, in pixels
#define SCREEN_HEIGHT 64                                                  // OLED display height, in pixels
#define OLED_RESET -1                                                     // Reset pin (not used)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Create an instance of the SSD1306 display

DHT dht(DHT_PIN, DHT11); // Initialize DHT sensor for DHT11

const int greenLedChannel = 0;  // PWM channel for Green LED
const int redLedChannel = 1;    // PWM channel for Red LED
const int buzzerChannel = 2;    // PWM channel for Buzzer
const int pwmFrequency = 5000;  // Frequency for PWM
const int pwmResolution = 8;    // 8-bit resolution (0-255)
const int buzzerDutyCycle = 64; // Duty cycle for the buzzer to lower the volume

constexpr float TEMP_HIGH_THRESHOLD_F = 80.0; // High temperature threshold in Fahrenheit

constexpr unsigned long ALARM_HIGH_FREQUENCY = 2000;    // High frequency for the alarm
constexpr unsigned long ALARM_LOW_FREQUENCY = 500;      // Low frequency for the alarm
constexpr unsigned long ALARM_TONE_DURATION_MS = 100;   // Duration of each alarm tone
constexpr unsigned long ALARM_TOTAL_DURATION_MS = 3000; // Total duration of the alarm
constexpr unsigned long NTP_SYNC_DELAY_MS = 1000;       // Delay between NTP time sync attempts
constexpr int maxRetries = 3;                           // Maximum number of retries for WiFi, Ping, and NTP
constexpr unsigned long RESTART_DELAY_MS = 3000;        // Delay before restarting the ESP32 in milliseconds
constexpr unsigned long WIFI_RETRY_DELAY_MS = 500;      // Delay between WiFi connection attempts
constexpr unsigned long WIFI_RETRY_WAIT_MS = 1000;      // Wait time before next WiFi connection attempt
constexpr unsigned long PING_RETRY_DELAY_MS = 500;      // Delay between Ping attempts
constexpr unsigned long OLED_INIT_DELAY_MS = 2000;      // Delay to ensure OLED starts correctly
constexpr unsigned long LOOP_DELAY_MS = 10000;          // Delay for the main loop

constexpr const char *ssid = "Brown-Guest";   // WiFi SSID
constexpr const char *password = "";          // WiFi Password
constexpr const char *host = PING_HOST;       // Host to ping
constexpr const char *ntpServer = NTP_SERVER; // NTP server

// * AWS IoT Core access settings
WiFiClientSecure net = WiFiClientSecure();              // Create a WiFiClientSecure to handle the MQTT connection
PubSubClient mqttClient(net);                           // Create a PubSubClient to handle the MQTT connection
constexpr unsigned long MQTT_RECONNECT_DELAY_MS = 3000; // Delay between reconnect attempts
String deviceID;                                        // Device ID for the AWS IoT Core
String AWS_IOT_PUBLISH_TOPIC;                           // MQTT topic to publish messages

// * Functions declaration
void setup();                                                                                                                          // Setup function declaration
void initOLED();                                                                                                                       // Function to initialize OLED display
void initPWM();                                                                                                                        // Function to configure PWM functionalities
void loop();                                                                                                                           // Loop function declaration
void readAndDisplayDHT();                                                                                                              // Function to read, display, and check DHT data
bool connectToWiFi();                                                                                                                  // Connects the ESP32 to the specified WiFi network
bool pingHost();                                                                                                                       // Pings the specified host to check network connectivity
bool syncNTP();                                                                                                                        // Synchronizes the ESP32's time with an NTP (Network Time Protocol) server
void buzzerAndBlinkAlarm();                                                                                                            // Function to trigger alarm
bool readAndDisplayTiltSensor();                                                                                                       // Function to read and display tilt sensor data
bool readAndDisplayMotionSensor();                                                                                                     // Function to read and display motion sensor data
bool readAndDisplayFlameSensor();                                                                                                      // Function to read and display flame sensor data
bool connectAWS();                                                                                                                     // Function to connect to AWS IoT Core
void mqttPublishMessage(float humidity, float temperatureC, float temperatureF, bool tiltStatus, bool motionStatus, bool flameStatus); // Function to publish MQTT messages

// * setup() function
void setup()
{
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate

    initOLED(); // Initialize OLED display

    dht.begin(); // Initialize DHT sensor

    initPWM(); // Initialize PWM functionalities

    pinMode(TILT_PIN, INPUT);   // Initialize tilt sensor pin as input
    pinMode(MOTION_PIN, INPUT); // Initialize motion sensor pin as input
    pinMode(FLAME_PIN, INPUT);  // Initialize flame sensor pin as input

    deviceID = String(ESP.getEfuseMac(), HEX); // Get the device ID
    AWS_IOT_PUBLISH_TOPIC = deviceID + "/pub"; // Set the MQTT topic to publish messages

    // ! Connect to WiFi
    if (!connectToWiFi())
    {
        Serial.println("Failed to connect to WiFi: " + String(ssid)); // Print error message to serial
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Failed to connect to WiFi: " + String(ssid)); // Print error message to OLED
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

    // ! Ping Google's DNS server
    if (!pingHost())
    {
        Serial.println("Failed to ping host: " + String(host)); // Print error message to serial
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Failed to ping host: " + String(host)); // Print error message to OLED
        display.print("ESP32 is rebooting in ");
        display.print(RESTART_DELAY_MS / 1000);
        display.println(" seconds.");
        display.display();
        ledcWrite(redLedChannel, 255); // Turn on Red LED
        delay(RESTART_DELAY_MS);       // Wait before restarting
        ESP.restart();                 // Reboot the MCU
    }

    // ! Synchronize NTP time
    if (!syncNTP())
    {
        Serial.println("Failed to sync NTP: " + String(ntpServer)); // Print error message to serial
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Failed to sync NTP: " + String(ntpServer)); // Print error message to OLED
        display.print("ESP32 is rebooting in ");
        display.print(RESTART_DELAY_MS / 1000);
        display.println(" seconds.");
        display.display();
        ledcWrite(redLedChannel, 255); // Turn on Red LED
        delay(RESTART_DELAY_MS);       // Wait before restarting
        ESP.restart();                 // Reboot the MCU
    }

    // ! Connect to AWS Cloud
    if (!connectAWS())
    {
        Serial.println("Failed to connect to AWS Cloud: " + String(AWS_IOT_MQTT_SERVER)); // Print error message to serial
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Failed to connect to AWS Cloud: " + String(AWS_IOT_MQTT_SERVER)); // Print error message to OLED
        display.print("ESP32 is rebooting in ");
        display.print(RESTART_DELAY_MS / 1000);
        display.println(" seconds.");
        display.display();
        ledcWrite(redLedChannel, 255); // Turn on Red LED
        delay(RESTART_DELAY_MS);       // Wait before restarting
        ESP.restart();                 // Reboot the ESP32 if AWS connection fails
    }

    // Play music when NTP sync is successful
    playMusic(buzzerChannel);
}

// * loop() function
void loop()
{
    Serial.println("........ Loop iteration started.........."); // Print loop iteration message

    // Check if WiFi is connected
    if (WiFi.status() == WL_CONNECTED)
    {
        ledcWrite(greenLedChannel, 255);      // Set Green LED brightness to maximum
        ledcWrite(redLedChannel, 0);          // Turn off Red LED
        Serial.println("WiFi is connected."); // Print message to serial
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("WiFi is connected."); // Print message to OLED
        display.display();
    }
    else
    {
        Serial.println("WiFi connection lost. Attempting to reconnect..."); // Print error message to serial
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("WiFi connection lost."); // Print error message to OLED
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
            Serial.println("Failed to reconnect to WiFi after 3 attempts."); // Print error message to serial
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Failed to reconnect to WiFi."); // Print error message to OLED
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
        display.println(timeStr); // Display formatted time on OLED
    }
    else
    {
        display.println("Time: N/A"); // Display "Time: N/A" if time is not available
    }

    // Print LED status
    display.print("Green LED: ");
    display.println(ledcRead(greenLedChannel) ? "On" : "Off");
    display.print("Red LED: ");
    display.println(ledcRead(redLedChannel) ? "On" : "Off");

    display.display();

    readAndDisplayDHT();                              // Read, display, and check DHT data
    bool tiltStatus = readAndDisplayTiltSensor();     // Read and display tilt sensor data
    bool motionStatus = readAndDisplayMotionSensor(); // Read and display motion sensor data
    bool flameStatus = readAndDisplayFlameSensor();   // Read and display flame sensor data

    Serial.print("Tilt Status: ");
    Serial.println(tiltStatus ? "Tilted" : "Not Tilted");
    Serial.print("Motion Status: ");
    Serial.println(motionStatus ? "Motion Detected" : "No Motion");
    Serial.print("Flame Status: ");
    Serial.println(flameStatus ? "Flame Detected" : "No Flame");

    // Publish sensor data to AWS IoT Core
    mqttPublishMessage(dht.readHumidity(), dht.readTemperature(), dht.readTemperature(true), tiltStatus, motionStatus, flameStatus);

    delay(LOOP_DELAY_MS); // Delay before next loop iteration
}

// * Functions definition

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

// ! Function to connect to WiFi
bool connectToWiFi()
{
    Serial.println("Connecting to WiFi: " + String(ssid)); // Print message to serial
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting to WiFi: " + String(ssid)); // Print message to OLED
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
            Serial.println("");                                // Print new line for readability
            Serial.println("WiFi is connected successfully."); // Print message to serial
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("WiFi is connected successfully."); // Print message to OLED
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
    Serial.println("Synchronizing NTP: " + String(ntpServer)); // Print message to serial
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Synchronizing NTP: " + String(ntpServer)); // Print message to OLED
    display.display();

    String ntpMessage = "Fetching date and time from NTP server: " + String(ntpServer);
    Serial.println(ntpMessage); // Print message to serial
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(ntpMessage); // Print message to OLED
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
        display.println(timeStr); // Print formatted time to OLED
        display.display();
        return true; // Return true if NTP sync was successful
    }
    else
    {
        return false;
    }
}

bool connectAWS() // Function to connect to AWS IoT Core
{
    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_ROOT_CA);         // Set the AWS Root CA certificate
    net.setCertificate(AWS_CERT_CRT);   // Set the device certificate
    net.setPrivateKey(AWS_PRIVATE_KEY); // Set the private key

    // Set the AWS IoT endpoint and port
    mqttClient.setServer(AWS_IOT_MQTT_SERVER, AWS_IOT_MQTT_PORT);

    Serial.println("Connecting to AWS IoT Core...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting to AWS IoT Core...");
    display.display();

    int attempt = 0;
    while (!mqttClient.connect(deviceID.c_str()) && attempt < 3)
    {
        attempt++;
        Serial.print("Attempt ");
        Serial.print(attempt);
        Serial.println("/3 failed, retrying...");

        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Attempt ");
        display.print(attempt);
        display.println("/3 failed, retrying...");
        display.display();

        delay(MQTT_RECONNECT_DELAY_MS); // Delay before retrying
    }

    if (mqttClient.connected())
    {
        Serial.println("Connected to AWS Cloud successfully!");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Connected to AWS Cloud successfully!");
        display.display();
        return true;
    }
    else
    {
        Serial.println("Failed to connect to AWS Cloud after 3 attempts.");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Failed to connect to AWS Cloud after 3 attempts.");
        display.display();

        return false;
    }
}

// ! Function to read, display, and check DHT data
void readAndDisplayDHT()
{
    float humidity = dht.readHumidity();             // Read humidity
    float temperature_c = dht.readTemperature();     // Read temperature in Celsius
    float temperature_f = dht.readTemperature(true); // Read temperature in Fahrenheit

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature_c) || isnan(temperature_f))
    {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    // Print to serial port
    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.print(F("%  "));
    Serial.print(F("Temperature: "));
    Serial.print(temperature_c);
    Serial.print((char)176); // ASCII code for degree symbol
    Serial.print(F("C "));
    Serial.print(temperature_f);
    Serial.print((char)176); // ASCII code for degree symbol
    Serial.println(F("F"));

    // Print to OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(F("Humidity_%: "));
    display.println(humidity);
    display.print(F("Temperature_C: "));
    display.println(temperature_c);
    display.print(F("Temperature_F: "));
    display.println(temperature_f);
    display.display();

    // Check temperature and control LEDs and buzzer
    if (temperature_f < TEMP_HIGH_THRESHOLD_F)
    {
        ledcWrite(greenLedChannel, 255); // Turn on green LED at full brightness
        ledcWrite(redLedChannel, 0);     // Turn off red LED
        ledcWrite(buzzerChannel, 0);     // Turn off buzzer
    }
    else
    {
        ledcWrite(greenLedChannel, 0); // Turn off green LED
        Serial.println(F("Warning!"));
        Serial.println(F("High Temperature!")); // Print warning to serial port
        Serial.print(F("Temperature: "));
        Serial.print(temperature_f);
        Serial.print((char)176); // ASCII code for degree symbol
        Serial.println(F("F"));

        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("Warning!"));
        display.println(F("High Temp!"));
        display.print(F("Temp: "));
        display.print(temperature_f);
        display.print((char)176); // ASCII code for degree symbol
        display.println(F("F"));
        display.display();

        buzzerAndBlinkAlarm(); // Trigger alarm if temperature exceeds threshold
    }
}

// ! Function to read and display tilt sensor data
bool readAndDisplayTiltSensor()
{
    int tiltState = digitalRead(TILT_PIN); // Read the state of the tilt sensor
    display.setCursor(0, 32);              // Set cursor position for tilt sensor status
    display.print("Tilt Status: ");
    display.println(tiltState ? "Yes" : "No"); // Display tilt status on OLED
    display.display();                         // Update OLED display
    return tiltState;                          // Return the tilt sensor state
}

// ! Function to read and display motion sensor data
bool readAndDisplayMotionSensor()
{
    int motionState = digitalRead(MOTION_PIN); // Read the state of the motion sensor
    display.setCursor(0, 42);                  // Set cursor position for motion sensor status
    display.print("Motion Status: ");
    display.println(motionState ? "Yes" : "No"); // Display motion status on OLED
    display.display();                           // Update OLED display
    return motionState;                          // Return the motion sensor state
}

// ! Function to read and display flame sensor data
bool readAndDisplayFlameSensor()
{
    int flameState = digitalRead(FLAME_PIN); // Read the state of the flame sensor
    display.setCursor(0, 52);                // Set cursor position for flame sensor status
    display.print("Flame Status: ");
    display.println(flameState ? "Yes" : "No"); // Display flame status on OLED
    display.display();                          // Update OLED display
    return flameState;                          // Return the flame sensor state
}

// Function to publish MQTT messages
void mqttPublishMessage(float humidity, float temperature_c, float temperature_f, bool tiltStatus, bool motionStatus, bool flameStatus)
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
        doc["timeStamp"] = unixTime;                        // Add timestamp to JSON document
        doc["deviceModel"] = "ESP32";                       // Add device model to JSON document
        doc["owner"] = "Jun Wen";                           // Add owner to JSON document
        doc["group"] = "Group_1";                           // Add group to JSON document
        doc["deviceID"] = String(ESP.getEfuseMac(), HEX);   // Add device ID to JSON document
        JsonObject data = doc.createNestedObject("data");   // Create a nested object for data
        data["temp_c"] = round(temperature_c);              // Add temperature in Celsius to JSON document
        data["temp_f"] = round(temperature_f);              // Add temperature in Fahrenheit to JSON document
        data["humidity_%"] = humidity;                      // Add humidity to JSON document
        data["tiltStatus"] = tiltStatus ? "Yes" : "No";     // Add tilt status to JSON document
        data["motionStatus"] = motionStatus ? "Yes" : "No"; // Add motion status to JSON document
        data["flameStatus"] = flameStatus ? "Yes" : "No";   // Add flame status to JSON document

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
    }
}

// ! Function to trigger alarm
void buzzerAndBlinkAlarm()
{
    unsigned long startTime = millis(); // Record the start time

    while (millis() - startTime < LOOP_DELAY_MS) // Continue the alarm during the loop time
    {
        // Play high frequency and turn on Red LED
        ledcWriteTone(buzzerChannel, ALARM_HIGH_FREQUENCY);
        ledcWrite(buzzerChannel, buzzerDutyCycle); // Set the duty cycle for lower volume
        ledcWrite(redLedChannel, 255);             // Turn on Red LED
        delay(ALARM_TONE_DURATION_MS);

        // Play low frequency and turn off Red LED
        ledcWriteTone(buzzerChannel, ALARM_LOW_FREQUENCY);
        ledcWrite(buzzerChannel, buzzerDutyCycle); // Set the duty cycle for lower volume
        ledcWrite(redLedChannel, 0);               // Turn off Red LED
        delay(ALARM_TONE_DURATION_MS);
    }

    // Turn off the Buzzer
    ledcWriteTone(buzzerChannel, 0);
}
