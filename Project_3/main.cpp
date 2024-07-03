// **********************************
// * Project 3
// * Connect a green LED on MCU Pin GPIO-10 and turn it on when Wi-fi access is successful.
// **********************************
// * Libraries
#include <Arduino.h> // Includes the core Arduino functions for the ESP32
#include <WiFi.h>    // Includes the WiFi library to connect to WiFi networks
#include <ESP32Ping.h> // Includes the ESP32 Ping library for ping functionality

// * Constant and Variables Declaration
const char* ssid = "Brown-Guest";       // The name of the WiFi network (SSID)
const char* password = ""; // The password for the WiFi network
const char *host = PING_HOST;           // Host to ping
const char *ntpServer = NTP_SERVER;     // NTP server
constexpr unsigned long NTP_SYNC_DELAY_MS = 1000; // Delay between NTP time sync attempts

// * Function Declarations
void connectToWiFi(); // Declares the function that will connect the ESP32 to WiFi
void pingHost(); // Declares the function that will ping a host
void syncNTP();

// * setup() function
void setup() {
  // ! Initialize serial communication
  Serial.begin(115200); // Sets up serial communication at a baud rate of 115200 bits per second
  
  // ! Initialize LED
  pinMode(GREEN_LED_PIN, OUTPUT); // Sets the LED pin as an output

  // ! Connect to WiFi
  connectToWiFi(); // Calls the function to connect to WiFi
}

// * loop() function
void loop() {
    Serial.println("****************************"); // Print a separator for readability
  // Check the WiFi status
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(GREEN_LED_PIN, HIGH); // Turn on the LED when WiFi is connected
    Serial.println("LED is On!"); // Print the message when the LED is on
    
    // ! Print the current WiFi SSID and IP address
    Serial.print("Connected to: "); // Prints a message to the serial monitor
    Serial.println(WiFi.SSID());    // Prints the current WiFi network name (SSID)
    Serial.print("IP Address: ");   // Prints a message to the serial monitor
    Serial.println(WiFi.localIP()); // Prints the IP address assigned to the ESP32
    Serial.print("MAC Address: ");  // Prints a message to the serial monitor
    Serial.println(WiFi.macAddress()); // Prints the MAC address of the ESP32

    // ! Ping Google's DNS server
    pingHost(); // Calls the function to ping Google's DNS server

    // ! Synchronize NTP time
    syncNTP();
  } else {
    digitalWrite(GREEN_LED_PIN, LOW); // Turn off the LED when WiFi is not connected
  }

  // ! Wait for 10 seconds (10000 milliseconds)
  delay(10000); // Pauses the program for 10000 milliseconds (10 seconds) before repeating
}

// * Function Definitions
void connectToWiFi() {
  Serial.print("Connecting to "); // Prints a message to the serial monitor
  Serial.println(ssid);           // Prints the SSID of the WiFi network

  WiFi.begin(ssid, password); // Starts the WiFi connection process with the given SSID and password

  // ! Wait until the ESP32 is connected to the WiFi network
  while (WiFi.status() != WL_CONNECTED) { // Checks if the WiFi is not connected
    delay(500); // Waits for 500 milliseconds before checking again
    Serial.print("."); // Prints a dot to the serial monitor to indicate connection progress
  }

  // ! Once connected, print a confirmation message and the IP address
  Serial.println(""); // Prints a new line for readability
  Serial.println("WiFi connected."); // Prints a confirmation message
  Serial.println("IP address: ");    // Prints a message to the serial monitor
  Serial.println(WiFi.localIP());    // Prints the IP address assigned to the ESP32
  Serial.print("MAC Address: ");     // Prints a message to the serial monitor
  Serial.println(WiFi.macAddress()); // Prints the MAC address of the ESP32
}

void pingHost() // Function to ping a host
{
    Serial.print("Pinging host: " + String(host) + "...");

    if (Ping.ping(host))
    {
        Serial.println("Ping successful.");
    }
    else
    {
        Serial.println("Ping failed.");
    }
}

void syncNTP() // Function to synchronize the NTP time
{
    Serial.println("Synchronizing NTP now..."); // Display the message

    // Initialize and start the SNTP service.
    configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, ntpServer);

    // Wait for time to be set
    Serial.println("Waiting for NTP time sync...");
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to fetch NTP time.");
        return;
    }

    // Once synchronized, print the current time
    char timeStr[64];
    strftime(timeStr, sizeof(timeStr), "%A, %B %d %Y %H:%M:%S", &timeinfo);
    Serial.println(timeStr);
}
