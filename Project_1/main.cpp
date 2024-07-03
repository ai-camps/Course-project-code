// **********************************
// * Project 1
// * Compile, upload and run an example C++ code (Connect a MCU to a mobile hotspot) 
// **********************************

// * Libraries
#include <Arduino.h> // Includes the core Arduino functions for the ESP32
#include <WiFi.h>    // Includes the WiFi library to connect to WiFi networks

// * Constant and Variables Declaration
const char* ssid = "your_SSID";       // The name of the WiFi network (SSID)
const char* password = "your_PASSWORD"; // The password for the WiFi network

// * Function Declarations
void connectToWiFi(); // Declares the function that will connect the ESP32 to WiFi

// * setup() function
void setup() {
  // ! Initialize serial communication
  Serial.begin(115200); // Sets up serial communication at a baud rate of 115200 bits per second
  
  // ! Connect to WiFi
  connectToWiFi(); // Calls the function to connect to WiFi
}

// * loop() function
void loop() {
  // ! Print the current WiFi SSID and IP address
  Serial.print("Connected to: "); // Prints a message to the serial monitor
  Serial.println(WiFi.SSID());    // Prints the current WiFi network name (SSID)
  Serial.print("IP Address: ");   // Prints a message to the serial monitor
  Serial.println(WiFi.localIP()); // Prints the IP address assigned to the ESP32

  // ! Wait for 30 seconds (30000 milliseconds)
  delay(30000); // Pauses the program for 30000 milliseconds (30 seconds) before repeating
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
}
