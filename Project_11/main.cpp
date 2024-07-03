// * Import libraries
#include <Arduino.h>          // Arduino core functionality
#include <DHT.h>              // Library for DHT sensor

// * Constants and variable declaration

DHT dht(DHT_PIN, DHT11); // Initialize DHT sensor for DHT11

const int greenLedChannel = 0; // PWM channel for Green LED
const int redLedChannel = 1;   // PWM channel for Red LED
const int buzzerChannel = 2;   // PWM channel for Buzzer
const int pwmFrequency = 5000; // Frequency for PWM
const int pwmResolution = 8;   // 8-bit resolution (0-255)

constexpr float TEMP_HIGH_THRESHOLD_F = 80.0;          // High temperature threshold in Fahrenheit
constexpr unsigned long ALARM_HIGH_FREQUENCY = 2000;   // High frequency for the alarm
constexpr unsigned long ALARM_LOW_FREQUENCY = 500;     // Low frequency for the alarm
constexpr unsigned long ALARM_TONE_DURATION_MS = 100;  // Duration of each alarm tone
constexpr unsigned long ALARM_TOTAL_DURATION_MS = 3000;// Total duration of the alarm

// * Functions declaration
void setup();               // Setup function declaration
void initPWM();             // Function to configure PWM functionalities
void loop();                // Loop function declaration
void readAndDisplayDHT();   // Function to read, display, and check DHT data
void buzzerAndBlinkAlarm(); // Function to trigger alarm

// * setup() function
void setup() {
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate
    dht.begin();          // Initialize DHT sensor
    initPWM();            // Configure PWM functionalities
}

// * loop() function
void loop() {
    readAndDisplayDHT(); // Read, display, and check DHT data
    delay(10000);        // Wait for 10 seconds
}

// * Functions definition

// ! Function to configure PWM functionalities
void initPWM() {
    // Configure LED PWM functionalities
    ledcSetup(greenLedChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(GREEN_LED_PIN, greenLedChannel);

    ledcSetup(redLedChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(RED_LED_PIN, redLedChannel);

    // Configure Buzzer PWM functionalities
    ledcSetup(buzzerChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(BUZZER_PIN, buzzerChannel);
}

// ! Function to read, display, and check DHT data
void readAndDisplayDHT() {
    float humidity = dht.readHumidity();             // Read humidity
    float temperature_c = dht.readTemperature();     // Read temperature in Celsius
    float temperature_f = dht.readTemperature(true); // Read temperature in Fahrenheit

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature_c) || isnan(temperature_f)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    // Print to serial port
    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.print(F("%  "));
    Serial.print(F("Temperature: "));
    Serial.print(temperature_c);
    Serial.print(F("°C "));
    Serial.print(temperature_f);
    Serial.println(F("°F"));

    // Check temperature and control LEDs and buzzer
    if (temperature_f < TEMP_HIGH_THRESHOLD_F) {
        ledcWrite(greenLedChannel, 255); // Turn on green LED at full brightness
        ledcWrite(redLedChannel, 0);     // Turn off red LED
        ledcWrite(buzzerChannel, 0);     // Turn off buzzer
    } else {
        ledcWrite(greenLedChannel, 0);                   // Turn off green LED
        buzzerAndBlinkAlarm();                           // Trigger alarm if temperature exceeds threshold
        Serial.println(F("Warning: High Temperature!")); // Print warning to serial port
    }
}

// ! Function to trigger alarm
void buzzerAndBlinkAlarm() {
    unsigned long startTime = millis(); // Record the start time

    while (millis() - startTime < ALARM_TOTAL_DURATION_MS) {
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
