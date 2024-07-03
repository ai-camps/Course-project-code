// ! Import libraries
#include <Arduino.h>          // Arduino core functionality
#include <Wire.h>             // Library for I2C communication
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_SSD1306.h> // Library for SSD1306 OLED display
#include <DHT.h>              // Library for DHT sensor

// ! Constants and variable declaration
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin (not used)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

constexpr unsigned long ALARM_HIGH_FREQUENCY = 2000;    // High frequency for the alarm
constexpr unsigned long ALARM_LOW_FREQUENCY = 500;      // Low frequency for the alarm
constexpr unsigned long ALARM_TONE_DURATION_MS = 100;   // Duration of each alarm tone
constexpr unsigned long ALARM_TOTAL_DURATION_MS = 3000; // Total duration of the alarm

constexpr unsigned long LOOP_DELAY_MS = 10000;          // Delay for the main loop

DHT dht(DHT_PIN, DHT11); // Initialize DHT sensor for DHT11

constexpr float TEMP_HIGH_THRESHOLD_F = 80.0; // High temperature threshold in Fahrenheit

const int greenLedChannel = 0; // PWM channel for Green LED
const int redLedChannel = 1;   // PWM channel for Red LED
const int buzzerChannel = 2;   // PWM channel for Buzzer
const int pwmFrequency = 5000; // Frequency for PWM
const int pwmResolution = 8;   // 8-bit resolution (0-255)
const int buzzerDutyCycle = 64; // Duty cycle for the buzzer to lower the volume

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
  display.clearDisplay();
  display.setTextSize(1);              // Set text size
  display.setTextColor(SSD1306_WHITE); // Set text color
  display.setCursor(0, 0);             // Set cursor position
  display.println(F("Initializing..."));
  display.display();
}

// ! Functions declaration
void setup();                               // Setup function declaration
void loop();                                // Loop function declaration
void readAndDisplayDHT();                   // Function to read and display DHT data
void buzzerAndBlinkAlarm();                 // Function to trigger alarm

// ! setup() function
void setup()
{
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  initOLED();           // Initialize OLED display
  dht.begin();          // Initialize DHT sensor

  // Configure PWM channels
  ledcSetup(greenLedChannel, pwmFrequency, pwmResolution); // Configure PWM channel for Green LED
  ledcSetup(redLedChannel, pwmFrequency, pwmResolution);   // Configure PWM channel for Red LED
  ledcSetup(buzzerChannel, pwmFrequency, pwmResolution);   // Configure PWM channel for Buzzer

  // Attach the channels to the GPIO pins
  ledcAttachPin(GREEN_LED_PIN, greenLedChannel); // Attach green LED pin to PWM channel
  ledcAttachPin(RED_LED_PIN, redLedChannel);     // Attach red LED pin to PWM channel
  ledcAttachPin(BUZZER_PIN, buzzerChannel);      // Attach buzzer pin to PWM channel
}

// * loop() function
void loop()
{
  readAndDisplayDHT(); // Read and display DHT data
  delay(10000);        // Wait for 10 seconds
}

// (6) Functions definition

// Function to read and display DHT data
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
    display.print(F("Humidity: "));
    display.print(humidity);
    display.println(F("%"));
    display.print(F("Temp: "));
    display.print(temperature_c);
    display.print((char)176); // ASCII code for degree symbol
    display.println(F("C"));
    display.print(F("Temp: "));
    display.print(temperature_f);
    display.print((char)176); // ASCII code for degree symbol
    display.println(F("F"));
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
