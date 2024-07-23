#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <Arduino.h>

const char dashboardHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-C3 Sensor Dashboard</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .dashboard {
            max-width: 800px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #333;
            text-align: center;
        }
        .sensor-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin-top: 20px;
        }
        .sensor-card {
            background-color: #f9f9f9;
            padding: 15px;
            border-radius: 5px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        .sensor-name {
            font-weight: bold;
            margin-bottom: 10px;
        }
        .sensor-value {
            font-size: 1.2em;
            color: #0066cc;
        }
        .status-on {
            color: #cc0000;
        }
        .status-off {
            color: #00cc00;
        }
        .aws-connected {
            color: green;
        }
        .aws-disconnected {
            color: red;
        }
        .button-container {
            display: flex;
            justify-content: space-between;
            margin-top: 20px;
        }
        .button {
            width: 48%;
            padding: 10px;
            font-size: 1em;
            cursor: pointer;
            border: none;
            border-radius: 5px;
            color: white;
            text-align: center;
        }
        .button-on {
            background-color: #00cc00;
        }
        .button-off {
            background-color: #cc0000;
        }
    </style>
</head>
<body>
    <div class="dashboard">
        <h1>Jun's Smart Home Dashboard</h1>
        <div class="sensor-grid">
            <div class="sensor-card">
                <div class="sensor-name">Current Date and Time</div>
                <div class="sensor-value" id="current-datetime">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Wi-Fi SSID</div>
                <div class="sensor-value" id="wifi-ssid">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">IP Address</div>
                <div class="sensor-value" id="ip-address">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Sensor Temperature (°C)</div>
                <div class="sensor-value" id="temp-c">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Sensor Temperature (°F)</div>
                <div class="sensor-value" id="temp-f">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Sensor Humidity (%)</div>
                <div class="sensor-value" id="humidity">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Motion Detection Status</div>
                <div class="sensor-value" id="motion">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Tilt Detection Status</div>
                <div class="sensor-value" id="tilt">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Flame Detection Status</div>
                <div class="sensor-value" id="flame">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name" id="city-weather">Local Weather</div>
                <div class="sensor-value" id="weather-desc">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Weather Temperature (°C)</div>
                <div class="sensor-value" id="weather-temp-c">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Weather Temperature (°F)</div>
                <div class="sensor-value" id="weather-temp-f">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name" id="city2-weather">City 2 Weather</div>
                <div class="sensor-value" id="weather-desc2">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Weather Temperature 2 (°C)</div>
                <div class="sensor-value" id="weather-temp-c2">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Weather Temperature 2 (°F)</div>
                <div class="sensor-value" id="weather-temp-f2">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Device ID</div>
                <div class="sensor-value" id="device-id">--</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">AWS Connection Status</div>
                <div class="sensor-value aws-disconnected" id="aws-status">Disconnected</div>
            </div>
            <div class="sensor-card">
                <div class="sensor-name">Next Weather Update Time</div>
                <div class="sensor-value" id="next-weather-update">--</div>
            </div>
        </div>
        <div class="button-container">
            <button class="button button-on" onclick="toggleGreenLed(true)">Green LED On</button>
            <button class="button button-off" onclick="toggleGreenLed(false)">Green LED Off</button>
            <button class="button button-on" onclick="toggleRedLed(true)">Red LED On</button>
            <button class="button button-off" onclick="toggleRedLed(false)">Red LED Off</button>
        </div>
    </div>

    <script>
        function updateSensorData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('current-datetime').textContent = data.currentDateTime;
                    document.getElementById('wifi-ssid').textContent = data.wifiSSID;
                    document.getElementById('ip-address').textContent = data.ipAddress;
                    document.getElementById('city-weather').textContent = data.city + ' Weather';
                    document.getElementById('weather-desc').textContent = data.weatherDescription;
                    document.getElementById('weather-temp-c').textContent = data.weatherTempC.toFixed(2) + ' °C';
                    document.getElementById('weather-temp-f').textContent = data.weatherTempF.toFixed(2) + ' °F';
                    document.getElementById('temp-c').textContent = data.temperatureC.toFixed(2) + ' °C';
                    document.getElementById('temp-f').textContent = data.temperatureF.toFixed(2) + ' °F';
                    document.getElementById('humidity').textContent = data.humidity.toFixed(2) + '%';
                    document.getElementById('motion').textContent = data.motion ? 'ON' : 'OFF';
                    document.getElementById('motion').className = 'sensor-value ' + (data.motion ? 'status-on' : 'status-off');
                    document.getElementById('tilt').textContent = data.tilt ? 'ON' : 'OFF';
                    document.getElementById('tilt').className = 'sensor-value ' + (data.tilt ? 'status-on' : 'status-off');
                    document.getElementById('flame').textContent = data.flame ? 'ON' : 'OFF';
                    document.getElementById('flame').className = 'sensor-value ' + (data.flame ? 'status-on' : 'status-off');
                    document.getElementById('city2-weather').textContent = data.city2 + ' Weather';
                    document.getElementById('weather-desc2').textContent = data.weatherDescription2;
                    document.getElementById('weather-temp-c2').textContent = data.weatherTempC2.toFixed(2) + ' °C';
                    document.getElementById('weather-temp-f2').textContent = data.weatherTempF2.toFixed(2) + ' °F';
                    document.getElementById('device-id').textContent = data.deviceID;
                    document.getElementById('aws-status').textContent = data.awsStatus ? 'Connected' : 'Disconnected';
                    document.getElementById('aws-status').className = data.awsStatus ? 'aws-connected' : 'aws-disconnected';
                    document.getElementById('next-weather-update').textContent = data.nextWeatherUpdate;
                })
                .catch(error => console.error('Error fetching sensor data:', error));
        }

        function updateTime() {
            fetch('/time')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('current-datetime').textContent = data.currentDateTime;
                })
                .catch(error => console.error('Error fetching time:', error));
        }

        function toggleGreenLed(state) {
            fetch(`/greenLed?state=${state ? 1 : 0}`);
        }

        function toggleRedLed(state) {
            fetch(`/redLed?state=${state ? 1 : 0}`);
        }

        // Update sensor data every 10 seconds
        setInterval(updateSensorData, 10000);

        // Update time every second
        setInterval(updateTime, 1000);

        // Initial update
        updateSensorData();
        updateTime();
    </script>
</body>
</html>
)rawliteral";

#endif // WEBPAGE_H
