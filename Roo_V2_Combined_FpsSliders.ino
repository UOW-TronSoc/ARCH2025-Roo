#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// ============================================================================
// Pin Assignments (unchanged)
// ============================================================================
const int MOTOR1_PWM = 25;  // PWM1 (Blue)
const int MOTOR1_DIR = 26;  // DIR1 (Purple)
const int MOTOR2_PWM = 27;  // PWM2 (Yellow)
const int MOTOR2_DIR = 14;  // DIR2 (Green)

const int IMU_SDA = 21;    // SDA for GY85
const int IMU_SCL = 22;    // SCL for GY85
#define ADXL345_ADDR 0x53  // ADXL345 I2C address

const int PIN_RESET_GIMBAL_CAM = 18;  // Gimbal camera reset (transistor)
const int PIN_RESET_STATIC_CAM = 19;  // Static camera reset (transistor)

const int SERVO_VERTICAL_PIN = 17;    // Vertical servo signal
const int SERVO_HORIZONTAL_PIN = 16;  // Horizontal servo signal

const int LED_PIN = 2;  // Built-in LED

unsigned long lastServoUpdate = 0;
const unsigned long servoInterval = 30; // Update servo every 30ms

// ============================================================================
// Software PWM Settings (unchanged)
// ============================================================================
const unsigned long pwmPeriodMicros = 20000;  // PWM period in microseconds

// ============================================================================
// Global Variables for Motor Speed (unchanged)
// ============================================================================
int desiredSpeedMotor1 = 0;
int desiredSpeedMotor2 = 0;

// ============================================================================
// Other Global Variables (unchanged)
// ============================================================================
float g_pitch = 0.0f;
float g_roll = 0.0f;
int g_signal = 0;
int g_power = 1;
int g_connected = 1;
int g_attached = 1;
int g_speed = 50;  // Speed from web slider (0-100%)

bool g_forward = false;
bool g_reverse = false;
bool g_left = false;
bool g_right = false;
bool g_stop = false;
bool g_gimbalUp = false;
bool g_gimbalDown = false;
bool g_gimbalLeft = false;
bool g_gimbalRight = false;

// Servo objects for gimbal control (unchanged)
Servo servoVertical;
Servo servoHorizontal;
int servoVerticalPos = 145;    // Default vertical position (centered, 0°)
int servoHorizontalPos = 90;   // Default horizontal position (0°)

// ============================================================================
// Web server instance (unchanged)
// ============================================================================
WebServer server(80);

// ============================================================================
// Modified Embedded HTML Page using a raw string literal
// ============================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>UOW Rover: Roo Control Panel</title>
  <style>
    body { font-family: Arial, sans-serif; background-color: #282c34; color: white; margin: 0; padding: 0; display: flex; flex-direction: column; height: 100vh; }
    .container { display: flex; flex-direction: column; flex: 1; }
    /* Row 1: Video Feeds */
    .video-feed { display: flex; flex-direction: row; justify-content: space-around; align-items: center; width: 100%; padding: 20px; box-sizing: border-box; }
    .video-container { width: 32%; transform: scaleY(0.9); transform-origin: top; }
    .video-title { text-align: center; margin-bottom: 10px; }
    img { width: 100%; height: auto; border: 2px solid #ccc; border-radius: 10px; }
    /* Row 2: Top Controls Row (Pitch, Level, Speed, etc.) */
    .controls-row { display: flex; flex-direction: row; justify-content: space-around; align-items: center; padding: 20px; box-sizing: border-box; gap: 20px; flex-wrap: nowrap; }
    /* Pitch & Roll Group with Gimbal Indicators added */
    .pitch-roll-group { display: flex; flex-direction: row; align-items: center; gap: 20px; min-width: 600px; }
    /* New: Gimbal Indicator Group (placed to the left of pitch indicator) */
    .gimbal-indicator-group { display: flex; flex-direction: row; gap: 10px; }
    .gimbal-indicator-group .indicator-box {
      width: 100px;
      height: 100px;
      background-color: #61dafb;
      position: relative;
      border-radius: 5px;
    }
    .gimbal-indicator-group .indicator-title {
      text-align: center;
      font-size: 12px;
      margin-bottom: 2px;
    }
    /* Pitch and Roll Indicators */
    .pitch-indicator, .roll-indicator { text-align: center; }
    .pitch, .roll { width: 200px; height: 20px; background-color: #444; position: relative; border-radius: 10px; overflow: hidden; }
    .pitch::before, .roll::before { content: ''; position: absolute; width: 100%; height: 100%; background-color: #61dafb; transform-origin: center; transition: transform 0.3s; }
    .numeric-value { position: relative; z-index: 2; }
    /* Level Indicators */
    .level-group { display: flex; flex-direction: row; gap: 20px; min-width: 300px; align-items: flex-end; }
    .level-indicator { text-align: center; }
    .level-wrapper { display: flex; align-items: center; justify-content: center; }
    .level-labels { margin-right: 5px; text-align: right; font-size: 12px; display: flex; flex-direction: column; justify-content: space-between; height: 150px; }
    .level-bar { width: 30px; height: 150px; background-color: #444; border: 1px solid #ccc; position: relative; }
    #signal-level .fill { background-color: #61dafb; width: 100%; height: 0; position: absolute; bottom: 0; }
    .motor-level { position: relative; }
    .motor-level .fill { background-color: #61dafb; width: 100%; position: absolute; }
    .motor-level::before { content: ''; position: absolute; top: 50%; left: 0; width: 100%; height: 1px; background-color: #fff; }
    /* Speed Slider */
    .speed-group { display: flex; flex-direction: column; align-items: center; min-width: 120px; }
    .speed-slider-container { display: flex; flex-direction: column; align-items: center; }
    .speed-slider-container label { margin-bottom: 10px; }
    .speed-slider-container input[type=range] { -webkit-appearance: none; -moz-appearance: none; appearance: none; width: 200px; height: 20px; margin: 20px 0; }
    .speed-slider-container input[type=range]::-webkit-slider-runnable-track { height: 8px; background: #444; border: 1px solid #ccc; border-radius: 5px; }
    .speed-slider-container input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; height: 20px; width: 20px; background: #61dafb; border: none; border-radius: 50%; margin-top: -6px; }
    /* Toggle Indicators */
    .toggle-group { display: grid; grid-template-columns: 1fr 1fr; grid-gap: 10px; min-width: 120px; }
    .toggle-indicator, .stop-button { height: 50px; width: 100%; box-sizing: border-box; font-size: 16px; border-radius: 10px; cursor: pointer; transition: background-color 0.2s; text-align: center; line-height: 50px; background-color: #61dafb; color: #282c34; font-weight: bold; }
    .toggle-indicator.off { background-color: #555; color: #ccc; }
    .stop-button { background-color: #f23430; color: white; border: none; }
    .stop-button:hover { background-color: #d12c2b; }
    /* Row 3: FPS Sliders and Gimbal Reset Buttons */
    .camera-fps-row { display: flex; justify-content: space-around; align-items: center; padding: 20px; }
    .camera-fps-control { text-align: center; }
    .camera-fps-control label { display: block; margin-bottom: 6px; }
    .camera-fps-control input[type=range] {
      -webkit-appearance: none;
      -moz-appearance: none;
      appearance: none;
      width: 200px;
      height: 10px;
      margin: 6px 0;
    }
    .camera-fps-control input[type=range]::-webkit-slider-runnable-track {
      height: 8px;
      background: #444;
      border: 1px solid #ccc;
      border-radius: 5px;
    }
    .camera-fps-control input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      height: 18px;
      width: 18px;
      background: #61dafb;
      border: none;
      border-radius: 50%;
      margin-top: -5px;
    }
    /* Row 4: Controls Buttons Row */
    .controls-buttons-row { 
      display: flex; 
      flex-direction: column; 
      align-items: center; 
      padding: 20px; 
    }
    .controls-buttons-row .group-title { 
      margin-bottom: 10px; 
      font-size: 18px; 
    }
    .controls-buttons-row .control-buttons { 
      display: flex; 
      flex-direction: row; 
      gap: 10px; 
    }
    .control-buttons button { 
      padding: 10px; 
      font-size: 16px; 
      background-color: #61dafb; 
      border: none; 
      border-radius: 5px; 
      cursor: pointer; 
      transition: background-color 0.2s; 
    }
    .control-buttons button:hover { background-color: #21a1f1; }
    button.active { background-color: #f23430 !important; }
  </style>
</head>
<body>
  <div class="container">
    <!-- Row 1: Video Feeds -->
    <div class="video-feed">
      <div class="video-container">
        <div class="video-title">Gimbal</div>
        <img id="feed1" src="" />
      </div>
      <div class="video-container">
        <div class="video-title">Front</div>
        <img id="feed2" src="" />
      </div>
      <div class="video-container">
        <div class="video-title">Back</div>
        <img id="feed3" src="" />
      </div>
    </div>

    <!-- Row 2: Top Controls Row (Pitch, Level, Speed, etc.) -->
    <div class="controls-row">
      <!-- New: Gimbal Position Indicators Group (moved to row 2, left of pitch indicator) -->
      <div class="gimbal-indicator-group">
        <div class="indicator-box">
          <div class="indicator-title">Vertical</div>
          <svg width="100" height="100">
            <!-- Default: a horizontal line from the center (50,50) to the left (0,50) -->
            <line id="vertical-indicator-line" x1="50" y1="50" x2="0" y2="50" stroke="grey" stroke-width="4" transform-origin="50 50"/>
          </svg>
        </div>
        <div class="indicator-box">
          <div class="indicator-title">Horizontal</div>
          <svg width="100" height="100">
            <!-- Default: a vertical line from the center (50,50) to the top (50,0) -->
            <line id="horizontal-indicator-line" x1="50" y1="50" x2="50" y2="0" stroke="grey" stroke-width="4" transform-origin="50 50"/>
          </svg>
        </div>
      </div>
      <!-- Pitch & Roll Indicators -->
      <div class="pitch-indicator">
        <h3 class="indicator-label">(F) Pitch (B)</h3>
        <div class="pitch" id="pitch-bar"></div>
        <div class="numeric-value" id="pitch-value">0°</div>
      </div>
      <div class="roll-indicator">
        <h3 class="indicator-label">(L) Roll (R)</h3>
        <div class="roll" id="roll-bar"></div>
        <div class="numeric-value" id="roll-value">0°</div>
      </div>
      <!-- Other groups remain unchanged -->
      <div class="level-group">
        <div class="level-indicator">
          <div class="level-wrapper">
            <div class="level-labels">
              <div>1000<br>ms</div>
              <div>500<br>ms</div>
              <div>0<br>ms</div>
            </div>
            <div class="level-bar" id="signal-level"><div class="fill"></div></div>
          </div>
          <div class="level-value" id="signal-value">0 ms</div>
          <div class="level-title">Signal</div>
        </div>
        <div class="level-indicator">
          <div class="level-wrapper">
            <div class="level-labels">
              <div>0.56<br>m/s</div>
              <div>0<br>m/s</div>
              <div>-0.56<br>m/s</div>
            </div>
            <div class="level-bar motor-level" id="port-level"><div class="fill"></div></div>
          </div>
          <div class="level-value" id="port-value">0 m/s</div>
          <div class="level-title">Motor A</div>
        </div>
        <div class="level-indicator">
          <div class="level-wrapper">
            <div class="level-labels">
              <div>0.56<br>m/s</div>
              <div>0<br>m/s</div>
              <div>-0.56<br>m/s</div>
            </div>
            <div class="level-bar motor-level" id="starboard-level"><div class="fill"></div></div>
          </div>
          <div class="level-value" id="starboard-value">0 m/s</div>
          <div class="level-title">Motor B</div>
        </div>
      </div>
      <div class="speed-group">
        <div class="group-title">Speed</div>
        <div class="speed-slider-container">
          <label id="speed-label">50%</label>
          <input type="range" id="speed-slider" min="0" max="100" step="25" value="50">
        </div>
      </div>
      <div class="toggle-group">
        <div class="toggle-indicator" id="power-indicator">Power</div>
        <div class="toggle-indicator off" id="connected-indicator">Connected</div>
        <div class="toggle-indicator" id="attached-indicator">Attached</div>
        <button class="stop-button" id="stop-button">STOP</button>
      </div>
    </div>

    <!-- Row 3: FPS Sliders and Gimbal Reset Buttons -->
    <div class="camera-fps-row">
      <div class="camera-fps-control">
        <label for="feed1-fps-slider">Gimbal FPS</label>
        <input type="range" id="feed1-fps-slider" min="1" max="20" value="4" step="1">
        <div id="feed1-fps-label">4 fps</div>
      </div>
      <div class="camera-fps-control">
        <label for="feed2-fps-slider">Front FPS</label>
        <input type="range" id="feed2-fps-slider" min="1" max="20" value="4" step="1">
        <div id="feed2-fps-label">4 fps</div>
      </div>
      <div class="camera-fps-control">
        <label for="feed3-fps-slider">Back FPS</label>
        <input type="range" id="feed3-fps-slider" min="1" max="20" value="4" step="1">
        <div id="feed3-fps-label">4 fps</div>
      </div>
      <div class="control-buttons" style="display: flex; flex-direction: column;">
        <button id="centre-vertical">Centre Vertical</button>
        <button id="centre-horizontal">Centre Horizontal</button>
      </div>
    </div>

    <!-- Row 4: Controls Buttons Row -->
    <div class="controls-buttons-row">
      <div class="group-title">Controls</div>
      <div class="control-buttons">
          <button id="forward">Forward (W)</button>
          <button id="reverse">Reverse (S)</button>
          <button id="left">Left (A)</button>
          <button id="right">Right (D)</button>
          <button id="gimbal-up">Up (Arrow Up)</button>
          <button id="gimbal-down">Down (Arrow Down)</button>
          <button id="gimbal-left">Left (Arrow Left)</button>
          <button id="gimbal-right">Right (Arrow Right)</button>
      </div>
    </div>

    <div class="heartbeat-log" id="heartbeat-log"></div>
  </div>
  <script>
    window.addEventListener('load', function() {
      // ----------------------------------------------------------------------
      // (A) Camera feed update functions
      // ----------------------------------------------------------------------
      function updateFeed1() {
        document.getElementById('feed1').src = 'http://192.168.10.211/capture?t=' + new Date().getTime();
      }
      function updateFeed2() {
        document.getElementById('feed2').src = 'http://192.168.10.212/capture?t=' + new Date().getTime();
      }
      function updateFeed3() {
        document.getElementById('feed3').src = 'http://192.168.10.213/capture?t=' + new Date().getTime();
      }

      // ----------------------------------------------------------------------
      // (B) FPS setters for each feed
      // ----------------------------------------------------------------------
      let feed1Timer = null;
      let feed2Timer = null;
      let feed3Timer = null;

      function setFeed1Fps(fps) {
        if (feed1Timer) clearInterval(feed1Timer);
        const intervalMs = 1000 / fps;
        feed1Timer = setInterval(updateFeed1, intervalMs);
        document.getElementById('feed1-fps-label').textContent = fps + ' fps';
      }
      function setFeed2Fps(fps) {
        if (feed2Timer) clearInterval(feed2Timer);
        const intervalMs = 1000 / fps;
        feed2Timer = setInterval(updateFeed2, intervalMs);
        document.getElementById('feed2-fps-label').textContent = fps + ' fps';
      }
      function setFeed3Fps(fps) {
        if (feed3Timer) clearInterval(feed3Timer);
        const intervalMs = 1000 / fps;
        feed3Timer = setInterval(updateFeed3, intervalMs);
        document.getElementById('feed3-fps-label').textContent = fps + ' fps';
      }

      // ----------------------------------------------------------------------
      // (C) Initialize default FPS
      // ----------------------------------------------------------------------
      setFeed1Fps(4);
      setFeed2Fps(4);
      setFeed3Fps(4);

      // ----------------------------------------------------------------------
      // (D) Slider handlers for FPS adjustment
      // ----------------------------------------------------------------------
      const feed1Slider = document.getElementById('feed1-fps-slider');
      feed1Slider.addEventListener('input', function() {
        setFeed1Fps(parseInt(this.value));
      });
      const feed2Slider = document.getElementById('feed2-fps-slider');
      feed2Slider.addEventListener('input', function() {
        setFeed2Fps(parseInt(this.value));
      });
      const feed3Slider = document.getElementById('feed3-fps-slider');
      feed3Slider.addEventListener('input', function() {
        setFeed3Fps(parseInt(this.value));
      });

      // ----------------------------------------------------------------------
      // Existing dynamic functionality
      // ----------------------------------------------------------------------
      let lastStatusTime = performance.now();
      let sliderActive = false;

      function sendCommand(cmd, state) {
        console.log('Command sent:', cmd, state);
        fetch('http://' + location.host + '/command?cmd=' + encodeURIComponent(cmd) + '&state=' + state)
          .then(response => response.text())
          .then(data => console.log('ESP32 responded:', data))
          .catch(error => console.error('Error sending command:', error));
      }

      function fetchStatus() {
        let startTime = performance.now();
        fetch('http://' + location.host + '/status')
          .then(response => response.json())
          .then(data => {
            lastStatusTime = performance.now();
            let endTime = performance.now();
            let latency = Math.round(endTime - startTime);
            document.getElementById('signal-value').textContent = latency + ' ms';
            let signalFill = document.querySelector('#signal-level .fill');
            let signalPercent = Math.min(100, (latency / 1000) * 100);
            signalFill.style.height = signalPercent + '%';

            document.getElementById('pitch-value').textContent = data.pitch + '°';
            document.getElementById('roll-value').textContent = data.roll + '°';
            document.getElementById('pitch-bar').style.transform = 'rotate(' + data.pitch + 'deg)';
            document.getElementById('roll-bar').style.transform = 'rotate(' + data.roll + 'deg)';

            document.getElementById('port-value').textContent = data.port.toFixed(2) + ' m/s';
            let portFill = document.querySelector('#port-level .fill');
            let portPercent = (Math.abs(data.port) / 0.56) * 50;
            if (data.port >= 0) {
              portFill.style.bottom = '50%';
              portFill.style.top = 'auto';
              portFill.style.height = portPercent + '%';
            } else {
              portFill.style.top = '50%';
              portFill.style.bottom = 'auto';
              portFill.style.height = portPercent + '%';
            }

            document.getElementById('starboard-value').textContent = data.starboard.toFixed(2) + ' m/s';
            let starboardFill = document.querySelector('#starboard-level .fill');
            let starboardPercent = (Math.abs(data.starboard) / 0.56) * 50;
            if (data.starboard >= 0) {
              starboardFill.style.bottom = '50%';
              starboardFill.style.top = 'auto';
              starboardFill.style.height = starboardPercent + '%';
            } else {
              starboardFill.style.top = '50%';
              starboardFill.style.bottom = 'auto';
              starboardFill.style.height = starboardPercent + '%';
            }

            if (!sliderActive) {
              document.getElementById('speed-label').textContent = 'Speed: ' + data.speed + '%';
            }
            
            // Update gimbal position indicators.
            // Using defaults: vertical centered at 145 and horizontal centered at 90.
            var verticalDiff = data.vertical - 145;
            var verticalAngle = -verticalDiff; // Reversed mapping: increasing vertical value gives negative angle.
            var horizontalDiff = data.horizontal - 90;
            var horizontalAngle = -horizontalDiff; // Reversed mapping.
            horizontalAngle = Math.max(-90, Math.min(90, horizontalAngle));
            
            document.getElementById('vertical-indicator-line').style.transform = 'rotate(' + verticalAngle + 'deg)';
            document.getElementById('horizontal-indicator-line').style.transform = 'rotate(' + horizontalAngle + 'deg)';
          })
          .catch(error => console.error('Error fetching status:', error));
      }

      const speedSlider = document.getElementById('speed-slider');
      speedSlider.addEventListener('input', function() {
        sliderActive = true;
        let val = this.value;
        document.getElementById('speed-label').textContent = 'Speed: ' + val + '%';
        sendCommand('speed', val);
      });
      speedSlider.addEventListener('change', function() {
        sliderActive = false;
      });

      setInterval(fetchStatus, 200);
      setInterval(function() {
        if (performance.now() - lastStatusTime > 400) {
          let connectedIndicator = document.getElementById('connected-indicator');
          connectedIndicator.classList.add('off');
        } else {
          let connectedIndicator = document.getElementById('connected-indicator');
          connectedIndicator.classList.remove('off');
        }
      }, 200);

      let keysPressed = {};
      const keyMapping = {
        'w': 'forward',
        'a': 'left',
        's': 'reverse',
        'd': 'right',
        'arrowup': 'gimbal-up',
        'arrowdown': 'gimbal-down',
        'arrowleft': 'gimbal-left',
        'arrowright': 'gimbal-right',
        'o': 'stop'
      };
      document.addEventListener('keydown', function(e) {
          if (['arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(e.key.toLowerCase())) {
               e.preventDefault();
          }
        let key = e.key.toLowerCase();
        if (keysPressed[key]) return;
        keysPressed[key] = true;
        const btnId = keyMapping[key];
        if (btnId) {
          let btn = document.getElementById(btnId);
          if (btn) btn.classList.add('active');
          sendCommand(btnId, 1);
        }
      });
      document.addEventListener('keyup', function(e) {
        let key = e.key.toLowerCase();
        if (!keysPressed[key]) return;
        delete keysPressed[key];
        const btnId = keyMapping[key];
        if (btnId) {
          let btn = document.getElementById(btnId);
          if (btn) btn.classList.remove('active');
          sendCommand(btnId, 0);
        }
      });

      document.querySelectorAll('.control-buttons button').forEach(btn => {
        btn.addEventListener('mousedown', function() {
          btn.classList.add('active');
          sendCommand(btn.id, 1);
        });
        btn.addEventListener('mouseup', function() {
          btn.classList.remove('active');
          sendCommand(btn.id, 0);
        });
        btn.addEventListener('mouseleave', function() {
          btn.classList.remove('active');
          sendCommand(btn.id, 0);
        });
      });

      // Event listeners for the Gimbal Reset Buttons
      document.getElementById('centre-vertical').addEventListener('mousedown', function() {
          sendCommand('centre-vertical', 1);
      });
      document.getElementById('centre-vertical').addEventListener('mouseup', function() {
          sendCommand('centre-vertical', 0);
      });
      document.getElementById('centre-vertical').addEventListener('mouseleave', function() {
          sendCommand('centre-vertical', 0);
      });
      document.getElementById('centre-horizontal').addEventListener('mousedown', function() {
          sendCommand('centre-horizontal', 1);
      });
      document.getElementById('centre-horizontal').addEventListener('mouseup', function() {
          sendCommand('centre-horizontal', 0);
      });
      document.getElementById('centre-horizontal').addEventListener('mouseleave', function() {
          sendCommand('centre-horizontal', 0);
      });
    });
  </script>
</body>
</html>
)rawliteral";

// ============================================================================
// Forward Declarations (unchanged)
// ============================================================================
void handleRoot();
void handleStatus();
void handleCommand();
void initIMU();
void updateIMU();
void processSerialInput();
void setMotorSpeed(int motor, int speed);
void updateSoftwarePWM();

void setMotorSpeed(int motor, int speed) {
  int duty = abs(speed);
  if (duty > 255) duty = 255;
  if (motor == 1) {
    if (speed > 0)
      digitalWrite(MOTOR1_DIR, HIGH);
    else if (speed < 0)
      digitalWrite(MOTOR1_DIR, LOW);
    else
      digitalWrite(MOTOR1_DIR, LOW);
    desiredSpeedMotor1 = duty;
  } else {
    if (speed > 0)
      digitalWrite(MOTOR2_DIR, HIGH);
    else if (speed < 0)
      digitalWrite(MOTOR2_DIR, LOW);
    else
      digitalWrite(MOTOR2_DIR, LOW);
    desiredSpeedMotor2 = duty;
  }
}

void updateSoftwarePWM() {
  unsigned long now = micros();
  unsigned long phase = now % pwmPeriodMicros;
  unsigned long duty1 = map(desiredSpeedMotor1, 0, 255, 0, pwmPeriodMicros);
  if (phase < duty1)
    digitalWrite(MOTOR1_PWM, HIGH);
  else
    digitalWrite(MOTOR1_PWM, LOW);
  unsigned long duty2 = map(desiredSpeedMotor2, 0, 255, 0, pwmPeriodMicros);
  if (phase < duty2)
    digitalWrite(MOTOR2_PWM, HIGH);
  else
    digitalWrite(MOTOR2_PWM, LOW);
}

void initIMU() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x2D);  // POWER_CTL register
  Wire.write(0x08);  // Measurement mode
  Wire.endTransmission();
  delay(10);
}

void updateIMU() {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x32);  // DATAX0 register
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDR, 6, true);
  if (Wire.available() < 6) return;
  int16_t x = Wire.read() | (Wire.read() << 8);
  int16_t y = Wire.read() | (Wire.read() << 8);
  int16_t z = Wire.read() | (Wire.read() << 8);
  float xs = x * 0.0039f;
  float ys = y * 0.0039f;
  float zs = z * 0.0039f;
  g_roll = atan2(xs, sqrtf(ys * ys + zs * zs)) * (180.0f / 3.14159f);
  g_pitch = -atan2(ys, zs) * (180.0f / 3.14159f) + 180;
}

void processSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      g_pitch = input.toFloat();
      Serial.print("g_pitch updated from Serial to: ");
      Serial.println(g_pitch);
    }
  }
}

void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleStatus() {
  updateIMU();
  float maxSpeedVal = 0.56f * (g_speed / 100.0f);
  float halfSpeedVal = maxSpeedVal * 0.5f;
  bool f = g_forward;
  bool rv = g_reverse;
  bool l = g_left;
  bool r = g_right;
  float motorA_UI = 0.0f;
  float motorB_UI = 0.0f;
  if (f && l) {
    motorA_UI = halfSpeedVal;
    motorB_UI = maxSpeedVal;
  } else if (f && r) {
    motorA_UI = maxSpeedVal;
    motorB_UI = halfSpeedVal;
  } else if (rv && l) {
    motorA_UI = -halfSpeedVal;
    motorB_UI = -maxSpeedVal;
  } else if (rv && r) {
    motorA_UI = -maxSpeedVal;
    motorB_UI = -halfSpeedVal;
  } else if (f) {
    motorA_UI = maxSpeedVal;
    motorB_UI = maxSpeedVal;
  } else if (rv) {
    motorA_UI = -maxSpeedVal;
    motorB_UI = -maxSpeedVal;
  } else if (l) {
    motorA_UI = -maxSpeedVal;
    motorB_UI = maxSpeedVal;
  } else if (r) {
    motorA_UI = maxSpeedVal;
    motorB_UI = -maxSpeedVal;
  } else {
    motorA_UI = 0.0f;
    motorB_UI = 0.0f;
  }
  String json = "{";
  json += "\"pitch\":" + String(g_pitch, 2) + ",";
  json += "\"roll\":" + String(g_roll, 2) + ",";
  json += "\"signal\":" + String(g_signal) + ",";
  json += "\"port\":" + String(motorA_UI, 2) + ",";
  json += "\"starboard\":" + String(motorB_UI, 2) + ",";
  json += "\"power\":" + String(g_power) + ",";
  json += "\"connected\":" + String(g_connected) + ",";
  json += "\"attached\":" + String(g_attached) + ",";
  json += "\"speed\":" + String(g_speed) + ",";
  json += "\"forward\":" + String(g_forward ? 1 : 0) + ",";
  json += "\"reverse\":" + String(g_reverse ? 1 : 0) + ",";
  json += "\"left\":" + String(g_left ? 1 : 0) + ",";
  json += "\"right\":" + String(g_right ? 1 : 0) + ",";
  json += "\"stop\":" + String(g_stop ? 1 : 0) + ",";
  json += "\"gimbalUp\":" + String(g_gimbalUp ? 1 : 0) + ",";
  json += "\"gimbalDown\":" + String(g_gimbalDown ? 1 : 0) + ",";
  json += "\"gimbalLeft\":" + String(g_gimbalLeft ? 1 : 0) + ",";
  json += "\"gimbalRight\":" + String(g_gimbalRight ? 1 : 0) + ",";
  json += "\"vertical\":" + String(servoVerticalPos) + ",";
  json += "\"horizontal\":" + String(servoHorizontalPos);
  json += "}";
  server.send(200, "application/json", json);
}

void handleCommand() {
  if (server.hasArg("cmd") && server.hasArg("state")) {
    String cmd = server.arg("cmd");
    int state = server.arg("state").toInt();
    Serial.print("Command received: ");
    Serial.print(cmd);
    Serial.print(" | State: ");
    Serial.println(state);
    if (cmd == "forward") {
      g_forward = (state == 1);
    } else if (cmd == "reverse") {
      g_reverse = (state == 1);
    } else if (cmd == "left") {
      g_left = (state == 1);
    } else if (cmd == "right") {
      g_right = (state == 1);
    } else if (cmd == "stop") {
      g_stop = (state == 1);
    } else if (cmd == "gimbal-up") {
      g_gimbalUp = (state == 1);
    } else if (cmd == "gimbal-down") {
      g_gimbalDown = (state == 1);
    } else if (cmd == "gimbal-left") {
      g_gimbalLeft = (state == 1);
    } else if (cmd == "gimbal-right") {
      g_gimbalRight = (state == 1);
    } else if (cmd == "speed") {
      int newSpeed = state;
      if (newSpeed == 0) return;
      g_speed = newSpeed;
    } else if (cmd == "rst-stat-cam") {
      digitalWrite(PIN_RESET_STATIC_CAM, (state == 1) ? HIGH : LOW);
    } else if (cmd == "rst-gim-cam") {
      digitalWrite(PIN_RESET_GIMBAL_CAM, (state == 1) ? HIGH : LOW);
    } else if (cmd == "na") {
    } else if (cmd == "centre-vertical") {
      if (state == 1) {
        g_gimbalUp = false;
        g_gimbalDown = false;
        servoVerticalPos = 145;
        servoVertical.write(servoVerticalPos);
      }
    } else if (cmd == "centre-horizontal") {
      if (state == 1) {
        g_gimbalLeft = false;
        g_gimbalRight = false;
        servoHorizontalPos = 90;
        servoHorizontal.write(servoHorizontalPos);
      }
    }
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));
  initIMU();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Setup ESP32-CAM reset pins
  pinMode(PIN_RESET_GIMBAL_CAM, OUTPUT);
  digitalWrite(PIN_RESET_GIMBAL_CAM, LOW);
  pinMode(PIN_RESET_STATIC_CAM, OUTPUT);
  digitalWrite(PIN_RESET_STATIC_CAM, LOW);
  // Setup motor driver direction pins
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  // Setup software PWM pins as outputs
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  digitalWrite(MOTOR1_PWM, LOW);
  digitalWrite(MOTOR2_PWM, LOW);
  desiredSpeedMotor1 = 0;
  desiredSpeedMotor2 = 0;
  // Attach servo motors for gimbal control
  servoVertical.attach(SERVO_VERTICAL_PIN);
  servoHorizontal.attach(SERVO_HORIZONTAL_PIN);
  servoVertical.write(servoVerticalPos);
  servoHorizontal.write(servoHorizontalPos);
  // --- WiFi Setup for Static IP via Router Mode ---
  WiFi.mode(WIFI_STA);
  IPAddress local_IP(192, 168, 10, 210);
  IPAddress gateway(192, 168, 10, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial.print("Connecting to WiFi network: ");
  Serial.println("UOWRoverTeam-RooAP");
  WiFi.begin("UOWRoverTeam-RooAP", "RooRoverAP22");
  esp_wifi_set_ps(WIFI_PS_NONE);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("Device IP address: ");
  Serial.println(WiFi.localIP());
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11N);
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/command", handleCommand);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  processSerialInput();
  updateSoftwarePWM();
  int currentSpeedPWM = map(g_speed, 0, 100, 0, 255);
  if (g_stop) {
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
  } else if (g_forward && g_left) {
    setMotorSpeed(1, currentSpeedPWM / 2);
    setMotorSpeed(2, currentSpeedPWM);
  } else if (g_forward && g_right) {
    setMotorSpeed(1, currentSpeedPWM);
    setMotorSpeed(2, currentSpeedPWM / 2);
  } else if (g_reverse && g_left) {
    setMotorSpeed(1, -currentSpeedPWM);
    setMotorSpeed(2, -currentSpeedPWM / 2);
  } else if (g_reverse && g_right) {
    setMotorSpeed(1, -currentSpeedPWM / 2);
    setMotorSpeed(2, -currentSpeedPWM);
  } else if (g_forward) {
    setMotorSpeed(1, currentSpeedPWM);
    setMotorSpeed(2, currentSpeedPWM);
  } else if (g_reverse) {
    setMotorSpeed(1, -currentSpeedPWM);
    setMotorSpeed(2, -currentSpeedPWM);
  } else if (g_left) {
    setMotorSpeed(1, -currentSpeedPWM / 1.5);
    setMotorSpeed(2, currentSpeedPWM / 1.5);
  } else if (g_right) {
    setMotorSpeed(1, currentSpeedPWM / 1.5);
    setMotorSpeed(2, -currentSpeedPWM / 1.5);
  } else {
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
  }
  unsigned long currentMillis = millis();
  if (currentMillis - lastServoUpdate >= servoInterval) {
    lastServoUpdate = currentMillis;
    if (g_gimbalUp && servoVerticalPos > 75) servoVerticalPos--;
    if (g_gimbalDown && servoVerticalPos < 155) servoVerticalPos++;
    if (g_gimbalRight && servoHorizontalPos > 0) servoHorizontalPos--;
    if (g_gimbalLeft && servoHorizontalPos < 180) servoHorizontalPos++;
    servoVertical.write(servoVerticalPos);
    servoHorizontal.write(servoHorizontalPos);
  }
}
