#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// ============================================================================
// Pin Assignments
// ============================================================================
const int MOTOR1_PWM = 25;   // PWM1 for Motor 1 (software PWM)
const int MOTOR1_DIR = 26;   // DIR1 for Motor 1
const int MOTOR2_PWM = 27;   // PWM2 for Motor 2 (software PWM)
const int MOTOR2_DIR = 14;   // DIR2 for Motor 2

const int IMU_SDA = 21;      // SDA for GY85
const int IMU_SCL = 22;      // SCL for GY85
#define ADXL345_ADDR 0x53   // ADXL345 I2C address

const int PIN_RESET_GIMBAL_CAM = 18;   // Gimbal camera reset (transistor)
const int PIN_RESET_STATIC_CAM = 19;   // Static camera reset (transistor)

const int SERVO_VERTICAL_PIN   = 16;  // Vertical servo signal
const int SERVO_HORIZONTAL_PIN = 17;  // Horizontal servo signal

const int LED_PIN = 2;       // Built-in LED

// ============================================================================
// Software PWM Settings
// ============================================================================
const unsigned long pwmPeriodMicros = 20000;  // PWM period in microseconds (1ms = 1kHz)

// ============================================================================
// Global Variables for Motor Speed
// ============================================================================
int desiredSpeedMotor1 = 0;
int desiredSpeedMotor2 = 0;

// ============================================================================
// Other Global Variables
// ============================================================================
float g_pitch = 0.0f;
float g_roll  = 0.0f;
int   g_signal    = 0;
int   g_power     = 1;
int   g_connected = 1;
int   g_attached  = 1;
int   g_speed     = 60;  // Speed from web slider (0-100%), default 60%

bool  g_forward    = false;
bool  g_reverse    = false;
bool  g_left       = false;
bool  g_right      = false;
bool  g_stop       = false;
bool  g_gimbalUp   = false;
bool  g_gimbalDown = false;
bool  g_gimbalLeft = false;
bool  g_gimbalRight= false;

// Servo objects for gimbal control
Servo servoVertical;
Servo servoHorizontal;
int servoVerticalPos = 90;
int servoHorizontalPos = 90;

// Web server instance
WebServer server(80);

// ============================================================================
// New HTML Embedded Page (Full HTML code integrated)
// ============================================================================
const char index_html[] PROGMEM =
"<!DOCTYPE html>"
"<html lang=\"en\">"
"<head>"
"  <meta charset=\"UTF-8\" />"
"  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\" />"
"  <title>UOW Rover: Roo Control Panel</title>"
"  <style>"
"    body { font-family: Arial, sans-serif; background-color: #282c34; color: white; margin: 0; padding: 0; display: flex; flex-direction: column; height: 100vh; }"
"    .header { text-align: center; padding: 20px 0; background-color: #20232a; border-bottom: 2px solid #61dafb; }"
"    .container { display: flex; flex-direction: column; flex: 1; }"
"    /* Row 1: Video Feeds */"
"    .video-feed { display: flex; flex-direction: row; justify-content: space-around; align-items: center; width: 100%; padding: 20px; box-sizing: border-box; }"
"    .video-container { width: 48%; }"
"    .video-title { text-align: center; margin-bottom: 10px; }"
"    img { width: 100%; height: auto; border: 2px solid #ccc; border-radius: 10px; }"
"    /* Row 2: All Controls in one horizontal row */"
"    .controls-row { display: flex; flex-direction: row; justify-content: space-around; align-items: flex-start; padding: 20px; box-sizing: border-box; gap: 20px; flex-wrap: nowrap; }"
"    /* Group 1: Pitch & Roll Indicators */"
"    .pitch-roll-group { display: flex; flex-direction: column; gap: 20px; min-width: 220px; }"
"    .pitch, .roll { width: 200px; height: 20px; background-color: #444; position: relative; border-radius: 10px; overflow: hidden; }"
"    .pitch::before, .roll::before { content: ''; position: absolute; width: 100%; height: 100%; background-color: #61dafb; transform-origin: center; transition: transform 0.3s; }"
"    .pitch-indicator, .roll-indicator { text-align: center; }"
"    .pitch-indicator .indicator-label, .pitch-indicator .numeric-value, .roll-indicator .indicator-label, .roll-indicator .numeric-value { position: relative; z-index: 2; }"
"    /* Group 2: Level Indicators */"
"    .level-group { display: flex; flex-direction: row; gap: 20px; min-width: 300px; align-items: flex-end; }"
"    .level-indicator { text-align: center; }"
"    .level-wrapper { display: flex; align-items: center; justify-content: center; }"
"    .level-labels { margin-right: 5px; text-align: right; font-size: 12px; display: flex; flex-direction: column; justify-content: space-between; height: 150px; }"
"    .level-bar { width: 30px; height: 150px; background-color: #444; border: 1px solid #ccc; position: relative; }"
"    #signal-level .fill { background-color: #61dafb; width: 100%; height: 0; position: absolute; bottom: 0; }"
"    .motor-level { position: relative; }"
"    .motor-level .fill { background-color: #61dafb; width: 100%; position: absolute; }"
"    .motor-level::before { content: ''; position: absolute; top: 50%; left: 0; width: 100%; height: 1px; background-color: #fff; }"
"    /* Group 3: Speed Slider (Horizontal) */"
"    .speed-group { display: flex; flex-direction: column; align-items: center; min-width: 120px; }"
"    .speed-slider-container { display: flex; flex-direction: column; align-items: center; }"
"    .speed-slider-container label { margin-bottom: 10px; }"
"    .speed-slider-container input[type=range] { -webkit-appearance: none; -moz-appearance: none; appearance: none; width: 200px; height: 20px; margin: 20px 0; }"
"    .speed-slider-container input[type=range]::-webkit-slider-runnable-track { height: 8px; background: #444; border: 1px solid #ccc; border-radius: 5px; }"
"    .speed-slider-container input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; height: 20px; width: 20px; background: #61dafb; border: none; border-radius: 50%; margin-top: -6px; }"
"    /* Group 4: Toggle Indicators (Status) */"
"    .toggle-group { display: flex; flex-direction: column; gap: 10px; min-width: 120px; align-items: center; }"
"    .toggle-indicator, .stop-button { height: 50px; width: 100%; box-sizing: border-box; font-size: 16px; border-radius: 10px; cursor: pointer; transition: background-color 0.2s; text-align: center; line-height: 50px; background-color: #61dafb; color: #282c34; font-weight: bold; }"
"    .toggle-indicator.off { background-color: #555; color: #ccc; }"
"    .stop-button { background-color: #f23430; color: white; border: none; }"
"    .stop-button:hover { background-color: #d12c2b; }"
"    /* Group 5: Button Controls */"
"    .button-group { display: flex; flex-direction: column; gap: 20px; min-width: 220px; }"
"    .control-buttons { display: flex; flex-direction: column; gap: 10px; }"
"    .control-buttons button { width: 100%; padding: 10px; font-size: 16px; background-color: #61dafb; border: none; border-radius: 5px; cursor: pointer; transition: background-color 0.2s; }"
"    .control-buttons button:hover { background-color: #21a1f1; }"
"    .group-title { text-align: center; font-size: 18px; margin-bottom: 10px; }"
"    button.active { background-color: #f23430 !important; }"
"  </style>"
"</head>"
"<body>"
"  <div class=\"header\"><h1>UOW Rover: Roo Control Panel</h1></div>"
"  <div class=\"container\">"
"    <!-- Row 1: Camera Feeds -->"
"    <div class=\"video-feed\">"
"      <div class=\"video-container\">"
"        <div class=\"video-title\">Gimbal</div>"
"        <img id=\"feed1\" src=\"http://192.168.10.211/stream\" />"
"      </div>"
"      <div class=\"video-container\">"
"        <div class=\"video-title\">Front</div>"
"        <img id=\"feed2\" src=\"http://192.168.10.212/stream\" />"
"      </div>"
"    </div>"
"    <!-- Row 2: Controls Row -->"
"    <div class=\"controls-row\">"
"      <!-- Group 1: Pitch & Roll -->"
"      <div class=\"pitch-roll-group\">"
"        <div class=\"pitch-indicator\">"
"          <h3 class=\"indicator-label\">(F) Pitch (B)</h3>"
"          <div class=\"pitch\" id=\"pitch-bar\"></div>"
"          <div class=\"numeric-value\" id=\"pitch-value\">0°</div>"
"        </div>"
"        <div class=\"roll-indicator\">"
"          <h3 class=\"indicator-label\">(L) Roll (R)</h3>"
"          <div class=\"roll\" id=\"roll-bar\"></div>"
"          <div class=\"numeric-value\" id=\"roll-value\">0°</div>"
"        </div>"
"      </div>"
"      <!-- Group 2: Level Indicators -->"
"      <div class=\"level-group\">"
"        <div class=\"level-indicator\">"
"          <div class=\"level-wrapper\">"
"            <div class=\"level-labels\">"
"              <div>1000<br>ms</div>"
"              <div>500<br>ms</div>"
"              <div>0<br>ms</div>"
"            </div>"
"            <div class=\"level-bar\" id=\"signal-level\"><div class=\"fill\"></div></div>"
"          </div>"
"          <div class=\"level-value\" id=\"signal-value\">0 ms</div>"
"          <div class=\"level-title\">Signal</div>"
"        </div>"
"        <div class=\"level-indicator\">"
"          <div class=\"level-wrapper\">"
"            <div class=\"level-labels\">"
"              <div>0.56<br>m/s</div>"
"              <div>0<br>m/s</div>"
"              <div>-0.56<br>m/s</div>"
"            </div>"
"            <div class=\"level-bar motor-level\" id=\"port-level\"><div class=\"fill\"></div></div>"
"          </div>"
"          <div class=\"level-value\" id=\"port-value\">0 m/s</div>"
"          <div class=\"level-title\">Motor A</div>"
"        </div>"
"        <div class=\"level-indicator\">"
"          <div class=\"level-wrapper\">"
"            <div class=\"level-labels\">"
"              <div>0.56<br>m/s</div>"
"              <div>0<br>m/s</div>"
"              <div>-0.56<br>m/s</div>"
"            </div>"
"            <div class=\"level-bar motor-level\" id=\"starboard-level\"><div class=\"fill\"></div></div>"
"          </div>"
"          <div class=\"level-value\" id=\"starboard-value\">0 m/s</div>"
"          <div class=\"level-title\">Motor B</div>"
"        </div>"
"      </div>"
"      <!-- Group 3: Speed Slider (Horizontal) -->"
"      <div class=\"speed-group\">"
"        <div class=\"group-title\">Speed</div>"
"        <div class=\"speed-slider-container\">"
"          <label id=\"speed-label\">60%</label>"
"          <input type=\"range\" id=\"speed-slider\" min=\"0\" max=\"100\" step=\"25\" value=\"60\">"
"        </div>"
"      </div>"
"      <!-- Group 4: Status Indicators -->"
"      <div class=\"toggle-group\">"
"        <div class=\"group-title\">Status</div>"
"        <div class=\"toggle-indicator\" id=\"power-indicator\">Power</div>"
"        <div class=\"toggle-indicator off\" id=\"connected-indicator\">Connected</div>"
"        <div class=\"toggle-indicator\" id=\"attached-indicator\">Attached</div>"
"        <button class=\"stop-button\" id=\"stop-button\">STOP</button>"
"      </div>"
"      <!-- Group 5: Button Controls -->"
"      <div class=\"button-group\">"
"        <div class=\"group-title\">Controls</div>"
"        <div class=\"control-buttons drive-controls\">"
"          <button id=\"forward\">Forward (W)</button>"
"          <button id=\"reverse\">Reverse (S)</button>"
"          <button id=\"left\">Left (A)</button>"
"          <button id=\"right\">Right (D)</button>"
"        </div>"
"        <div class=\"control-buttons gimbal-controls\">"
"          <button id=\"gimbal-up\">Up (Arrow Up)</button>"
"          <button id=\"gimbal-down\">Down (Arrow Down)</button>"
"          <button id=\"gimbal-left\">Left (Arrow Left)</button>"
"          <button id=\"gimbal-right\">Right (Arrow Right)</button>"
"        </div>"
"      </div>"
"    </div>"
"  </div>"
"  <script>"
"    let lastStatusTime = performance.now();"
"    let sliderActive = false;"
"    function sendCommand(cmd, state) {"
"      console.log('Command sent:', cmd, state);"
"      fetch('http://' + location.host + '/command?cmd=' + encodeURIComponent(cmd) + '&state=' + state)"
"        .then(response => response.text())"
"        .then(data => console.log('ESP32 responded:', data))"
"        .catch(error => console.error('Error sending command:', error));"
"    }"
"    function fetchStatus() {"
"      let startTime = performance.now();"
"      fetch('http://' + location.host + '/status')"
"        .then(response => response.json())"
"        .then(data => {"
"          lastStatusTime = performance.now();"
"          let endTime = performance.now();"
"          let latency = Math.round(endTime - startTime);"
"          document.getElementById('signal-value').textContent = latency + ' ms';"
"          let signalFill = document.querySelector('#signal-level .fill');"
"          let signalPercent = Math.min(100, (latency / 1000) * 100);"
"          signalFill.style.height = signalPercent + '%';"
"          document.getElementById('pitch-value').textContent = data.pitch + '°';"
"          document.getElementById('roll-value').textContent = data.roll + '°';"
"          document.getElementById('pitch-bar').style.transform = 'rotate(' + data.pitch + 'deg)';"
"          document.getElementById('roll-bar').style.transform = 'rotate(' + data.roll + 'deg)';"
"          document.getElementById('port-value').textContent = data.port.toFixed(2) + ' m/s';"
"          let portFill = document.querySelector('#port-level .fill');"
"          let portPercent = (Math.abs(data.port) / 0.56) * 50;"
"          if (data.port >= 0) {"
"            portFill.style.bottom = '50%';"
"            portFill.style.top = 'auto';"
"            portFill.style.height = portPercent + '%';"
"          } else {"
"            portFill.style.top = '50%';"
"            portFill.style.bottom = 'auto';"
"            portFill.style.height = portPercent + '%';"
"          }"
"          document.getElementById('starboard-value').textContent = data.starboard.toFixed(2) + ' m/s';"
"          let starboardFill = document.querySelector('#starboard-level .fill');"
"          let starboardPercent = (Math.abs(data.starboard) / 0.56) * 50;"
"          if (data.starboard >= 0) {"
"            starboardFill.style.bottom = '50%';"
"            starboardFill.style.top = 'auto';"
"            starboardFill.style.height = starboardPercent + '%';"
"          } else {"
"            starboardFill.style.top = '50%';"
"            starboardFill.style.bottom = 'auto';"
"            starboardFill.style.height = starboardPercent + '%';"
"          }"
"          if (!sliderActive) {"
"            document.getElementById('speed-label').textContent = data.speed + '%';"
"          }"
"        })"
"        .catch(error => console.error('Error fetching status:', error));"
"    }"
"    const speedSlider = document.getElementById('speed-slider');"
"    speedSlider.addEventListener('input', function() {"
"      sliderActive = true;"
"      let val = this.value;"
"      document.getElementById('speed-label').textContent = val + '%';"
"      sendCommand('speed', val);"
"    });"
"    speedSlider.addEventListener('change', function() {"
"      sliderActive = false;"
"    });"
"    setInterval(fetchStatus, 200);"
"    setInterval(function() {"
"      if (performance.now() - lastStatusTime > 400) {"
"        let connectedIndicator = document.getElementById('connected-indicator');"
"        connectedIndicator.classList.add('off');"
"      } else {"
"        let connectedIndicator = document.getElementById('connected-indicator');"
"        connectedIndicator.classList.remove('off');"
"      }"
"    }, 200);"
"    let keysPressed = {};"
"    const keyMapping = {"
"      'w': 'forward',"
"      'a': 'left',"
"      's': 'reverse',"
"      'd': 'right',"
"      'arrowup': 'gimbal-up',"
"      'arrowdown': 'gimbal-down',"
"      'arrowleft': 'gimbal-left',"
"      'arrowright': 'gimbal-right',"
"      'o': 'stop'"
"    };"
"    document.addEventListener('keydown', function(e) {"
"      let key = e.key.toLowerCase();"
"      if (keysPressed[key]) return;"
"      keysPressed[key] = true;"
"      const btnId = keyMapping[key];"
"      if (btnId) {"
"        let btn = document.getElementById(btnId);"
"        if (btn) btn.classList.add('active');"
"        sendCommand(btnId, 1);"
"      }"
"    });"
"    document.addEventListener('keyup', function(e) {"
"      let key = e.key.toLowerCase();"
"      if (!keysPressed[key]) return;"
"      delete keysPressed[key];"
"      const btnId = keyMapping[key];"
"      if (btnId) {"
"        let btn = document.getElementById(btnId);"
"        if (btn) btn.classList.remove('active');"
"        sendCommand(btnId, 0);"
"      }"
"    });"
"    document.querySelectorAll('.control-buttons button').forEach(btn => {"
"      btn.addEventListener('mousedown', function() {"
"        btn.classList.add('active');"
"        sendCommand(btn.id, 1);"
"      });"
"      btn.addEventListener('mouseup', function() {"
"        btn.classList.remove('active');"
"        sendCommand(btn.id, 0);"
"      });"
"      btn.addEventListener('mouseleave', function() {"
"        btn.classList.remove('active');"
"        sendCommand(btn.id, 0);"
"      });"
"    });"
"    document.getElementById('rst-stat-cam').addEventListener('mousedown', function() {"
"      this.classList.add('active');"
"      sendCommand('rst-stat-cam', 1);"
"    });"
"    document.getElementById('rst-stat-cam').addEventListener('mouseup', function() {"
"      this.classList.remove('active');"
"      sendCommand('rst-stat-cam', 0);"
"    });"
"    document.getElementById('rst-gim-cam').addEventListener('mousedown', function() {"
"      this.classList.add('active');"
"      sendCommand('rst-gim-cam', 1);"
"    });"
"    document.getElementById('rst-gim-cam').addEventListener('mouseup', function() {"
"      this.classList.remove('active');"
"      sendCommand('rst-gim-cam', 0);"
"    });"
"    document.getElementById('na').addEventListener('mousedown', function() {"
"      this.classList.add('active');"
"      sendCommand('na', 1);"
"    });"
"    document.getElementById('na').addEventListener('mouseup', function() {"
"      this.classList.remove('active');"
"      sendCommand('na', 0);"
"    });"
"    const stopBtn = document.getElementById('stop-button');"
"    stopBtn.addEventListener('mousedown', function() {"
"      stopBtn.classList.add('active');"
"      sendCommand('stop', 1);"
"    });"
"    stopBtn.addEventListener('mouseup', function() {"
"      stopBtn.classList.remove('active');"
"      sendCommand('stop', 0);"
"    });"
"    stopBtn.addEventListener('mouseleave', function() {"
"      stopBtn.classList.remove('active');"
"      sendCommand('stop', 0);"
"    });"
"  </script>"
"</body>"
"</html>";


// ============================================================================
// Forward Declarations
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
  Wire.write(0x2D); // POWER_CTL register
  Wire.write(0x08); // Measurement mode
  Wire.endTransmission();
  delay(10);
}

void updateIMU() {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x32); // DATAX0 register
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
  if (f && l) { motorA_UI = halfSpeedVal; motorB_UI = maxSpeedVal; }
  else if (f && r) { motorA_UI = maxSpeedVal; motorB_UI = halfSpeedVal; }
  else if (rv && l) { motorA_UI = -halfSpeedVal; motorB_UI = -maxSpeedVal; }
  else if (rv && r) { motorA_UI = -maxSpeedVal; motorB_UI = -halfSpeedVal; }
  else if (f) { motorA_UI = maxSpeedVal; motorB_UI = maxSpeedVal; }
  else if (rv) { motorA_UI = -maxSpeedVal; motorB_UI = -maxSpeedVal; }
  else if (l) { motorA_UI = -maxSpeedVal; motorB_UI = maxSpeedVal; }
  else if (r) { motorA_UI = maxSpeedVal; motorB_UI = -maxSpeedVal; }
  else { motorA_UI = 0.0f; motorB_UI = 0.0f; }
  
  String json = "{";
  json += "\"pitch\":" + String(g_pitch, 2) + ",";
  json += "\"roll\":"  + String(g_roll, 2) + ",";
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
  json += "\"gimbalRight\":" + String(g_gimbalRight ? 1 : 0);
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
    
    if (cmd == "forward") { g_forward = (state == 1); }
    else if (cmd == "reverse") { g_reverse = (state == 1); }
    else if (cmd == "left") { g_left = (state == 1); }
    else if (cmd == "right") { g_right = (state == 1); }
    else if (cmd == "stop") { g_stop = (state == 1); }
    else if (cmd == "gimbal-up") { g_gimbalUp = (state == 1); }
    else if (cmd == "gimbal-down") { g_gimbalDown = (state == 1); }
    else if (cmd == "gimbal-left") { g_gimbalLeft = (state == 1); }
    else if (cmd == "gimbal-right") { g_gimbalRight = (state == 1); }
    else if (cmd == "speed") {
      int newSpeed = state;
      if (newSpeed == 0) return;
      g_speed = newSpeed;
    }
    else if (cmd == "rst-stat-cam") {
      digitalWrite(PIN_RESET_STATIC_CAM, (state == 1) ? HIGH : LOW);
    }
    else if (cmd == "rst-gim-cam") {
      digitalWrite(PIN_RESET_GIMBAL_CAM, (state == 1) ? HIGH : LOW);
    }
    else if (cmd == "na") { }
    
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
  
  Serial.print("Connecting to WiFi network: ");
  Serial.println("UOWRoverTeam-RooAP");
  WiFi.begin("UOWRoverTeam-RooAP", "RooRoverAP22");
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("Device IP address: ");
  Serial.println(WiFi.localIP());
  
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
    setMotorSpeed(1, -currentSpeedPWM);
    setMotorSpeed(2, -currentSpeedPWM / 2);
  } else if (g_forward && g_right) {
    setMotorSpeed(1, -currentSpeedPWM / 2);
    setMotorSpeed(2, -currentSpeedPWM);
  } else if (g_reverse && g_left) {
    setMotorSpeed(1, currentSpeedPWM / 2);
    setMotorSpeed(2, currentSpeedPWM);
  } else if (g_reverse && g_right) {
    setMotorSpeed(1, currentSpeedPWM);
    setMotorSpeed(2, currentSpeedPWM / 2);
  } else if (g_forward) {
    setMotorSpeed(1, -currentSpeedPWM);
    setMotorSpeed(2, -currentSpeedPWM);
  } else if (g_reverse) {
    setMotorSpeed(1, currentSpeedPWM);
    setMotorSpeed(2, currentSpeedPWM);
  } else if (g_left) {
    setMotorSpeed(1, -currentSpeedPWM / 4);
    setMotorSpeed(2, currentSpeedPWM / 4);
  } else if (g_right) {
    setMotorSpeed(1, currentSpeedPWM / 4);
    setMotorSpeed(2, -currentSpeedPWM / 4);
  } else {
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
  }
  
  if (g_gimbalUp && servoVerticalPos < 180) servoVerticalPos++;
  if (g_gimbalDown && servoVerticalPos > 0) servoVerticalPos--;
  if (g_gimbalLeft && servoHorizontalPos > 0) servoHorizontalPos--;
  if (g_gimbalRight && servoHorizontalPos < 180) servoHorizontalPos++;
  
  servoVertical.write(servoVerticalPos);
  servoHorizontal.write(servoHorizontalPos);
  
  delay(1);
}
