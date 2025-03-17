#include "esp_camera.h"
#include <WiFi.h>
#include "esp_wifi.h"  // Include to control WiFi power saving mode

// Replace with your network credentials
const char* ssid = "UOWRoverTeam-RooAP";
const char* password = "RooRoverAP22";

// Set your static IP configuration (adjust for each camera)
IPAddress local_IP(192, 168, 10, 212); // For example: adjust per device
IPAddress gateway(192, 168, 10, 1);      
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// Camera pin configuration for AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WiFiServer server(80);

void handleCapture(WiFiClient &client) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    client.print("HTTP/1.1 500 Internal Server Error\r\n\r\n");
    return;
  }
  
  // Build header with Content-Length to avoid caching issues.
  String header = "HTTP/1.1 200 OK\r\n";
  header += "Content-Type: image/jpeg\r\n";
  header += "Content-Length: " + String(fb->len) + "\r\n";
  header += "Cache-Control: no-cache, no-store, must-revalidate\r\n\r\n";
  client.print(header);
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  // Configure camera settings
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Use QVGA resolution to reduce bandwidth (320x240)
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  // Configure Wi-Fi with static IP
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  
  // Set maximum transmit power (20.5 dBm is available on many boards)
  WiFi.setTxPower(WIFI_POWER_20_5dBm);
  
  // Start WiFi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Camera IP: ");
  Serial.println(WiFi.localIP());
  
  // Disable Wi-Fi power saving mode to ensure low latency and continuous connectivity
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    // Wait until client sends data
    while (client.connected() && !client.available()) {
      delay(1);
    }
    String req = client.readStringUntil('\r');
    Serial.println("Request: " + req);
    // If the request URL contains /capture, serve a single JPEG frame.
    if (req.indexOf("/capture") != -1) {
      handleCapture(client);
    } else {
      client.print("HTTP/1.1 404 Not Found\r\n\r\n");
    }
    delay(1);
    client.stop();
  }
}
