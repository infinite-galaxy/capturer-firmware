#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <SD_MMC.h>
#include <base64.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RTClib.h>

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define GPS_RX 16
#define GPS_BAUD 9600

#define EEPROM_SIZE 64

#define LAT_ADDR 0
#define LON_ADDR 4

#define CUSTOM_SDA_PIN 13
#define CUSTOM_SCL_PIN 15

const char* wifi_ssid = "infinitegalaxy";
const char* wifi_password = "12345678";
const char* mqtt_server = "mqtt.thingsboard.falkia34.dev";
const char* mqtt_tele_topic = "v1/devices/me/telemetry";
const char* mqtt_attr_topic = "v1/devices/me/attributes";
const char* mqtt_username = "d3slu0dxxiyhmrcq5nff";
const char* mqtt_password = "";
const int mqtt_port = 1883;
const int image_buffer_size = 150000;

unsigned long global_unix_time = 0;

unsigned long last_attr_sent = 0;
const unsigned long attr_interval = 60000;

unsigned long last_print_gps = 0;
const unsigned long gps_interval = 1000;

String time_str;
float latitude, longitude;
int retry_counter;
bool file_flag = true;

WiFiClient esp_client;
PubSubClient mqtt_client(esp_client);

WiFiUDP ntp_udp;
NTPClient time_client(ntp_udp, "pool.ntp.org", 0, 60000);

SoftwareSerial gps_serial(GPS_RX, -1);
TinyGPSPlus gps;

RTC_DS3231 rtc;

void setup_camera() {
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 30;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (true);
  }
}

void setup_wifi() {
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void reconnect_mqtt() {
  while (!mqtt_client.connected()) {
    mqtt_client.setBufferSize(image_buffer_size);
    Serial.print("Connecting to MQTT...");
    String client_id = "ESP32Client-" + String(random(0xffff), HEX);
    if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.print(mqtt_client.state());
      delay(2000);
    }
  }
}

void save_json_to_sd(const String& filename, const String& json) {
  if (file_flag) {
    file_flag = false;
    File file = SD_MMC.open("/" + filename, FILE_WRITE);
    file.print(json);
    file.close();
    file_flag = true;
  }
  Serial.println("JSON saved: " + filename);
}

void setup() {
  Serial.begin(115200);
  gps_serial.begin(GPS_BAUD);
  EEPROM.begin(EEPROM_SIZE);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  rtc.begin();

  latitude = EEPROM.readFloat(LAT_ADDR);
  longitude = EEPROM.readFloat(LON_ADDR);

  if (isnan(latitude)) latitude = 0;
  if (isnan(longitude)) longitude = 0;

  setup_camera();
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return;
  }

  setup_wifi();

  mqtt_client.setBufferSize(image_buffer_size);

  time_client.begin();
  if (time_client.update()) {
    rtc.adjust(DateTime(time_client.getEpochTime()));
    Serial.println("RTC updated from NTP");
  } else {
    time_client.forceUpdate();
    if (time_client.update()) {
      rtc.adjust(DateTime(time_client.getEpochTime()));
      Serial.println("RTC updated from NTP");
    } else Serial.println("NTP unavailable, using RTC only");
  }
  global_unix_time = rtc.now().unixtime();

  mqtt_client.setServer(mqtt_server, mqtt_port);

  xTaskCreatePinnedToCore(Task2code, "Task1", 4096, NULL, 0, NULL, 0);
  delay(500);
}

void loop() {
  while (gps_serial.available() > 0) {
    gps.encode(gps_serial.read());
  }
  if (millis() - last_print_gps > gps_interval) {
    last_print_gps = millis();
    if (gps.location.isValid()) {
      float read_latitude = gps.location.lat();
      float read_longitude = gps.location.lng();
      EEPROM.writeFloat(LAT_ADDR, read_latitude);
      EEPROM.writeFloat(LON_ADDR, read_longitude);
      EEPROM.commit();
      latitude = read_latitude;
      longitude = read_longitude;
      Serial.print("Latitude : ");
      Serial.println(read_latitude, 6);
      Serial.print("Longitude: ");
      Serial.println(read_latitude, 6);
    }
  }
  if (latitude && longitude) {
    global_unix_time = rtc.now().unixtime(); 
    time_str = String(global_unix_time) + "000";
    camera_fb_t* fb = esp_camera_fb_get();

    if (!fb) {
      Serial.println("Camera capture failed");
      delay(10000);
      return;
    }

    String image_base64 = base64::encode(fb->buf, fb->len);

    String payload = "{";
    payload += "\"ts\":" + time_str + ",";
    payload += "\"values\":{";
    payload += "\"gps\":{";
    payload += "\"lat\":" + String(latitude, 6) + ",";
    payload += "\"long\":" + String(longitude, 6);
    payload += "},";
    payload += "\"image\":\"" + image_base64 + "\"";
    payload += "}}";

    Serial.println(payload);
    Serial.printf("Payload size: %d chars\n", payload.length());

    String filename = time_str + ".txt";
    save_json_to_sd(filename, payload);

    payload = "";

    esp_camera_fb_return(fb);
    delay(5000);
  }
}

void Task2code(void* parameter) {
  Serial.print("Task2 is running on core ");
  Serial.println(xPortGetCoreID());
  for (;;) {
    if (latitude && longitude) {
      mqtt_client.setBufferSize(image_buffer_size);
      if (!mqtt_client.connected()) {
        reconnect_mqtt();
      }
      mqtt_client.loop();
      if (file_flag) {
        file_flag = false;
        File root = SD_MMC.open("/");
        if (!root) {
          file_flag = true;
          Serial.println("Failed to open the root directory!");
          delay(3000);
          return;
        }

        String oldest_file = "";
        File file;
        while ((file = root.openNextFile())) {
          if (!file.isDirectory() && String(file.name()).endsWith(".txt")) {
            if (oldest_file == "" || String(file.name()) < oldest_file) {
              oldest_file = file.name();
            }
          }
          file.close();
        }
        root.close();

        if (oldest_file != "") {
          File txt_file = SD_MMC.open("/" + oldest_file);
          if (!txt_file) {
            Serial.println("Failed to open " + oldest_file);
            file_flag = true;
            delay(2000);
            return;
          }

          String payload_sdcard = "";
          while (txt_file.available()) {
            payload_sdcard += (char)txt_file.read();
          }
          txt_file.close();
          if (!mqtt_client.connected()) reconnect_mqtt();
          bool success = mqtt_client.publish(mqtt_tele_topic, payload_sdcard.c_str());

          if (success) {
            SD_MMC.remove("/" + oldest_file);
            Serial.println("Successfully to send and delete file " + oldest_file);
          } else {
            retry_counter++;
            Serial.println("Failed to send file: " + oldest_file);
            if (retry_counter >= 2) {
              SD_MMC.remove("/" + oldest_file);
              Serial.println("Failed to send file " + oldest_file + " twice, file deleted!");
              retry_counter = 0;
            }
          }
        } else {
          Serial.println("No file file .txt to send!");
        }
        file_flag = true;
      }
      if (millis() - last_attr_sent > attr_interval) {
        String attr_payload = "{";
        attr_payload += "\"lastLatAt\":" + String(latitude, 6) + ",";
        attr_payload += "\"lastLongAt\":" + String(longitude, 6);
        attr_payload += "}";

        if (mqtt_client.publish(mqtt_attr_topic, attr_payload.c_str())) {
          Serial.println("GPS attributes sent.");
        } else {
          Serial.println("Failed to send attributes!");
        }
        attr_payload = "";
        last_attr_sent = millis();
      }
    } else Serial.println("No GPS!");
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}