#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Inisialisasi GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // RX=16, TX=17

// WiFi & MQTT
const char* ssid = "infinitegalaxy";
const char* password = "12345678";
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// Buzzer dan LED
const int buzzerPin = 14;
const int ledPin = 27;

bool alarmActive = false;
unsigned long alarmStartTime = 0;
bool sentLow = false;
unsigned long lowSentTime = 0;

void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Menghubungkan WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Tersambung");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == "infinitegalaxy/alarm") {
    if (message == "high") {
      digitalWrite(buzzerPin, HIGH);
      digitalWrite(ledPin, HIGH);
      alarmStartTime = millis();
      alarmActive = true;
      sentLow = false;
      Serial.println("Alarm diaktifkan!");
    }
  }
}

void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Menghubungkan MQTT...");
    if (client.connect("ESP32-GPS-Client")) {
      Serial.println("Berhasil");
      client.subscribe("infinitegalaxy/alarm");
    } else {
      Serial.print("Gagal, kode=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("GPS Matek M10 - Kirim ke MQTT");

  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  // Proses alarm
  if (alarmActive && millis() - alarmStartTime >= 5000) {
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    if (!sentLow) {
      lowSentTime = millis();
      sentLow = true;
    }
  }

  if (sentLow && millis() - lowSentTime >= 1000) {
    client.publish("infinitegalaxy/alarm", "low");
    Serial.println("Alarm dimatikan (low dikirim)");
    sentLow = false;
    alarmActive = false;
  }

  // Proses GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      float lat = gps.location.lat();
      float lng = gps.location.lng();
      int sats = gps.satellites.value();

      Serial.print("Lat: "); Serial.print(lat, 6);
      Serial.print(" | Lng: "); Serial.println(lng, 6);
      Serial.print("Satelit: "); Serial.println(sats);

      String koordinatStr = String(lat, 6) + "," + String(lng, 6);
      client.publish("infinitegalaxy/gps", koordinatStr.c_str());
    }
  }
}
