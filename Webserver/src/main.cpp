#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#define RX2_PIN 16
#define SERIAL2_BAUD 921600
#define DNS_PORT 53

#define CAPTIVE_PORTAL_DOMAIN "patientportal"
#define CAPTIVE_PORTAL_URL ("http://" CAPTIVE_PORTAL_DOMAIN ".local")
DNSServer dnsServer;
AsyncWebServer CaptiveServer(80);
AsyncEventSource events("/sensor_readings");

// #pragma pack(1)
typedef struct
{
  uint8_t start_byte;
  int16_t temperature;
  int16_t hr_bpm;
  int16_t hrv_mean_rr;
  int16_t hrv_sdnn;
  int16_t hrv_rmssd;
  int16_t hrv_pnn50;
  int16_t ecg_sig;
  bool isbeat_ecg;
  int16_t ppg_sig;
  bool isbeat_ppg;
  int16_t ppg_hr;
  uint8_t ppg_spo2;
  bool ecg_idle;
  bool ppg_idle;
  uint8_t stop_byte;
} data_pkt;

union DataUnion
{
  data_pkt packet;
  uint8_t buffer[sizeof(data_pkt)];
};

DataUnion dataUnion;
uint32_t bufferIndex = 0;
const IPAddress localIP(4, 3, 2, 1);          // the IP address the web server, Samsung requires the IP to be in public space
const IPAddress gatewayIP(4, 3, 2, 1);        // IP address of the network should be the same as the local IP for captive portals
const IPAddress subnetMask(255, 255, 255, 0); // no need to change: https://avinetworks.com/glossary/subnet-mask/

void setup()
{
  Serial.begin(115200);
  Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, RX2_PIN, -1); // RX only
  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAPConfig(localIP, gatewayIP, subnetMask);
  WiFi.softAP("Patient Portal", "12345678", 6 /* channel */, 0 /* hidden */, 4 /* max client*/);

  dnsServer.setTTL(3600);
  // Setup mDNS
  if (MDNS.begin(CAPTIVE_PORTAL_DOMAIN))
  {
    Serial.println("MDNS responder started");
  }
  MDNS.addService("http", "tcp", 80);

  // Setup Web Server
  // server.on("/", handleRoot);
  dnsServer.start(53, "*", localIP);

  // server.begin();
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

  LittleFS.begin();
  CaptiveServer.on("/connecttest.txt", [](AsyncWebServerRequest *request)
                   { request->redirect("http://logout.net"); }); // windows 11 captive portal workaround
  CaptiveServer.on("/wpad.dat", [](AsyncWebServerRequest *request)
                   { request->send(404); }); // Honestly don't understand what this is but a 404 stops win 10 keep calling this repeatedly and panicking the esp32 :
  CaptiveServer.on("/generate_204", [](AsyncWebServerRequest *request)
                   { request->redirect(CAPTIVE_PORTAL_URL); }); // android captive portal redirect
  CaptiveServer.on("/generate204", [](AsyncWebServerRequest *request)
                   { request->redirect(CAPTIVE_PORTAL_URL); }); // android captive portal redirect
  CaptiveServer.on("/redirect", [](AsyncWebServerRequest *request)
                   { request->redirect(CAPTIVE_PORTAL_URL); }); // microsoft redirect
  CaptiveServer.on("/hotspot-detect.html", [](AsyncWebServerRequest *request)
                   { request->redirect(CAPTIVE_PORTAL_URL); }); // apple call home
  CaptiveServer.on("/canonical.html", [](AsyncWebServerRequest *request)
                   { request->redirect(CAPTIVE_PORTAL_URL); }); // firefox captive portal call home
  CaptiveServer.on("/success.txt", [](AsyncWebServerRequest *request)
                   { request->send(200); }); // firefox captive portal call home
  CaptiveServer.on("/ncsi.txt", [](AsyncWebServerRequest *request)
                   { request->redirect(CAPTIVE_PORTAL_URL); }); // windows call home
  CaptiveServer.on("/service/update2/json", [](AsyncWebServerRequest *request)
                   { request->send(200); }); // firefox?
  CaptiveServer.on("/chat", [](AsyncWebServerRequest *request)
                   { request->send(404); }); // No stop asking Whatsapp, there is no internet connection
  CaptiveServer.on("/favicon.ico", [](AsyncWebServerRequest *request)
                   { request->send(404); }); // webpage icon

  CaptiveServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  CaptiveServer.onNotFound(
      [](AsyncWebServerRequest *request)
      {
        request->send(404);
      });
  // CaptiveServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  //           { request->send_P(200, "text/html", index_html, processor); });

  CaptiveServer.addHandler(&events);
  CaptiveServer.begin();
}
StaticJsonDocument<512> doc;
char json_buffer[512];
uint32_t start_time;
uint8_t downsample_idx = 0;
void loop()
{
  start_time = millis();
  dnsServer.processNextRequest();
  yield();
  if (Serial2.available())
  {
    uint8_t incoming = Serial2.read();
    // webSocket.loop();
    if (bufferIndex == 0 && incoming != '\r')
    {
      return; // Wait for start byte
    }

    dataUnion.buffer[bufferIndex++] = incoming;

    if (bufferIndex == sizeof(data_pkt))
    {
      // Serial.printf("buffer read took: %u ms read: %u size: %u\r\n", millis() - start_time, bufferIndex, sizeof(data_pkt));
      if (dataUnion.packet.stop_byte == '\n')
      {
        // if (downsample_idx == 1)
        // {
        //   downsample_idx = 0;
        // }
        // else
        // {
        //   downsample_idx++;
        // }
        // if (downsample_idx == 0)
        // {
          doc["ppgIdle"] = dataUnion.packet.ppg_idle;
          doc["ecgIdle"] = dataUnion.packet.ecg_idle;
          doc["temp"] = dataUnion.packet.temperature;
          doc["hr"] = dataUnion.packet.hr_bpm;
          doc["meanRR"] = dataUnion.packet.hrv_mean_rr;
          doc["sdnn"] = dataUnion.packet.hrv_sdnn;
          doc["rmssd"] = dataUnion.packet.hrv_rmssd;
          doc["pnn50"] = dataUnion.packet.hrv_pnn50;
          doc["ecg"] = dataUnion.packet.ecg_sig;
          doc["ecgBeat"] = dataUnion.packet.isbeat_ecg;
          doc["ppg"] = dataUnion.packet.ppg_sig;
          doc["ppgBeat"] = dataUnion.packet.isbeat_ppg;
          doc["ppgHR"] = dataUnion.packet.ppg_hr;
          doc["spo2"] = dataUnion.packet.ppg_spo2;

          // Serial.printf("json took: %u ms\r\n", millis() - start_time);

          serializeJson(doc, json_buffer);
          events.send(json_buffer, "readings", millis());
        // }
      }
      bufferIndex = 0;
    }
  }
}