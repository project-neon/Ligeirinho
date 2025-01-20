// Referência: https://esp32io.com/tutorials/esp32-web-server#content_reading_the_sensor_value_from_esp32_via_web
#include <WiFi.h>
#include <ESPAsyncWebServer.h> //v3.1.0
#include "SiteHtml.h"

AsyncWebServer server(80);
WiFiClient client;

const char *ssid = "----";
const char *password = "----";
bool isLedOn = false;


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Conectamos na internet
  Serial.println((String) "Connecting to: " + ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Enquanto não conectar, printa pontinhos
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // IP para acessar o servidor
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // ENDPOINTS
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("GET /");
    request->send(200, "text/html", webpage);
  });

  server.on("/led", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("GET /led");
    isLedOn = !isLedOn;
    digitalWrite(LED_BUILTIN, isLedOn);
    request->send(200, "text/plain", String(isLedOn));
  });

  //Inicia o servidor
  server.begin();
}

void loop() {
 
}