//PS: A Internet e o USB costumam bater de frente quando usados juntos! Parece que pode ser tanto porque o WIFI consome muita energia do USB, como a falta de uso da flag phy_bbpll_en_usb(true);

int strategySelected = 1;
#define INTERNET_MODE 0 //0 -> Desliga a internet, > 0 -> Liga a internet, 1 -> Printa os valores em http://<192.168.15.24>/webserial

#if INTERNET_MODE > 0

//&#129302 = 🤖

// Referência: https://esp32io.com/tutorials/esp32-web-server#content_reading_the_sensor_value_from_esp32_via_web
#include <WiFi.h>
#include <ESPAsyncWebServer.h> //v3.1.0
#include <WebSerial.h> //1.1.0 - Ayush Sharma

AsyncWebServer server(80);
WiFiClient client;

const char *ssid = "----";
const char *password = "----";
bool isLedOn = false;

const char* webpage = R"=====(
  <!DOCTYPE html>
  <html style='text-align:center; background-color: #0db037'>
    <head>
      <!-- Define o nome da aba aberta -->
      <title>Controle do Robo</title>

      <!-- Define favicon https://stackoverflow.com/questions/59431371/use-emoji-as-favicon-in-websites-->
      <link rel="icon" href="data:image/svg+xml,
        <svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22>
          <text y=%22.9em%22 font-size=%2290%22>  &#129302  </text>
        </svg>">

      <h1> Controle do Robo &#129302 - NEON </h1>
    </head>
    <body>

      <script type='text/javascript'>
        function turnLedOn() {
          fetch('/led').then( response => response.text())
            .then(isLedOn => {
            isLedOn = isLedOn === '1';

            document.getElementById('ledID').textContent = isLedOn ? 'Desligar o LED' : 'Ligar o LED';
            document.getElementById('circle').style.backgroundColor = isLedOn ?'blue' : 'lightGray';
          });
        }

        function setStrategy1() {
          fetch('/strategy1').then(response => {
            document.getElementById('strategyParagraph').textContent = 'Estrategia 1 selecionada'
          });
        }

        function setStrategy2() {
          fetch('/strategy2').then(response => {
            document.getElementById('strategyParagraph').textContent = 'Estrategia 2 selecionada'
          });
        }

        function setStrategy3() {
          fetch('/strategy3').then(response => {
            document.getElementById('strategyParagraph').textContent = 'Estrategia 3 selecionada'
          });
        }
      </script>

      <style>
        .dot {
          height: 25px;
          width: 25px;
          background-color: lightGray;
          border-radius: 50%;
          display: inline-block;
        }
      </style>

      <button id='ledID' onclick='turnLedOn()'> Ligar o LED </button> <br> <br>
      <span id='circle' class="dot"></span> <br> <br>

      <button id='ledID' onclick='setStrategy1()'> Setar Estrategia 1 </button> <br> <br>
      <button id='ledID' onclick='setStrategy2()'> Setar Estrategia 2  </button> <br> <br>
      <button id='ledID' onclick='setStrategy3()'> Setar Estrategia 3 </button> <br> <br>

      <p id='strategyParagraph'></p> <br> <br>

      <a href=/webserial> <button> Ir para pagina /Webserial </button> </a> <br> <br>


    </body>
  </html>
  )=====";


void InternetInit() {

  // Conectamos na internet
  Serial.println((String) "Connecting to: " + ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // // Use o IP 192.168.15.24
  // IPAddress local_IP(192, 168, 15, 24);
  // IPAddress gateway(192, 168, 1, 1);
  // IPAddress subnet(255, 255, 0, 0);
  // IPAddress primaryDNS(8, 8, 8, 8);
  // IPAddress secondaryDNS(8, 8, 4, 4);
  // // Configures static IP address
  // if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
  //   Serial.println("STA Failed to configure");
  // }

  Serial.print("Tentando se conectar na internet, aguarde");
  int timer = millis();
  // Enquanto não conectar, printa pontinhos
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - timer > 7000) {
      Serial.println("Não foi possível se conectar a internet");
      break;
    }
  }

  // IP para acessar o servidor
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // WebSerial is accessible at "192.168.15.24/webserial" in browser
  WebSerial.begin(&server);

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

  //TODO: Dá pra aprimorar usando só um request /strategy e query params
  server.on("/strategy1", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("GET /estrategy1");
    strategySelected = 1;
    request->send(200, "text/plain", String(strategySelected));
  });

  server.on("/strategy2", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("GET /estrategy2");
    strategySelected = 2;
    request->send(200, "text/plain", String(strategySelected));
  });

  server.on("/strategy3", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("GET /estrategy3");
    strategySelected = 3;
    request->send(200, "text/plain", String(strategySelected));
  });

  //Inicia o servidor
  server.begin();
}
#endif