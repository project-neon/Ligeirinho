#include <WiFi.h>

const char *ssid = "----";
const char *password = "----";
bool isLedOn = false;

WiFiServer server(80);
WiFiClient client;

#include "SiteHtml.h"



void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // We start by connecting to a WiFi network
  Serial.println((String) "Connecting to: " + ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

    // if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //   Serial.printf("WiFi Failed!\n");
    //   return;
    // }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  client = server.accept();  // listen for incoming clients

  if (client) {                     // if you get a client,
    Serial.println("New Client.");  // print a message out the serial port
    String currentLine = "";        // make a String to hold incoming data from the client
    while (client.connected()) {    // loop while the client's connected
      if (client.available()) {     // if there's bytes to read from the client,
        char c = client.read();     // read a byte, then
        Serial.print(c);
        //Serial.write(c);            // print it out the serial monitor
        if (c == '\n') {            // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            generateHtml();
            // client.println(htmlString);
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /led")) {
          isLedOn = !isLedOn;
          digitalWrite(LED_BUILTIN, isLedOn);  // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(LED_BUILTIN, LOW);  // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}