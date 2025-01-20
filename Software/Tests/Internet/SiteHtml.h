#include <WiFi.h>
#include <string.h>

char ROBOT_EMOJI[] = "&#129302"; //🤖

void testFunc() {
  Serial.println("Clicou");
}

char *htmlList[] = {
  "<!DOCTYPE html>",
  "<html>",
    "<head>",
      //Define o nome da aba aberta
      "<title>Controle do Robo</title>",
      //Define favicon
      "<link rel='icon' href='data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>", ROBOT_EMOJI, "</text></svg>'>",
      "<h1> Controle do Robo", ROBOT_EMOJI, "- NEON </h1>",
    "</head>",
    "<body>",
      "<script type='text/javascript'>",
        "function myFunction() {",
          "fetch('/led');",
          "document.getElementById('demo').style.color = 'red';",
          "console.log('Clicou');",
        "}"
      "</script>",


      "<button id='demo' onclick='myFunction()' >Clique LED</button> <br> <br>",
      "<a href=\"/L\"> <button> Turn LED OFF </button> </a> <br> <br>",
      "Click <a href=\"/L\">here</a> to turn the LED on pin 2 off.<br>",
    "</body>",
  "</html>"
};

void generateHtml() {
  int listSize = sizeof(htmlList)/sizeof(htmlList[0]);
  for (int i = 0; i < listSize; i++) client.println(htmlList[i]);
}





//char htmlList[][100]

// char htmlString[] = "Inicial";
// void generateHtml() {
//     int listSize = sizeof(htmlList)/sizeof(htmlList[0]);
//     for (int i = 0; i < listSize; i++) strcat(htmlString, htmlList[i]);
// } 

//"<a href=\"/H\"> <button id='demo' onclick='myFunction()' > Turn LED ON </button> </a> <br> <br>",
//"<a href=\"/L\"> <button> Turn LED OFF </button> </a> <br> <br>",