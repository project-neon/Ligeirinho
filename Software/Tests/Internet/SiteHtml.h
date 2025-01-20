//&#129302 = 🤖

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
        function myFunction() {
          fetch('/led').then( response => response.text())
            .then(isLedOn => {
            isLedOn = isLedOn === '1';

            document.getElementById('demo').textContent = isLedOn ? 'Desligar o LED' : 'Ligar o LED';
            document.getElementById('circle').style.backgroundColor = isLedOn ?'blue' : 'lightGray';
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
      
      <button id='demo' onclick='myFunction()'> Ligar o LED </button> <br> <br>
      <span id='circle' class="dot"></span>

    </body>
  </html>
  )=====";
