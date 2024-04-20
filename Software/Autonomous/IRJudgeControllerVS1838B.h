#include <IRremote.hpp>  // Version 4.2.0

int isRobotAllowedToMove = false; // Define se o robô está andando ou não

void IRJudgeControllerVS1838BInit() {
  IrReceiver.begin(JUDGE_CONTROLLER_PIN, ENABLE_LED_FEEDBACK);
}

void checkSensorIR() {
  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.decodedIRData.command, HEX);

    switch (IrReceiver.decodedIRData.command) {
      case 0:
        Serial.println("Apertou o botão 1");
        break;
      case 0x1:
      case 0x11:
        Serial.println("Apertou o botão 2");
        lastTime = millis(); // Para o controle PID
        isRobotAllowedToMove = true;
        break;
      case 0x2:
      case 0x12:
        Serial.println("Apertou o botão 3");
        if (isRobotAllowedToMove) {
          velMotorL = velMotorR = 0;
          sendPWMToMotors();
          Serial.println(" Robô Morreu ✖╭╮✖");
          while(1);
        }
        break;
      case 0x3:
      case 0x13:
        Serial.println("Apertou o botão 4");
        isRobotAllowedToMove = false;
        break;
      case 0x4:
      case 0x14:
        Serial.println("Apertou o botão 5");
        break;
      case 0x5:
      case 0x15:
        Serial.println("Apertou o botão 6");
        break;
    }

    IrReceiver.resume();
  }
}