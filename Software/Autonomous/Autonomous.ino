#include "Pinout.h"   // Arquivo com os valores dos pinos
#include "Constants.h"
#include "DistanceSensorVL53L0X.h"
#include "GyroscopeMPU6050.h"
#include "MotorsHBridgeDRV8833.h"
#include "IRJudgeControllerVS1838B.h"
#include "MouseSensorADNS9500.h"

int timer = millis();

void printDebugInfos() {
  printDistanceSensorsValues();
  printGyroscopeAngle();
  printMotorsSpeed();
  printMouseXYAbsolute();
  Serial.println("");
}

void setup() {
  // Inicia a comunicação serial UART
  Serial.begin(115200);

  // Iniciar o sensor de Infra Vermelho do Juiz
  IRJudgeControllerVS1838BInit();

  // Inicia e endereça os 3 sensores de distância
  DistanceSensorVL53L0XInit();

  // Inicia o Giroscópio
  GyroscopeMPU6050Init();

  // Inicia os motores configurando os seus canais PWM
  MotorsHBridgeDRV8833Init();

  // Inicia o sensor de mouse
  MouseSensorADNS9500Init();
}

void preventFromFalling() {
  if (radius >= maxRadius) {
    velMotorL = velMotorR = -40;
    sendPWMToMotors();
    delay(200);
  }
}

void simpleStrategy() {
  if (distC < distAtk and (distL < distAtk or distR < distAtk)) {
    Serial.print("ATACANDO \t\t");
    preventFromFalling();
    velMotorL = velMotorR = speedStandard;
  } else if (distL < distAtk or distR < distAtk) {
    (distL < distAtk) ? Serial.print("ESQ \t\t") : Serial.print("DIR \t\t");
    preventFromFalling();
    velMotorL = (distL < distAtk) ? speedStandard * 0.9 : speedStandard;
    velMotorR = (distL < distAtk) ? speedStandard : speedStandard * 0.9;
    enemyLastTimeSeenLeft = (distL < distAtk) ? true : false;
  } else {
    enemyLastTimeSeenLeft ? Serial.print("PROCURANDO ESQ \t\t") : Serial.print("PROCURANDO DIR \t\t");
    velMotorL = enemyLastTimeSeenLeft ? -searchSpeed : searchSpeed;
    velMotorR = enemyLastTimeSeenLeft ? searchSpeed : -searchSpeed;
  }
}

void goBackAndForth() {
  if (velMotorL == 0) velMotorL = velMotorR = -speedStandard;
  if (radius >= maxRadius && millis()-timer > 500) {
    timer = millis();
    velMotorL = velMotorR = -velMotorL;
    sendPWMToMotors();
  }
}

void loop() {

  // Checa se recebemos algum comando do controle do Juiz e muda de estágio
  checkSensorIR();

  readDistanceSensorsValues();
  readGyroscopeAngles();
  readMouseXY();
  updateRobotRadius();

  if (isRobotAllowedToMove) {
    goBackAndForth();
  } else {
    velMotorL = velMotorR = 0;
  }

  sendPWMToMotors();
  printDebugInfos();
}