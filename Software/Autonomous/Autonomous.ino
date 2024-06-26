#include "Pinout.h"  // Arquivo com os valores dos pinos
#include "Constants.h"
#include "DistanceSensorVL53L0X.h"
#include "GyroscopeMPU6050.h"
#include "MotorsHBridgeDRV8833.h"
#include "MouseSensorADNS9500.h"
#include "PidController.h"
#include "IRJudgeControllerVS1838B.h"


int timer = millis();

void printDebugInfos() {
  printDistanceSensorsValues();
  printGyroscopeAngle();
  printMotorsSpeed();
  printMouseXYRelative();
  printMouseRadius();
  printPidOutput();
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
  calculatePidAngleError();
  if (radius <= minRadius) canRevertVel = true;
  if (radius >= maxRadius && canRevertVel) {
    speedStandard = -speedStandard;
    canRevertVel = false;
  }
  velMotorL = constrain(speedStandard + pidOutput, -100, 100);
  velMotorR = constrain(speedStandard - pidOutput, -100, 100);
}

void notMoveYAxis() {
  calculatePidYAxisError();
  velMotorL = constrain(-pidOutput, -100, 100);
  velMotorR = constrain(-pidOutput, -100, 100);
}

void checkIfBeingPushed() {
  for (int i = 0; i < (9); i++) listXDistanceValues[i] = listXDistanceValues[i + 1];
  listXDistanceValues[9] = dx;

  int sum = 0;
  for (int i = 0; i < 10; i++) sum += listXDistanceValues[i];
  int meanValueXDistance = sum / 10;
  // Serial.println(meanValueXDistance);
  if (meanValueXDistance >= 400) velMotorL = velMotorR = -maxSpeed;
  else if (distL < distAtk/3 || distC < distAtk/3 || distR < distAtk/3) velMotorL = velMotorR = maxSpeed;
}

void loop() {

  // Checa se recebemos algum comando do controle do Juiz e muda de estágio
  checkSensorIR();

  readDistanceSensorsValues();
  readGyroscopeAngles();
  readMouseXY();
  updateRobotRadius();
  

  if (isRobotAllowedToMove) {
    // simpleStrategy();
    // goBackAndForth();
    checkIfBeingPushed();
    // notMoveYAxis();
  } else {
    velMotorL = velMotorR = 0;
  }

  sendPWMToMotors();
  printDebugInfos();
}