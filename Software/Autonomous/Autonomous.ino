#include "Pinout.h"   // Arquivo com os valores dos pinos
#include "Constants.h"
#include "DistanceSensorVL53L0X.h"
#include "GyroscopeMPU6050.h"
#include "MotorsHBridgeDRV8833.h"
#include "IRJudgeControllerVS1838B.h"
#include "MouseSensorADNS9500.h"


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
  if(radius >= maxRadius) {
    velMotorL = velMotorR = -40;
    sendPWMToMotors();
    delay(200);
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
    if(distC < distAtk and (distL < distAtk or distR < distAtk)) {
      Serial.print("ATACANDO \t\t");
      preventFromFalling();
      velMotorL = velMotorR = speedStandard;
    } else if (distL < distAtk or distR < distAtk) {
      (distL < distAtk) ? Serial.print("ESQ \t\t") : Serial.print("DIR \t\t");
      preventFromFalling();
      velMotorL = (distL < distAtk) ? speedStandard*0.9 : speedStandard;
      velMotorR = (distL < distAtk) ? speedStandard : speedStandard*0.9;
      enemyLastTimeSeenLeft = (distL < distAtk) ? true : false;
    } else {
      Serial.print("PROCURANDO \t\t");
      if (inwardSpiralMovement) {
        velMotorL = enemyLastTimeSeenLeft ? velMotorL-- : searchSpeed;
        velMotorR = enemyLastTimeSeenLeft ? searchSpeed : velMotorR--;
      } else {
        velMotorL = enemyLastTimeSeenLeft ? velMotorL-- : searchSpeed;
        velMotorR = enemyLastTimeSeenLeft ? searchSpeed : velMotorR--;
      }
      
    }
  } else {
    velMotorL = velMotorR = 0;
  }

  sendPWMToMotors();
  printDebugInfos();
}