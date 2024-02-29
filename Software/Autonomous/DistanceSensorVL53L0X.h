#include <VL53L0X.h>  // Sensores de distância
#include <Wire.h>     // Auxiliar dos sensores

VL53L0X sensorL;  // Sensor da esquerda
VL53L0X sensorC;  // Sensor da frente
VL53L0X sensorR;  // Sensor da direita

VL53L0X* sensorsList[3] = { &sensorL, &sensorC, &sensorR };  // Lista de ponteiros com os sensores

uint8_t sensorsAddresses[3] = { 0x1, 0x2, 0x3 };  // Endereços pro I2C

int distL;  // Valor lido pelo sensor da esquerda
int distC;  // Valor lido pelo sensor da frente
int distR;  // Valor lido pelo sensor da direita

int enemyLastTimeSeenLeft = false; // Define qual o último lado que vimos o inimigo

void DistanceSensorVL53L0XInit() {

  Wire.begin();  // Inicia o I2C

  // Desliga todos os sensores
  for (int i = 0; i < 3; i++) {
    pinMode(VL53LOX_XSHUT_LIST_PINS[i], OUTPUT);
    digitalWrite(VL53LOX_XSHUT_LIST_PINS[i], LOW);
  }

  delay(10);

  // Endereça todos os sensores
  for (int i = 0; i < 3; i++) {
    pinMode(VL53LOX_XSHUT_LIST_PINS[i], INPUT); // XSHUT volta pra HIGH
    sensorsList[i]->init(); // Inicia o sensor usando a função da lib
    sensorsList[i]->setAddress(sensorsAddresses[i]); // Endereça o sensor
    sensorsList[i]->setTimeout(100); // Tempo que pode ficar sem responder
    sensorsList[i]->startContinuous(); // Modo contínuo, ↓ Precisão ↑ Velocidade
    delay(10);
  }
}

void readDistanceSensorsValues() {
  // Armazena os valores lidos nos sensores
  distL = sensorL.readRangeContinuousMillimeters();
  distC = sensorC.readRangeContinuousMillimeters();
  distR = sensorR.readRangeContinuousMillimeters();
}

void printDistanceSensorsValues() {
  // Mostra o valor de cada sensor na tela
  Serial.print("L: ");
  Serial.print(distL);
  Serial.print("  C: ");
  Serial.print(distC);
  Serial.print("  R: ");
  Serial.print(distR);
  Serial.print("\t\t");
}