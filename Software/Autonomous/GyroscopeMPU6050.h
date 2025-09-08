
#include <MPU6050_neon.h> //1.0.0
#include <Wire.h>

MPU6050 mpu6050(Wire);
float absoluteAngle = 0;
// Como a placa modular inicia com um pequeno movimento da roda
// vamos setar para o MPU iniciar ao apertarmos 1 no botão do Juiz
bool hasMpuAlreadyInitialize = false;

void GyroscopeMPU6050Init() {
  mpu6050.begin();                // Inicia o sensor pela lib
  mpu6050.calcGyroOffsets(true);  // Calcula o OffSet
  mpu6050.restartXYZAnglesValues();
}

void printGyroscopeAngle() {
  Serial.print("Angulo Z: ");
  Serial.print(absoluteAngle);
  Serial.print("\t");
}

String printInternetGyroscopeAngle() {
  return "  Angulo Z: " + (String) absoluteAngle;
}


void readGyroscopeAngles() {
  mpu6050.update(); // Atualiza os valores de X, Y, Z
  float angleZ = mpu6050.getAngleZ();
  bool isNegative = angleZ < 0;
  while(abs(angleZ) > 360) {
    isNegative ? angleZ += 360 : angleZ -= 360;
  }
  absoluteAngle = angleZ;
}