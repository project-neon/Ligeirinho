
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);
float absoluteAngle = 0;

void GyroscopeMPU6050Init() {
  mpu6050.begin();                // Inicia o sensor pela lib
  mpu6050.calcGyroOffsets(true);  // Calcula o OffSet
}

void printGyroscopeAngle() {
  Serial.print("Angulo Z: ");
  Serial.print(absoluteAngle);
  Serial.print("\t\t");
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