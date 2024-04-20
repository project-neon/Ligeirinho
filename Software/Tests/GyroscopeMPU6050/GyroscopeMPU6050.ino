
#include <MPU6050_tockn.h>
#include <Wire.h>

// TODO: Testar se com outros dispositivos I2C ainda funciona;
// Código tese retirado da biblioteca MPU6050_tockn;
// Por padrão, parece que o Endereço do MPU é 68.
// A biblioteca já cuida disso automaticamente,
// então desde que não setemos nenhum endereço como 68, não devemos ter problema.

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Inicia o I2C

  mpu6050.begin();                // Inicia o sensor pela lib
  mpu6050.calcGyroOffsets(true);  // Calcula o OffSet
}

void loop() {
  mpu6050.update();  // Atualiza os valores de X, Y, Z
  Serial.print("Angulo X: ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tAngulo Y : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tAngulo Z : ");
  Serial.println(mpu6050.getAngleZ());
}
