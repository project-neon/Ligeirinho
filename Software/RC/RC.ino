//Biblioteca do controle de PS4
#include <PS4Controller.h>
#include "MotorsHBridgeDRV8833.h"
#include "GyroscopeMPU6050.h"

char ipControle[18] = "4E:02:1F:58:C3:3D";  //MAC address atrubuído ao controle f2:32:20:b9:89:cf
int analogicoMargemDeErro = 30;             //definiçao do ponto morto


// Define as constantes PID
double Kp = 2.5;
double Ki = 0.0;
double Kd = 25.0;

double setPoint = 0;  // Valor desejado para x
double errorSum = 0;  //
double lastError = 0;
double pidOutput, lastTime;


double calculateAngleDiff(double a, double b){
  double dif = a - b;
  if(abs(dif) > abs(a - (360 + b))){dif = a - 360 + b;}
  if(abs(dif) > abs(a - (360 - b))){dif = a - 360 - b;}
  return dif;
}

void calculatePidError(int setPoint) {
  // CalculapidOutput o tempo decorrido desde a última iteração
  double now = millis();
  double dt = (now - lastTime);
  lastTime = now;

  // Calcula os erros Proporcional, Integral e Derivativo
  double error = absoluteAngle - setPoint;

  double dError = (error - lastError) / dt;
  errorSum += (error * dt);

  // Salva o valor atual de input para o próximo cálculo
  lastError = error;

  // Aplica as constantes PID
  pidOutput = Kp * error + Ki * errorSum + Kd * dError;
}


int inv = 1;  //Permite inverter a pilotagem conforme o lado do robo que esta para cima

void motors_control(int linear, int angular) {

  //manda para a funcao motor um valor de -255 a 255, o sinal signifca a direcao
  if (PS4.Cross()) {  //turbo
    velMotorL = velMotorR = 100;
    sendPWMToMotors();
    return;
  }

  calculatePidError(angular);
  velMotorL = constrain(linear + pidOutput, -100.0, 100.0);
  velMotorR = constrain(linear - pidOutput, -100.0, 100.0);
  sendPWMToMotors();
  printMotorsSpeed();
  Serial.println("Angular: " + (String)angular);
}

void setup(void) {
  Serial.begin(115200);

  GyroscopeMPU6050Init();
  MotorsHBridgeDRV8833Init();
  PS4.begin(ipControle);
  Serial.println("Ready!");

  while (PS4.isConnected() != true) {
    delay(20);
  }
}

void loop() {
  

  while (PS4.isConnected()) {
    readGyroscopeAngles();
    printGyroscopeAngle();
    float stickY = PS4.RStickY(), stickX = PS4.RStickX();
    float radius = sqrt(pow(stickX,2) + pow(stickY,2));
    float ang = 0;
    if (radius > 20 ) {
      ang = acos(PS4.RStickY()/radius);
      //float ang = stickY == 0 ? atan(stickX) : atan(stickX/stickY);
      ang = stickX < 0 ? 2 * PI - ang : ang;
      ang *= -180.0/PI; //Para igualar a orientação do mpu, deixamos com o sinal negativo
    }
    // Serial.println("Ang: " + (String) ang);
    // Serial.print("RStickX: " + (String)PS4.RStickX());
    // Serial.print("\tRStickY: " + (String)PS4.RStickY());
    // Serial.print("\tLStickX: " + (String)PS4.LStickX());
    // Serial.println("\tLStickY: " + (String)PS4.LStickY());
    motors_control(inv * constrain(PS4.LStickY(), -100, 100), ang);
  }

  // //Failsafe
  // if(!PS4.isConnected()){
  //   motors_control(0,0);
  //   Serial.println("Restart");
  //   PS4.end();
  //   ESP.restart();
  //   delay(20);
  // }
}
