// Define as constantes PID
double Kp = 2.5;
double Ki = 0.0;
double Kd = 25.0;

double setPoint = 0;  // Valor desejado para x
double errorSum = 0; //
double lastError = 0;
double pidOutput, lastTime;

void calculatePidError () {
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
  lastAbsoluteX = absoluteX;

	// Aplica as constantes PID
  pidOutput = Kp * error + Ki * errorSum + Kd * dError;
}

void printPidOutput() {
  Serial.print("PID: ");
  Serial.print(pidOutput);
  // Serial.print("\tP:");Serial.print(Kp * error);
  // Serial.print("\tI:");Serial.print(Ki * errorSum);
  // Serial.print("\tD:");Serial.print(Kd * dError);
  Serial.print("\t");
}