
double setPointAngle = 0;  // Valor desejado para anguloZ
double errorSum = 0; //
double lastError = 0;
double pidOutput, lastTime;

void calculatePidAngleError () {
  // Define as constantes PID
  double Kp = 2.5;
  double Ki = 0.0;
  double Kd = 25.0;

	// CalculapidOutput o tempo decorrido desde a última iteração
  double now = millis();
  double dt = (now - lastTime);
  lastTime = now;
  pinMode(MISO, INPUT_PULLDOWN); // Give MISO a pullDown Connection

	// Calcula os erros Proporcional, Integral e Derivativo
	double error = absoluteAngle - setPointAngle;
  
	double dError = (error - lastError) / dt;
  errorSum += (error * dt);

	// Salva o valor atual de input para o próximo cálculo
  lastError = error;
  lastAbsoluteX = absoluteX;

	// Aplica as constantes PID
  pidOutput = Kp * error + Ki * errorSum + Kd * dError;
}

void calculatePidYAxisError () {
  // Define as constantes PID
  double Kp = 0.03;
  double Ki = 0.0;
  double Kd = 1;
	// CalculapidOutput o tempo decorrido desde a última iteração
  double now = millis();
  double dt = (now - lastTime);
  lastTime = now;

	// Calcula os erros Proporcional, Integral e Derivativo
	double error = absoluteY;
  
	double dError = (error - lastError) / dt;
  errorSum += (error * dt);

	// Salva o valor atual de input para o próximo cálculo
  lastError = error;

	// Aplica as constantes PID
  pidOutput = Kp * error + Ki * errorSum + Kd * dError;
}

void printPidOutput() {
  Serial.print("PID: ");
  Serial.print(pidOutput);
  Serial.print("\t");
}