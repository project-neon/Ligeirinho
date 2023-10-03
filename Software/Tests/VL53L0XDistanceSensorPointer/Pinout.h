// Motores da Esquerda e da direita
#define MOTOR_LEFT_PIN 16
#define MOTOR_RIGHT_PIN 17

// SCL, SDA para a comunicação I2C
// Como usamos os pinos padrões da I2C, a biblioteca Wire.h cuida
// disso automaticamente para nós. Então essas variáveis não seriam necessárias.
#define SCL_PIN 22
#define SDA_PIN 21

// Define o pino XSHUT do sensor de distância VL53L0X
// Quando LOW, esse pino desliga o sensor e permite o endereçamento dos outros
int VL53LOX_XSHUT_LIST_PINS[3] = {23, 19, 18};

// Pino que retorna o valor recebido pelo controle do Juiz
#define JUDGE_CONTROLLER_PIN 4

// Backup analogs pins
#define ANALOG_1_PIN 25
#define ANALOG_2_PIN 26
#define ANALOG_3_PIN 32
#define ANALOG_4_PIN 33