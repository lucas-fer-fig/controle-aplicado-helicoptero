#include <Servo.h>  // Biblioteca para controle do ESC com Servo

// Definindo pinos para sinal dos motores (ESCs) e encoders
#define signalPinPitch 45    // Pino de controle do ESC para pitch
#define signalPinYaw 46      // Pino de controle do ESC para yaw
#define encoderPinAPitch 18  // Pino do sinal A do encoder de pitch
#define encoderPinBPitch 19  // Pino do sinal B do encoder de pitch
#define encoderPinAYaw 2     // Pino do sinal A do encoder de yaw
#define encoderPinBYaw 3     // Pino do sinal B do encoder de yaw

// Declaração dos objetos Servo para ESCs
Servo escPitch;
Servo escYaw;

// Variáveis do encoder
volatile long pulseCountClockwisePitch = 0;
volatile long pulseCountCounterClockwisePitch = 0;
volatile long pulseCountClockwiseYaw = 0;
volatile long pulseCountCounterClockwiseYaw = 0;
const float PPR = 4132.0;  // Pulsos por revolução (ajustar conforme necessário)

// Estado anterior dos encoders
volatile bool lastAPitch = LOW;
volatile bool lastBPitch = LOW;
volatile bool lastAYaw = LOW;
volatile bool lastBYaw = LOW;

// Limites de ângulo para pitch e yaw em radianos
const float pitchLimitRad = radians(40);  // Limite de 40 graus
const float yawLimitRad = radians(180);   // Limite de 180 graus

// Variáveis de estado para cálculo de velocidade angular
float previousPitchAngleRad = 0.0;
float previousYawAngleRad = 0.0;
unsigned long lastTimePitch = 0;
unsigned long lastTimeYaw = 0;

// Setpoints (em radianos, mas entrada será em graus)
float setpointPitch = 0.0;
float setpointYaw = 0.0;

// Matrizes de ganho K mocadas
float K[2][4] = {
  {31.6148, 7.8464, 0.0, 0.0},     // Ganhos para pitch
  {0.0, 0.0, 7.9776, 31.1976}      // Ganhos para yaw
};

// Função de configuração do sistema
void setup() {
  // Inicializa ESCs
  escPitch.attach(signalPinPitch, 1000, 2000);
  escYaw.attach(signalPinYaw, 1000, 2000);

  // Configura pinos dos encoders
  pinMode(encoderPinAPitch, INPUT_PULLUP);
  pinMode(encoderPinBPitch, INPUT_PULLUP);
  pinMode(encoderPinAYaw, INPUT_PULLUP);
  pinMode(encoderPinBYaw, INPUT_PULLUP);

  // Configura interrupções para os pinos A e B de pitch e yaw
  attachInterrupt(digitalPinToInterrupt(encoderPinAPitch), handleEncoderPitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinBPitch), handleEncoderPitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinAYaw), handleEncoderYaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinBYaw), handleEncoderYaw, CHANGE);

  Serial.begin(9600);  // Inicia comunicação serial
}

// Função principal de controle
void loop() {
  // Verifica se há dados disponíveis na serial (para setpoints)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    processSetpoint(input);  // Processa o input serial para definir os setpoints
  }

  // Calcula os ângulos baseados nos encoders (em radianos)
  float pitchAngleRad = constrain(calculateAngle(pulseCountCounterClockwisePitch, pulseCountClockwisePitch), -pitchLimitRad, pitchLimitRad);
  float yawAngleRad = constrain(calculateAngle(pulseCountCounterClockwiseYaw, pulseCountClockwiseYaw), -yawLimitRad, yawLimitRad);

  // Calcula a velocidade angular com base na variação do tempo
  float pitchAngularVelocity = calculateAngularVelocity(pitchAngleRad, previousPitchAngleRad, lastTimePitch);
  float yawAngularVelocity = calculateAngularVelocity(yawAngleRad, previousYawAngleRad, lastTimeYaw);

  // Calcula os sinais de controle (tensões)
  float V_pitch, V_yaw;
  calculateControlSignal(pitchAngleRad, yawAngleRad, pitchAngularVelocity, yawAngularVelocity, V_pitch, V_yaw);

  // Controla os motores com as tensões calculadas
  controlMotor(escPitch, V_pitch);
  controlMotor(escYaw, V_yaw);

  // Debug na serial, convertendo para graus
  Serial.print("Pitch Angle (deg): ");
  Serial.print(degrees(pitchAngleRad));  // Convertendo de radianos para graus
  Serial.print(" | Yaw Angle (deg): ");
  Serial.println(degrees(yawAngleRad));  // Convertendo de radianos para graus

  delay(100);  // Atraso para suavizar a leitura
}

// Função para calcular o sinal de controle (tensões) usando realimentação de estados
void calculateControlSignal(float pitchAngle, float yawAngle, float pitchAngularVelocity, float yawAngularVelocity, float &V_pitch, float &V_yaw) {
  // Vetor de estados X = [ângulo pitch, ângulo yaw, velocidade pitch, velocidade yaw]
  float X[4] = { pitchAngle, yawAngle, pitchAngularVelocity, yawAngularVelocity };

  // Cálculo das tensões de controle usando u = -K * X
  V_pitch = -(K[0][0] * X[0] + K[0][1] * X[1] + K[0][2] * X[2] + K[0][3] * X[3]);
  V_yaw = -(K[1][0] * X[0] + K[1][1] * X[1] + K[1][2] * X[2] + K[1][3] * X[3]);
}

// Função para controlar o motor (ESC) com base na tensão de controle
void controlMotor(Servo &esc, float voltage) {
  // Mapeia o valor de tensão para o intervalo de 0 a 180 (velocidade do motor)
  int pwmValue = constrain(map(voltage, -10, 10, 0, 180), 0, 180);  // Ajuste de -10 a 10 conforme necessário
  esc.write(pwmValue);  // Define o valor de controle do motor
}

// Função para calcular a velocidade angular (variação do ângulo por variação de tempo)
float calculateAngularVelocity(float currentAngle, float &previousAngle, unsigned long &lastTime) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // Tempo em segundos
  lastTime = currentTime;

  float angularVelocity = (currentAngle - previousAngle) / deltaTime;  // Velocidade angular em rad/s
  previousAngle = currentAngle;

  return angularVelocity;
}

// Função para calcular o ângulo com base nos pulsos do encoder (em radianos)
float calculateAngle(long pulseCount1, long pulseCount2) {
  long totalPulseCount = pulseCount1 - pulseCount2;
  return (2 * PI * totalPulseCount) / PPR;  // Retorna o ângulo em radianos
}

// Função para processar a entrada serial para definir os setpoints (entrada em graus, cálculo em radianos)
void processSetpoint(String input) {
  input.trim();

  // Caso o comando seja apenas para pitch (pX)
  if (input.startsWith("p") && input.indexOf('y') == -1) {
    String pitchValue = input.substring(1);
    setpointPitch = constrain(radians(pitchValue.toFloat()), -pitchLimitRad, pitchLimitRad);  // Limita pitch em radianos
    Serial.println("Setpoint de Pitch atualizado.");
  }
  // Caso o comando seja apenas para yaw (yX)
  else if (input.startsWith("y") && input.indexOf('p') == -1) {
    String yawValue = input.substring(1);
    setpointYaw = constrain(radians(yawValue.toFloat()), -yawLimitRad, yawLimitRad);  // Limita yaw em radianos
    Serial.println("Setpoint de Yaw atualizado.");
  }
  // Caso o comando seja para pitch e yaw (pXyY)
  else if (input.startsWith("p") && input.indexOf('y') != -1) {
    int splitIndex = input.indexOf('y');
    String pitchValue = input.substring(1, splitIndex);
    String yawValue = input.substring(splitIndex + 1);

    setpointPitch = constrain(radians(pitchValue.toFloat()), -pitchLimitRad, pitchLimitRad);
    setpointYaw = constrain(radians(yawValue.toFloat()), -yawLimitRad, yawLimitRad);
    Serial.println("Setpoints de Pitch e Yaw atualizados.");
  }
}

// Função para lidar com o encoder de pitch
void handleEncoderPitch() {
  bool currentA = digitalRead(encoderPinAPitch);
  bool currentB = digitalRead(encoderPinBPitch);

  // Decodificação do encoder de quadratura
  if (currentA != lastAPitch || currentB != lastBPitch) {
    if (lastAPitch == currentB) {
      pulseCountClockwisePitch++;
    } else {
      pulseCountCounterClockwisePitch++;
    }
  }

  lastAPitch = currentA;
  lastBPitch = currentB;
}

// Função para lidar com o encoder de yaw
void handleEncoderYaw() {
  bool currentA = digitalRead(encoderPinAYaw);
  bool currentB = digitalRead(encoderPinBYaw);

  // Decodificação do encoder de quadratura
  if (currentA != lastAYaw || currentB != lastBYaw) {
    if (lastAYaw == currentB) {
      pulseCountClockwiseYaw++;
    } else {
      pulseCountCounterClockwiseYaw++;
    }
  }

  lastAYaw = currentA;
  lastBYaw = currentB;
}
