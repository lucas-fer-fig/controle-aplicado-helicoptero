#include <Servo.h>  // Biblioteca para controle do ESC com Servo

// Definindo pinos para sinal dos motores (ESCs) e encoders
#define signalPinPitch 45    // Pino de controle do ESC para pitch
#define signalPinYaw 46      // Pino de controle do ESC para yaw
#define encoderPinAPitch 18  // Pino do sinal A do encoder de pitch
#define encoderPinBPitch 19  // Pino do sinal B do encoder de pitch
#define encoderPinAYaw 20    // Pino do sinal A do encoder de yaw
#define encoderPinBYaw 21    // Pino do sinal B do encoder de yaw

/*========================================================================================================
****************************************** Secção de Calibração ******************************************
===========================================================================================================*/
// Limites de ângulo para pitch
const float pitchLimitRadMax = radians(25);
const float pitchLimitRadMin = radians(-15);

const float controlPotMaxPitch = 70;
const float controlPotMaxYaw = 40;

const float maxVPitch = 30.0;
const float minVPitch = -30.0;
const float maxVYaw = 50.0;
const float minVYaw = 0.0;

const float integralMaxPitch = 1000.0;
const float integralMaxYaw = 0.25;

// Variáveis para o controle PID do pitch
const float KpPitch = 65, KiPitch = 45.0, KdPitch = 5.5;
float integralPitch = 0.0, previousErrorPitch = 0.0;

// Variáveis para o controle PID do yaw
const float KpYaw = 80.0, KiYaw = 90.0, KdYaw = 25.0;
float integralYaw = 0.0, previousErrorYaw = 0.0;
//========================================================================================================
const float PotMaxPitch = (controlPotMaxPitch / 100.0) * 180.0;
const float PotMaxYaw = (controlPotMaxYaw / 100.0) * 180.0;

// Declaração dos objetos Servo para ESCs
Servo escPitch;
Servo escYaw;

float setpointPitch = 0.0;
float setpointYaw = 0.0;

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

// Variáveis de tempo para cálculo do intervalo de tempo dinâmico
unsigned long lastTimePID = 0;

void setup() {
  // Inicializa ESCs
  escPitch.attach(signalPinPitch, 1000, 2000);
  escYaw.attach(signalPinYaw, 1000, 2000);

  // Configura pinos dos encoders
  pinMode(encoderPinAPitch, INPUT_PULLUP);
  pinMode(encoderPinBPitch, INPUT_PULLUP);
  pinMode(encoderPinAYaw, INPUT_PULLUP);
  pinMode(encoderPinBYaw, INPUT_PULLUP);

  Serial.begin(250000);  // Inicia comunicação serial
  Serial.println("Dê o comando de inicialização.");

  while (true) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      if (input == "a") break;  // Processa o input serial para definir os setpoints
    }
    escPitch.write(0);
    escYaw.write(0);
  }

  // Configura interrupções para os pinos A e B de pitch e yaw
  attachInterrupt(digitalPinToInterrupt(encoderPinAPitch), handleEncoderPitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinBPitch), handleEncoderPitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinAYaw), handleEncoderYaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinBYaw), handleEncoderYaw, CHANGE);
}

void loop() {
  // Calcula o intervalo de tempo desde o último ciclo
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTimePID) / 1000.0;  // Convertendo para segundos

  // Verificação para garantir que dt nunca seja zero ou extremamente pequeno
  if (dt < 0.001) dt = 0.001;  // Define um limite mínimo para dt de 1ms

  lastTimePID = currentTime;

  // Verifica se há dados disponíveis na serial (para setpoints)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    processSetpoint(input, setpointPitch, pitchLimitRadMin, pitchLimitRadMax);  // Processa o input serial para definir os setpoints
  }

  // Calcula os ângulos baseados nos encoders (em radianos)
  float pitchAngleRad = calculateAngle(pulseCountCounterClockwisePitch, pulseCountClockwisePitch);
  float yawAngleRad = calculateAngle(pulseCountClockwiseYaw, pulseCountCounterClockwiseYaw);

  // Calcula os sinais de controle (tensões) usando PID
  float VPitch = pidControl(setpointPitch, pitchAngleRad, integralPitch, previousErrorPitch, KpPitch, KiPitch, KdPitch, dt, minVPitch, maxVPitch, -integralMaxPitch, integralMaxPitch);
  float VYaw = pidControl(setpointYaw, yawAngleRad, integralYaw, previousErrorYaw, KpYaw, KiYaw, KdYaw, dt, minVYaw, maxVYaw, -integralMaxYaw, integralMaxYaw);

  // Controla os motores com as tensões calculadas
  int pwmPitch = controlMotor(escPitch, VPitch, minVPitch, maxVPitch, PotMaxPitch);
  int pwmYaw = controlMotor(escYaw, VYaw, minVYaw, maxVYaw, PotMaxYaw);

  // Calcula a porcentagem de potência para cada motor
  float powerPercentPitch = (pwmPitch / PotMaxPitch) * 100.0;
  float powerPercentYaw = (pwmYaw / PotMaxYaw) * 100.0;

  // Exibe os ângulos, tensões e porcentagem de potência no monitor serial
  Serial.print(degrees(pitchAngleRad));  // Convertendo de radianos para graus
  Serial.print(" ");
  Serial.print(degrees(yawAngleRad));  // Convertendo de radianos para graus
  Serial.print(" ");
  Serial.print(VPitch);
  Serial.print(" ");
  Serial.print(VYaw);
  Serial.print(" ");
  Serial.print(powerPercentPitch);
  Serial.print(" ");
  Serial.print(powerPercentYaw);
  Serial.print(" ");
  Serial.println(dt, 5);
}

void processSetpoint(String input, float &setpoint, float minAngle, float maxAngle) {
  input.trim();
  // Caso o comando seja apenas para pitch (pX)
  if (input.startsWith("p")) {
    String pitchValue = input.substring(1);
    setpoint = constrain(radians(pitchValue.toFloat()), minAngle, maxAngle);
    Serial.println("Setpoint de Pitch atualizado.");
  }
}

float pidControl(float setpoint, float currentAngle, float &integral, float &previousError, float Kp, float Ki, float Kd, float dt, float minVoltage, float maxVoltage, float integralMin, float integralMax) {
  float error = setpoint - currentAngle;

  integral += error * dt;  // Integral baseada no tempo de ciclo
  integral = constrain(integral, integralMin, integralMax);

  float derivative = (error - previousError) / dt;  // Derivada baseada no tempo de ciclo
  previousError = error;

  // Calcula o sinal de controle
  float output = Kp * error + Ki * integral + Kd * derivative;

  return constrain(output, minVoltage, maxVoltage);
}

// Função para controlar o motor (ESC) com base na tensão de controle
int controlMotor(Servo &esc, float voltage, float minVoltage, float maxVoltage, float maxPwm) {
  int pwmValue = map(voltage, minVoltage, maxVoltage, 0, maxPwm);
  pwmValue = constrain(pwmValue, 0, maxPwm);  // Garante que o valor do PWM esteja entre 0 e Pitch
  esc.write(pwmValue);                        // Define o valor de controle do motor

  return pwmValue;  // Retorna o valor do PWM para cálculo da porcentagem de potência
}

// Função para calcular o ângulo com base nos pulsos do encoder (em radianos)
float calculateAngle(long pulseCount1, long pulseCount2) {
  long totalPulseCount = pulseCount1 - pulseCount2;

  return (2 * PI * totalPulseCount) / PPR;
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
