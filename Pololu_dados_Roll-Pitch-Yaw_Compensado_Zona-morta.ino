#include <Wire.h>
#include <Servo.h>
#include <LSM303.h>
#include <L3G.h>

LSM303 compass;
L3G gyro;

// Declaração dos 4 ESCs / Motores
Servo motor1; // Frente-Esquerda (Pino 3)
Servo motor2; // Frente-Direita (Pino 5)
Servo motor3; // Trás-Direita (Pino 6)
Servo motor4; // Trás-Esquerda (Pino 9)

const int PINO_M1 = 3, PINO_M2 = 5, PINO_M3 = 6, PINO_M4 = 9;

// --- VARIÁVEIS DO RECEPTOR RC (MICROSEGUNDOS) ---
volatile uint16_t rc_ch1_raw = 1500; // Roll
volatile uint16_t rc_ch2_raw = 1500; // Pitch
volatile uint16_t rc_ch3_raw = 1000; // Throttle
volatile uint16_t rc_ch4_raw = 1500; // Yaw

// Temporizadores para medir o comprimento dos pulsos
uint32_t timer_ch1 = 0, timer_ch2 = 0, timer_ch3 = 0, timer_ch4 = 0;

// Variáveis da IMU
float roll_filtrado = 0.0, pitch_filtrado = 0.0;
float gyroX_offset = 0.0, gyroY_offset = 0.0;
float roll_tare = 0.0, pitch_tare = 0.0;
unsigned long tempo_anterior = 0;

// Ganhos PID
float Kp_roll = 1.2, Ki_roll = 0.01, Kd_roll = 0.5;
float Kp_pitch = 1.2, Ki_pitch = 0.01, Kd_pitch = 0.5;

float erro_roll = 0, erro_anterior_roll = 0, integral_roll = 0;
float erro_pitch = 0, erro_anterior_pitch = 0, integral_pitch = 0;

// =========================================================================
// INTERRUPÇÕES DO RECEPTOR RC (LEITURA EM SEGUNDO PLANO)
// =========================================================================

// Interrupção no Pino 2 (CH1 - Roll)
void ISR_CH1() {
  if (digitalRead(2) == HIGH) timer_ch1 = micros();
  else if (timer_ch1 > 0) rc_ch1_raw = micros() - timer_ch1;
}

// Interrupções gerais para os pinos 7, 8 e 11
ISR(PCINT2_vect) { // Pino 7 (CH2 - Pitch)
  if (digitalRead(7) == HIGH) timer_ch2 = micros();
  else if (timer_ch2 > 0) rc_ch2_raw = micros() - timer_ch2;
}

ISR(PCINT0_vect) { // Pinos 8 (CH3 - Throttle) e 11 (CH4 - Yaw)
  uint32_t agora = micros();
  // Pino 8
  if (digitalRead(8) == HIGH) timer_ch3 = agora;
  else if (timer_ch3 > 0) rc_ch3_raw = agora - timer_ch3;

  // Pino 11
  if (digitalRead(11) == HIGH) timer_ch4 = agora;
  else if (timer_ch4 > 0) rc_ch4_raw = agora - timer_ch4;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Configura Pinos do Receptor como Entrada
  pinMode(2, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(11, INPUT);

  // Ativa Interrupção Externa no Pino 2
  attachInterrupt(digitalPinToInterrupt(2), ISR_CH1, CHANGE);

  // Ativa Interrupções Pin Change para os Pinos 7, 8 e 11
  PCICR  |= (1 << PCIE2) | (1 << PCIE0); // Habilita Banco PCINT2 (Pino 7) e PCINT0 (Pinos 8 e 11)
  PCMSK2 |= (1 << PCINT23);             // Pino 7
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT3); // Pinos 8 e 11

  // Associa ESCs
  motor1.attach(PINO_M1, 1000, 2000);
  motor2.attach(PINO_M2, 1000, 2000);
  motor3.attach(PINO_M3, 1000, 2000);
  motor4.attach(PINO_M4, 1000, 2000);

  pararMotores();

  if (!compass.init() || !gyro.init()) {
    Serial.println("ERRO: Falha na IMU Pololu!");
    while (1);
  }

  compass.enableDefault();
  gyro.enableDefault();

  delay(1000);

  // 1. Calibração do Giroscópio
  long sumX = 0, sumY = 0;
  for (int i = 0; i < 100; i++) {
    gyro.read();
    sumX += gyro.g.x;
    sumY += gyro.g.y;
    delay(10);
  }
  gyroX_offset = (sumX / 100.0) * 0.00875;
  gyroY_offset = (sumY / 100.0) * 0.00875;

  // 2. Tara Inicial da IMU
  compass.read();
  float ax = compass.a.x, ay = compass.a.y, az = compass.a.z;
  roll_tare = atan2(ay, az) * 180.0 / M_PI;
  pitch_tare = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

  tempo_anterior = micros();
  Serial.println("Receptor e IMU Inicializados!");
}

void loop() {
  // 1. Cálculo do Delta Time (dt)
  unsigned long tempo_atual = micros();
  float dt = (tempo_atual - tempo_anterior) / 1000000.0;
  tempo_anterior = tempo_atual;
  if (dt <= 0) dt = 0.001;

  // 2. Leitura dos Sensores
  compass.read();
  gyro.read();

  // --- FILTRO COMPLEMENTAR DA IMU ---
  float ax = compass.a.x, ay = compass.a.y, az = compass.a.z;
  float roll_acc = (atan2(ay, az) * 180.0 / M_PI) - roll_tare;
  float pitch_acc = (atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI) - pitch_tare;

  float gx = (gyro.g.x * 0.00875) - gyroX_offset;
  float gy = (gyro.g.y * 0.00875) - gyroY_offset;

  const float alpha = 0.98;
  roll_filtrado = alpha * (roll_filtrado + gx * dt) + (1.0 - alpha) * roll_acc;
  pitch_filtrado = alpha * (pitch_filtrado + gy * dt) + (1.0 - alpha) * pitch_acc;

  // --- CONVERSÃO DOS SINAIS DO RÁDIO RC EM SETPOINTS ---
  // Limita a leitura física do rádio caso haja ruído
  int rc_roll  = constrain(rc_ch1_raw, 1000, 2000);
  int rc_pitch = constrain(rc_ch2_raw, 1000, 2000);
  int rc_throt = constrain(rc_ch3_raw, 1000, 2000);

  // Mapeia os Sticks de Roll e Pitch para Setpoints de ângulo entre -30° e +30°
  float setpoint_roll  = map(rc_roll, 1000, 2000, -30, 30);
  float setpoint_pitch = map(rc_pitch, 1000, 2000, -30, 30);

  // Aplica uma zona morta central nos sticks (entre 1480us e 1520us = 0 graus)
  if (abs(rc_roll - 1500) < 20) setpoint_roll = 0.0;
  if (abs(rc_pitch - 1500) < 20) setpoint_pitch = 0.0;

  // --- CÁLCULO DO CONTROLADOR PID ---
  erro_roll = setpoint_roll - roll_filtrado;
  integral_roll = constrain(integral_roll + (erro_roll * dt), -100, 100);
  float deriv_roll = (erro_roll - erro_anterior_roll) / dt;
  float output_roll = (Kp_roll * erro_roll) + (Ki_roll * integral_roll) + (Kd_roll * deriv_roll);
  erro_anterior_roll = erro_roll;

  erro_pitch = setpoint_pitch - pitch_filtrado;
  integral_pitch = constrain(integral_pitch + (erro_pitch * dt), -100, 100);
  float deriv_pitch = (erro_pitch - erro_anterior_pitch) / dt;
  float output_pitch = (Kp_pitch * erro_pitch) + (Ki_pitch * integral_pitch) + (Kd_pitch * deriv_pitch);
  erro_anterior_pitch = erro_pitch;

  // --- MISTURA DOS MOTORES (MIXER QUAD-X) ---
  // Apenas aciona se o stick de aceleração (Throttle) estiver acima do mínimo de segurança
  if (rc_throt > 1100) {
    int pwm_m1 = rc_throt + output_pitch + output_roll;
    int pwm_m2 = rc_throt + output_pitch - output_roll;
    int pwm_m3 = rc_throt - output_pitch - output_roll;
    int pwm_m4 = rc_throt - output_pitch + output_roll;

    motor1.writeMicroseconds(constrain(pwm_m1, 1000, 1800));
    motor2.writeMicroseconds(constrain(pwm_m2, 1000, 1800));
    motor3.writeMicroseconds(constrain(pwm_m3, 1000, 1800));
    motor4.writeMicroseconds(constrain(pwm_m4, 1000, 1800));
  } else {
    // Se o Throttle estiver totalmente abaixado, zera as integrais e desliga os motores
    integral_roll = 0;
    integral_pitch = 0;
    pararMotores();
  }

  // Debug Serial
  Serial.print("Throttle RC: "); Serial.print(rc_throt);
  Serial.print("us | Setpoint Roll: "); Serial.print(setpoint_roll, 1);
  Serial.print("° | Roll Real: "); Serial.print(roll_filtrado, 1);
  Serial.println("°");

  delay(10);
}

void pararMotores() {
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
}