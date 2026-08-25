/*
 * ===================================================================
 * CONTROLADORA DE VOO - QUADRICOPTERO MODELO X - Arduino UNO
 * VERSAO CORRIGIDA - RAM otimizada com macro F() nas strings
 * ===================================================================
 * O UNO so tem 2048 bytes de RAM. Strings de texto usadas em
 * Serial.print("...") sao copiadas para a RAM por padrao no AVR.
 * A macro F("...") mantem o texto na memoria Flash (32KB, bem mais
 * folgada) em vez de RAM. Isso resolve o erro:
 *   "data section exceeds available space in board"
 *
 * *** ATENCAO - SEGURANCA ***
 * - NUNCA teste com helices instaladas antes de validar toda a logica.
 * - Os motores SO respondem se o canal ARM estiver ligado E o throttle
 *   estiver baixo no momento de armar.
 * - Failsafe: se o receptor parar de enviar sinal por mais de 500ms,
 *   os motores sao desarmados automaticamente.
 * ===================================================================
 */

#include <Wire.h>
#include <Servo.h>
#include <LSM303.h>
#include <L3G.h>
#include <Adafruit_VL53L0X.h>

LSM303 compass;
L3G gyro;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ===================== MOTORES (ESC) =====================
Servo motor1; // Frente-Esquerda
Servo motor2; // Frente-Direita
Servo motor3; // Tras-Direita
Servo motor4; // Tras-Esquerda
const int PINO_M1 = 3, PINO_M2 = 5, PINO_M3 = 6, PINO_M4 = 9;

// ===================== RECEPTOR RC (5 canais) =====================
volatile uint16_t rc_ch1_raw = 1500;
volatile uint16_t rc_ch2_raw = 1500;
volatile uint16_t rc_ch3_raw = 1500;
volatile uint16_t rc_ch4_raw = 1500;
volatile uint16_t rc_ch5_raw = 1500;

volatile uint32_t rc_ultima_atualizacao = 0;

uint32_t timer_ch1 = 0, timer_ch2 = 0, timer_ch3 = 0, timer_ch4 = 0, timer_ch5 = 0;

// ===================== IMU =====================
float roll_filtrado = 0.0, pitch_filtrado = 0.0;
float gyroX_offset = 0.0, gyroY_offset = 0.0, gyroZ_offset = 0.0;
float roll_tare = 0.0, pitch_tare = 0.0;
float gz_filtrado = 0.0;
unsigned long tempo_anterior_imu = 0;
float dt_controle = 0.01;

float Kp_roll = 1.2, Ki_roll = 0.01, Kd_roll = 0.5;
float Kp_pitch = 1.2, Ki_pitch = 0.01, Kd_pitch = 0.5;
float erro_roll = 0, erro_anterior_roll = 0, integral_roll = 0;
float erro_pitch = 0, erro_anterior_pitch = 0, integral_pitch = 0;

float Kp_yaw = 1.5, Ki_yaw = 0.0;
float integral_yaw = 0;

// ===================== ARM / SEGURANCA =====================
bool armado = false;
const uint16_t ARM_SWITCH_LIGADO   = 1700;
const uint16_t THROTTLE_MINIMO_ARM = 1100;
const uint32_t RECEPTOR_TIMEOUT_MS = 500;

// ===================== ALTURA (VL53L0X) =====================
const int OFFSET_ALTURA_MM = 145;
bool altimetro_ok = false;
int altura_atual_mm = -1;
uint32_t ultima_leitura_altura = 0;
const uint32_t INTERVALO_ALTURA_MS = 150;

// ===================== DEBUG =====================
uint32_t ultimo_debug = 0;

// ===================================================================
// INTERRUPCOES DO RECEPTOR
// ===================================================================
void ISR_CH1() {
  if (digitalRead(2) == HIGH) {
    timer_ch1 = micros();
  } else if (timer_ch1 > 0) {
    rc_ch1_raw = micros() - timer_ch1;
    rc_ultima_atualizacao = millis();
  }
}

ISR(PCINT2_vect) {
  if (digitalRead(7) == HIGH) {
    timer_ch2 = micros();
  } else if (timer_ch2 > 0) {
    rc_ch2_raw = micros() - timer_ch2;
    rc_ultima_atualizacao = millis();
  }
}

ISR(PCINT0_vect) {
  uint32_t agora = micros();

  if (digitalRead(8) == HIGH) {
    timer_ch3 = agora;
  } else if (timer_ch3 > 0) {
    rc_ch3_raw = agora - timer_ch3;
    rc_ultima_atualizacao = millis();
  }

  if (digitalRead(11) == HIGH) {
    timer_ch4 = agora;
  } else if (timer_ch4 > 0) {
    rc_ch4_raw = agora - timer_ch4;
    rc_ultima_atualizacao = millis();
  }

  if (digitalRead(12) == HIGH) {
    timer_ch5 = agora;
  } else if (timer_ch5 > 0) {
    rc_ch5_raw = agora - timer_ch5;
    rc_ultima_atualizacao = millis();
  }
}

// ===================================================================
// SETUP - com print em cada etapa (agora usando F() p/ economizar RAM)
// ===================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("### BOOT: Serial OK ###"));

  Wire.begin();
  Wire.setWireTimeout(25000, true);
  Serial.println(F("[1/6] I2C iniciado."));

  configurarPinosReceptor();
  configurarInterrupcoes();
  Serial.println(F("[2/6] Receptor configurado."));

  motor1.attach(PINO_M1, 1000, 2000);
  motor2.attach(PINO_M2, 1000, 2000);
  motor3.attach(PINO_M3, 1000, 2000);
  motor4.attach(PINO_M4, 1000, 2000);
  pararMotores();
  Serial.println(F("[3/6] Motores parados (1000us)."));

  Serial.println(F("Iniciando IMU..."));
  bool compassOk = compass.init();
  Serial.println(compassOk ? F("  LSM303 OK.") : F("  LSM303 FALHOU!"));

  bool gyroOk = gyro.init();
  Serial.println(gyroOk ? F("  L3G OK.") : F("  L3G FALHOU!"));

  if (!compassOk || !gyroOk) {
    Serial.println(F("### ERRO FATAL: IMU nao encontrada. Verifique fiacao I2C. ###"));
    while (1) { delay(1000); }
  }
  compass.enableDefault();
  gyro.enableDefault();
  Serial.println(F("[4/6] IMU OK."));

  if (!lox.begin()) {
    Serial.println(F("  AVISO: VL53L0X nao encontrado."));
    altimetro_ok = false;
  } else {
    altimetro_ok = true;
    Serial.println(F("  Altimetro OK."));
  }
  Serial.println(F("[5/6] Altimetro concluido."));

  delay(500);
  Serial.println(F("Calibrando giroscopio - NAO MEXA..."));
  calibrarGiroscopio();
  tararIMU();
  Serial.println(F("[6/6] Calibracao concluida."));

  tempo_anterior_imu = micros();
  Serial.println(F("=== SISTEMA PRONTO. DESARMADO. ==="));
}

// ===================================================================
// LOOP PRINCIPAL
// ===================================================================
void loop() {
  atualizarFailsafe();
  lerIMU();
  atualizarAltura();
  processarControle();

  if (millis() - ultimo_debug > 100) {
    ultimo_debug = millis();
    imprimirDebug();
  }
}

void configurarPinosReceptor() {
  pinMode(2, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
}

void configurarInterrupcoes() {
  attachInterrupt(digitalPinToInterrupt(2), ISR_CH1, CHANGE);
  PCICR  |= (1 << PCIE2) | (1 << PCIE0);
  PCMSK2 |= (1 << PCINT23);
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT3) | (1 << PCINT4);
}

void calibrarGiroscopio() {
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 200; i++) {
    gyro.read();
    sumX += gyro.g.x;
    sumY += gyro.g.y;
    sumZ += gyro.g.z;
    delay(5);
  }
  gyroX_offset = (sumX / 200.0) * 0.00875;
  gyroY_offset = (sumY / 200.0) * 0.00875;
  gyroZ_offset = (sumZ / 200.0) * 0.00875;
}

void tararIMU() {
  compass.read();
  float ax = compass.a.x, ay = compass.a.y, az = compass.a.z;
  roll_tare  = atan2(ay, az) * 180.0 / M_PI;
  pitch_tare = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
}

void lerIMU() {
  unsigned long agora = micros();
  float dt = (agora - tempo_anterior_imu) / 1000000.0;
  tempo_anterior_imu = agora;
  if (dt <= 0) dt = 0.001;
  dt_controle = dt;

  compass.read();
  gyro.read();

  float ax = compass.a.x, ay = compass.a.y, az = compass.a.z;
  float roll_acc  = (atan2(ay, az) * 180.0 / M_PI) - roll_tare;
  float pitch_acc = (atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI) - pitch_tare;

  float gx = (gyro.g.x * 0.00875) - gyroX_offset;
  float gy = (gyro.g.y * 0.00875) - gyroY_offset;
  float gz = (gyro.g.z * 0.00875) - gyroZ_offset;

  const float alpha = 0.98;
  roll_filtrado  = alpha * (roll_filtrado + gx * dt)  + (1.0 - alpha) * roll_acc;
  pitch_filtrado = alpha * (pitch_filtrado + gy * dt) + (1.0 - alpha) * pitch_acc;
  gz_filtrado = gz;
}

void atualizarAltura() {
  if (!altimetro_ok) return;

  uint32_t agora = millis();
  if (agora - ultima_leitura_altura < INTERVALO_ALTURA_MS) return;
  ultima_leitura_altura = agora;

  VL53L0X_RangingMeasurementData_t medida;
  lox.rangingTest(&medida, false);

  if (medida.RangeStatus != 4) {
    int altura = medida.RangeMilliMeter - OFFSET_ALTURA_MM;
    if (altura < 0) altura = 0;
    altura_atual_mm = altura;
  }
}

void atualizarFailsafe() {
  if (millis() - rc_ultima_atualizacao > RECEPTOR_TIMEOUT_MS) {
    if (armado) {
      Serial.println(F("!!! FAILSAFE: sinal perdido. Desarmando. !!!"));
    }
    armado = false;
  }
}

void processarControle() {
  int rc_roll  = constrain(rc_ch1_raw, 1000, 2000);
  int rc_pitch = constrain(rc_ch2_raw, 1000, 2000);
  int rc_throt = constrain(rc_ch3_raw, 1000, 2000);
  int rc_yaw   = constrain(rc_ch4_raw, 1000, 2000);
  int rc_arm   = constrain(rc_ch5_raw, 1000, 2000);

  bool switch_ligado = rc_arm > ARM_SWITCH_LIGADO;

  if (switch_ligado && !armado && rc_throt < THROTTLE_MINIMO_ARM) {
    armado = true;
    integral_roll = 0; integral_pitch = 0; integral_yaw = 0;
    Serial.println(F(">>> ARMADO <<<"));
  }
  if (!switch_ligado && armado) {
    armado = false;
    Serial.println(F(">>> DESARMADO <<<"));
  }

  if (!armado) {
    pararMotores();
    integral_roll = 0; integral_pitch = 0; integral_yaw = 0;
    return;
  }

  float setpoint_roll      = map(rc_roll,  1000, 2000, -30, 30);
  float setpoint_pitch     = map(rc_pitch, 1000, 2000, -30, 30);
  float setpoint_yaw_rate  = map(rc_yaw,   1000, 2000, -180, 180);

  if (abs(rc_roll  - 1500) < 20) setpoint_roll = 0.0;
  if (abs(rc_pitch - 1500) < 20) setpoint_pitch = 0.0;
  if (abs(rc_yaw   - 1500) < 30) setpoint_yaw_rate = 0.0;

  float dt = dt_controle;

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

  float erro_yaw = setpoint_yaw_rate - gz_filtrado;
  integral_yaw = constrain(integral_yaw + (erro_yaw * dt), -100, 100);
  float output_yaw = (Kp_yaw * erro_yaw) + (Ki_yaw * integral_yaw);

  int pwm_m1 = rc_throt + output_pitch + output_roll - output_yaw;
  int pwm_m2 = rc_throt + output_pitch - output_roll + output_yaw;
  int pwm_m3 = rc_throt - output_pitch - output_roll - output_yaw;
  int pwm_m4 = rc_throt - output_pitch + output_roll + output_yaw;

  motor1.writeMicroseconds(constrain(pwm_m1, 1000, 2000));
  motor2.writeMicroseconds(constrain(pwm_m2, 1000, 2000));
  motor3.writeMicroseconds(constrain(pwm_m3, 1000, 2000));
  motor4.writeMicroseconds(constrain(pwm_m4, 1000, 2000));
}

void pararMotores() {
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
}

void imprimirDebug() {
  Serial.print(armado ? F("ARMADO") : F("desarmado"));
  Serial.print(F(" | T: "));  Serial.print(rc_ch3_raw);
  Serial.print(F(" | R: "));  Serial.print(roll_filtrado, 1);
  Serial.print(F(" | P: "));  Serial.print(pitch_filtrado, 1);
  Serial.print(F(" | Y: "));  Serial.print(gz_filtrado, 1);
  Serial.print(F(" | Alt: ")); Serial.print(altura_atual_mm);
  Serial.println(F(" mm"));
}
