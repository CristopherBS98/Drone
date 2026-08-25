#include <Servo.h>

// Pinos dos ESCs
#define ESC1_PIN 3
#define ESC2_PIN 5
#define ESC3_PIN 9
#define ESC4_PIN 6

Servo esc1, esc2, esc3, esc4;

// Valores de pulso (em microssegundos)
const int deixa_motor_parado = 1000; // motor parado / armado
const int maxima_potencia = 2000; // potência máxima
const int ARM_DELAY = 2000;    // tempo para o ESC armar

void setup() {
  Serial.begin(9600);

  esc1.attach(ESC1_PIN);
  esc2.attach(ESC2_PIN);
  esc3.attach(ESC3_PIN);
  esc4.attach(ESC4_PIN);

  Serial.println("Armando ESCs...");
  setAllMotors(deixa_motor_parado);
  delay(ARM_DELAY); // espera o beep de confirmação dos ESCs

  Serial.println("ESCs armados. Iniciando teste em 3 segundos...");
  delay(3000);
}

void loop() {
  // Rampa de aceleração suave (0 a ~30% de potência, por segurança)
  Serial.println("Acelerando motores...");
  for (int pulse = deixa_motor_parado; pulse <= 1300; pulse += 4) {
   // setAllMotors(pulse);
    //delay(30);
  }

  delay(1000000); // mantém rodando por 2s

  Serial.println("Desacelerando motores...");
  for (int pulse = 1300; pulse >= deixa_motor_parado; pulse -= 5) {
    setAllMotors(pulse);
    delay(30);
  }

  Serial.println("Motores parados. Aguardando 5s...");
  delay(5000);
}

void setAllMotors(int pulse) {
  esc1.writeMicroseconds(pulse);
  esc2.writeMicroseconds(pulse);
  esc3.writeMicroseconds(pulse);
  esc4.writeMicroseconds(pulse);
}