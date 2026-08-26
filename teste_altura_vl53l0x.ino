/*
 * ===================================================================
 * TESTE - SENSOR DE ALTURA VL53L0X - Arduino UNO
 * ===================================================================
 * Le a distancia via I2C (A4=SDA, A5=SCL), aplica o offset de
 * montagem (145mm) e mostra a altura em relacao ao solo.
 *
 * Se este for o VL53L1X (sucessor, mais alcance), o codigo muda -
 * veja nota no final da explicacao.
 * ===================================================================
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int OFFSET_MM = 145;      // altura do sensor ate o solo, com o drone pousado
const int ALTURA_MAX_MM = 2000; // altura maxima esperada de voo (2m)

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  Serial.println("Iniciando VL53L0X...");

  if (!lox.begin()) {
    Serial.println("Erro: sensor VL53L0X nao encontrado. Verifique as conexoes!");
    while (1) { delay(1000); }
  }

  // Ajustes moderados para melhorar estabilidade sem exagerar no alcance
  lox.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.2 * 65536));
  lox.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(30 * 65536));
  lox.setMeasurementTimingBudgetMicroSeconds(66000); // 66ms - bom equilibrio range x velocidade

  Serial.println("Sensor pronto!");
  Serial.println("---------------------------------------------");
  Serial.println("Distancia bruta (mm) | Status | Sinal (Mcps) | Altura (mm)");
  Serial.println("---------------------------------------------");
}

void loop() {
  VL53L0X_RangingMeasurementData_t medida;
  lox.rangingTest(&medida, false);

  Serial.print("RAW: ");
  Serial.print(medida.RangeMilliMeter);
  Serial.print(" mm | Status: ");
  Serial.print(medida.RangeStatus);
  Serial.print(" | Sinal: ");
  Serial.print(medida.SignalRateRtnMegaCps / 65536.0, 3);

  if (medida.RangeStatus != 4) { // 4 = fora de alcance / invalido
    int altura = medida.RangeMilliMeter - OFFSET_MM;
    if (altura < 0) altura = 0;
    if (altura > ALTURA_MAX_MM) altura = ALTURA_MAX_MM;

    Serial.print(" | Altura: ");
    Serial.print(altura);
    Serial.println(" mm");
  } else {
    Serial.println(" | FORA DE ALCANCE");
  }

  delay(100);
}
