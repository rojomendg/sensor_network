#include <Arduino.h>

#define MOISTURE_SOIL_SENSOR_PIN_0 A0
#define MOISTURE_SOIL_SENSOR_PIN_1 A1
#define SENS_PWR 7   // Pin de control del TIP42C (alto-lado)

// Calibración humedad suelo
#define DRY_SOIL 600  // Valor en seco
#define WET_SOIL 200   // Valor en húmedo

void setup() {
  Serial.begin(9600);
  pinMode(SENS_PWR, OUTPUT);

  // Mantener siempre encendido el TIP42C (LOW = ON)
  digitalWrite(SENS_PWR, LOW);

  Serial.println("Leyendo humedad del suelo continuamente...");
}

void loop() {
  // Leer 3 veces para mayor estabilidad
  

  int soil_sensor_0 = analogRead(MOISTURE_SOIL_SENSOR_PIN_0);

  // Convertir a porcentaje (0 = seco, 100 = húmedo)
  int soil_moisture_0 = map(soil_sensor_0, DRY_SOIL, WET_SOIL, 0, 100);
  soil_moisture_0 = constrain(soil_moisture_0, 0, 100);

  int soil_sensor_1 = analogRead(MOISTURE_SOIL_SENSOR_PIN_1);

  // Convertir a porcentaje (0 = seco, 100 = húmedo)
  int soil_moisture_1 = map(soil_sensor_1, DRY_SOIL, WET_SOIL, 0, 100);
  soil_moisture_1 = constrain(soil_moisture_1, 0, 100);


  // Mostrar resultados
  Serial.print("Raw 1: ");
  Serial.print(soil_sensor_0);
  Serial.print("  |  Humedad 1 (%): ");
  Serial.print(soil_moisture_0);
  Serial.print("  |  Raw 2: ");
  Serial.print(soil_sensor_1);
  Serial.print("  |  Humedad 2 (%): ");
  Serial.println(soil_moisture_1);

  delay(500); // Actualiza cada segundo
}
