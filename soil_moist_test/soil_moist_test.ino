#include <Arduino.h>

#define MOISTURE_SOIL_SENSOR_PIN A0
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
  

  int soil_sensor = analogRead(MOISTURE_SOIL_SENSOR_PIN);

  // Convertir a porcentaje (0 = seco, 100 = húmedo)
  int soil_moisture = map(soil_sensor, DRY_SOIL, WET_SOIL, 0, 100);
  soil_moisture = constrain(soil_moisture, 0, 100);

  // Mostrar resultados
  Serial.print("Lectura raw: ");
  Serial.print(soil_sensor);
  Serial.print("  |  Humedad (%): ");
  Serial.println(soil_moisture);

  delay(1000); // Actualiza cada segundo
}
