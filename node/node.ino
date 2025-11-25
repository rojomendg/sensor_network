#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <LowPower.h>
#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT22
#define MOISTURE_SOIL_SENSOR_PIN A0

// --- NUEVO: pin que controla el TIP42C (alto-lado de VCC para DHT22 + YL69)
#define SENS_PWR 7   // LOW = ON, HIGH = OFF (con pull-up en la base del TIP42C)

// Calibración humedad suelo
#define DRY_SOIL 1023//785
#define WET_SOIL 0 //350

const uint8_t MAX_RETRIES = 5;
const unsigned long ACK_TIMEOUT = 1000;
bool ack = false;

DHT dht(DHTPIN, DHTTYPE);

// RF24
RF24 radio(9, 10);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

#define nodeID 2

// RTC DS3231
RTC_DS3231 rtc;
const int alarmPin = 3;
const int outputPin = 4;
volatile bool alarmFlag = false;

int intervalMinutes = 10;
bool confirmationReceived = false;

struct __attribute__((packed)) SensorData {
  float    humidity;
  float    temperature;
  int16_t  moistureLevel;
  uint8_t  nodeId;
};
SensorData data;

struct TimeData {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};
TimeData timeData;

bool reqAndReceiveTime();
void configureAlarm(int intervalMinutes);
void readSensor();
bool sendSensorData();
bool tryRecoverRadioMesh();

uint32_t displayTimer = 0;

struct payload_t {
  unsigned long ms;
  unsigned long counter;
};

void onAlarm() { alarmFlag = true; }

// =========================
// PATCH: Join rápido con timeout (no bloqueante)
// =========================
bool tryMeshJoinWithin(uint32_t join_ms) {
  uint32_t t0 = millis();

  // Arranque del stack sin bucles bloqueantes
  mesh.begin();

  // Ventana corta de intentos de DHCP
  while (millis() - t0 < join_ms) {
    mesh.update();
    if (mesh.checkConnection()) return true;
    mesh.renewAddress();         // intento rápido; si no hay master, falla enseguida
    delay(50);
  }
  return mesh.checkConnection();
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { Serial.println("Stuck 1"); }

  pinMode(alarmPin, INPUT_PULLUP);
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, LOW);

  // TIP42C: arranca APAGADO por defecto
  pinMode(SENS_PWR, OUTPUT);
  digitalWrite(SENS_PWR, HIGH); // HIGH = OFF

  // RTC
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, resetting time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    EEPROM.update(0, 0);
  }
  attachInterrupt(digitalPinToInterrupt(alarmPin), onAlarm, FALLING);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.disable32K();
  rtc.writeSqwPinMode(DS3231_OFF);

  // Mesh
  mesh.setNodeID(nodeID);
  data.nodeId = nodeID;

  radio.begin();
  radio.setPALevel(RF24_PA_MIN, 0);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.setRetries(5, 15);

  // =========================
  // PATCH: Join rápido en setup (sin bloqueo)
  // =========================
  Serial.println(F("Fast mesh join..."));
  bool joined = tryMeshJoinWithin(1500);  // ~1.5s
  if (!joined) {
    Serial.println(F("No master -> sleep until next window"));
    configureAlarm(intervalMinutes);
    radio.powerDown();
    delay(100); //ADDED
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }

  // =========================
  // PATCH: Sin bucle infinito de sincronización
  // (intento único y continúas; el DS3231 mantiene buen tiempo)
  // =========================
  if (!reqAndReceiveTime()) {
    Serial.println("Not able to synchronize time now");
  }

  Serial.println("Sensor Node Ready");
  DateTime now = rtc.now();
  Serial.print("Current time: ");
  Serial.print(now.year()); Serial.print("-");
  Serial.print(now.month()); Serial.print("-");
  Serial.print(now.day()); Serial.print(" ");
  Serial.print(now.hour()); Serial.print(":");
  Serial.print(now.minute()); Serial.print(":");
  Serial.println(now.second());

  configureAlarm(intervalMinutes);
  // dht.begin() justo antes de leer tras energizar
}

void loop() {
  mesh.update();

  if (alarmFlag) {
    displayTimer = millis();
    digitalWrite(outputPin, HIGH);
    alarmFlag = false;
    Serial.println("Waking up and sending data...");

    // Radio arriba
    radio.powerUp();
    delay(5);

    // =========================
    // PATCH: Join corto dentro de la ventana
    // =========================
    if (!mesh.checkConnection()) {
      if (!tryMeshJoinWithin(1200)) { // ~1.2s
        Serial.println(F("No mesh this window -> sleep again"));
        configureAlarm(intervalMinutes);
        radio.powerDown();
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
        return;
      }
    }

    // --- ENCENDER sensores con TIP42C ---
    digitalWrite(SENS_PWR, LOW);   // LOW = ON (base baja en PNP)
    delay(1800);                   // DHT22 ~1s; dejamos margen
    dht.begin();                   // init tras energizar
    delay(50); //ADDED

    // Leer
    readSensor();

    // =========================
    // PATCH: Retries limitados por ventana (deadline corto)
    // =========================
    bool sent = false;
    uint32_t deadline = millis() + 1800; // ~1.8s de ventana de envío
    for (uint8_t i = 0; i < 2 && !sent && millis() < deadline; i++) {
      if (!mesh.checkConnection()) {
        mesh.renewAddress();
        delay(50);
        if (!mesh.checkConnection()) continue;
      }
      sent = sendSensorData();
      if (!sent) delay(80);
    }

    if (!sent) {
      Serial.println(F("No send this window -> sleep"));
      // --- APAGAR sensores antes de dormir ---
      pinMode(DHTPIN, INPUT);
      pinMode(MOISTURE_SOIL_SENSOR_PIN, INPUT);
      digitalWrite(SENS_PWR, HIGH); // OFF
      delay(100);

      configureAlarm(intervalMinutes);
      radio.powerDown();
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
      return;
    }

    Serial.print("Send OK at ms: ");
    Serial.println(displayTimer);

    // --- APAGAR sensores ---
    pinMode(DHTPIN, INPUT);
    pinMode(MOISTURE_SOIL_SENSOR_PIN, INPUT);
    digitalWrite(SENS_PWR, HIGH); // OFF
    delay(100);

    configureAlarm(intervalMinutes);
  }

  delay(50);
  Serial.flush();
  radio.powerDown();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

  Serial.println("Woke up!");
  delay(100); //ADDED
}

bool reqAndReceiveTime() {
  mesh.update();
  RF24NetworkHeader h;
  unsigned int req = nodeID;
  bool reqSent = false;

  Serial.print("Sending time update request");
  // (deja el bucle de envío, pero no bloqueamos más allá del timeout de abajo)
  while (!reqSent) {
    reqSent = mesh.write(&req, 'R', sizeof(req));
    Serial.print(".");
    delay(200);
    // PATCH: si no hay conexión, sal pronto
    if (!mesh.checkConnection()) break;
  }
  Serial.println("Request sent!");

  unsigned long startTime = millis();
  while (millis() - startTime < 1500) { // PATCH: timeout corto (antes eran 5s)
    mesh.update();
    if (network.available()) {
      network.peek(h);
      if (h.type == 'T') {
        network.read(h, &timeData, sizeof(timeData));
        rtc.adjust(DateTime(timeData.year, timeData.month, timeData.day,
                            timeData.hour, timeData.minute, timeData.second));
        Serial.println("RTC synchronized!");
        return true;
      } else {
        network.read(h, NULL, 0);
      }
    }
  }
  return false;
}

void configureAlarm(int intervalMinutes) {
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  DateTime now = rtc.now();

  int totalMin = now.hour() * 60 + now.minute();
  int nextTotalMin = ((totalMin / intervalMinutes) + 1) * intervalMinutes;

  int nextHour = (nextTotalMin / 60) % 24;
  int nextMinute = nextTotalMin % 60;

  int year = now.year();
  int month = now.month();
  int day = now.day();
  if (nextTotalMin >= 24 * 60) {
    DateTime nextDay = now + TimeSpan(1, 0, 0, 0);
    year = nextDay.year();
    month = nextDay.month();
    day = nextDay.day();
  }

  bool ok = rtc.setAlarm1(DateTime(year, month, day, nextHour, nextMinute, 0),
                          DS3231_A1_Minute);
  if (!ok) {
    Serial.println("Error, alarm wasn't set!");
  } else {
    Serial.print("Alarm programmed for: ");
    Serial.print(nextHour); Serial.print(":");
    Serial.print(nextMinute); Serial.println(":00");
  }
}

void readSensor() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  auto invalid = [&](float h, float t){
    return isnan(h) || h < 0 || h > 100 || isnan(t) || t < -40 || t > 80;
  };

  if (invalid(humidity, temperature)) {
    // esperar la ventana mínima del DHT22
    delay(2000);
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
  }

  // Promedia 3 lecturas del suelo para estabilidad
  long acc = 0;
  for (int i = 0; i < 3; i++) {
    acc += analogRead(MOISTURE_SOIL_SENSOR_PIN);
    delay(10);
  }
  int soil_sensor = acc / 3;
  int soil_moisture = map(soil_sensor, DRY_SOIL, WET_SOIL, 0, 100);
  soil_moisture = constrain(soil_moisture, 0, 100);

  if (invalid(humidity, temperature)) {
    Serial.println("DHT invalid -> set zeros");
    data.humidity = 0;
    data.temperature = 0;
  } else {
    data.humidity = humidity;
    data.temperature = temperature;
  }
  data.moistureLevel = soil_moisture;

  Serial.print("humidity: ");   Serial.print(data.humidity);
  Serial.print(", temperature: "); Serial.print(data.temperature);
  Serial.print(", moisture: ");  Serial.println(data.moistureLevel);
}

bool sendSensorData() {
  byte dataBuffer[sizeof(SensorData)];
  memcpy(dataBuffer, &data, sizeof(SensorData));
  return mesh.write(dataBuffer, 'M', sizeof(dataBuffer));
}

bool tryRecoverRadioMesh() {
  Serial.println(F("[RECOVER] attempting radio/mesh recovery"));

  SPI.end(); delay(2);
  SPI.begin(); delay(2);

  if (!radio.begin()) {
    Serial.println(F("[RECOVER] radio.begin() failed"));
  }

  radio.setPALevel(RF24_PA_MIN, 0);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.setRetries(5, 15);
  radio.flush_rx();
  radio.flush_tx();
  radio.powerUp();
  delay(5);

  if (!radio.isChipConnected()) {
    Serial.println(F("[RECOVER] nRF24 not detected"));
    return false;
  }

  if (!mesh.checkConnection()) {
    Serial.println(F("[RECOVER] renewing mesh address"));
    if (mesh.renewAddress() == MESH_DEFAULT_ADDRESS) {
      Serial.println(F("[RECOVER] renewAddress failed -> mesh.begin()"));
      if (!mesh.begin()) {
        Serial.println(F("[RECOVER] mesh.begin() failed"));
        return false;
      }
    }
  }

  unsigned int ping = nodeID;
  bool ok = mesh.write(&ping, 'P', sizeof(ping));
  Serial.println(ok ? F("[RECOVER] write test OK") : F("[RECOVER] write test FAIL"));
  return ok;
}
