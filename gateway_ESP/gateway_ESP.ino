#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <RTClib.h>

#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include "esp_sleep.h"

// ---------- Wi-Fi AP ----------
const char* AP_SSID = "MiESP32-Gateway";
const char* AP_PASS = "12345678";

// ---------- RF24 / Mesh (ajusta pines si hace falta) ----------
static const int CE_PIN  = 27;
static const int CSN_PIN = 15;

RF24 radio(CE_PIN, CSN_PIN);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

// ---------- RTC ----------
RTC_DS3231 rtc;

// ---------- Web ----------
WebServer server(80);
const char* LOG_PATH = "/sensor_data_p4.csv";

// ---------- Datos ----------
struct __attribute__((packed)) SensorData {
  float    humidity;
  float    temperature;
  int16_t  moistureLevel;
  uint8_t  nodeId;
};

struct TimeData {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

// ---------- Persistencia (entre deep-sleep) ----------
RTC_DATA_ATTR bool     g_sleepMode = false;     // modo ahorro ON/OFF
RTC_DATA_ATTR uint32_t g_interval_s = 900;     // intervalo entre ventanas (segundos)
RTC_DATA_ATTR uint32_t g_window_s   = 12;      // duración ventana RX (segundos)
RTC_DATA_ATTR bool     g_align      = true;    // alinear a múltiplos exactos

// ---------- Utilidad de log ----------
static inline void LOG(const String& s) {
  Serial.print('['); Serial.print(millis()); Serial.print(" ms] ");
  Serial.println(s);
}

// ---------- Boot log ----------
const char* BOOTLOG_PATH = "/boot_log_p4.csv";

static inline const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

static inline const char* wakeupCauseStr(esp_sleep_wakeup_cause_t c) {
  switch (c) {
    case ESP_SLEEP_WAKEUP_UNDEFINED: return "UNDEFINED";
    case ESP_SLEEP_WAKEUP_ALL:       return "ALL";
    case ESP_SLEEP_WAKEUP_EXT0:      return "EXT0";
    case ESP_SLEEP_WAKEUP_EXT1:      return "EXT1";
    case ESP_SLEEP_WAKEUP_TIMER:     return "TIMER";
    case ESP_SLEEP_WAKEUP_TOUCHPAD:  return "TOUCHPAD";
    case ESP_SLEEP_WAKEUP_ULP:       return "ULP";
    case ESP_SLEEP_WAKEUP_GPIO:      return "GPIO";
    case ESP_SLEEP_WAKEUP_UART:      return "UART";
    default:                         return "UNKNOWN";
  }
}

void ensureBootLogHeader() {
  if (!SPIFFS.exists(BOOTLOG_PATH)) {
    File f = SPIFFS.open(BOOTLOG_PATH, "w");
    if (f) {
      f.println("Timestamp,ResetReason,WakeupCause");
      f.close();
      LOG("BootLog: creado con cabecera.");
    } else {
      LOG("ERROR: no se pudo crear BootLog.");
    }
  }
}

bool appendBootLogEntry() {
  File f = SPIFFS.open(BOOTLOG_PATH, "a");
  if (!f) return false;
  esp_reset_reason_t rr = esp_reset_reason();
  esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
  String line;
  line.reserve(64);
  line += nowTimestamp(); line += ",";
  line += resetReasonStr(rr); line += ",";
  line += wakeupCauseStr(wc);
  f.println(line);
  f.close();
  return true;
}

// ---------- Node log ----------
const char* NODELOG_PATH = "/node_log_p4.csv";

// Mapa de nodos vistos (IDs 0..255) persistente entre deep-sleep
RTC_DATA_ATTR uint8_t g_seen_bitmap[32] = {0}; // 32*8 = 256 bits

inline bool nodeSeen(uint8_t id) {
  return (g_seen_bitmap[id >> 3] >> (id & 7)) & 0x01;
}
inline void nodeMarkSeen(uint8_t id) {
  g_seen_bitmap[id >> 3] |= (1 << (id & 7));
}

void ensureNodeLogHeader() {
  if (!SPIFFS.exists(NODELOG_PATH)) {
    File f = SPIFFS.open(NODELOG_PATH, "w");
    if (f) {
      f.println("Timestamp,Event,NodeID,MeshAddrOct");
      f.close();
      LOG("NodeLog: creado con cabecera.");
    } else {
      LOG("ERROR: no se pudo crear NodeLog.");
    }
  }
}

bool appendNodeEvent(const char* eventName, uint8_t nodeId, int addr /* -1 si no hay */) {
  File f = SPIFFS.open(NODELOG_PATH, "a");
  if (!f) return false;
  String line; line.reserve(64);
  line += nowTimestamp(); line += ",";
  line += eventName;      line += ",";
  line += String(nodeId); line += ",";
  if (addr >= 0) {
    line += "0"; line += String((uint16_t)addr, OCT);
  } else {
    line += "-";
  }
  f.println(line);
  f.close();
  return true;
}

// ---------- Utilidades ----------
String nowTimestamp() {
  DateTime t = rtc.now();
  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
  return String(buf);
}

void ensureCsvHeader() {
  if (!SPIFFS.exists(LOG_PATH)) {
    LOG("CSV no existe, creando con cabecera...");
    File f = SPIFFS.open(LOG_PATH, "w");
    if (f) {
      f.println("Timestamp,NodeID,Humidity,Temperature,Moisture");
      f.close();
      LOG("Cabecera CSV creada.");
    } else {
      LOG("ERROR: no se pudo crear CSV inicial.");
    }
  } else {
    LOG("CSV existente detectado.");
  }
}

size_t fileSize(const char* path) {
  if (!SPIFFS.exists(path)) return 0;
  File f = SPIFFS.open(path, "r");
  if (!f) return 0;
  size_t s = f.size();
  f.close();
  return s;
}

String humanSize(size_t bytes) {
  const char* u[] = {"B","KB","MB","GB"};
  int i=0; double v=bytes;
  while (v>=1024.0 && i<3) { v/=1024.0; i++; }
  char b[32]; snprintf(b, sizeof(b), "%.2f %s", v, u[i]);
  return String(b);
}

bool appendCsv(const SensorData& d) {
  File f = SPIFFS.open(LOG_PATH, "a");
  if (!f) return false;
  String line;
  line.reserve(64);
  line += nowTimestamp(); line += ",";
  line += String(d.nodeId); line += ",";
  line += String(d.humidity, 2); line += ",";
  line += String(d.temperature, 2); line += ",";
  line += String(d.moistureLevel);
  f.println(line);
  f.close();
  return true;
}

// ==================== helpers /time ====================
bool parseDateTimeLocal(const String& dt, DateTime& out) {
  int y, m, d, hh, mm, ss = 0;
  int tpos = dt.indexOf('T');
  if (tpos < 0) return false;
  String dpart = dt.substring(0, tpos);
  String tpart = dt.substring(tpos + 1);
  if (sscanf(dpart.c_str(), "%d-%d-%d", &y,&m,&d) != 3) return false;
  if (tpart.length() >= 8) {
    if (sscanf(tpart.c_str(), "%d:%d:%d", &hh,&mm,&ss) < 2) return false;
  } else {
    if (sscanf(tpart.c_str(), "%d:%d", &hh,&mm) != 2) return false;
  }
  if (y<2000||m<1||m>12||d<1||d>31||hh<0||hh>23||mm<0||mm>59||ss<0||ss>59) return false;
  out = DateTime(y,m,d,hh,mm,ss);
  return true;
}

// ==================== Wi-Fi AP control ====================
void startAP() {
  WiFi.mode(WIFI_AP);
  // Baja potencia si quieres (ahorra algo): WiFi.setTxPower(WIFI_POWER_8_5dBm);
  if (WiFi.softAP(AP_SSID, AP_PASS)) {
    LOG(String("AP listo: SSID=") + AP_SSID + " PASS=" + AP_PASS);
    LOG(String("IP AP: ") + WiFi.softAPIP().toString());
  } else {
    LOG("ERROR: softAP() falló.");
  }
}

void stopAP() {
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
}

// ==================== RF window helpers ====================
void rfStart() {
  // RF24 Mesh Master
  LOG("Inicializando RF24 + Mesh...");
  mesh.setNodeID(0); // Maestro
  if (!radio.begin()) {
    LOG("ERROR: radio.begin() falló (NRF24 no detectado).");
  } else {
    radio.setPALevel(RF24_PA_LOW);    // ajusta según hardware
    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    radio.setRetries(5,15);
    // radio.setChannel(76);
    // radio.setDataRate(RF24_250KBPS);
    LOG("RF24 OK. Iniciando mesh como master...");
    bool mb = mesh.begin(); // en master, retorna false solo si radio.begin fue false
    LOG(String("mesh.begin() -> ") + (mb ? "OK" : "WARN/false (master)"));
  }
}

void rfStop() {
  // No hay “end()” formal; limpia buffers
  radio.flush_rx();
  radio.flush_tx();
}

// Procesa tráfico RF durante “window_s” segundos
void runReceiveWindow(uint32_t window_s) {
  LOG(String("Ventana RX de ") + window_s + " s");
  uint32_t t0 = millis();
  while ((millis() - t0) < window_s * 1000UL) {
    mesh.update();
    mesh.DHCP();

    while (network.available()) {
      RF24NetworkHeader header; network.peek(header);
      if (header.type == 'M') {
        SensorData d;
        if (network.read(header, &d, sizeof(d))) {
          appendCsv(d);
        } else {
          network.read(header, 0, 0);
        }
      } else if (header.type == 'R') {
        unsigned int reqID=0;
        if (network.read(header, &reqID, sizeof(reqID))) {
          uint16_t addr=0; bool found=false;
          for (int i=0;i<mesh.addrListTop;i++){
            if (mesh.addrList[i].nodeID==reqID){addr=mesh.addrList[i].address; found=true; break;}
          }
          if (found) {
            // ---- NUEVO: logging de eventos de nodo ----
            appendNodeEvent("TIME_REQ", (uint8_t)reqID, addr);
            if (!nodeSeen((uint8_t)reqID)) {
              nodeMarkSeen((uint8_t)reqID);
              appendNodeEvent("JOIN", (uint8_t)reqID, addr);
            }

            DateTime now = rtc.now();
            TimeData td{(uint16_t)now.year(), (uint8_t)now.month(), (uint8_t)now.day(),
                        (uint8_t)now.hour(), (uint8_t)now.minute(), (uint8_t)now.second()};
            byte buf[sizeof(TimeData)]; memcpy(buf, &td, sizeof(td));
            RF24NetworkHeader h(addr, 'T');
            network.write(h, &buf, sizeof(buf));
          }
        } else {
          network.read(header, 0, 0);
        }
      } else {
        network.read(header, 0, 0);
      }
    }
    delay(2); // ceder CPU
  }
  LOG("Ventana RX terminada");
}

// Calcula microsegundos hasta la próxima “ventana”
// Si g_align=true alinea a múltiplos de g_interval_s (p.ej. 00, 15, 30, 45…)
// Si g_align=false, simplemente “ahora + g_interval_s”
uint64_t microsToNextWindow() {
  DateTime now = rtc.now();
  uint32_t now_s = (uint32_t)now.unixtime();
  uint32_t next_start_s;
  if (g_align) {
    uint32_t r = now_s % g_interval_s;
    uint32_t delta = (r == 0) ? g_interval_s : (g_interval_s - r);
    next_start_s = now_s + delta;
  } else {
    next_start_s = now_s + g_interval_s;
  }
  uint64_t us = (uint64_t)(next_start_s - now_s) * 1000000ULL;
  return us;
}

// ==================== Páginas web ====================
void handleRoot() {
  String html;
  html.reserve(4000);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Gateway ESP32</title>";
  html += "<style>body{font-family:system-ui;margin:2rem;max-width:800px}a.btn,button{padding:.6rem 1rem;border:1px solid #999;border-radius:.5rem;text-decoration:none;display:inline-block;margin-right:.5rem}code{background:#eee;padding:.1rem .3rem;border-radius:.3rem}</style>";
  html += "</head><body>";
  html += "<h1>Gateway ESP32</h1>";

  html += "<p>Modo ahorro: <b>";
  html += (g_sleepMode ? "ON" : "OFF");
  html += "</b></p>";

  html += "<p>Intervalo: <code>";
  html += String(g_interval_s);
  html += " s</code> — Ventana: <code>";
  html += String(g_window_s);
  html += " s</code> — Alinear: <code>";
  html += (g_align ? "true" : "false");
  html += "</code></p>";

  html += "<p>AP: <b>";
  html += AP_SSID;
  html += "</b> — IP: <code>";
  html += WiFi.softAPIP().toString();
  html += "</code></p>";

  html += "<p>Hora (RTC): <code>";
  html += nowTimestamp();
  html += "</code></p>";

  // --- Sección archivo de datos (igual que antes, solo agrupado) ---
  html += "<h2>Datos de sensores</h2>";
  html += "<p>Archivo: <code>";
  html += LOG_PATH;
  html += "</code> (";
  html += humanSize(fileSize(LOG_PATH));
  html += ")</p>";
  html += "<p><a class='btn' href='/download'>Descargar CSV</a> ";
  html += "<form style='display:inline' method='POST' action='/erase' onsubmit='return confirm(\"¿Borrar el archivo de datos?\")'><button type='submit'>Borrar archivo</button></form></p>";

  // --- NUEVO: Sección Boot Log ---
  html += "<h2>Registro de arranques</h2>";
  html += "<p>Archivo: <code>";
  html += BOOTLOG_PATH;
  html += "</code> (";
  html += humanSize(fileSize(BOOTLOG_PATH));
  html += ")</p>";
  html += "<p><a class='btn' href='/bootlog'>Descargar boot log</a> ";
  html += "<form style='display:inline' method='POST' action='/bootlog_erase' onsubmit='return confirm(\"¿Borrar el boot log?\")'><button type='submit'>Borrar boot log</button></form></p>";

  // --- NUEVO: Sección Node Log ---
  html += "<h2>Registro de nodos</h2>";
  html += "<p>Archivo: <code>";
  html += NODELOG_PATH;
  html += "</code> (";
  html += humanSize(fileSize(NODELOG_PATH));
  html += ")</p>";
  html += "<p><a class='btn' href='/nodelog'>Descargar node log</a> ";
  html += "<form style='display:inline' method='POST' action='/nodelog_erase' onsubmit='return confirm(\"¿Borrar el node log?\")'><button type='submit'>Borrar node log</button></form></p>";

  // --- Acciones generales ---
  html += "<p><a class='btn' href='/time'>Ajustar fecha/hora RTC</a> ";
  html += "<a class='btn' href='/config'>Config ahorro</a></p>";

  html += "</body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}


void handleDownload() {
  if (!SPIFFS.exists(LOG_PATH)) { server.send(404, "text/plain", "No existe"); return; }
  File f = SPIFFS.open(LOG_PATH, "r");
  if (!f) { server.send(500, "text/plain", "Error abriendo archivo"); return; }
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=\"sensor_data_p4.csv\"");
  server.streamFile(f, "text/csv");
  f.close();
}

void handleErase() {
  LOG("HTTP POST /erase");
  if (SPIFFS.exists(LOG_PATH)) {
    if (SPIFFS.remove(LOG_PATH)) {
      LOG("CSV borrado.");
    } else {
      LOG("ERROR: no se pudo borrar CSV.");
    }
  } else {
    LOG("CSV no existía al borrar.");
  }
  ensureCsvHeader();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleBootDownload() {
  if (!SPIFFS.exists(BOOTLOG_PATH)) { server.send(404, "text/plain", "No existe"); return; }
  File f = SPIFFS.open(BOOTLOG_PATH, "r");
  if (!f) { server.send(500, "text/plain", "Error abriendo archivo"); return; }
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=\"boot_log_p4.csv\"");
  server.streamFile(f, "text/csv");
  f.close();
}

void handleBootErase() {
  LOG("HTTP POST /bootlog_erase");
  if (SPIFFS.exists(BOOTLOG_PATH)) {
    if (SPIFFS.remove(BOOTLOG_PATH)) {
      LOG("BootLog borrado.");
    } else {
      LOG("ERROR: no se pudo borrar BootLog.");
    }
  } else {
    LOG("BootLog no existía al borrar.");
  }
  ensureBootLogHeader();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleNodeLogDownload() {
  if (!SPIFFS.exists(NODELOG_PATH)) { server.send(404, "text/plain", "No existe"); return; }
  File f = SPIFFS.open(NODELOG_PATH, "r");
  if (!f) { server.send(500, "text/plain", "Error abriendo archivo"); return; }
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=\"node_log_p4.csv\"");
  server.streamFile(f, "text/csv");
  f.close();
}

void handleNodeLogErase() {
  LOG("HTTP POST /nodelog_erase");
  if (SPIFFS.exists(NODELOG_PATH)) {
    if (SPIFFS.remove(NODELOG_PATH)) {
      LOG("NodeLog borrado.");
    } else {
      LOG("ERROR: no se pudo borrar NodeLog.");
    }
  } else {
    LOG("NodeLog no existía al borrar.");
  }
  ensureNodeLogHeader();
  server.sendHeader("Location", "/");
  server.send(303);
}

// /time (GET form / POST aplica) — igual a tu versión anterior
void handleTimeForm() {
  DateTime t = rtc.now();
  char preset[17];
  snprintf(preset, sizeof(preset), "%04d-%02d-%02dT%02d:%02d", t.year(), t.month(), t.day(), t.hour(), t.minute());

  String html;
  html.reserve(3500);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Ajustar fecha/hora RTC</title>";
  html += "<style>body{font-family:system-ui;margin:2rem;max-width:650px}label{display:block;margin:.5rem 0}.row{margin:.5rem 0}input,button{padding:.5rem}</style>";
  html += "</head><body>";
  html += "<h1>Ajustar fecha/hora del RTC</h1>";
  html += "<p>Hora actual (RTC): <code>" + nowTimestamp() + "</code></p>";
  html += "<form method='POST' action='/time'>";
  html += "<label>Fecha y hora (local):</label>";
  html += "<input type='datetime-local' name='dt' id='dt' value='"; html += preset; html += "' required>";
  html += "<div class='row'><button type='submit'>Guardar</button></div>";
  html += "</form>";
  html += "<p><a href='/'>Volver</a></p>";
  html += "</body></html>";

  server.send(200, "text/html; charset=utf-8", html);
}

bool parseDateTimeLocal(const String& dt, DateTime& out); // fwd ya declarado

void handleTimeSet() {
  if (!server.hasArg("dt")) { server.send(400, "text/plain", "Falta 'dt'"); return; }
  DateTime newT;
  if (!parseDateTimeLocal(server.arg("dt"), newT)) {
    server.send(400, "text/plain", "Formato invalido. Usa YYYY-MM-DDTHH:MM(:SS)");
    return;
  }
  rtc.adjust(newT);
  server.sendHeader("Location", "/time");
  server.send(303);
}

// ---------- /config ----------
void handleConfigGet() {
  String html;
  html.reserve(4000);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Config Ahorro</title>";
  html += "<style>body{font-family:system-ui;margin:2rem;max-width:650px}label{display:block;margin:.5rem 0}input,button,select{padding:.5rem}</style>";
  html += "</head><body>";
  html += "<h1>Configuración de ahorro</h1>";
  html += "<p>Estado actual: modo ahorro <b>"; html += (g_sleepMode?"ON":"OFF"); html += "</b></p>";
  html += "<form method='POST' action='/config'>";
  html += "<label>Modo ahorro:</label>";
  html += "<select name='sleepMode'><option value='0'"; if(!g_sleepMode)html+=" selected"; html+=">OFF</option>";
  html += "<option value='1'"; if(g_sleepMode)html+=" selected"; html+=">ON</option></select>";
  html += "<label>Intervalo (segundos):</label>";
  html += "<input type='number' name='interval' min='10' max='86400' value='"; html += String(g_interval_s); html += "'>";
  html += "<label>Ventana (segundos):</label>";
  html += "<input type='number' name='window' min='3' max='120' value='"; html += String(g_window_s); html += "'>";
  html += "<label>Alinear a múltiplos exactos:</label>";
  html += "<select name='align'><option value='0'"; if(!g_align)html+=" selected"; html+=">false</option>";
  html += "<option value='1'"; if(g_align)html+=" selected"; html+=">true</option></select>";
  html += "<div style='margin-top:1rem'><button type='submit'>Guardar</button> <a href='/'>Volver</a></div>";
  html += "</form>";
  html += "<p>Nota: si activas el modo ahorro, el AP se apagará y el equipo entrará en deep-sleep hasta la próxima ventana.</p>";
  html += "</body></html>";

  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfigPost() {
  if (server.hasArg("interval")) {
    uint32_t v = server.arg("interval").toInt();
    if (v >= 10 && v <= 86400) g_interval_s = v;
  }
  if (server.hasArg("window")) {
    uint32_t v = server.arg("window").toInt();
    if (v >= 3 && v <= 120) g_window_s = v;
  }
  if (server.hasArg("align")) {
    g_align = (server.arg("align").toInt() != 0);
  }
  bool newSleep = g_sleepMode;
  if (server.hasArg("sleepMode")) {
    newSleep = (server.arg("sleepMode").toInt() != 0);
  }

  g_sleepMode = newSleep;

  // Feedback inmediato
  String msg = String("OK. sleepMode=") + (g_sleepMode?"ON":"OFF")
             + " interval=" + g_interval_s
             + " window="   + g_window_s
             + " align="    + (g_align?"true":"false") + "\n";
  server.send(200, "text/plain; charset=utf-8", msg);

  // Si activaste ahorro, apagamos AP y nos vamos a dormir hacia la próxima ventana
  if (g_sleepMode) {
    stopAP();
    uint64_t us = microsToNextWindow();
    LOG(String("Entrando a deep-sleep hasta la próxima ventana: ") + (us/1000000ULL) + " s");
    esp_sleep_enable_timer_wakeup(us);
    esp_deep_sleep_start();
  }
}

// ---------- RF: enviar hora a un nodo ----------
bool sendTimeToNode(uint16_t address) {
  DateTime now = rtc.now();
  TimeData td{(uint16_t)now.year(), (uint8_t)now.month(), (uint8_t)now.day(),
              (uint8_t)now.hour(), (uint8_t)now.minute(), (uint8_t)now.second()};
  byte buf[sizeof(TimeData)];
  memcpy(buf, &td, sizeof(td));
  RF24NetworkHeader h(address, 'T');
  return network.write(h, &buf, sizeof(buf));
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(150);
  LOG("==== Arranque ESP32 Gateway ====");

  // FS
  LOG("Montando SPIFFS...");
  if (!SPIFFS.begin(true)) {
    LOG("ERROR: SPIFFS mount failed (incluso tras format).");
  } else {
    LOG("SPIFFS montado OK.");
  }
  ensureCsvHeader();

  // RTC
  LOG("Inicializando I2C + RTC DS3231...");
  Wire.begin(21,22); // SDA, SCL
  if (!rtc.begin()) {
    LOG("ERROR: RTC no encontrado. (Verifica conexiones)");
  } else {
    if (rtc.lostPower()) {
      LOG("RTC lost power -> ajustando a fecha de compilación.");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    LOG("RTC OK. Hora actual: " + nowTimestamp());
  }

  // Boot log
  ensureBootLogHeader();
  ensureNodeLogHeader();

  if (appendBootLogEntry()) {
    LOG(String("BootLog OK: ") +
        resetReasonStr(esp_reset_reason()) + " / " +
        wakeupCauseStr(esp_sleep_get_wakeup_cause()));
  } else {
    LOG("ERROR: appendBootLogEntry() falló.");
  }


  // Si NO estamos en modo ahorro -> AP ON y servidor web
  if (!g_sleepMode) {
    startAP();
    server.on("/", HTTP_GET, handleRoot);
    server.on("/download", HTTP_GET, handleDownload);
    server.on("/erase", HTTP_POST, handleErase);
    server.on("/time",  HTTP_GET, handleTimeForm);
    server.on("/time",  HTTP_POST, handleTimeSet);
    server.on("/config",HTTP_GET, handleConfigGet);
    server.on("/config",HTTP_POST, handleConfigPost);
    server.on("/bootlog",       HTTP_GET,  handleBootDownload);
    server.on("/bootlog_erase", HTTP_POST, handleBootErase);
    server.on("/nodelog",        HTTP_GET,  handleNodeLogDownload);
    server.on("/nodelog_erase",  HTTP_POST, handleNodeLogErase);


    server.begin();

    // RF24 Mesh activo continuo (como antes)
    rfStart();
  } else {
    // Estamos despertando para abrir una ventana RX
    rfStart();
    runReceiveWindow(g_window_s);
    rfStop();

    // Programar el próximo wake y dormir
    uint64_t us = microsToNextWindow();
    LOG(String("Ventana cerrada. Próximo wake en ") + (us/1000000ULL) + " s");
    esp_sleep_enable_timer_wakeup(us);
    esp_deep_sleep_start();
  }

  // Info SPIFFS
  size_t total = SPIFFS.totalBytes();
  size_t used  = SPIFFS.usedBytes();
  Serial.printf("SPIFFS total=%u bytes, usado=%u bytes\n", (unsigned)total, (unsigned)used);
  LOG("==== Setup completo ====");
}

// ---------- Loop ----------
void loop() {
  if (!g_sleepMode) {
    // Modo normal (AP + RF24 siempre activos)
    server.handleClient();

    mesh.update();
    mesh.DHCP();

    while (network.available()) {
      RF24NetworkHeader header; network.peek(header);
      if (header.type == 'M') {
        
        byte buf[sizeof(SensorData)];
        if (network.read(header, &buf, sizeof(buf))) {
          SensorData d;
          memcpy(&d, buf, sizeof(d));
          LOG(String("RX M: node=") + d.nodeId +
              " hum=" + String(d.humidity,2) +
              " temp=" + String(d.temperature,2) +
              " moist=" + d.moistureLevel);
          if (!appendCsv(d)) {
            LOG("ERROR: appendCsv() falló.");
          }
        } else {
          LOG("WARN: network.read(M) devolvió false, descartando.");
          network.read(header, 0, 0); // descarta
        }

      } else if (header.type == 'R') {
        unsigned int reqID = 0;
      if (network.read(header, &reqID, sizeof(reqID))) {
        LOG(String("RX R: solicitud de hora de nodeID=") + reqID);
        // buscar address del nodo en la tabla del mesh
        uint16_t addr = 0;
        bool found = false;
        for (int i = 0; i < mesh.addrListTop; i++) {
          if (mesh.addrList[i].nodeID == reqID) {
            addr = mesh.addrList[i].address;
            found = true;
            break;
          }
        }
        if (found) {

          // ---- NUEVO: logging de eventos de nodo ----
          appendNodeEvent("TIME_REQ", (uint8_t)reqID, addr);
          if (!nodeSeen((uint8_t)reqID)) {
            nodeMarkSeen((uint8_t)reqID);
            appendNodeEvent("JOIN", (uint8_t)reqID, addr);
          }

          LOG(String("Addr para nodeID ") + reqID + " = 0" + String(addr, OCT));
          sendTimeToNode(addr);
          } else {
          LOG("WARN: nodeID no encontrado en addrList.");
          }
        } else {
          LOG("WARN: network.read(R) devolvió false.");
          network.read(header, 0, 0);
        }
      } else {
        LOG(String("RX tipo desconocido: '") + char(header.type) + "' -> descartando");
        network.read(header, 0, 0); // descartar tipos no usados
      }
    }
    delay(2);
  } else {
    // En modo ahorro nunca deberíamos permanecer aquí: setup() ya duerme.
    // Pero por seguridad, si caemos, reprogramamos sleep.
    uint64_t us = microsToNextWindow();
    esp_sleep_enable_timer_wakeup(us);
    esp_deep_sleep_start();
  }
}
