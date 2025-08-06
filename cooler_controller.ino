// Подключение необходимых библиотек
#include <ESP8266WiFi.h>          // Для работы с WiFi на ESP8266
#include <ESP8266WebServer.h>     // Для создания веб-сервера
#include <EEPROM.h>               // Для работы с энергонезависимой памятью
#include <OneWire.h>              // Для работы с 1-Wire устройствами (датчики температуры)
#include <DallasTemperature.h>    // Для работы с датчиками температуры DS18B20
#include <QuickPID.h>             // Библиотека PID-регулятора
#include <Ticker.h>               // Для создания периодических событий
#include <NTPClient.h>            // Для работы с NTP (получение времени)
#include <WiFiUdp.h>              // Для работы с UDP (необходимо для NTP)
#include <ESP8266mDNS.h>          // Для mDNS (OTA)
#include <ArduinoOTA.h>           // Для OTA обновлений

// Конфигурация WiFi
const char PROGMEM _ssid[] = "WIFI";      // SSID WiFi сети
const char PROGMEM _password[] = "12345678"; // Пароль WiFi сети

// Пин-конфигурация
#define ONE_WIRE_BUS D1           // Пин для подключения датчика температуры
#define FAN_PIN D2                // Пин для управления вентилятором (ШИМ)
#define FAN_MIN_PWM 55            // Минимальное значение ШИМ для вентилятора

// Настройки ШИМ
#define PWM_FREQ 76               // Частота ШИМ в Гц (оптимально для вентиляторов)
#define PWM_RANGE 255             // Диапазон значений ШИМ (8 бит)

// Интервалы работы
#define TEMP_UPDATE_INTERVAL 1000 // Интервал обновления температуры (мс)
#define WIFI_CHECK_INTERVAL 300000 // Интервал проверки соединения WiFi (5 минут)

// Конфигурация восстановления датчика
#define MAX_SENSOR_ERRORS 3       // Максимальное количество ошибок перед аварией
#define SENSOR_RECOVERY_ATTEMPTS 2 // Количество попыток восстановления датчика
#define SENSOR_RECOVERY_DELAY 2000 // Задержка между попытками восстановления (мс)

// Максимальные значения параметров
#define MAX_SETPOINT 100.0        // Максимальная уставка температуры
#define MIN_SETPOINT 0.0          // Минимальная уставка температуры
#define MAX_PID_VALUE 1000.0      // Максимальное значение PID коэффициентов
#define MAX_OFFSET 10.0           // Максимальное смещение датчика
#define MAX_ANTI_WINDUP 255       // Максимальное значение антивиндапа

// Настройки NTP
#define NTP_SERVER "europe.pool.ntp.org" // NTP сервер
#define GMT_OFFSET_SEC 7200              // Смещение по времени для Калининграда (UTC+2) в секундах.
#define DAYLIGHT_OFFSET_SEC 0            // Смещение летнего времени (0 для Калининграда)
#define RESTART_HOUR 0                   // Час перезагрузки (0 = полночь)
#define RESTART_MINUTE 0                 // Минута перезагрузки (0 = ровно)

// Структура для хранения настроек в EEPROM (без гистерезиса!)
struct __attribute__((packed)) Settings {
  float setpoint = 37.5;          // Уставка температуры (целевое значение)
  float Kp = 90.0;                // Пропорциональный коэффициент PID
  float Ki = 1.5;                 // Интегральный коэффициент PID
  float Kd = 8.5;                 // Дифференциальный коэффициент PID
  float sensorOffset = 2.5;       // Коррекция показаний датчика температуры
  uint8_t antiWindup = 65;        // Настройка антивиндапа PID
  uint32_t resetCount = 0;        // Счетчик перезагрузок
  uint32_t errorCount = 0;        // Счетчик ошибок
  uint32_t sensorErrorCount = 0;  // Счетчик ошибок датчика

  // Метод проверки валидности настроек
  bool isValid() const {
    return !isnan(setpoint) && !isnan(Kp) && !isnan(Ki) && !isnan(Kd) &&
           !isnan(sensorOffset) &&
           (setpoint >= MIN_SETPOINT) && (setpoint <= MAX_SETPOINT) &&
           (Kp >= 0) && (Kp <= MAX_PID_VALUE) &&
           (Ki >= 0) && (Ki <= MAX_PID_VALUE) &&
           (Kd >= 0) && (Kd <= MAX_PID_VALUE) &&
           (fabs(sensorOffset) <= MAX_OFFSET) &&
           (antiWindup <= MAX_ANTI_WINDUP);
  }
} settings;

// Глобальные объекты и переменные
ESP8266WebServer server(80);      // Веб-сервер на порту 80
OneWire oneWire(ONE_WIRE_BUS);    // Объект для работы с 1-Wire
DallasTemperature sensors(&oneWire); // Объект для работы с датчиками температуры
DeviceAddress tempDeviceAddress;   // Адрес датчика температуры
float Setpoint, Input, Output;    // Переменные для PID (уставка, вход, выход)
QuickPID myPID(&Input, &Output, &Setpoint); // PID-регулятор
Ticker restartTicker;             // Таймер для периодического рестарта

// Объекты для работы с NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, GMT_OFFSET_SEC, 600000); // Обновление раз в минуту

// Глобальные переменные состояния
volatile float currentTemp = 0.0; // Текущая температура (с учетом коррекции)
volatile int currentPWM = 0;      // Текущее значение ШИМ вентилятора
volatile bool sensorError = false; // Флаг ошибки датчика
volatile uint8_t sensorErrorCounter = 0; // Счетчик ошибок датчика
unsigned long startTime = 0;      // Время старта системы
unsigned long lastTempRequest = 0; // Время последнего запроса температуры
unsigned long lastWiFiCheck = 0;  // Время последней проверки WiFi
bool timeInitialized = false;     // Флаг инициализации времени
unsigned long lastRestartCheck = 0; // Время последней проверки времени для перезагрузки

// Переменные для асинхронных операций
unsigned long rebootStart = 0;    // Таймер для перезагрузки (без delay)
unsigned long lastSave = 0;       // Защита от частого сохранения в EEPROM

/* ========== HTML ШАБЛОНЫ ========== */
const char PROGMEM HTML_HEADER[] = R"html(
<!DOCTYPE html>
<html lang='ru'>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width,initial-scale=1.0'>
  <title>CryoMind</title>
  <style>
    /* Сброс и базовые стили */
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
    }
    body {
      font-family: 'Segoe UI', sans-serif;
      max-width: 800px;
      margin: 0 auto;
      padding: 15px;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      color: #333;
      font-size: 14px;
      min-height: 100vh;
    }
    .container {
      background: rgba(255, 255, 255, 0.95);
      border-radius: 16px;
      padding: 25px;
      box-shadow: 0 8px 32px rgba(31, 38, 135, 0.37);
      backdrop-filter: blur(8px);
      -webkit-backdrop-filter: blur(8px);
      border: 1px solid rgba(255, 255, 255, 0.2);
    }
    .header {
      text-align: center;
      margin-bottom: 20px;
    }
    .header h1 {
      margin: 0;
      font-size: 32px;
      font-weight: 800;
      background: linear-gradient(90deg, #4f46e5, #7c3aed);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      text-shadow: 0 2px 4px rgba(0,0,0,0.1);
    }
    .time {
      font-size: 18px;
      color: #555;
      font-weight: 500;
    }
    /* Сетка 2x2 для основных данных */
    .status-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
      margin-bottom: 20px;
    }
    .status-card {
      background: linear-gradient(145deg, #e6f7ff, #cceeff);
      border-radius: 12px;
      padding: 14px 10px;
      text-align: center;
      box-shadow: 0 3px 10px rgba(0, 0, 0, 0.08);
      border: 1px solid rgba(173, 216, 230, 0.5);
      transition: all 0.3s ease;
    }
    .status-card:hover {
      transform: translateY(-2px);
      box-shadow: 0 5px 14px rgba(0,0,0,0.12);
    }
    .status-label {
      font-size: 13px;
      color: #666;
      margin-bottom: 4px;
      font-weight: 600;
    }
    .status-value {
      font-size: 18px;
      font-weight: 800;
      font-family: 'Courier New', monospace;
    }
    .status-value.temp { color: #e74c3c; }
    .status-value.pwm { color: #f39c12; }
    .status-value.setpoint { color: #21de71; }
    .status-value.uptime { color: #8e44ad; }

    /* Сетка 3x1 для счётчиков */
    .counters-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
      margin-bottom: 20px;
    }
    .counter-card {
      background: #f8f9fa;
      border-radius: 10px;
      padding: 12px 8px;
      text-align: center;
      box-shadow: 0 2px 8px rgba(0,0,0,0.06);
      border: 1px solid #eee;
      display: flex;
      flex-direction: column;
      justify-content: center;
      align-items: center;
      height: 70px;
    }
    .counter-card .status-label {
      font-size: 12px;
      color: #555;
      margin-bottom: 4px;
    }
    .counter-card .status-value {
      font-size: 16px;
      font-weight: bold;
      color: #3498db;
      margin: 0;
    }

    /* Вкладки — ВСЕГДА В ОДНУ СТРОКУ, РОВНО */
    .tabs {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 8px;
      margin-bottom: 20px;
    }
    .tab {
      padding: 12px 0;
      background: rgba(248, 249, 250, 0.8);
      cursor: pointer;
      border-radius: 8px;
      font-size: 14px;
      font-weight: 600;
      color: #555;
      text-align: center;
      border: 1px solid #ddd;
      transition: all 0.3s ease;
    }
    .tab:hover {
      background: rgba(233, 236, 239, 0.9);
    }
    .tab.active {
      background: linear-gradient(to bottom, #2980b9, #1a5276);
      color: white;
      box-shadow: 0 4px 10px rgba(0,0,0,0.2);
    }

    /* Вкладки контент — без скачков */
    .tab-content {
      display: none;
      padding: 20px;
      background: rgba(248, 249, 250, 0.7);
      border-radius: 10px;
      box-shadow: 0 2px 8px rgba(0,0,0,0.05);
      border: 1px solid #ddd;
    }
    .tab-content.active {
      display: block;
    }

    /* Формы */
    .form-row {
      display: flex;
      align-items: center;
      margin-bottom: 15px;
    }
    .form-row label {
      flex: 1;
      font-size: 14px;
      font-weight: 600;
      color: #444;
      margin-right: 10px;
    }
    .input-tiny {
      width: 80px;
      padding: 8px 10px;
      border: 1px solid #ced4da;
      border-radius: 6px;
      font-size: 14px;
      background: white;
      box-shadow: 0 1px 4px rgba(0,0,0,0.05);
    }
    .input-tiny:focus {
      border-color: #2980b9;
      box-shadow: 0 0 0 3px rgba(41, 128, 185, 0.2);
      outline: none;
    }

    /* Сетка 2x2 для PID */
    .pid-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 15px;
      margin-bottom: 20px;
    }
    .pid-grid .form-row {
      margin-bottom: 0;
    }

    /* Кнопки */
    button {
      background: linear-gradient(to right, #2980b9, #1a5276);
      color: #fff;
      border: none;
      padding: 12px 0;
      border-radius: 8px;
      cursor: pointer;
      font-size: 15px;
      font-weight: 600;
      width: 100%;
      margin: 10px 0;
      box-shadow: 0 4px 6px rgba(0,0,0,0.1);
      transition: all 0.3s ease;
    }
    button:hover {
      background: linear-gradient(to right, #3498db, #2980b9);
      transform: translateY(-2px);
      box-shadow: 0 6px 10px rgba(0,0,0,0.15);
    }
    button:active {
      transform: translateY(0);
    }
    button.danger {
      background: linear-gradient(to right, #e74c3c, #c0392b);
    }
    button.danger:hover {
      background: linear-gradient(to right, #c0392b, #a93226);
    }
    button.warning {
      background: linear-gradient(to right, #f39c12, #d35400);
    }
    button.warning:hover {
      background: linear-gradient(to right, #d35400, #ba4a00);
    }

    /* Ошибки */
    .error {
      color: #e74c3c;
      font-weight: bold;
      padding: 12px;
      background: #fdeded;
      border-radius: 8px;
      border-left: 5px solid #e74c3c;
      margin-bottom: 15px;
      text-align: center;
    }

    /* Адаптивность — вкладки ВСЕГДА в строку */
    @media (max-width: 768px) {
      .tabs { grid-template-columns: 1fr 1fr 1fr; }
      .tab { padding: 10px 0; font-size: 13px; }
      .status-card { padding: 12px 8px; }
      .status-label { font-size: 12px; }
      .status-value { font-size: 16px; }
      .counter-card { padding: 10px 6px; }
      .counter-card .status-label { font-size: 11px; }
      .counter-card .status-value { font-size: 15px; }
    }
  </style>
</head>
<body>
  <div class='container'>
    <div class='header'>
      <h1>CryoMind</h1>
      <div class='time' id='ntpTime'>--:--:--</div>
    </div>

    <!-- 4 информационных блока -->
    <div class='status-grid'>
      <div class='status-card'><div class='status-label'>Температура</div><div class='status-value temp' id='currentTemp'>--.-- °C</div></div>
      <div class='status-card'><div class='status-label'>Обороты</div><div class='status-value pwm' id='currentPWM'>--</div></div>
      <div class='status-card'><div class='status-label'>Цель</div><div class='status-value setpoint' id='setpoint'>--.-- °C</div></div>
      <div class='status-card'><div class='status-label'>Время работы</div><div class='status-value uptime' id='uptime'>00:00:00</div></div>
    </div>

    <!-- Счётчики -->
    <div class='counters-grid'>
      <div class='counter-card'><div class='status-label'>Перезагрузки</div><div class='status-value' id='resetCount'>0</div></div>
      <div class='counter-card'><div class='status-label'>Ошибки системы</div><div class='status-value' id='errorCount'>0</div></div>
      <div class='counter-card'><div class='status-label'>Ошибки датчика</div><div class='status-value' id='sensorErrorCount'>0</div></div>
    </div>

    <!-- Вкладки -->
    <div class='tabs'>
      <div class='tab active' onclick="showTab('controlTab')">Управление</div>
      <div class='tab' onclick="showTab('settingsTab')">Настройки</div>
      <div class='tab' onclick="showTab('pidTab')">PID</div>
    </div>
)html";

const char PROGMEM HTML_CONTROL_TAB[] = R"html(
    <!-- Вкладка Управление -->
    <div id='controlTab' class='tab-content active'>
      <div id='sensorStatusContainer'></div>
      <form action='/set' method='post'>
        <div class='form-row'>
          <label for='setpoint'>Цель (°C)</label>
          <input type='number' step='0.1' min='0' max='100' id='setpoint' name='setpoint' value='%SETPOINT%' class='input-tiny' required>
        </div>
        <button type='submit'>Применить</button>
      </form>
      <form action='/save' method='post'><button type='submit'>Сохранить настройки</button></form>
    </div>
)html";

const char PROGMEM HTML_SETTINGS_TAB[] = R"html(
    <!-- Вкладка Настройки -->
    <div id='settingsTab' class='tab-content'>
      <form action='/set' method='post'>
        <div class='form-row'>
          <label for='sensorOffset'>Поправка (°C)</label>
          <input type='number' step='0.1' min='-10' max='10' id='sensorOffset' name='sensorOffset' value='%OFFSET%' class='input-tiny' required>
        </div>
        <button type='submit'>Применить</button>
      </form>
      <form action='/save' method='post'><button type='submit'>Сохранить настройки</button></form>
      <form action='/resetCounters' method='post'><button type='submit' class='warning'>Сбросить счётчики</button></form>
      <form action='/reset' method='post'><button type='submit' class='danger'>Сбросить настройки</button></form>
      <form action='/reboot' method='post'><button type='submit'>Перезагрузить</button></form>
    </div>
)html";

const char PROGMEM HTML_PID_TAB[] = R"html(
    <!-- Вкладка PID -->
    <div id='pidTab' class='tab-content'>
      <form action='/set' method='post'>
        <div class='pid-grid'>
          <div class='form-row'>
            <label for='Kp'>Kp</label>
            <input type='number' step='0.1' min='0' max='1000' id='Kp' name='Kp' value='%KP%' class='input-tiny' required>
          </div>
          <div class='form-row'>
            <label for='Ki'>Ki</label>
            <input type='number' step='0.01' min='0' max='100' id='Ki' name='Ki' value='%KI%' class='input-tiny' required>
          </div>
          <div class='form-row'>
            <label for='Kd'>Kd</label>
            <input type='number' step='0.1' min='0' max='1000' id='Kd' name='Kd' value='%KD%' class='input-tiny' required>
          </div>
          <div class='form-row'>
            <label for='antiWindup'>Anti-windup</label>
            <input type='number' min='0' max='255' id='antiWindup' name='antiWindup' value='%ANTI_WINDUP%' class='input-tiny' required>
          </div>
        </div>
        <button type='submit'>Применить</button>
      </form>
      <form action='/save' method='post'><button type='submit'>Сохранить настройки</button></form>
    </div>
)html";

const char PROGMEM HTML_FOOTER[] = R"html(
  </div>
  <script>
    let serverUptime = 0;
    function formatTime(totalSeconds) {
      const days = Math.floor(totalSeconds / 86400);
      const hours = Math.floor((totalSeconds % 86400) / 3600);
      const minutes = Math.floor((totalSeconds % 3600) / 60);
      const seconds = totalSeconds % 60;
      return days + 'д ' + [hours, minutes, seconds].map(v => v < 10 ? '0' + v : v).join(':');
    }
    function updateStatus() {
      fetch('/api/status')
        .then(r => r.json())
        .then(d => {
          document.getElementById('currentTemp').textContent = d.temp.toFixed(2) + ' °C';
          document.getElementById('currentPWM').textContent = d.pwm;
          document.getElementById('setpoint').textContent = d.setpoint.toFixed(2) + ' °C';
          document.getElementById('resetCount').textContent = d.resetCount;
          document.getElementById('errorCount').textContent = d.errorCount;
          document.getElementById('sensorErrorCount').textContent = d.sensorErrorCount;
          const statusContainer = document.getElementById('sensorStatusContainer');
          if (d.error) {
            statusContainer.innerHTML = '<div class="error">Ошибка датчика!</div>';
          } else {
            statusContainer.innerHTML = '';
          }
          if (d.ntpTime) {
            document.getElementById('ntpTime').textContent = d.ntpTime;
          }
          document.getElementById('uptime').textContent = d.uptime;
        })
        .catch(e => console.error('Ошибка:', e));
    }
    function showTab(tabId) {
      document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
      document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
      document.getElementById(tabId).classList.add('active');
      document.querySelector(`.tab[onclick="showTab('${tabId}')"]`).classList.add('active');
    }
    document.addEventListener('DOMContentLoaded', () => {
      showTab('controlTab');
      fetch('/api/status')
        .then(r => r.json())
        .then(d => {
          const timeParts = d.uptime.split(' ');
          const days = parseInt(timeParts[0]);
          const [h, m, s] = timeParts[1].split(':').map(Number);
          serverUptime = days * 86400 + h * 3600 + m * 60 + s;
          document.getElementById('uptime').textContent = d.uptime;
          if (d.ntpTime) document.getElementById('ntpTime').textContent = d.ntpTime;
        });
      setInterval(() => { serverUptime++; document.getElementById('uptime').textContent = formatTime(serverUptime); }, 1000);
      setInterval(updateStatus, 1000);
    });
  </script>
</body>
</html>
)html";

// Прототипы
void emergencyMode();
void loadSettings();
void saveSettings();
void updatePIDParameters();
void handleRoot();
void handleApiStatus();
void handleSet();
void handleSave();
void handleReset();
void handleResetCounters();
void handleReboot();
void checkWiFiConnection();
bool tryRecoverSensor();
void initializeNTP();
void checkScheduledRestart();
void setupOTA();

// Реализации функций

void emergencyMode() {
  analogWrite(FAN_PIN, 255);
  currentPWM = 255;
  sensorError = true;
  settings.sensorErrorCount++;
  saveSettings();
  for (int i = 0; i < SENSOR_RECOVERY_ATTEMPTS; i++) {
    if (tryRecoverSensor()) {
      sensorError = false;
      sensorErrorCounter = 0;
      return;
    }
    delay(SENSOR_RECOVERY_DELAY);
  }
  ESP.restart();
}

bool tryRecoverSensor() {
  oneWire.reset(); delay(100);
  if (!sensors.getAddress(tempDeviceAddress, 0)) return false;
  sensors.setResolution(tempDeviceAddress, 12);
  delay(100);
  sensors.requestTemperatures();
  delay(100);
  float temp = sensors.getTempC(tempDeviceAddress);
  return !(temp == -127.0 || temp == 85.0 || temp == 0.0);
}

void loadSettings() {
  EEPROM.get(0, settings);
  if (!settings.isValid()) {
    settings.setpoint = 37.5;
    settings.Kp = 90.0;
    settings.Ki = 1.5;
    settings.Kd = 8.5;
    settings.sensorOffset = 2.5;
    settings.antiWindup = 65;
    settings.resetCount = 0;
    settings.errorCount = 0;
    settings.sensorErrorCount = 0;
    saveSettings();
  }
  updatePIDParameters();
}

void saveSettings() {
  if (millis() - lastSave < 500) return;
  EEPROM.put(0, settings);
  EEPROM.commit();
  lastSave = millis();
}

void updatePIDParameters() {
  myPID.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(QuickPID::Control::automatic);
  myPID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  myPID.SetControllerDirection(QuickPID::Action::reverse);
  Setpoint = settings.setpoint;
}

void handleRoot() {
  String html;
  html.reserve(3000);
  html = FPSTR(HTML_HEADER);

  String control = FPSTR(HTML_CONTROL_TAB);
  control.replace("%SETPOINT%", String(settings.setpoint, 1));
  html += control;

  String settingsTab = FPSTR(HTML_SETTINGS_TAB);
  settingsTab.replace("%OFFSET%", String(settings.sensorOffset, 1));
  html += settingsTab;

  String pidTab = FPSTR(HTML_PID_TAB);
  pidTab.replace("%KP%", String(settings.Kp, 1));
  pidTab.replace("%KI%", String(settings.Ki, 2));
  pidTab.replace("%KD%", String(settings.Kd, 1));
  pidTab.replace("%ANTI_WINDUP%", String(settings.antiWindup));
  html += pidTab;

  html += FPSTR(HTML_FOOTER);
  server.send(200, "text/html", html);
}

void handleApiStatus() {
  static char json[350];
  unsigned long totalSeconds = millis() / 1000;
  unsigned long days = totalSeconds / 86400;
  unsigned long hours = (totalSeconds % 86400) / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;
  char uptimeStr[20];
  snprintf(uptimeStr, sizeof(uptimeStr), "%luд %02lu:%02lu:%02lu", days, hours, minutes, seconds);

  char ntpTimeStr[20] = "--:--:--";
  if (timeInitialized && timeClient.isTimeSet()) {
    timeClient.update();
    strncpy(ntpTimeStr, timeClient.getFormattedTime().c_str(), 19);
    ntpTimeStr[19] = '\0';
  }

  snprintf(json, sizeof(json),
    "{\"temp\":%.2f,\"pwm\":%d,\"setpoint\":%.2f,\"uptime\":\"%s\","
    "\"resetCount\":%u,\"errorCount\":%u,\"sensorErrorCount\":%u,\"error\":%s,\"ntpTime\":\"%s\"}",
    currentTemp, currentPWM, Setpoint, uptimeStr,
    settings.resetCount, settings.errorCount, settings.sensorErrorCount,
    sensorError ? "true" : "false", ntpTimeStr
  );
  server.send(200, "application/json", json);
}

void handleSet() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }
  bool pidChanged = false;

  if (server.hasArg("setpoint")) {
    float sp = server.arg("setpoint").toFloat();
    if (sp >= MIN_SETPOINT && sp <= MAX_SETPOINT) {
      settings.setpoint = sp;
      Setpoint = sp;
    }
  }
  if (server.hasArg("Kp")) {
    float kp = server.arg("Kp").toFloat();
    if (kp >= 0 && kp <= MAX_PID_VALUE) {
      settings.Kp = kp;
      pidChanged = true;
    }
  }
  if (server.hasArg("Ki")) {
    float ki = server.arg("Ki").toFloat();
    if (ki >= 0 && ki <= MAX_PID_VALUE) {
      settings.Ki = ki;
      pidChanged = true;
    }
  }
  if (server.hasArg("Kd")) {
    float kd = server.arg("Kd").toFloat();
    if (kd >= 0 && kd <= MAX_PID_VALUE) {
      settings.Kd = kd;
      pidChanged = true;
    }
  }
  if (server.hasArg("sensorOffset")) {
    float offset = server.arg("sensorOffset").toFloat();
    if (offset >= -MAX_OFFSET && offset <= MAX_OFFSET) {
      settings.sensorOffset = offset;
    }
  }
  if (server.hasArg("antiWindup")) {
    uint8_t aw = server.arg("antiWindup").toInt();
    if (aw <= MAX_ANTI_WINDUP) {
      settings.antiWindup = aw;
      pidChanged = true;
    }
  }

  if (pidChanged) updatePIDParameters();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSave() {
  saveSettings();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleReset() {
  settings.setpoint = 37.5;
  settings.Kp = 90.0;
  settings.Ki = 1.5;
  settings.Kd = 8.5;
  settings.sensorOffset = 2.5;
  settings.antiWindup = 65;
  updatePIDParameters();
  saveSettings();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleResetCounters() {
  settings.resetCount = 0;
  settings.errorCount = 0;
  settings.sensorErrorCount = 0;
  saveSettings();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleReboot() {
  server.send(200, "text/html; charset=utf-8", "<html><head><meta charset='UTF-8'></head><body><h2 style='text-align:center;'>Перезагрузка...</h2><script>setTimeout(function(){window.location.href='/'},10000);</script></body></html>");
  rebootStart = millis();
}

void checkWiFiConnection() {
  if (millis() - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
}

void initializeNTP() {
  if (WiFi.status() == WL_CONNECTED && !timeInitialized) {
    timeClient.begin();
    unsigned long timeout = millis() + 10000;
    while (!timeClient.update() && millis() < timeout) {
      timeClient.forceUpdate();
      delay(1000);
      ESP.wdtFeed();
    }
    if (timeClient.isTimeSet()) timeInitialized = true;
  }
}

void checkScheduledRestart() {
  if (millis() - lastRestartCheck >= 60000) {
    lastRestartCheck = millis();
    if (timeInitialized && timeClient.isTimeSet()) {
      timeClient.update();
      unsigned long epoch = timeClient.getEpochTime();
      struct tm *ptm = gmtime((time_t*)&epoch);
      if (ptm->tm_hour == RESTART_HOUR && ptm->tm_min == RESTART_MINUTE) ESP.restart();
    }
  }
}

void setupOTA() {
  ArduinoOTA.setHostname("cryomind-esp");
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  system_update_cpu_freq(160);
  startTime = millis();
  ESP.wdtDisable();
  ESP.wdtEnable(4000);
  pinMode(FAN_PIN, OUTPUT);
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  analogWrite(FAN_PIN, 0);
  sensors.begin();
  if (!sensors.getAddress(tempDeviceAddress, 0)) {
    if (!tryRecoverSensor()) emergencyMode();
  } else {
    sensors.setResolution(tempDeviceAddress, 12);
  }
  EEPROM.begin(sizeof(Settings));
  loadSettings();
  settings.resetCount++;
  saveSettings();
  Input = currentTemp;
  Setpoint = settings.setpoint;
  updatePIDParameters();
  WiFi.begin(FPSTR(_ssid), FPSTR(_password));
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStartTime) < 15000) {
    delay(500);
    ESP.wdtFeed();
  }
  initializeNTP();
  setupOTA();
  server.on("/", handleRoot);
  server.on("/api/status", handleApiStatus);
  server.on("/set", handleSet);
  server.on("/save", handleSave);
  server.on("/reset", handleReset);
  server.on("/resetCounters", handleResetCounters);
  server.on("/reboot", handleReboot);
  server.begin();
  sensors.requestTemperatures();
}

void loop() {
  ESP.wdtFeed();

  // Асинхронная перезагрузка
  if (rebootStart && millis() - rebootStart > 1000) {
    ESP.restart();
  }

  server.handleClient();
  ArduinoOTA.handle();
  checkWiFiConnection();
  checkScheduledRestart();

  if (millis() - lastTempRequest >= TEMP_UPDATE_INTERVAL) {
    lastTempRequest = millis();
    sensors.requestTemperatures();
    if (!sensorError) {
      float rawTemp = sensors.getTempC(tempDeviceAddress);
      if (rawTemp == -127.0 || rawTemp == 85.0 || rawTemp == 0.0) {
        sensorErrorCounter++;
        if (sensorErrorCounter >= MAX_SENSOR_ERRORS) emergencyMode();
      } else {
        sensorErrorCounter = 0;
        currentTemp = rawTemp + settings.sensorOffset;
        Input = currentTemp;
        myPID.Compute();
        if (currentTemp < Setpoint - 0.2) {
          currentPWM = 0;
        } else if (currentTemp < Setpoint - 0.1) {
          currentPWM = FAN_MIN_PWM;
        } else if (currentTemp < Setpoint) {
          float progress = (currentTemp - (Setpoint - 0.1)) / 0.1;
          currentPWM = FAN_MIN_PWM + (Output - FAN_MIN_PWM) * progress;
        } else {
          currentPWM = Output;
        }
        analogWrite(FAN_PIN, currentPWM);
      }
    }
  }
}
