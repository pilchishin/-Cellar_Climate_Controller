/* ============================================================================
   Климат-контроллер для погреба v3.0 - Стабильная Промышленная Редакция
   Автор: П. Ильчишин + ассистент

   ОСНОВНЫЕ КОМПОНЕНТЫ:
   - Микроконтроллер: Arduino Nano (ATmega328P)
   - Датчики: BME280 (в погребе), HTU21D (снаружи), DS18B20 (резервный в погребе)
   - Дисплей: LCD1602 I2C @ 0x27 (строка 1 = погреб, строка 2 = улица)
   - Исполнительные устройства: Твердотельные реле (SSR) для вентилятора и озонатора
   - Управление: Одна кнопка (короткое/длинное/сверхдлинное нажатие)

   КЛЮЧЕВЫЕ ФУНКЦИИ:
   - Выравнивание износа EEPROM (8 слотов) с проверкой целостности данных (CRC16)
   - Периодическое сохранение рабочего состояния (каждые 30 минут)
   - Неблокирующий интерфейс меню калибровки
   - Сторожевой таймер (WDT) для защиты от зависаний
   ============================================================================ */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BME280.h>
#include <Adafruit_HTU21DF.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <math.h>
#include <stdarg.h>

// ================== КОНФИГУРАЦИЯ ===================
// --- Аппаратные настройки ---
#define LCD_ADDR            0x27  // I2C адрес дисплея
#define BUTTON_PIN          6     // Пин кнопки (подключен к GND, используется внутренний подтягивающий резистор)
#define FAN_PIN             7     // Пин управления реле вентилятора (HIGH = ВКЛ)
#define OZONE_PIN           9     // Пин управления реле озонатора (HIGH = ВКЛ)
#define ONE_WIRE_BUS        8     // Пин шины OneWire для датчика DS18B20

// --- Целевые значения и гистерезис для климат-контроля ---
#define CELLAR_TEMP_TARGET      4.0f   // Целевая температура в погребе
#define CELLAR_TEMP_MIN         2.0f   // Минимально допустимая температура (вентилятор выключится)
#define CELLAR_TEMP_HYST        0.5f   // Гистерезис для температуры (зона нечувствительности)
#define CELLAR_RH_MIN           80.0f  // Минимальная влажность (ниже - вентилятор выключается)
#define CELLAR_RH_MAX           95.0f  // Максимальная влажность (выше - вентилятор включается)
#define AH_DIFF_THRESHOLD       0.5f   // Минимальная разница абсолютной влажности (г/м^3) для включения вентилятора

// --- Временные интервалы (в миллисекундах) ---
#define SENSOR_UPDATE_INTERVAL  10000UL   // Интервал опроса датчиков (10 сек)
#define FAN_MIN_SWITCH_INTERVAL 300000UL  // Минимальный интервал между переключениями вентилятора (5 мин)
#define WORK_SAVE_INTERVAL      1800000UL // Интервал сохранения рабочего состояния в EEPROM (30 мин)
#define BACKLIGHT_TIMEOUT       60000UL   // Время до автоматического отключения подсветки (1 мин)
#define BUTTON_DEBOUNCE_MS      50UL      // Время для подавления дребезга контактов кнопки
#define BUTTON_LONG_MS          2000UL    // Длительность удержания для "длинного" нажатия (вход в меню/выбор)
#define BUTTON_RESET_MS         5000UL    // Длительность удержания для "сверхдлинного" нажатия (сброс калибровки)
#define OZONE_MIN_SAFE_AFTER_BOOT_MS (5UL * 60UL * 1000UL) // Задержка перед запуском озонатора после включения (5 мин)

// --- Настройки цикла озонирования ---
#define OZONE_DURATION_MS       (15UL * 60UL * 1000UL)      // Длительность фазы генерации озона (15 мин)
#define OZONE_REST_MS           (2UL * 60UL * 60UL * 1000UL)// Период "отдыха" после генерации для реакции озона (2 ч)
#define OZONE_VENT_MS           (15UL * 60UL * 1000UL)      // Период принудительной вентиляции для удаления остатков озона (15 мин)
#define OZONE_INTERVAL_MS       (7UL * 24UL * 60UL * 60UL * 1000UL) // Интервал между циклами озонирования (7 дней)
#define OZONE_FIRST_DELAY_MS    (24UL * 60UL * 60UL * 1000UL) // Задержка перед самым первым циклом после настройки (24 ч)

#define DEBUG_SERIAL true // Включить вывод отладочной информации в Serial порт

// --- Диапазоны и шаги для калибровки ---
#define CAL_MIN_VAL   -5.0f  // Минимальное значение коррекции
#define CAL_MAX_VAL    5.0f  // Максимальное значение коррекции
#define CAL_STEP_TEMP  0.1f  // Шаг изменения для температуры
#define CAL_STEP_HUM   0.1f  // Шаг изменения для влажности

// ================== НАСТРОЙКИ EEPROM (Выравнивание износа) =================
#define EE_SLOTS 8             // Количество ячеек для циклической записи
#define EE_SLOT_SIZE 64        // Размер одной ячейки (в байтах)
#define EE_BASE_ADDR 0         // Начальный адрес в памяти EEPROM
#define EE_MAGIC 0xC0DE        // "Магическое" число для проверки, что в ячейке есть наши данные

// ================== ОБЪЕКТЫ ОБОРУДОВАНИЯ ===================
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2); // Дисплей
Adafruit_BME280 bme;                     // Датчик BME280 (погреб)
Adafruit_HTU21DF htu;                    // Датчик HTU21D (улица)
OneWire oneWire(ONE_WIRE_BUS);           // Шина OneWire
DallasTemperature ds(&oneWire);          // Датчик DS18B20 (резервный, погреб)

// ================== СТРУКТУРА ДЛЯ ХРАНЕНИЯ В EEPROM ===================
// Все данные, которые должны сохраняться между перезагрузками
struct Persist {
  uint16_t magic;              // "Магическое" число для идентификации
  uint16_t slotID;             // Уникальный номер записи для определения самой новой
  uint16_t wdtCount;           // Счетчик срабатываний сторожевого таймера
  uint32_t uptimeMin;          // Общее время работы в минутах
  uint32_t fanMin;             // Общее время работы вентилятора в минутах
  uint32_t ozoneMin;           // Общее время работы озонатора в минутах
  uint32_t lastOzoneRunUptime; // Время (uptime) последнего запуска озонатора
  uint32_t nextOzoneAtMin;     // Планируемое время следующего запуска озонатора
  uint16_t ozoneCycles;        // Счетчик циклов озонирования
  float cal_t_cellar_ds18;     // Коррекция температуры DS18B20
  float cal_t_cellar_bme;      // Коррекция температуры BME280
  float cal_rh_cellar;         // Коррекция влажности BME280
  float cal_t_out;             // Коррекция температуры HTU21D
  float cal_rh_out;            // Коррекция влажности HTU21D
  uint16_t crc16;              // Контрольная сумма для проверки целостности
};
Persist persisted;        // Глобальный объект для хранения персистентных данных
uint8_t currentSlot = 0;  // Текущая ячейка EEPROM для записи
bool persistedDirty = false; // Флаг, указывающий на необходимость сохранить данные

// ================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ СОСТОЯНИЯ ===================
float t_cellar = NAN, rh_cellar = NAN, ah_cellar = NAN; // Данные с датчиков в погребе
float t_out = NAN, rh_out = NAN, ah_out = NAN;         // Данные с уличных датчиков
float t_ds18 = NAN;                                    // Температура с резервного датчика

bool bmeOk=false, htuOk=false, dsOk=false; // Флаги состояния датчиков
bool fanState=false;                       // Текущее состояние вентилятора (ВКЛ/ВЫКЛ)
unsigned long lastFanSwitchMillis = 0;     // Время последнего переключения вентилятора
unsigned long lastSensorMillis = 0;        // Время последнего опроса датчиков
unsigned long lastSaveMillis = 0;          // Время последнего сохранения в EEPROM
unsigned long backlightTimer = 0;          // Таймер для автоотключения подсветки
bool backlightOn = false;                  // Состояние подсветки

// --- Переменные для обработки кнопки ---
bool lastButtonRaw = HIGH;        // Предыдущее "сырое" состояние кнопки
unsigned long lastButtonChange = 0; // Время последнего изменения состояния
unsigned long buttonPressedAt = 0;  // Время, когда кнопка была нажата

unsigned long systemBootMillis = 0; // Время старта системы

// --- Конечный автомат (FSM) для озонатора ---
enum OzoneState : uint8_t { OZ_IDLE=0, OZ_RUNNING=1, OZ_REST=2, OZ_VENT=3 };
OzoneState ozoneState = OZ_IDLE;
unsigned long ozoneStateStart = 0;     // Время начала текущего состояния озонатора

// --- Состояние меню и калибровки ---
enum MenuMode : uint8_t { MENU_NONE=0, MENU_SERVICE=1 };
MenuMode menuMode = MENU_NONE;
int menuIndex = 0;                 // Текущий выбранный пункт меню
unsigned long menuLastTick = 0;    // Время последнего обновления экрана меню
unsigned long menuSince = 0;       // Время входа в меню
const int SERVICE_ITEMS = 6;       // Общее количество пунктов в сервисном меню (5 параметров + Выход)
bool calibEditing = false;         // Флаг: true, когда мы находимся в режиме редактирования параметра

// --- Кэш дисплея для минимизации записей в I2C ---
char lastLine1[17] = {0};
char lastLine2[17] = {0};

// ============== ПРОТОТИПЫ ФУНКЦИЙ ================
// --- Основные функции ---
uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t seed = 0xFFFF);
uint16_t ee_slot_addr(uint8_t slot);
bool loadPersisted();
void writePersistSlot(uint8_t slot);
void savePersistRotate();
void savePersistIfNeeded();
void updateSensors();
float calcAbsoluteHumidity(float tempC, float rh);
void ozoneStateMachine(unsigned long now);
void controlFan(unsigned long now);
void updateDisplay();
void handleButton(unsigned long now);
void persistSaveNow();

// --- Функции меню и кнопок (после рефакторинга) ---
void startServiceMenu();
void exitServiceMenu();
void updateServiceMenu(unsigned long now);
void enterCalibrationMode();
void calibShortPress();
void calibHoldSave();
void calibHoldReset();
void handleButton_NormalMode(unsigned long now, unsigned long held);
void handleButton_MenuNavMode(unsigned long now, unsigned long held);
void handleButton_CalibEditMode(unsigned long now, unsigned long held);


// Функция для отладочного вывода в Serial
void debugPrintf(const char *fmt, ...) {
#if DEBUG_SERIAL
  va_list args; va_start(args, fmt);
  char buf[256]; vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
#endif
}

// ============== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ================
// Возвращает текущее время работы в минутах
uint32_t minuteNow() { return (uint32_t)(millis() / 60000UL); }

// Рассчитывает адрес ячейки в EEPROM по ее номеру
uint16_t ee_slot_addr(uint8_t slot) { return (uint16_t)(EE_BASE_ADDR + slot * EE_SLOT_SIZE); }

// Реализация алгоритма CRC16-CCITT для проверки целостности данных
uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t seed) {
  uint16_t crc = seed;
  while (len--) {
    crc ^= ((uint16_t)*data++) << 8;
    for (uint8_t i=0;i<8;i++) crc = (crc & 0x8000) ? (crc<<1) ^ 0x1021 : (crc<<1);
  }
  return crc;
}

// ============== ЛОГИКА РАБОТЫ С EEPROM ================
// Загружает наиболее свежие данные из EEPROM
bool loadPersisted() {
  Persist best; bool found=false;
  uint32_t bestSlotID = 0;
  Persist tmp;
  // Перебираем все ячейки
  for (uint8_t s=0;s<EE_SLOTS;s++){
    int addr = ee_slot_addr(s);
    EEPROM.get(addr, tmp);
    // Проверяем "магическое" число
    if (tmp.magic != EE_MAGIC) continue;
    // Проверяем контрольную сумму
    uint16_t saved = tmp.crc16;
    tmp.crc16 = 0;
    uint16_t calc = crc16_ccitt((const uint8_t*)&tmp, sizeof(Persist));
    if (calc != saved) continue;
    // Выбираем запись с самым большим slotID (самую новую)
    if (!found || tmp.slotID >= bestSlotID) {
      best = tmp; bestSlotID = tmp.slotID; found = true; currentSlot = s;
    }
  }
  if (!found) return false; // Не найдено ни одной корректной записи
  persisted = best; persistedDirty = false;
  debugPrintf("[EEPROM] Загружена ячейка %u (ID=%u)\n", currentSlot, persisted.slotID);
  return true;
}

// Записывает данные в указанную ячейку EEPROM
void writePersistSlot(uint8_t slot) {
  persisted.magic = EE_MAGIC;
  persisted.slotID++; // Увеличиваем ID, чтобы эта запись стала самой новой
  persisted.crc16 = 0; // Считаем новую CRC
  persisted.crc16 = crc16_ccitt((const uint8_t*)&persisted, sizeof(Persist));
  int addr = ee_slot_addr(slot);
  EEPROM.put(addr, persisted);
  debugPrintf("[EEPROM] Записана ячейка %u (ID=%u)\n", slot, persisted.slotID);
}

// Сохраняет данные в следующую ячейку (циклический буфер)
void savePersistRotate() {
  currentSlot = (currentSlot + 1) % EE_SLOTS;
  writePersistSlot(currentSlot);
  persistedDirty = false;
}

// Сохраняет данные, если установлен флаг `persistedDirty`
void savePersistIfNeeded() {
  if (!persistedDirty) return;
  savePersistRotate();
}

// Немедленное сохранение данных (используется при калибровке)
void persistSaveNow() {
  savePersistRotate();
}

// ============== РАБОТА С ДАТЧИКАМИ ================
// Рассчитывает абсолютную влажность (г/м^3) по формуле Магнуса
float calcAbsoluteHumidity(float tempC, float rh) {
  float expPart = exp((17.67f * tempC) / (tempC + 243.5f));
  float e = 6.112f * expPart * (rh / 100.0f); // Давление насыщенного пара в гПа
  return 2.1674f * e / (273.15f + tempC);
}

// Опрашивает все датчики и обновляет глобальные переменные
void updateSensors() {
  // Датчик BME280 (погреб)
  if (bmeOk) {
    float t = bme.readTemperature() + persisted.cal_t_cellar_bme;
    float rh = bme.readHumidity() + persisted.cal_rh_cellar;
    if (!isnan(t) && !isnan(rh)) {
      t_cellar = t;
      rh_cellar = rh;
      ah_cellar = calcAbsoluteHumidity(t, rh);
    } else { bmeOk = false; debugPrintf("[ERR] BME: нет данных (NAN)\n"); }
  }
  // Датчик HTU21D (улица)
  if (htuOk) {
    float t = htu.readTemperature() + persisted.cal_t_out;
    float rh = htu.readHumidity() + persisted.cal_rh_out;
    if (!isnan(t) && !isnan(rh)) {
      t_out = t;
      rh_out = rh;
      ah_out = calcAbsoluteHumidity(t, rh);
    } else { htuOk = false; debugPrintf("[ERR] HTU: нет данных (NAN)\n"); }
  }
  // Датчик DS18B20 (резервный, погреб)
  ds.requestTemperatures();
  float td = ds.getTempCByIndex(0);
  // Проверяем на стандартные ошибки DS18B20: -127 (нет подключения) и 85 (ошибка чтения)
  if (td != DEVICE_DISCONNECTED_C && td != 85.0) {
    t_ds18 = td + persisted.cal_t_cellar_ds18;
    dsOk = true;
    // Если основной датчик (BME) отказал, используем резервный
    if (!bmeOk) t_cellar = t_ds18;
  } else { 
    dsOk = false; 
    debugPrintf("[WARN] DS18B20 не отвечает или вернул ошибку (Код: %f)\n", td); 
  }

  // Попытка переинициализировать отказавшие датчики
  if (!bmeOk) { bmeOk = bme.begin(0x76) || bme.begin(0x77); if (bmeOk) debugPrintf("[INFO] BME восстановлен\n"); }
  if (!htuOk) { htuOk = htu.begin(); if (htuOk) debugPrintf("[INFO] HTU восстановлен\n"); }
  if (!dsOk) { ds.begin(); ds.requestTemperatures(); if (ds.getTempCByIndex(0) != DEVICE_DISCONNECTED_C) { dsOk=true; debugPrintf("[INFO] DS восстановлен\n"); } }
}

// ============== КОНЕЧНЫЙ АВТОМАТ ОЗОНАТОРА ================
void ozoneStateMachine(unsigned long now) {
  uint32_t nowMin = minuteNow();
  switch (ozoneState) {
    case OZ_IDLE: // Состояние ожидания
      // Условия для старта: подошло время, уличный датчик работает, на улице не мороз,
      // подсветка выключена (нет человека), прошла безопасная задержка после включения.
      if ((uint32_t)nowMin >= persisted.nextOzoneAtMin && htuOk && (t_out >= 0.0f) && !backlightOn && (millis() - systemBootMillis >= OZONE_MIN_SAFE_AFTER_BOOT_MS)) {
        ozoneState = OZ_RUNNING;
        ozoneStateStart = now;
        digitalWrite(OZONE_PIN, HIGH); // Включаем озонатор
        persisted.ozoneCycles++;
        persisted.lastOzoneRunUptime = persisted.uptimeMin;
        persisted.nextOzoneAtMin = nowMin + (uint32_t)(OZONE_INTERVAL_MS / 60000UL); // Планируем следующий запуск
        persistedDirty = true;
        savePersistIfNeeded();
        debugPrintf("[OZONE] Цикл запущен\n");
      }
      break;
    case OZ_RUNNING: // Фаза генерации озона
      if (now - ozoneStateStart >= OZONE_DURATION_MS) {
        ozoneState = OZ_REST; // Переходим к фазе отдыха
        ozoneStateStart = now;
        digitalWrite(OZONE_PIN, LOW); // Выключаем озонатор
        persistedDirty = true;
        debugPrintf("[OZONE] Генерация завершена -> Отдых\n");
      } else {
        // Если во время работы включилась подсветка (появился человек), аварийно прерываем
        if (backlightOn) {
          ozoneState = OZ_REST;
          ozoneStateStart = now;
          digitalWrite(OZONE_PIN, LOW);
          persistedDirty = true;
          debugPrintf("[OZONE] Прервано присутствием -> Отдых\n");
        }
      }
      break;
    case OZ_REST: // Фаза отдыха (озон реагирует)
      if (now - ozoneStateStart >= OZONE_REST_MS) {
        ozoneState = OZ_VENT; // Переходим к вентиляции
        ozoneStateStart = now;
        digitalWrite(FAN_PIN, HIGH); // Принудительно включаем вентилятор
        fanState = true;
        lastFanSwitchMillis = now;
        debugPrintf("[OZONE] Отдых завершен -> Вентиляция\n");
      }
      break;
    case OZ_VENT: // Фаза вентиляции
      if (now - ozoneStateStart >= OZONE_VENT_MS) {
        ozoneState = OZ_IDLE; // Возвращаемся в режим ожидания
        ozoneStateStart = 0;
        digitalWrite(FAN_PIN, LOW); // Выключаем принудительную вентиляцию
        fanState = false;
        debugPrintf("[OZONE] Вентиляция завершена -> Ожидание\n");
      }
      break;
  }
}

// ============== ЛОГИКА УПРАВЛЕНИЯ ВЕНТИЛЯТОРОМ ================
void controlFan(unsigned long now) {
  // --- Проверка безопасности: отключаем вентилятор, если датчики влажности неисправны ---
  // Логика зависит от данных по абсолютной влажности, поэтому оба датчика (внутренний и внешний) должны быть исправны.
  if (!bmeOk || !htuOk) {
    if (fanState) {
      digitalWrite(FAN_PIN, LOW);
      fanState = false;
      lastFanSwitchMillis = now;
      debugPrintf("[FAN] Принудительно выключен из-за ошибки датчика влажности\n");
    }
    return; // Запрещаем дальнейшую работу функции
  }

  // Озонатор имеет приоритет над автоматикой
  if (ozoneState == OZ_VENT) return; // Вентилятор уже принудительно включен
  if (ozoneState == OZ_RUNNING || ozoneState == OZ_REST) {
    if (fanState) { // Вентилятор должен быть выключен
      digitalWrite(FAN_PIN, LOW);
      fanState = false;
      lastFanSwitchMillis = now;
      debugPrintf("[FAN] Принудительно выключен во время озонирования\n");
    }
    return;
  }

  // --- Логика управления с гистерезисом, зависящим от состояния ---

  // 1. Определяем, нужна ли вентиляция, исходя из показателей в погребе
  bool needs_ventilation;
  if (fanState) {
    // ВЕНТИЛЯТОР РАБОТАЕТ: Проверяем условия для ВЫКЛЮЧЕНИЯ.
    // Выключаем, если и температура, и влажность в норме, ИЛИ если стало слишком холодно.
    bool temp_ok_to_stop = isnan(t_cellar) || t_cellar <= (CELLAR_TEMP_TARGET - CELLAR_TEMP_HYST);
    bool rh_ok_to_stop = isnan(rh_cellar) || rh_cellar <= CELLAR_RH_MIN;
    bool cellar_too_cold = !isnan(t_cellar) && t_cellar <= CELLAR_TEMP_MIN;
    if ((temp_ok_to_stop && rh_ok_to_stop) || cellar_too_cold) {
      needs_ventilation = false;
    } else {
      needs_ventilation = true; // Продолжаем работать
    }
  } else {
    // ВЕНТИЛЯТОР ВЫКЛЮЧЕН: Проверяем условия для ВКЛЮЧЕНИЯ.
    // Включаем, если ИЛИ температура, ИЛИ влажность превысили верхний порог.
    bool temp_too_high = !isnan(t_cellar) && t_cellar >= (CELLAR_TEMP_TARGET + CELLAR_TEMP_HYST);
    bool rh_too_high = !isnan(rh_cellar) && rh_cellar >= CELLAR_RH_MAX;
    if (temp_too_high || rh_too_high) {
      needs_ventilation = true;
    } else {
      needs_ventilation = false; // Остаемся выключенными
    }
  }

  // 2. Проверяем, подходят ли уличные условия для вентиляции
  bool ah_ok = !isnan(ah_cellar) && !isnan(ah_out) && (ah_cellar - ah_out >= AH_DIFF_THRESHOLD); // Снаружи суше?
  bool outdoor_temp_ok = !isnan(t_out) && (t_out > -10.0f); // На улице не слишком холодно?

  // 3. Итоговое решение
  bool want_on = needs_ventilation && ah_ok && outdoor_temp_ok;

  // 4. Выполняем переключение с учетом минимального интервала
  bool canSwitch = ((now - lastFanSwitchMillis) >= FAN_MIN_SWITCH_INTERVAL);
  if (want_on && !fanState && canSwitch) {
    digitalWrite(FAN_PIN, HIGH);
    fanState = true;
    lastFanSwitchMillis = now;
    debugPrintf("[FAN] АВТО ВКЛ\n");
  } else if ((!want_on) && fanState && canSwitch) {
    digitalWrite(FAN_PIN, LOW);
    fanState = false;
    lastFanSwitchMillis = now;
    debugPrintf("[FAN] АВТО ВЫКЛ\n");
  }
}

// ============== ОБНОВЛЕНИЕ ДИСПЛЕЯ ================
void updateDisplay() {
  char l1[17], l2[17];
  // Формируем строки для дисплея
  if (!isnan(t_cellar) && !isnan(rh_cellar) && !isnan(ah_cellar)) {
    snprintf(l1, sizeof(l1), "C:%4.1fC %2.0f%% %4.1fg", t_cellar, rh_cellar, ah_cellar);
  } else {
    snprintf(l1, sizeof(l1), "C: --.-C --%% ----g");
  }
  if (!isnan(t_out) && !isnan(rh_out) && !isnan(ah_out)) {
    snprintf(l2, sizeof(l2), "O:%4.1fC %2.0f%% %4.1fg", t_out, rh_out, ah_out);
  } else {
    snprintf(l2, sizeof(l2), "O: --.-C --%% ----g");
  }

  // Добавляем индикатор состояния озонатора в конец первой строки
  char tag[4] = "   ";
  if (ozoneState == OZ_RUNNING) strncpy(tag, "O3", 3);
  else if (ozoneState == OZ_REST) strncpy(tag, "RST", 4);
  else if (ozoneState == OZ_VENT) strncpy(tag, "VNT", 4);

  char l1full[17];
  char left13[14]; strncpy(left13, l1, 13); left13[13]=0;
  snprintf(l1full, 17, "%-13s%s", left13, tag);

  // Обновляем дисплей, только если строки изменились (кэширование)
  if (memcmp(l1full, lastLine1, 16) != 0) {
    lcd.setCursor(0,0); lcd.print(l1full);
    memcpy(lastLine1, l1full, 16);
  }
  if (memcmp(l2, lastLine2, 16) != 0) {
    lcd.setCursor(0,1); lcd.print(l2);
    memcpy(lastLine2, l2, 16);
  }
}

// ============== МЕНЮ И КАЛИБРОВКА (после рефакторинга) ================

// --- Управление состоянием меню ---
void startServiceMenu() {
  menuMode = MENU_SERVICE;
  menuIndex = 0;
  menuSince = millis();
  menuLastTick = millis();
  lcd.backlight(); backlightOn = true; backlightTimer = millis();
  lcd.clear();
}

void enterCalibrationMode() {
  if (menuIndex >= 5) return; // Нельзя редактировать пункт "Выход"
  calibEditing = true;
  // Сразу обновляем экран, чтобы показать интерфейс редактирования
  lcd.clear();
  char buf[17];
  float *p = getCalPointerByIndex(menuIndex);
  snprintf(buf, sizeof(buf), "%s Edit", getCalNameByIndex(menuIndex));
  lcd.setCursor(0,0); lcd.print(buf);
  snprintf(buf, sizeof(buf), "Val:%+4.1f", *p);
  lcd.setCursor(0,1); lcd.print(buf);
}

void exitServiceMenu() {
    menuMode = MENU_NONE;
    calibEditing = false;
    lcd.clear();
    // Подсветка выключится по своему таймеру
}

// --- Отображение меню ---
// Возвращает указатель на переменную калибровки по индексу меню
float* getCalPointerByIndex(int idx) {
  switch (idx) {
    case 0: return &persisted.cal_t_cellar_bme;
    case 1: return &persisted.cal_rh_cellar;
    case 2: return &persisted.cal_t_out;
    case 3: return &persisted.cal_rh_out;
    case 4: return &persisted.cal_t_cellar_ds18;
    default: return NULL;
  }
}
// Возвращает имя параметра по индексу меню
const char* getCalNameByIndex(int idx) {
  switch (idx) {
    case 0: return "BME Temp";
    case 1: return "BME Hum ";
    case 2: return "HTU Temp";
    case 3: return "HTU Hum ";
    case 4: return "DS18 Temp";
    default: return "Выход";
  }
}

// Обновляет дисплей в режиме навигации по меню
void updateServiceMenu(unsigned long now) {
  if (menuMode != MENU_SERVICE || calibEditing) return; // Обновляем только в режиме навигации

  if (now - menuLastTick < 300) return; // Ограничиваем частоту обновления
  menuLastTick = now;

  // Формируем строки для текущего пункта меню
  char line1[17], line2[17];
  snprintf(line1, sizeof(line1), "> %s", getCalNameByIndex(menuIndex));

  if (menuIndex < 5) { // Если это параметр калибровки
      float *ptr = getCalPointerByIndex(menuIndex);
      snprintf(line2, sizeof(line2), "Val:%+4.1f Sel:hold", *ptr);
  } else { // Если это пункт "Выход"
      snprintf(line2, sizeof(line2), "Select: hold");
  }

  // Обновляем дисплей через кэш, чтобы избежать мерцания
  if (strncmp(line1, lastLine1, 16) != 0) {
    lcd.setCursor(0,0); lcd.print("                ");
    lcd.setCursor(0,0); lcd.print(line1);
    strncpy(lastLine1, line1, 16);
  }
  if (strncmp(line2, lastLine2, 16) != 0) {
    lcd.setCursor(0,1); lcd.print("                ");
    lcd.setCursor(0,1); lcd.print(line2);
    strncpy(lastLine2, line2, 16);
  }
}

// --- Действия в режиме калибровки ---
// Увеличить значение (короткое нажатие в режиме редактирования)
void calibShortPress() {
  if (!calibEditing) return;
  float *p = getCalPointerByIndex(menuIndex);
  if (!p) return;
  float step = (menuIndex == 1 || menuIndex == 3) ? CAL_STEP_HUM : CAL_STEP_TEMP;
  float v = *p + step;
  if (v > CAL_MAX_VAL) v = CAL_MIN_VAL; // Зацикливание значения
  *p = roundf(v * 10.0f) / 10.0f;
  // Обновляем значение на дисплее
  char buf[17];
  snprintf(buf, sizeof(buf), "Val:%+4.1f", *p);
  lcd.setCursor(0,1); lcd.print("                ");
  lcd.setCursor(0,1); lcd.print(buf);
}

// Сохранить значение (длинное нажатие)
void calibHoldSave() {
  if (!calibEditing) return;
  persistedDirty = true;
  persistSaveNow();
  calibEditing = false; // Возвращаемся в режим навигации
  menuLastTick = millis(); // Принудительно обновляем экран меню
  lcd.clear();
}

// Сбросить значение на 0 (сверхдлинное нажатие)
void calibHoldReset() {
  if (!calibEditing) return;
  float *p = getCalPointerByIndex(menuIndex);
  if (!p) return;
  *p = 0.0f;
  persistedDirty = true;
  persistSaveNow();
  calibEditing = false; // Возвращаемся в режим навигации
  menuLastTick = millis(); // Принудительно обновляем экран меню
  lcd.clear();
}

// ============== ОБРАБОТКА НАЖАТИЙ КНОПКИ (после рефакторинга) ==============

// Обработчик для обычного режима работы (меню выключено)
void handleButton_NormalMode(unsigned long now, unsigned long held) {
  if (held < BUTTON_LONG_MS) { // Короткое нажатие -> включить подсветку
    lcd.backlight();
    backlightOn = true;
    backlightTimer = now;
  } else { // Длинное нажатие -> войти в сервисное меню
    startServiceMenu();
  }
}

// Обработчик для режима навигации по меню
void handleButton_MenuNavMode(unsigned long now, unsigned long held) {
  if (held < BUTTON_LONG_MS) { // Короткое нажатие -> перейти к следующему пункту
    menuIndex = (menuIndex + 1) % SERVICE_ITEMS;
    menuLastTick = now; // Принудительно обновить экран
  } else { // Длинное нажатие -> выбрать текущий пункт
    if (menuIndex < 5) { // Если это параметр, входим в режим редактирования
      enterCalibrationMode();
    } else { // Если это "Выход", выходим из меню
      exitServiceMenu();
    }
  }
}

// Обработчик для режима редактирования калибровки
void handleButton_CalibEditMode(unsigned long now, unsigned long held) {
  if (held < BUTTON_LONG_MS) {
    calibShortPress(); // Увеличить значение
  } else if (held < BUTTON_RESET_MS) {
    calibHoldSave();   // Сохранить и выйти из редактирования
  } else {
    calibHoldReset();  // Сбросить на 0, сохранить и выйти
  }
}

// Главный диспетчер обработки кнопки
void handleButton(unsigned long now) {
  bool raw = digitalRead(BUTTON_PIN);
  // Подавление дребезга
  if (raw != lastButtonRaw) {
    lastButtonChange = now;
    lastButtonRaw = raw;
  }
  if (now - lastButtonChange <= BUTTON_DEBOUNCE_MS) return;

  // Логика на основе стабильного состояния
  static bool stable = HIGH;
  if (raw != stable) {
    stable = raw;
    if (raw == LOW) {
      // Кнопка была нажата, запоминаем время
      buttonPressedAt = now;
    } else {
      // Кнопка была отпущена, вычисляем длительность и вызываем нужный обработчик
      unsigned long held = now - buttonPressedAt;
      if (menuMode == MENU_NONE) {
        handleButton_NormalMode(now, held);
      } else if (menuMode == MENU_SERVICE) {
        if (calibEditing) {
          handleButton_CalibEditMode(now, held);
        } else {
          handleButton_MenuNavMode(now, held);
        }
      }
    }
  }
}

// ============== ПЕРИОДИЧЕСКОЕ СОХРАНЕНИЕ ================
void periodicSave(unsigned long now) {
  if (now - lastSaveMillis >= WORK_SAVE_INTERVAL) {
    lastSaveMillis = now;
    uint32_t addMin = (uint32_t)(WORK_SAVE_INTERVAL / 60000UL);
    persisted.uptimeMin += addMin;
    if (fanState) persisted.fanMin += addMin;
    if (ozoneState == OZ_RUNNING) persisted.ozoneMin += addMin;
    persistedDirty = true;
    savePersistIfNeeded();
    debugPrintf("[SAVE] Uptime=%lu, Fan=%lu, Ozone=%lu\n", persisted.uptimeMin, persisted.fanMin, persisted.ozoneMin);
  }
}

// ============== ФУНКЦИЯ НАСТРОЙКИ (SETUP) ================
void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FAN_PIN, OUTPUT); digitalWrite(FAN_PIN, LOW);
  pinMode(OZONE_PIN, OUTPUT); digitalWrite(OZONE_PIN, LOW);

  lcd.init(); lcd.clear(); lcd.backlight(); backlightOn = true;
  lcd.setCursor(0,0); lcd.print("Cellar v3.0 boot");

#if DEBUG_SERIAL
  Serial.begin(9600);
  Serial.println("Boot v3.0");
#endif

  Wire.begin();
  // --- Проверка инициализации датчиков ---
  bmeOk = bme.begin(0x76) || bme.begin(0x77);
  if (!bmeOk) {
    lcd.setCursor(0,1); lcd.print("BME280 Error");
    debugPrintf("[ERR] BME280 init failed\n");
    delay(2000);
  }
  htuOk = htu.begin();
  if (!htuOk) {
    lcd.setCursor(0,1); lcd.print("HTU21D Error");
    debugPrintf("[ERR] HTU21D init failed\n");
    delay(2000);
  }
  ds.begin();
  ds.requestTemperatures();
  if (ds.getTempCByIndex(0) == DEVICE_DISCONNECTED_C) {
      dsOk = false;
      lcd.setCursor(0,1); lcd.print("DS18B20 Error");
      debugPrintf("[WARN] DS18B20 not found at boot\n");
      delay(2000);
  } else {
      dsOk = true;
  }
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Cellar v3.0 boot");

  // Загрузка сохраненного состояния из EEPROM
  if (!loadPersisted()) {
    // Если не удалось загрузить, инициализируем значения по умолчанию
    memset(&persisted, 0, sizeof(persisted));
    persisted.magic = EE_MAGIC;
    // ... (инициализация всех полей)
    persisted.nextOzoneAtMin = minuteNow() + (uint32_t)(OZONE_FIRST_DELAY_MS / 60000UL);
    persistedDirty = true;
    savePersistIfNeeded();
    debugPrintf("[EEPROM] Инициализированы значения по умолчанию\n");
  }

  // Проверка, был ли сброс по сторожевому таймеру
  if (MCUSR & _BV(WDRF)) {
    persisted.wdtCount++;
    persisted.lastOzoneRunUptime = persisted.uptimeMin;
    persistedDirty = true;
    savePersistIfNeeded();
    lcd.setCursor(0,1); lcd.print("WDT reset logged");
    delay(700);
  }
  MCUSR = 0; // Сбрасываем флаги причины сброса
  wdt_enable(WDTO_8S); // Включаем сторожевой таймер с таймаутом 8 секунд

  // Инициализация переменных состояния
  lastSensorMillis = 0;
  lastSaveMillis = millis();
  backlightTimer = millis();
  systemBootMillis = millis();
  menuMode = MENU_NONE;
  ozoneState = OZ_IDLE;
  ozoneStateStart = 0;
  lastFanSwitchMillis = 0;

  lcd.clear();
}

// ============== ОСНОВНОЙ ЦИКЛ (LOOP) ================
void loop() {
  wdt_reset(); // Сбрасываем сторожевой таймер
  unsigned long now = millis();

  // Обрабатываем нажатия кнопки
  handleButton(now);

  // Автоматически выключаем подсветку по таймеру
  if (backlightOn && (now - backlightTimer >= BACKLIGHT_TIMEOUT)) {
    backlightOn = false; lcd.noBacklight();
  }

  // Обновляем дисплей, если мы в меню
  if (menuMode == MENU_SERVICE) updateServiceMenu(now);

  // Периодический опрос датчиков и обновление логики
  if (now - lastSensorMillis >= SENSOR_UPDATE_INTERVAL) {
    lastSensorMillis = now;
    updateSensors();
    // Запускаем конечные автоматы только после безопасной задержки
    if (millis() - systemBootMillis >= OZONE_MIN_SAFE_AFTER_BOOT_MS) {
      ozoneStateMachine(now);
    }
    controlFan(now);
    // Обновляем главный экран, только если не в меню
    if (menuMode == MENU_NONE) {
      updateDisplay();
    }
  }

  // Периодическое сохранение состояния
  periodicSave(now);

  // Сохраняем данные, если были изменения
  savePersistIfNeeded();

  delay(50); // Небольшая задержка для экономии энергии и стабильности
  wdt_reset();
}

/* ============================================================================
   Конец кода.

   ИНСТРУКЦИЯ ПО КАЛИБРОВКЕ:
   - Длинное нажатие (2 сек): вход в Сервисное Меню.
   - В меню:
     * Короткие нажатия: переключение между пунктами (BME Temp, BME Hum, ..., Выход).
     * Длинное нажатие: выбор текущего пункта.
       - Если выбран параметр -> вход в режим редактирования.
       - Если выбран "Выход" -> выход из меню.
   - В режиме редактирования:
     * Короткое нажатие: увеличение значения на 1 шаг.
     * Длинное нажатие (2-5 сек): сохранение текущего значения и выход из редактирования.
     * Сверхдлинное нажатие (>5 сек): сброс значения на 0, сохранение и выход.
   
   Примечание: Все значения калибровки сохраняются в энергонезависимую память (EEPROM).
   Безопасность: Озонатор не запустится, пока не пройдет 5 минут после включения,
   пока уличный датчик не покажет температуру выше нуля, и немедленно отключится,
   если будет обнаружено присутствие человека (по включению подсветки).
   ============================================================================ */
