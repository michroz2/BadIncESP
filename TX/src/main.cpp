/**
 * @file main.cpp
 * @project BadIncESP - Модуль TX (Датчик и вычислитель)
 * @version 1.6
 * * @changelog
 * v1.6 - Реализовано плавное гашение угла по оси Y в диапазоне 5-45 градусов.
 * - Добавлена калибровка нуля для оси Y (deltaZero_Y) с сохранением в Preferences.
 * v1.5 - Замена старой EEPROM на Preferences.h. Отложенная запись 15 секунд.
 * v1.1.1 - Исправление BLE: включен ScanResponse(true) для передачи UUID.
 * v1.1 - Замена LoRa на BLE Server с Notify. Статусный RGB LED (GPIO 21).
 * v1.0 - Первичное портирование периферии (BNO080, Кнопка, Статусный LED).
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include "SparkFun_BNO080_Arduino_Library.h"
 #include <OneButton.h>
 #include <FastLED.h>
 #include <Preferences.h> 
 
 // Подключение библиотек BLE
 #include <BLEDevice.h>
 #include <BLEServer.h>
 #include <BLEUtils.h>
 #include <BLE2902.h>
 
 // ================= ОПРЕДЕЛЕНИЕ ВЕРСИИ И ТИПА МОДУЛЯ =================
 #define MODULE_TYPE "TX (ДАТЧИК)"
 #define VERSION     "1.6"
 
 // ================= НАСТРОЙКИ ПИНОВ ESP32-S3 =================
 #define PIN_I2C_SDA 5
 #define PIN_I2C_SCL 6
 #define PIN_CONTROL_BUTTON 9
 #define PIN_FB_LED 10
 #define PIN_SYS_LED 21  // Встроенный адресный RGB LED
 
 // ================= НАСТРОЙКИ BLE =================
 #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
 #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
 
 BLEServer* pServer = NULL;
 BLECharacteristic* pCharacteristic = NULL;
 bool deviceConnected = false;
 bool oldDeviceConnected = false;
 
 // ================= ОСНОВНЫЕ НАСТРОЙКИ =================
 #define USE_X_AXIS 1  
 #define USE_Y_AXIS 0  
 bool usedAxis = USE_X_AXIS;    
 
 #define FB_LED_BRIGHTNESS 30   
 #define NUM_MODES 23           
 #define NUM_FADES 4            
 #define NUM_SENSITIVITIES 4    
 #define VERY_LONG_PRESS_MS 3000 
 
 // ================= НАСТРОЙКИ ГАШЕНИЯ ПО ОСИ Y =================
 const float DEADZONE_ANGLE_Y = 5.0f;      // До этого угла (градусы) влияние Y игнорируется
 const float MAX_DAMPING_ANGLE_Y = 45.0f;  // При этом угле и выше, основной угол принудительно гасится в 0.0
 
 // Команды для связи
 #define CMD_BRIGHTNESS       100 
 #define CMD_SENSITIVITY      101 
 #define CMD_SWITCHSIDES      102 
 #define CMD_LONGPRESS        103 
 #define CMD_CALIBRATION      104 
 #define CMD_MODE             111 
 #define CMD_GREETING         112 
 #define CMD_EEPROM_WRITE     113 
 
 // Настройки таймера сохранения памяти NVS
 #define WRITE_PREFERENCES_DELAY_MS 15000 
 Preferences prefs;
 bool writePrefsPending = false;
 uint32_t writePrefsTimer = 0;
 
 // Дефолтные значения параметров
 const byte defaultFade = 0;
 const byte defaultSensitivity = 0;
 const boolean defaultReverse = false;
 const float defaultDeltaZero = 0.0f;
 
 // ================= ГЛОБАЛЬНЫЕ ОБЪЕКТЫ =================
 BNO080 myIMU;
 OneButton buttonControl(PIN_CONTROL_BUTTON, true);
 CRGB sysLed[1]; 
 
 // ================= ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ СОСТОЯНИЯ =================
 byte curFade = defaultFade; 
 byte curMode = NUM_MODES / 2; 
 byte prevMode = -1; 
 float Roll = 0.0f;   
 float deltaZero = defaultDeltaZero; 
 float deltaZero_Y = defaultDeltaZero; // Калибровочный ноль перпендикулярной оси Y
 boolean revers = defaultReverse;   
 byte curSensitivity = defaultSensitivity;    
 
 boolean startCalibrationMode = false;
 uint32_t verylongPressTimer = 0;
 
 uint8_t fades[NUM_FADES] = {255, 64, 32, 8}; 
 
 const float modeRange[NUM_SENSITIVITIES][NUM_MODES - 1] PROGMEM = 
 {
   {10, 5.0, 2.2, 1.6, 1.1, 0.8, 0.6, 0.4, 0.3, 0.2, 0.1,  -0.1, -0.2, -0.3, -0.4, -0.6, -0.8, -1.1, -1.6, -2.2, -5.0, -10},
   {10, 5.0, 2.8, 2.1, 1.5, 1.1, 0.8, 0.5, 0.4, 0.3, 0.2,  -0.2, -0.3, -0.4, -0.5, -0.8, -1.1, -1.5, -2.1, -2.8, -5.0, -10},
   {10, 5.0, 3.4, 2.6, 1.9, 1.4, 1.0, 0.6, 0.5, 0.4, 0.3,  -0.3, -0.4, -0.5, -0.6, -1.0, -1.4, -1.9, -2.6, -3.4, -5.0, -10},
   {12, 6.0, 4.0, 3.1, 2.3, 1.7, 1.2, 0.7, 0.6, 0.5, 0.4,  -0.4, -0.5, -0.6, -0.7, -1.2, -1.7, -2.3, -3.1, -4.0, -6.0, -12},
 };
 
 // ================= ПРОТОТИПЫ ФУНКЦИЙ =================
 void updateFBLed(bool ledStatus);
 void flashFBLed(int times);
 void setSysLed(CRGB color);
 void sendLoRaMessage(byte msgCmd, byte sndData);
 void preparePreferencesWrite();
 void processPreferences();
 void loadSettings();
 void saveSettings();
 void setZERO();
 
 // ================= CALLBACKS ДЛЯ BLE =================
 class MyServerCallbacks: public BLEServerCallbacks {
     void onConnect(BLEServer* pServer) {
       deviceConnected = true;
     };
     void onDisconnect(BLEServer* pServer) {
       deviceConnected = false;
     }
 };
 
 // ================= РАБОТА С ЭНЕРГОНЕЗАВИМОЙ ПАМЯТЬЮ (NVS) =================
 void loadSettings() {
   Serial.println("Чтение настроек из памяти NVS...");
   prefs.begin("badinc", true); 
   
   curFade = prefs.getUChar("fade", defaultFade);
   curSensitivity = prefs.getUChar("sens", defaultSensitivity);
   revers = prefs.getBool("rev", defaultReverse);
   deltaZero = prefs.getFloat("zero", defaultDeltaZero);
   deltaZero_Y = prefs.getFloat("zero_y", defaultDeltaZero); // Загрузка нуля оси Y
   
   prefs.end();
 
   Serial.printf("Загружено: Яркость=%d, Чувствительность=%d, Реверс=%d, Ноль_X=%6.2f°, Ноль_Y=%6.2f°\n", 
                 curFade, curSensitivity, revers, deltaZero, deltaZero_Y);
 }
 
 void saveSettings() {
   Serial.println("ФАКТИЧЕСКОЕ СОХРАНЕНИЕ настроек в память NVS...");
   prefs.begin("badinc", false); 
   
   prefs.putUChar("fade", curFade);
   prefs.putUChar("sens", curSensitivity);
   prefs.putBool("rev", revers);
   prefs.putFloat("zero", deltaZero);
   prefs.putFloat("zero_y", deltaZero_Y); // Сохранение нуля оси Y
   
   prefs.end();
   Serial.println("Сохранение успешно выполнено.");
 }
 
 void preparePreferencesWrite() {
   writePrefsPending = true;
   writePrefsTimer = millis() + WRITE_PREFERENCES_DELAY_MS;
   Serial.println("Память: запланирована отложенная запись через 15 секунд.");
 }
 
 void processPreferences() {
   if (writePrefsPending && (millis() > writePrefsTimer)) {
     writePrefsPending = false;
     saveSettings();
     sendLoRaMessage(CMD_EEPROM_WRITE, 0);
     flashFBLed(3); 
     prevMode = -1; 
   }
 }
 
 // ================= АППАРАТНЫЕ ФУНКЦИИ (LEDs) =================
 void initFBled() {
   pinMode(PIN_FB_LED, OUTPUT);
   analogWrite(PIN_FB_LED, 0); 
   updateFBLed(1);
 }
 
 void updateFBLed(bool ledStatus) {
   analogWrite(PIN_FB_LED, ledStatus ? FB_LED_BRIGHTNESS : 0);
 }
 
 void flashFBLed(int times) {
   bool flash = false;
   for (int i = 0; i < times * 2; i++) {
     flash = !flash;
     updateFBLed(flash);
     delay(200);
   }
   delay(200);
 }
 
 void setSysLed(CRGB color) {
   sysLed[0] = color;
   FastLED.show();
 }
 
 // ================= ФУНКЦИИ СВЯЗИ BLE =================
 void sendLoRaMessage(byte msgCmd, byte sndData) {
   if (deviceConnected) {
     uint8_t packet[2];
     packet[0] = msgCmd;
     packet[1] = sndData;
     pCharacteristic->setValue(packet, 2);
     pCharacteristic->notify(); 
     Serial.printf("[BLE] Отправлено: Cmd=%d, Data=%d\n", msgCmd, sndData);
   } else {
     Serial.printf("[BLE] Нет связи. Пропуск отправки: Cmd=%d\n", msgCmd);
   }
 }
 
 // ================= ИНИЦИАЛИЗАЦИЯ ПЕРИФЕРИИ =================
 void initWire() {
   Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); 
   Wire.setClock(400000); 
   delay(100);
 }
 
 void initIMU() {
   if (myIMU.begin() == false) {
     Serial.println("ОШИБКА: BNO080 не найден!");
     setSysLed(CRGB::Red);
     while (1) { delay(10); } 
   }
   myIMU.enableGameRotationVector(25); 
   delay(200);
   myIMU.enableStabilityClassifier(25); 
   delay(200);
 }
 
 void initBLE() {
   BLEDevice::init("BadIncESP_TX");
   pServer = BLEDevice::createServer();
   pServer->setCallbacks(new MyServerCallbacks());
 
   BLEService *pService = pServer->createService(SERVICE_UUID);
   pCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID,
                       BLECharacteristic::PROPERTY_NOTIFY
                     );
   pCharacteristic->addDescriptor(new BLE2902());
   pService->start();
   
   BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
   pAdvertising->addServiceUUID(SERVICE_UUID); 
   pAdvertising->setScanResponse(true);        
   pAdvertising->setMinPreferred(0x06);        
   pAdvertising->setMaxPreferred(0x12);
   BLEDevice::startAdvertising();
   Serial.println("BLE Сервер запущен. Ожидание подключения...");
 }
 
 // ================= ФУНКЦИИ КНОПКИ настроек =================
 void clickControl() {
   curFade = (curFade + 1) % NUM_FADES;
   sendLoRaMessage(CMD_BRIGHTNESS, curFade);
   flashFBLed(1);
   prevMode = -1;
   preparePreferencesWrite();
 }
 
 void doubleclickControl() {
   curSensitivity = (curSensitivity + 1) % NUM_SENSITIVITIES;
   sendLoRaMessage(CMD_SENSITIVITY, curSensitivity);
   flashFBLed(2);
   prevMode = -1;
   preparePreferencesWrite();
 }
 
 void switchSides() {
   revers = !revers;
   sendLoRaMessage(CMD_SWITCHSIDES, revers);
   flashFBLed(2);
   prevMode = -1;
 }
 
 void longPressStartControl() {
   startCalibrationMode = true;
   verylongPressTimer = millis() + VERY_LONG_PRESS_MS;
   sendLoRaMessage(CMD_LONGPRESS, 1);
   flashFBLed(4);
 }
 
 void longPressControl() {
   if (millis() > verylongPressTimer) {
     verylongPressTimer = millis() + VERY_LONG_PRESS_MS; 
     startCalibrationMode = false;
     switchSides();
     preparePreferencesWrite();
   }
 }
 
 void longPressStopControl() {
   if (startCalibrationMode) {
     startCalibrationMode = false;
     sendLoRaMessage(CMD_CALIBRATION, 1);
     flashFBLed(2);
     prevMode = -1;
     preparePreferencesWrite();
     setZERO();
   }
 }
 
 void initButtons() {
   buttonControl.attachClick(clickControl);
   buttonControl.attachDoubleClick(doubleclickControl);
   buttonControl.attachLongPressStart(longPressStartControl);
   buttonControl.attachLongPressStop(longPressStopControl);
   buttonControl.attachDuringLongPress(longPressControl);
 }
 
 // ================= МАТЕМАТИКА ДАТЧИКА =================
 void setZERO() {
   byte moving_status = 5;
   do {
     delay(200);
     if (myIMU.dataAvailable()) {
       moving_status = myIMU.getStabilityClassifier();
     }
   } while (moving_status > 3);
 
   float delta0 = 0;
   float delta0_Y = 0;
   for (int i = 0; i < 100; i++) {
     delta0   += (usedAxis ? myIMU.getPitch() : myIMU.getRoll());
     delta0_Y += (usedAxis ? myIMU.getRoll() : myIMU.getPitch());
     delay(50);
   }
   deltaZero = (delta0 * 180.0 / PI) / 100.0;
   deltaZero_Y = (delta0_Y * 180.0 / PI) / 100.0; // Одновременная фиксация нуля Y
   Serial.printf("Калибровка выполнена. Новые нули: X=%6.2f°, Y=%6.2f°\n", deltaZero, deltaZero_Y);
 }
 
 void getNextRoll() {
   if (myIMU.dataAvailable()) {
     // 1. Получаем физические значения углов в градусах с учетом индивидуальных нулей
     Roll = (usedAxis ? myIMU.getPitch() : myIMU.getRoll()) * 57.29578 - deltaZero;
     float Pitch_Y = (usedAxis ? myIMU.getRoll() : myIMU.getPitch()) * 57.29578 - deltaZero_Y;
 
     // 2. Вычисляем модуль отклонения по перпендикулярной оси Y
     float absY = abs(Pitch_Y);
     float dampingFactor = 1.0f; 
 
     // 3. Линейная интерполяция коэффициента гашения
     if (absY >= MAX_DAMPING_ANGLE_Y) {
       dampingFactor = 0.0f; // Полное обнуление крена при 45 градусах тангажа и выше
     } 
     else if (absY > DEADZONE_ANGLE_Y) {
       // Пропорциональное уменьшение множителя от 1.0 до 0.0
       dampingFactor = 1.0f - ((absY - DEADZONE_ANGLE_Y) / (MAX_DAMPING_ANGLE_Y - DEADZONE_ANGLE_Y));
     }
 
     // 4. Модифицируем главный рабочий угол коэффициентом демпфирования
     Roll *= dampingFactor;
   }
 }
 
 byte getMode() {
   for (byte i = 0; i < (NUM_MODES - 1); i++) {
     if (Roll > modeRange[curSensitivity][i]) {
       return i;
     }
   }
   return (NUM_MODES - 1);
 }
 
 // ================= ОСНОВНОЙ ЦИКЛ СИСТЕМЫ =================
 void setup() {
   Serial.begin(115200);
   delay(2000); 
   
   Serial.println("\n========================================");
   Serial.printf("=== ПРОЕКТ BadIncESP | МОДУЛЬ: %s ===\n", MODULE_TYPE);
   Serial.printf("=== ВЕРСИЯ ПРОШИВКИ: %s                  ===\n", VERSION);
   Serial.println("========================================");
 
   FastLED.addLeds<WS2812B, PIN_SYS_LED, GRB>(sysLed, 1);
   FastLED.setBrightness(50);
   setSysLed(CRGB::Green); 
 
   initWire();
   initButtons();
   initFBled();
   loadSettings(); 
   initIMU();
   initBLE();
   
   flashFBLed(1);
   sendLoRaMessage(CMD_GREETING, 0); 
 }
 
 void loop() {
   buttonControl.tick();
   
   if (deviceConnected && !oldDeviceConnected) {
       oldDeviceConnected = true;
       setSysLed(CRGB::Blue); 
       Serial.println("RX Приемник подключился!");
       prevMode = -1; 
   }
   if (!deviceConnected && oldDeviceConnected) {
       delay(500); 
       pServer->startAdvertising(); 
       oldDeviceConnected = false;
       setSysLed(CRGB::Red); 
       Serial.println("RX Отключился. Повторный запуск Advertising...");
   }
   
   getNextRoll();
   curMode = getMode();
   
   if (curMode != prevMode) {
     sendLoRaMessage(CMD_MODE, curMode);
     prevMode = curMode;
   }
   
   processPreferences(); 
   delay(1); // Оптимизированный такт цикла
 }