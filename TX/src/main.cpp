/**
 * @file main.cpp
 * @project BadIncESP - Модуль TX (Датчик и вычислитель)
 * @version 1.0
 * * @description 
 * Инерционно-независимый уровень. Система считывает показания с датчика BNO080 (Pitch/Roll), 
 * обрабатывает их с учетом настроек (кнопка) и вычисляет нужный паттерн индикации.
 * * @changelog
 * v1.0 - Первичное портирование на ESP32-S3. 
 * - Настроены пины (I2C: SDA=5, SCL=6; Button=9; LED=10).
 * - Функции LoRa и EEPROM заменены заглушками (Mock) для вывода в Serial.
 * - Отключена библиотека PGMWrap.h, используется нативный PROGMEM ESP32.
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include "SparkFun_BNO080_Arduino_Library.h"
 #include <OneButton.h>
 
 // ================= НАСТРОЙКИ ПИНОВ ESP32-S3 =================
 #define PIN_I2C_SDA 5
 #define PIN_I2C_SCL 6
 #define PIN_CONTROL_BUTTON 9
 #define PIN_FB_LED 10
 
 // ================= ОСНОВНЫЕ НАСТРОЙКИ =================
 #define USE_X_AXIS 1  // true
 #define USE_Y_AXIS 0  // false
 bool usedAxis = USE_X_AXIS;    
 
 #define FB_LED_BRIGHTNESS 30   // 0 - 255 - Яркость леда обратной связи
 #define NUM_MODES 23           // Количество вариантов свечения ленты
 #define NUM_FADES 4            // Количество вариантов яркости
 #define NUM_SENSITIVITIES 4    // Количество вариантов чувствительности
 
 #define VERY_LONG_PRESS_MS 3000 // Длительность очень длинного нажатия (мс)
 
 // Команды для связи (пока используются только для логов)
 #define CMD_BRIGHTNESS       100 
 #define CMD_SENSITIVITY      101 
 #define CMD_SWITCHSIDES      102 
 #define CMD_LONGPRESS        103 
 #define CMD_CALIBRATION      104 
 #define CMD_MODE             111 
 #define CMD_GREETING         112 
 #define CMD_EEPROM_WRITE     113 
 
 // ================= ГЛОБАЛЬНЫЕ ОБЪЕКТЫ =================
 BNO080 myIMU;
 OneButton buttonControl(PIN_CONTROL_BUTTON, true);
 
 // ================= ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ =================
 byte curFade = 0; 
 byte curMode = NUM_MODES / 2; // Центр (LEVEL)   
 byte prevMode = -1; 
 float Roll = 0.0;   
 float deltaZero = 0.0; 
 boolean revers = false;   
 byte curSensitivity = 0;    
 
 boolean startCalibrationMode = false;
 uint32_t verylongPressTimer = 0;
 
 uint8_t fades[NUM_FADES] = {255, 64, 32, 8}; 
 
 // Границы диапазонов крена (в градусах) для разных чувствительностей
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
 void sendLoRaMessage(byte msgCmd, byte sndData);
 void prepareEEPROMWrite();
 void setZERO();
 
 // ================= ЗАГЛУШКИ (MOCKS) ДЛЯ ИЗОЛЯЦИИ =================
 void sendLoRaMessage(byte msgCmd, byte sndData) {
   Serial.printf("[MOCK RADIO] Отправка команды: Cmd=%d, Data=%d\n", msgCmd, sndData);
 }
 
 void prepareEEPROMWrite() {
   Serial.println("[MOCK EEPROM] Запланирована запись настроек в память.");
 }
 
 // ================= АППАРАТНЫЕ ФУНКЦИИ =================
 void initFBled() {
   pinMode(PIN_FB_LED, OUTPUT);
   analogWrite(PIN_FB_LED, 0); 
   updateFBLed(1);
 }
 
 void updateFBLed(bool ledStatus) {
   // ESP32 Arduino Core >= 3.0 поддерживает analogWrite нативно
   analogWrite(PIN_FB_LED, ledStatus ? FB_LED_BRIGHTNESS : 0);
 }
 
 void flashFBLed(int times) {
   Serial.printf("Мигание LED: %d раз\n", times);
   bool flash = false;
   for (int i = 0; i < times * 2; i++) {
     flash = !flash;
     updateFBLed(flash);
     delay(200);
   }
   delay(200);
 }
 
 void initWire() {
   Serial.println("Инициализация I2C...");
   Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); // Принудительно задаем пины для ESP32-S3
   Wire.setClock(400000); 
   delay(100);
 }
 
 void initIMU() {
   Serial.println("Подключение к BNO080...");
   if (myIMU.begin() == false) {
     Serial.println("ОШИБКА: BNO080 не найден! Проверьте подключение.");
     flashFBLed(7);
     while (1) { delay(10); } // Остановка
   }
   Serial.println("BNO080 найден. Включение GameRotationVector...");
   myIMU.enableGameRotationVector(25); 
   delay(200);
   myIMU.enableStabilityClassifier(25); 
   delay(200);
 }
 
 // ================= ФУНКЦИИ КНОПКИ =================
 void clickControl() {
   Serial.println("Кнопка: Короткое нажатие (Яркость)");
   curFade = (curFade + 1) % NUM_FADES;
   sendLoRaMessage(CMD_BRIGHTNESS, curFade);
   flashFBLed(1);
   prevMode = -1;
   prepareEEPROMWrite();
 }
 
 void doubleclickControl() {
   Serial.println("Кнопка: Двойное нажатие (Чувствительность)");
   curSensitivity = (curSensitivity + 1) % NUM_SENSITIVITIES;
   sendLoRaMessage(CMD_SENSITIVITY, curSensitivity);
   flashFBLed(2);
   prevMode = -1;
   prepareEEPROMWrite();
 }
 
 void switchSides() {
   Serial.print("Инверсия сторон: ");
   revers = !revers;
   Serial.println(revers ? "ВКЛ" : "ВЫКЛ");
   sendLoRaMessage(CMD_SWITCHSIDES, revers);
   flashFBLed(2);
   prevMode = -1;
 }
 
 void longPressStartControl() {
   Serial.println("Кнопка: Старт долгого нажатия");
   startCalibrationMode = true;
   verylongPressTimer = millis() + VERY_LONG_PRESS_MS;
   sendLoRaMessage(CMD_LONGPRESS, 1);
   flashFBLed(4);
 }
 
 void longPressControl() {
   if (millis() > verylongPressTimer) {
     Serial.println("Кнопка: ОЧЕНЬ долгое нажатие (Инверсия сторон)");
     verylongPressTimer = millis() + VERY_LONG_PRESS_MS; 
     startCalibrationMode = false;
     switchSides();
     prepareEEPROMWrite();
   }
 }
 
 void longPressStopControl() {
   Serial.println("Кнопка: Отпускание");
   if (startCalibrationMode) {
     Serial.println("Запуск калибровки нуля!");
     startCalibrationMode = false;
     sendLoRaMessage(CMD_CALIBRATION, 1);
     flashFBLed(2);
     prevMode = -1;
     prepareEEPROMWrite();
     setZERO();
   }
 }
 
 void initButtons() {
   Serial.println("Инициализация кнопки...");
   buttonControl.attachClick(clickControl);
   buttonControl.attachDoubleClick(doubleclickControl);
   buttonControl.attachLongPressStart(longPressStartControl);
   buttonControl.attachLongPressStop(longPressStopControl);
   buttonControl.attachDuringLongPress(longPressControl);
 }
 
 // ================= МАТЕМАТИКА =================
 void setZERO() {
   Serial.print("Текущий ноль: "); Serial.println(deltaZero);
   
   byte moving_status = 5;
   Serial.print("Ожидание стабилизации датчика... ");
   do {
     delay(200);
     if (myIMU.dataAvailable()) {
       moving_status = myIMU.getStabilityClassifier();
     }
   } while (moving_status > 3);
   Serial.println("OK (стабилен).");
 
   float delta0 = 0;
   for (int i = 0; i < 100; i++) {
     delta0 += (usedAxis ? myIMU.getPitch() : myIMU.getRoll());
     delay(50);
   }
   deltaZero = (delta0 * 180.0 / PI) / 100.0;
   Serial.print("Новый ноль установлен: "); Serial.println(deltaZero);
 }
 
 void getNextRoll() {
   if (myIMU.dataAvailable()) {
     Roll = (usedAxis ? myIMU.getPitch() : myIMU.getRoll());
     Roll *= 57.29578; // Радианы в градусы
     Roll -= deltaZero;
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
 
 // ================= ОСНОВНОЙ ЦИКЛ =================
 void setup() {
   Serial.begin(115200);
   delay(2000); // Пауза для открытия Serial Monitor по USB
   
   Serial.println("\n==== BadIncESP TX v1.0 Запуск ====");
   initWire();
   initButtons();
   initFBled();
   initIMU();
   
   flashFBLed(1);
   Serial.println("Система готова. Ожидание данных...");
 }
 
 void loop() {
   buttonControl.tick();
   
   getNextRoll();
   curMode = getMode();
   
   if (curMode != prevMode) {
     Serial.printf("Угол: %6.2f° | Режим (Mode): %d\n", Roll, curMode);
     sendLoRaMessage(CMD_MODE, curMode);
     prevMode = curMode;
   }
   
   delay(10); // Небольшая пауза для стабильности
 }