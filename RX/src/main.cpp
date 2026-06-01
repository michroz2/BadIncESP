/**
 * @file main.cpp
 * @project BadIncESP - Модуль RX (Индикатор)
 * @version 1.0
 * * @description 
 * Инерционно-независимый уровень. Модуль принимает команды отображения и выводит их 
 * на светодиодную ленту WS2812B.
 * * @changelog
 * v1.0 - Первичное портирование на ESP32-S3. 
 * - Настроен пин для ленты (GPIO 7).
 * - Удален код отладочной кнопки (в финале на RX она не нужна).
 * - Добавлен автономный тест (переключение режимов каждые 500мс) для проверки ленты.
 * - Радиосвязь и EEPROM изолированы.
 */

 #include <Arduino.h>
 #include <FastLED.h>
 
 // ================= НАСТРОЙКИ ПИНОВ =================
 #define PIN_LEDS 7    // Пин данных для ленты WS2812B
 
 // ================= ОСНОВНЫЕ НАСТРОЙКИ =================
 #define NUM_LEDS 13   // Количество ЛЕДов в ленте
 #define NUM_MODES 23  // Количество вариантов свечения ленты
 #define NUM_FADES 4   // Количество вариантов яркости ленты
 
 // Глобальные переменные состояния
 byte curFade = 0; 
 byte curMode = NUM_MODES / 2; // Центр (LEVEL)
 byte prevMode = -1; 
 boolean revers = false;   
 byte curSensitivity = 0;    
 
 uint8_t fades[NUM_FADES] = {255, 64, 32, 8}; 
 CRGB leds[NUM_LEDS];
 
 // ================= МАССИВЫ ЦВЕТОВ =================
 // Задаем левую половину, правая инициализируется симметрично в initMODS()
 CRGB modes[NUM_MODES][NUM_LEDS] =
 {
   {0xff0000, 0x640800, 0x321400,        0,        0,        0,        0,        0, 0, 0, 0, 0, 0},  //L11
   {0xff0000, 0x8c0000, 0x500a00, 0x3c1400, 0x200800,        0,        0,        0, 0, 0, 0, 0, 0},  //L10
   {0xff0000, 0x780000, 0x641000, 0x321E00, 0x180E00, 0x0F1400, 0x000400,        0, 0, 0, 0, 0, 0},  //L9
   {0x0a0000, 0x780000, 0x641000, 0x321E00, 0x321E00, 0x1E2D00, 0x000C00,        0, 0, 0, 0, 0, 0},  //L8
   {0x040000, 0x080000, 0x641000, 0x501E00, 0x463200, 0x283C00, 0x001E00,        0, 0, 0, 0, 0, 0},  //L7
   {0,        0x040000, 0x080000, 0x501E00, 0x5A4600, 0x3C5000, 0x003200,        0, 0, 0, 0, 0, 0},  //L6
   {0,        0,        0x040000, 0x501E00, 0x786400, 0x506400, 0x005000,        0, 0, 0, 0, 0, 0},  //L5
   {0,        0,        0,        0x321E00, 0x786400, 0x647800, 0x007800,        0, 0, 0, 0, 0, 0},  //L4
   {0,        0,        0,        0,        0x786400, 0x647800, 0x00A000,        0, 0, 0, 0, 0, 0},  //L3
   {0,        0,        0,        0,        0x2D1E00, 0x647800, 0x00C800,        0, 0, 0, 0, 0, 0},  //L2
   {0,        0,        0,        0,        0,        0x467800, 0x00FF00,        0, 0, 0, 0, 0, 0},  //L1
   {0,        0,        0,        0,        0,        0x002020, 0x00FF00, 0x002020, 0, 0, 0, 0, 0},  //!LEVEL MODE!
 };
 
 #define NUM_TEST_COLORS 3
 CRGB testColors[NUM_TEST_COLORS] = { CRGB::Red, CRGB::Green, CRGB::Blue };
 
 // ================= ФУНКЦИИ ИНДИКАЦИИ =================
 void initMODS() {
   Serial.println("Инициализация массивов симметрии...");
   for (byte i = 0; i < NUM_MODES / 2; i++) {
     for (byte j = 0; j < NUM_LEDS; j++) {
       modes[NUM_MODES - i - 1][NUM_LEDS - j - 1] = modes[i][j];
     }
   }
 }
 
 void copyMode() {
   byte k;
   for (byte i = 0; i < NUM_LEDS; i++) {
     k = revers ? (NUM_LEDS - 1 - i) : i;
     leds[i] = modes[curMode][k];
   }
 }
 
 void processLEDS() {
   if (curMode != prevMode) {
     copyMode();
     FastLED.show();
     prevMode = curMode;
   }
 }
 
 void initLEDs() {
   Serial.println("Инициализация WS2812B...");
   pinMode(PIN_LEDS, OUTPUT);
   FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
   FastLED.setBrightness(fades[curFade]);
   delay(100);
 }
 
 void playGreeting() {
   Serial.println("Анимация приветствия...");
   for (byte j = 0; j < 3; j++) {
     for (byte i = 0; i < NUM_LEDS; i++) {
       leds[i] = testColors[j];
       FastLED.show();
       delay(40);
     }
     prevMode = -1;
     delay(100);
   }
   // После приветствия очищаем ленту перед стартом
   for(int i=0; i<NUM_LEDS; i++) leds[i] = CRGB::Black;
   FastLED.show();
 }
 
 // ================= ОСНОВНОЙ ЦИКЛ =================
 void setup() {
   Serial.begin(115200);
   delay(2000); 
 
   Serial.println("\n==== BadIncESP RX v1.0 Запуск ====");
   initMODS();
   initLEDs();
   playGreeting();
   
   Serial.println("Система готова. Запуск автономного теста ленты...");
 }
 
 unsigned long lastChange = 0;
 
 void loop() {
   // АВТОНОМНЫЙ ТЕСТ: Переключаем режимы каждые 500 мс
   if (millis() - lastChange > 500) {
     lastChange = millis();
     curMode = (curMode + 1) % NUM_MODES;
     Serial.printf("Тест режима: %d\n", curMode);
   }
   
   processLEDS();
 }