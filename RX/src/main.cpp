/**
 * @file main.cpp
 * @project BadIncESP - Модуль RX (Индикатор)
 * @version 1.6
 * * @changelog
 * v1.6 - Синхронизация версии экосистемы (логика вынесена на сторону TX).
 * v1.5 - Замена старой EEPROM на Preferences.h. Мгновенная локальная запись.
 * v1.2.1 - Исправление BLE: асинхронный запуск сканера. Отладочный вывод статуса.
 * v1.2 - Внедрена поддержка двух аппаратных плат (WROOM и S3-Zero).
 * - Интегрирована защита от просадки напряжения при запуске (Brownout).
 */

 #include <Arduino.h>
 #include <FastLED.h>
 #include <BLEDevice.h>
 #include <Preferences.h> 
 
 // ================= ВЫБОР АППАРАТНОЙ ПЛАТЫ =================
 #define USE_WROOM_BOARD
 
 #ifdef USE_WROOM_BOARD
   #define PIN_LEDS 13     
   #define PIN_SYS_LED 2   
 #else
   #define PIN_LEDS 7      
   #define PIN_SYS_LED 21  
 #endif
 
 // ================= ОПРЕДЕЛЕНИЕ ВЕРСИИ И ТИПА МОДУЛЯ =================
 #define MODULE_TYPE "RX (ИНДИКАТОР)"
 #define VERSION     "1.6"
 
 // ================= НАСТРОЙКИ BLE =================
 #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
 #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
 
 static boolean doConnect = false;
 static boolean connected = false;
 static boolean doScan = false;
 static BLERemoteCharacteristic* pRemoteCharacteristic;
 static BLEAdvertisedDevice* myDevice;
 
 enum SystemState { STATE_INIT, STATE_SCANNING, STATE_CONNECTED };
 SystemState currentState = STATE_INIT;
 
 // ================= ОСНОВНЫЕ НАСТРОЙКИ =================
 #define NUM_LEDS 13   
 #define NUM_MODES 23  
 #define NUM_FADES 4   
 #define NUM_SENSITIVITIES 4 
 
 unsigned long StandByTime = 30000; 
 unsigned long nextStandByMillis;
 
 // Команды связи
 #define CMD_BRIGHTNESS       100 
 #define CMD_SENSITIVITY      101 
 #define CMD_SWITCHSIDES      102 
 #define CMD_LONGPRESS        103 
 #define CMD_CALIBRATION      104 
 #define CMD_MODE             111 
 #define CMD_GREETING         112 
 #define CMD_EEPROM_WRITE     113 
 
 // Настройки памяти NVS
 Preferences prefs;
 const byte defaultFade = 0;
 const byte defaultSensitivity = 0;
 const boolean defaultReverse = false;
 
 // Кольцевой буфер команд
 #define MAX_INDEX 8
 byte rcvCmd[MAX_INDEX];      
 byte rcvData[MAX_INDEX];     
 byte rcvIndex = 0, curIndex = 0;    
 boolean StandBy = false;            
 
 byte curFade = defaultFade; 
 byte curMode = NUM_MODES / 2; 
 byte prevMode = -1; 
 boolean revers = defaultReverse;   
 byte curSensitivity = defaultSensitivity;    
 
 uint8_t fades[NUM_FADES] = {255, 64, 32, 8}; 
 CRGB leds[NUM_LEDS];
 #ifndef USE_WROOM_BOARD
 CRGB sysLed[1]; 
 #endif
 
 // ================= МАССИВЫ ЦВЕТОВ =================
 CRGB modes[NUM_MODES][NUM_LEDS] = {
   {0xff0000, 0x640800, 0x321400,        0,        0,        0,        0,        0, 0, 0, 0, 0, 0}, 
   {0xff0000, 0x8c0000, 0x500a00, 0x3c1400, 0x200800,        0,        0,        0, 0, 0, 0, 0, 0}, 
   {0xff0000, 0x780000, 0x641000, 0x321E00, 0x180E00, 0x0F1400, 0x000400,        0, 0, 0, 0, 0, 0}, 
   {0x0a0000, 0x780000, 0x641000, 0x321E00, 0x321E00, 0x1E2D00, 0x000C00,        0, 0, 0, 0, 0, 0}, 
   {0x040000, 0x080000, 0x641000, 0x501E00, 0x463200, 0x283C00, 0x001E00,        0, 0, 0, 0, 0, 0}, 
   {0,        0x040000, 0x080000, 0x501E00, 0x5A4600, 0x3C5000, 0x003200,        0, 0, 0, 0, 0, 0}, 
   {0,        0,        0x040000, 0x501E00, 0x786400, 0x506400, 0x005000,        0, 0, 0, 0, 0, 0}, 
   {0,        0,        0,        0x321E00, 0x786400, 0x647800, 0x007800,        0, 0, 0, 0, 0, 0}, 
   {0,        0,        0,        0,        0x786400, 0x647800, 0x00A000,        0, 0, 0, 0, 0, 0}, 
   {0,        0,        0,        0,        0x2D1E00, 0x647800, 0x00C800,        0, 0, 0, 0, 0, 0}, 
   {0,        0,        0,        0,        0,        0x467800, 0x00FF00,        0, 0, 0, 0, 0, 0}, 
   {0,        0,        0,        0,        0,        0x002020, 0x00FF00, 0x002020, 0, 0, 0, 0, 0}, 
 };
 
 CRGB modeLongPressStart[NUM_LEDS] = {
   CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Indigo, CRGB::Violet,
   CRGB::Indigo, CRGB::Blue, CRGB::Green, CRGB::Yellow, CRGB::Orange, CRGB::Red
 };
 
 CRGB testColors[3] = { CRGB::Red, CRGB::Green, CRGB::Blue };
 
 // ================= ПРОТОТИПЫ ФУНКЦИЙ =================
 void showSwitchOff();
 void loadSettings();
 void saveSettings();
 
 // ================= РАБОТА С ЭНЕРГОНЕЗАВИМОЙ ПАМЯТЬЮ (NVS) =================
 void loadSettings() {
   Serial.println("Чтение настроек из памяти NVS RX...");
   prefs.begin("badinc_rx", true); 
   
   curFade = prefs.getUChar("fade", defaultFade);
   curSensitivity = prefs.getUChar("sens", defaultSensitivity);
   revers = prefs.getBool("rev", defaultReverse);
   
   prefs.end();
   Serial.printf("Загружено на RX: Яркость=%d, Чувствительность=%d, Реверс=%d\n", curFade, curSensitivity, revers);
 }
 
 void saveSettings() {
   Serial.println("Сохранение локальных копий параметров в NVS RX...");
   prefs.begin("badinc_rx", false); 
   
   prefs.putUChar("fade", curFade);
   prefs.putUChar("sens", curSensitivity);
   prefs.putBool("rev", revers);
   
   prefs.end();
 }
 
 // ================= СИСТЕМНАЯ СВЕТОДИОДНАЯ ИНДИКАЦИЯ СВЯЗИ =================
 void initSysLed() {
 #ifdef USE_WROOM_BOARD
   pinMode(PIN_SYS_LED, OUTPUT);
 #else
   FastLED.addLeds<WS2812B, PIN_SYS_LED, GRB>(sysLed, 1);
   FastLED.setBrightness(50);
 #endif
 }
 
 void updateSysLed() {
 #ifdef USE_WROOM_BOARD
   if (currentState == STATE_CONNECTED) {
     digitalWrite(PIN_SYS_LED, HIGH);
   } else {
     digitalWrite(PIN_SYS_LED, (millis() / 500) % 2); 
   }
 #else
   static SystemState lastState = (SystemState)-1;
   if (currentState != lastState) {
     if (currentState == STATE_INIT) sysLed[0] = CRGB::Green;
     else if (currentState == STATE_SCANNING) sysLed[0] = CRGB::Red;
     else if (currentState == STATE_CONNECTED) sysLed[0] = CRGB::Blue;
     FastLED.show();
     lastState = currentState;
   }
 #endif
 }
 
 // ================= КОЛЛБЭКИ BLE =================
 static void notifyCallback(
   BLERemoteCharacteristic* pBLERemoteCharacteristic,
   uint8_t* pData,
   size_t length,
   bool isNotify) {
   
   if (length == 2) {
     rcvCmd[rcvIndex] = pData[0];
     rcvData[rcvIndex] = pData[1];
     rcvIndex = (rcvIndex + 1) % MAX_INDEX;
     
     StandBy = false;
     nextStandByMillis = millis() + StandByTime;
   }
 }
 
 class MyClientCallback : public BLEClientCallbacks {
   void onConnect(BLEClient* pclient) { }
   void onDisconnect(BLEClient* pclient) {
     connected = false;
     doScan = true; 
     currentState = STATE_SCANNING;
     Serial.println("Потеряна связь с TX.");
   }
 };
 
 class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
   void onResult(BLEAdvertisedDevice advertisedDevice) {
     if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
       BLEDevice::getScan()->stop();
       myDevice = new BLEAdvertisedDevice(advertisedDevice);
       doConnect = true;
       doScan = true;
     }
   }
 };
 
 // ================= ПОДКЛЮЧЕНИЕ BLE К СЕРВЕРУ =================
 bool connectToServer() {
     Serial.print("Подключение к: ");
     Serial.println(myDevice->getAddress().toString().c_str());
     
     BLEClient* pClient  = BLEDevice::createClient();
     pClient->setClientCallbacks(new MyClientCallback());
 
     if (!pClient->connect(myDevice)) return false;
     Serial.println("Подключено к серверу!");
 
     BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
     if (pRemoteService == nullptr) {
       pClient->disconnect();
       return false;
     }
 
     pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
     if (pRemoteCharacteristic == nullptr) {
       pClient->disconnect();
       return false;
     }
 
     if(pRemoteCharacteristic->canNotify()) {
       pRemoteCharacteristic->registerForNotify(notifyCallback);
     }
 
     connected = true;
     currentState = STATE_CONNECTED;
     return true;
 }
 
 // ================= ФУНКЦИИ ИНДИКАЦИИ =================
 void initMODS() {
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
   pinMode(PIN_LEDS, OUTPUT);
   FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
   FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); 
   FastLED.setBrightness(fades[curFade]);
 }
 
 void showBrightness() {
   FastLED.setBrightness(fades[curFade]);
   prevMode = -1;
 }
 
 void showSensitivity() {
   for (byte i = 0; i < NUM_LEDS; i++) leds[i] = 0;
   leds[curSensitivity] = modeLongPressStart[curSensitivity];
   leds[NUM_LEDS - curSensitivity - 1] = modeLongPressStart[curSensitivity];
   FastLED.show();
   delay(1000);
   prevMode = -1;
 }
 
 void showSwitchSides() {
   for (byte i = 0; i < NUM_LEDS / 2; i++) {
     leds[i] = CRGB::Blue;
     leds[NUM_LEDS - i - 1] = CRGB::Red;
   }
   FastLED.show();
   delay(1000);
   for (byte i = 0; i < NUM_LEDS / 2; i++) {
     leds[i] = CRGB::Red;
     leds[NUM_LEDS - i - 1] = CRGB::Blue;
   }
   FastLED.show();
   delay(1000);
 }
 
 void showLongPressStart() {
   for (byte i = 0; i < NUM_LEDS; i++) leds[i] = modeLongPressStart[i];
   FastLED.show();
   delay(500);
 }
 
 void showCalibration() {
   for (byte i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Blue;
   FastLED.show();
   delay(500);
 }
 
 void playGreeting() {
   Serial.println("Анимация приветствия...");
   uint8_t currentBrightness = fades[curFade];
   FastLED.setBrightness(32); 
 
   for (byte j = 0; j < 3; j++) {
     for (byte i = 0; i < NUM_LEDS; i++) {
       leds[i] = testColors[j];
       FastLED.show();
       delay(40);
     }
     prevMode = -1;
     delay(100);
   }
   for(int i=0; i<NUM_LEDS; i++) leds[i] = CRGB::Black;
   FastLED.show();
   FastLED.setBrightness(currentBrightness);
 }
 
 void showSwitchOff() {
   byte centerLED = NUM_LEDS / 2;
   for (byte i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Black;
   for (byte j = 0; j < 3; j++) {
     leds[centerLED] = CRGB::Blue; FastLED.show(); delay(200);
     leds[centerLED] = CRGB::Black; FastLED.show(); delay(200);
   }
   delay(300);
 }
 
 void processEEPROMwrite() {
   Serial.println("Получена команда записи. Выполняю сохранение в NVS RX...");
   saveSettings(); 
   
   for (byte i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Black;
   FastLED.show();
   delay(200);
   for (byte j = 0; j < 2; j++) {
     leds[NUM_LEDS / 2] = CRGB::Red; FastLED.show(); delay(200);
     leds[NUM_LEDS / 2] = CRGB::Black; FastLED.show(); delay(200);
   }
   prevMode = -1;
 }
 
 // ================= ОБРАБОТКА КОМАНД ИЗ СТЕКА =================
 void processCommand() {
   unsigned long curMillis = millis();
   if (!StandBy && (curIndex == rcvIndex)) {
     if (curMillis > nextStandByMillis) {
       showSwitchOff();
       StandBy = true;
     }
     return;
   }
   while (curIndex != rcvIndex) {
     switch (rcvCmd[curIndex]) {
       case CMD_MODE:
         curMode = rcvData[curIndex];
         break;
       case CMD_BRIGHTNESS:
         curFade = rcvData[curIndex];
         showBrightness();
         break;
       case CMD_SENSITIVITY:
         curSensitivity = rcvData[curIndex];
         showSensitivity();
         break;
       case CMD_SWITCHSIDES:
         revers = rcvData[curIndex];
         showSwitchSides();
         prevMode = -1;
         break;
       case CMD_LONGPRESS:
         showLongPressStart();
         break;
       case CMD_CALIBRATION:
         showCalibration();
         prevMode = -1;
         break;
       case CMD_GREETING:
         playGreeting();
         break;
       case CMD_EEPROM_WRITE:
         processEEPROMwrite();
         break;
     }
     curIndex = (curIndex + 1) % MAX_INDEX;
     StandBy = false;
     nextStandByMillis = curMillis + StandByTime;
   }
 }
 
 // ================= ОСНОВНОЙ ЦИКЛ ПРИЕМНИКА =================
 void setup() {
   Serial.begin(115200);
   delay(2000); 
 
   Serial.println("\n========================================");
   Serial.printf("=== ПРОЕКТ BadIncESP | МОДУЛЬ: %s ===\n", MODULE_TYPE);
   Serial.printf("=== ВЕРСИЯ ПРОШИВКИ: %s                  ===\n", VERSION);
   Serial.println("========================================");
 
   initSysLed();
   currentState = STATE_INIT;
   updateSysLed();
 
   initMODS();
   loadSettings(); 
   initLEDs();
   playGreeting();
 
   BLEDevice::init("");
   BLEScan* pBLEScan = BLEDevice::getScan();
   pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
   pBLEScan->setInterval(1349);
   pBLEScan->setWindow(449);
   pBLEScan->setActiveScan(true);
   pBLEScan->start(0, nullptr, false); 
   
   currentState = STATE_SCANNING;
   nextStandByMillis = millis() + StandByTime;
   Serial.println("Сканирование эфира начато...");
 }
 
 void loop() {
   updateSysLed(); 
 
   static unsigned long lastDebug = 0;
   if (millis() - lastDebug > 2000) {
       Serial.printf("Status: connected=%d, doScan=%d, doConnect=%d\n", connected, doScan, doConnect);
       lastDebug = millis();
   }
 
   if (doConnect == true) {
     Serial.println("Попытка соединения...");
     if (connectToServer()) {
       Serial.println("Связь с TX установлена.");
     } else {
       Serial.println("Ошибка подключения к TX.");
       currentState = STATE_SCANNING;
     }
     doConnect = false;
   }
   
   if (!connected && doScan) {
     Serial.println("Возобновление сканирования...");
     BLEDevice::getScan()->start(0, nullptr, false);
     doScan = false;
   }
 
   processCommand();
   processLEDS();
 }