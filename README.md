# BadIncESP - Wireless BLE Inclinometer System

[🇷🇺 Русский](#русский) | [🇬🇧 English](#english)

---

## 🇷🇺 Русский

### Описание проекта
**BadIncESP** — это высокоточная двухмодульная беспроводная система измерения и визуализации углов наклона (инклинометр). Цель - обеспечить визуальный контроль «горизонта» при работе со стэдикамом. Может также использоваться для другой фото-видео аппаратуры и проч. Проект разработан для работы в реальном времени с минимальными задержками и состоит из 2-х модулей: передатчика (с безинерционным датчиком углов) и приемника (индикатора), связанных по протоколу Bluetooth Low Energy (BLE).
Система включает фильтр, который автоматически гасит показания крена, если устройство наклоняется вперед или назад.

### Архитектура системы
* **TX Модуль (Передатчик):** Отвечает за опрос I2C-датчика BNO080, вычисление кватернионов, фильтрацию, опрос кнопок управления и работу BLE-сервера. Отправляет данные (Notify) при каждом изменении состояния.
* **RX Модуль (Приемник):** Работает как BLE-клиент с функцией автоматического переподключения. Декодирует команды и выводит плавную анимацию на светодиодную ленту WS2812B (отклонение "капли" от центра с изменением цвета).
* **Память NVS (Preferences):** Все пользовательские настройки (яркость, чувствительность, реверс ленты) и калибровочные нули осей сохраняются в энергонезависимую память обоих устройств. Запись отложена на 15 секунд после последнего действия пользователя по настройке для защиты флэш-памяти от износа.

### Аппаратное обеспечение
| Компонент | Описание / Пин |
| :--- | :--- |
| **TX Микроконтроллер** | ESP32-S3-Zero |
| **Датчик IMU** | BNO080 (I2C: SDA - GPIO5, SCL - GPIO6) |
| **Кнопка управления** | Тактовая кнопка (GPIO9) |
| **Индикатор кнопки (TX)** | Обычный LED (GPIO10) |
| **Системный LED (TX/RX)** | Встроенный WS2812 (GPIO21) или GPIO2 (WROOM) |
| **RX Микроконтроллер** | ESP32-S3-Zero (или NodeMCU ESP32-WROOM) |
| **Основной Индикатор** | Лента WS2812B, 13 светодиодов (Пин данных: GPIO7 или GPIO13) |

### Динамическое демпфирование (Вторая ось)
Вычислитель TX имеет встроенный фильтр тангажа (наклон вперед/назад):
* **0° – 5°:** Зона нечувствительности. Показания крена точны на 100%.
* **5° – 45°:** Зона пропорционального гашения. Показания крена плавно умножаются на понижающий коэффициент от 1.0 до 0.0.
* **> 45°:** Полное гашение. Индикатор крена принудительно устанавливается в центр (0°), независимо от реального угла устройства.

### Управление и Настройка (Кнопка на TX)
Все настройки осуществляются единственной кнопкой на модуле передатчика.

| Действие | Функция | Индикация на ленте (RX) |
| :--- | :--- | :--- |
| **Короткое нажатие (Клик)** | Изменение яркости (4 уровня) | Мгновенное изменение яркости всей ленты |
| **Двойной клик** | Изменение чувствительности (4 уровня) | Симметричное отображение цвета по краям ленты |
| **Удержание (менее 3 сек)** | Установка нового "нуля" (Калибровка) | Вся лента загорается **СИНИМ** цветом |
| **Долгое удержание (> 3 сек)** | Реверс отображения (Смена сторон) | Левая и правая стороны ленты мигают красным/синим |

### Системная LED Индикация (Статус BLE)
Встроенные светодиоды на платах показывают состояние радиоканала:

| Состояние | Цвет системного LED |
| :--- | :--- |
| **Инициализация / Запуск** | Зеленый |
| **Поиск / Ожидание связи (Advertising/Scanning)** | Красный (или мигающий синий на WROOM) |
| **Соединение установлено** | Синий (непрерывный) |
| **Запись в память NVS (RX)** | Центральный пиксель ленты мигает красным 2 раза |

---

## 🇬🇧 English

### Project Overview
**BadIncESP** is a high-precision, dual-module wireless inclinometer system. Designed for real-time operation with minimal latency, it consists of a transmitter (sensor module) and a receiver (indicator module) connected via Bluetooth Low Energy (BLE).

The system not only broadcasts the primary roll angle but also features a built-in mathematical filter (dynamic damping) that automatically zeroes out the roll readings if the device is excessively pitched forward or backward.

### System Architecture
* **TX Module (Transmitter):** Handles BNO080 I2C sensor polling, quaternion math, dynamic filtering, button control logic, and acts as the BLE Server. Dispatches data via BLE Notify instantly upon state changes.
* **RX Module (Receiver):** Operates as a BLE Client with an auto-reconnect routine. Decodes incoming commands and drives smooth animations on a WS2812B LED strip (a shifting "drop" of light indicating angle and severity via color).
* **NVS Memory (Preferences):** User configurations (brightness, sensitivity, strip reverse) and calibration zeroes are saved to non-volatile memory on both devices. Writes are delayed by 15 seconds after the last user input to prevent flash wear.

### Hardware Requirements
| Component | Description / Pin Assignment |
| :--- | :--- |
| **TX Microcontroller** | ESP32-S3-Zero |
| **IMU Sensor** | BNO080 (I2C: SDA - GPIO5, SCL - GPIO6) |
| **Control Button** | Tactile Switch (GPIO9) |
| **Button Feedback (TX)** | Standard LED (GPIO10) |
| **System LED (TX/RX)** | Onboard WS2812 (GPIO21) or GPIO2 (WROOM) |
| **RX Microcontroller** | ESP32-S3-Zero (or NodeMCU ESP32-WROOM) |
| **Main Indicator** | WS2812B Strip, 13 LEDs (Data Pin: GPIO7 or GPIO13) |

### Dynamic Damping (Y-Axis Filter)
The TX processor includes a pitch (forward/backward tilt) filter:
* **0° – 5°:** Deadzone. Roll readings are 100% accurate and unaffected.
* **5° – 45°:** Proportional damping zone. The roll output is smoothly multiplied by a reduction factor from 1.0 down to 0.0.
* **> 45°:** Complete damping. The roll indicator is forced to the center (0°), regardless of the actual device roll.

### Controls & Configuration (TX Button)
All system settings are managed via a single button on the Transmitter module.

| Action | Function | RX Strip Indication |
| :--- | :--- | :--- |
| **Single Click** | Change Brightness (4 levels) | Immediate brightness update across the strip |
| **Double Click** | Change Sensitivity (4 levels) | Symmetrical color blocks light up on the strip edges |
| **Hold (< 3 sec) & Release** | Set Calibration "Zero" | The entire strip flashes **BLUE** |
| **Long Hold (> 3 sec)** | Reverse Display (Swap Left/Right) | Left and right halves alternate Red/Blue |

### System LED Status (BLE Link)
Onboard processor LEDs provide immediate feedback on the radio link status:

| State | System LED Color |
| :--- | :--- |
| **Booting / Init** | Green |
| **Scanning / Advertising (No Link)** | Red (or flashing Blue on WROOM boards) |
| **BLE Connected** | Blue (Solid) |
| **NVS Memory Write (RX)** | The center LED of the strip flashes Red twice |
