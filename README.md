# 🌐 Language / Język

**[🇬🇧 English](#english)** | **[🇵🇱 Polski](#polish)**

---

<a name="english"></a>
## 🇬🇧 ENGLISH VERSION

### 📖 Description

**Diesel Pilot** is a fully-featured ESP32 controller for Chinese diesel heaters communicating via 433 MHz RF. The project enables full heater control through a web browser, MQTT, and integration with Home Assistant.

⚠️ **IMPORTANT:** 
Use at your own risk!!!!

<img width="882" height="776" alt="Zrzut ekranu 2026-01-04 022007" src="https://github.com/user-attachments/assets/c2cef19e-e951-46b3-9e6f-20fe6c7df986" />

Compatibility:
I tested two controllers with red and black remote controls, both of which work correctly.
![IMG_20260103_165719](https://github.com/user-attachments/assets/43ec8d9f-6b02-4b90-9055-03c66882fab7)

### ✨ Features

- 🌐 **Web GUI** - elegant dark theme interface
- 📟 **OLED Display SH1106** - real-time status and IP
- 📡 **WiFi** - AP mode (default) + configurable STA mode
- 📨 **MQTT** - full Home Assistant integration
- 🔗 **Pairing** - automatic and manual
- 🎮 **Control** - POWER, UP, DOWN, MODE
- 💾 **NVS Memory** - configuration survives reset

Files:
- **DieselPilot.ino** - Application code

- **ForNerds.md** - Communication protocol analysis, used tools and debugging

- **HomeAssistantMQTT.txt** - MQTT configuration file for HA

### 🛠️ Required Hardware

| Component | Model | Notes |
|-----------|-------|-------|
| Microcontroller | ESP32 |
| RF Transceiver | CC1101 | 433 MHz |
| Display | SH1106 | OLED 128x64, I2C |

**CC1101 Wiring:**
```
ESP32    CC1101
-----    ------
GPIO4  - GDO2
GPIO18 - SCK
GPIO19 - MISO
GPIO23 - MOSI
GPIO5  - CSn
3.3V   - VCC
GND    - GND
```

**OLED Wiring:**
```
ESP32    SH1106
-----    ------
GPIO21 - SDA
GPIO22 - SCL
3.3V   - VCC
GND    - GND
```

### 🚀 Installation

#### 1. Arduino Libraries
```
- U8g2 (by oliver)
- PubSubClient (by Nick O'Leary)
```

#### 2. Arduino IDE Configuration
```
Board: ESP32 Dev Module
Upload Speed: 115200
Flash Frequency: 80MHz
CPU Frequency: 240MHz
```

#### 3. Upload
1. Open `DieselPilot.ino`
2. Upload to ESP32
3. Open Serial Monitor (115200 baud)


### 📱 First Run

1. ESP32 starts in **AP mode**
2. Connect to WiFi: `Diesel-Pilot` (password: `12345678`)
3. Open browser: `http://192.168.4.1`
4. Pair heater (AUTO or MANUAL)
5. (Optional) Configure home WiFi
6. (Optional) Configure MQTT


#### Automatic Pairing

1. Press **AUTO PAIR** in GUI
2. ESP32 listens for 60 seconds
3. **Press and hold pairing button on heater panel** (usually ~5-10 seconds)
   - Heater enters discovery mode
   - Sends STATUS frame with address
4. ESP32 catches address and saves in NVS memory
5. Done - heater paired!

### Manual Paring
Read: ForNerds.md


**No communication with heater:**
- Verify frequency (433.937 MHz)
- Check if heater is paired
- Make sure heater supports OLED remote 
- Check CC1101 power voltage (must be 3.3V!)

**OLED not working:**
- Check I2C address (default 0x3C)
- Verify SDA/SCL connections

### 📜 License

MIT License - use as you wish, at your own risk!

---

**CC1101 Debugging:**
- ⚠️ **IMPORTANT:** Every CC1101 module has minimal frequency deviations!
- Tested 5 different modules - all work
- Differences: ±10-30 kHz from nominal 433.92 MHz
- Use SDR# to verify actual TX frequency
- If weak reception → frequency tuning in CC1101 code

**CC1101 Module Calibration:**
```cpp
// In case of reception problems, frequency tuning:
// Default: 433.92 MHz (FREQ2=0x10, FREQ1=0xB1, FREQ0=0x3B)
// 
// Example from real test - module worked best at 433.937 MHz:
// Adjust FREQ registers to match your module's actual frequency
// Use SDR# to find signal center, then tune CC1101
// Deviations ±10-30 kHz are normal
```

**Recommended Tools:**
- ✅ rtl_433 - packet decoding
- ✅ SDR# / GQRX - spectrum visualization
- ✅ Inspectrum - IQ recording analysis
- ✅ Universal Radio Hacker - protocol RE

---

## 🚀 Project Development - What's Next?

### 🔮 Planned Features

**1. Fuel Level Sensor ⛽**
```
- Analog reading from fuel sensor
- Real-time level monitoring
- MQTT alerts when fuel < 20%
- Estimated runtime until depletion
- HA integration (fuel level sensor)
```

**2. Fake Heater Simulator 🎭**
```
- Heater simulator for testing remotes
- Responds like real heater
- Testing reverse engineering
- No need for actual device
- Coming soon to repo!
```

### 🤝 How to Help Development?

1. **Testing** - try with different heater models
2. **Bug reports** - report issues on GitHub Issues
3. **Pull requests** - share your improvements
4. **Documentation** - help translate to other languages
5. **Hardware** - test with different CC1101 modules

---

### 🙏 Acknowledgments

- **[merbanan/rtl_433](https://github.com/merbanan/rtl_433)** - THE tool for RF protocol reverse engineering! Without this project, protocol analysis would be impossible. Huge thanks for rtl_433! 📡
- **[DieselHeaterRF](https://github.com/jakkik/DieselHeaterRF)** - inspiration for parts of the protocol and CC1101 library - this is where it all started.
- **RTL-SDR community** - for accessible and affordable SDR tools (DVB-T dongles)
- **SDR#** - for excellent RF spectrum visualization software
- **Home Assistant Community** - for motivation to create MQTT integration

**Tools used in the project:**
- rtl_433 (merbanan) - RF transmission decoding
- SDR# / GQRX - spectrum analysis
- DVB-T R820T2 dongle - cheap SDR receiver
- Arduino IDE, Python (PyCharm) - development


<a name="polish"></a>
## 🇵🇱 WERSJA POLSKA

### 📖 Opis

**Diesel Pilot** to pełnoprawny kontroler ESP32 dla chińskich ogrzewaczy diesla komunikujących się przez RF 433 MHz. Projekt umożliwia pełną kontrolę ogrzewacza przez przeglądarkę internetową, MQTT oraz integrację z Home Assistant.

⚠️ **WAŻNE:** 
Używasz na własne ryzyko !!!!

<img width="882" height="776" alt="Zrzut ekranu 2026-01-04 022007" src="https://github.com/user-attachments/assets/c2cef19e-e951-46b3-9e6f-20fe6c7df986" />

Kompatybilność:

Przetestowałem 2 sterowniki z czerwonym i czarnym kolorem pilota, oba działają poprawnie.
![IMG_20260103_165719](https://github.com/user-attachments/assets/43ec8d9f-6b02-4b90-9055-03c66882fab7)

### ✨ Funkcje

- 🌐 **Web GUI** - elegancki interfejs w ciemnym motywie
- 📟 **Wyświetlacz OLED SH1106** - status i IP w czasie rzeczywistym
- 📡 **WiFi** - tryb AP (domyślnie) + konfigurowalny tryb STA
- 📨 **MQTT** - pełna integracja z Home Assistant
- 🔗 **Parowanie** - automatyczne i ręczne
- 🎮 **Sterowanie** - POWER, UP, DOWN, MODE
- 💾 **Pamięć NVS** - konfiguracja przetrwa reset

Pliki:
- **DieselPilot.ino** - Kod aplikacji

- **ForNerds.md** - Analiza protokołu komunikacji, narzędzia i debug

- **HomeAsistantMQTT.txt** - Plik konfiguracji MQTT dla HA

### 🛠️ Wymagany sprzęt

| Komponent | Model | Uwagi |
|-----------|-------|-------|
| Mikrokontroler | ESP32 
| Transceiver RF | CC1101 | 433 MHz |
| Wyświetlacz | SH1106 | OLED 128x64, I2C |

**Podłączenie CC1101:**
```
ESP32    CC1101
-----    ------
GPIO4  - GDO2
GPIO18 - SCK
GPIO19 - MISO
GPIO23 - MOSI
GPIO5  - CSn
3.3V   - VCC
GND    - GND
```

**Podłączenie OLED:**
```
ESP32    SH1106
-----    ------
GPIO21 - SDA
GPIO22 - SCL
3.3V   - VCC
GND    - GND
```

### 🚀 Instalacja

#### 1. Biblioteki Arduino
```
- U8g2 (by oliver)
- PubSubClient (by Nick O'Leary)
```

#### 2. Konfiguracja Arduino IDE
```
Board: ESP32 Dev Module
Upload Speed: 115200
Flash Frequency: 80MHz
CPU Frequency: 240mhz
```

#### 3. Upload
1. Otwórz `DieselPilot.ino`
2. Wgraj na ESP32
3. Otwórz Serial Monitor (115200 baud)


### 📱 Pierwsze uruchomienie

1. ESP32 uruchamia się w **trybie AP**
2. Podłącz się do WiFi: `Diesel-Pilot` (hasło: `12345678`)
3. Otwórz przeglądarkę: `http://192.168.4.1`
4. Sparuj ogrzewacz (AUTO lub MANUAL)
5. (Opcjonalnie) Skonfiguruj WiFi domowe
6. (Opcjonalnie) Skonfiguruj MQTT

#### Automatyczne parowanie

1. Wciśnij **AUTO PAIR** w GUI
2. ESP32 nasłuchuje przez 60 sekund
3. **Przytrzymaj przycisk parowania na panelu ogrzewacza** (zwykle ~5-10 sekund)
   - Ogrzewacz wejdzie w tryb discovery
   - Wyśle ramkę STATUS z adresem
4. ESP32 wyłapie adres i zapisze w pamięci NVS
5. Gotowe - ogrzewacz sparowany!

#### Manualne Parowanie 

Przeczytaj: ForNerds.md


**Brak komunikacji z ogrzewaczem:**
- Zweryfikuj częstotliwość (433.937 MHz)
- Sprawdź czy ogrzewacz jest sparowany
- Upewnij się że ogrzewacz wspiera pilot OLED
- Sprawdź napięcie zasilania CC1101 (musi być 3.3V!)

**OLED nie działa:**
- Sprawdź adres I2C (domyślnie 0x3C)
- Zweryfikuj połączenia SDA/SCL

### 📜 Licencja

MIT License - użyj jak chcesz, na własną odpowiedzialność!

---

**Debugowanie CC1101:**
- ⚠️ **WAŻNE:** Każdy moduł CC1101 ma minimalne odstępstwa częstotliwości!
- Testowałem 5 różnych modułów - wszystkie działają
- Różnice: ±10-30 kHz od nominalnej 433.92 MHz
- Używaj SDR# do weryfikacji rzeczywistej częstotliwości TX
- Jeśli odbiór słaby → dostrojenie freq w kodzie CC1101

**Kalibracja modułu CC1101:**
```cpp
// W razie problemów z odbiorem, dostrajanie freq:
// Domyślnie: 433.92 MHz (FREQ2=0x10, FREQ1=0xB1, FREQ0=0x3B)
// 
// Przykład z rzeczywistego testu - moduł działał najlepiej na 433.937 MHz:
// Dostosuj rejestry FREQ aby dopasować do rzeczywistej freq twojego modułu
// Użyj SDR# aby znaleźć środek sygnału, potem dostraj CC1101
// Odstępstwa ±10-30 kHz są normalne
```

**Polecane narzędzia:**
- ✅ rtl_433 - dekodowanie pakietów
- ✅ SDR# / GQRX - wizualizacja widma
- ✅ Inspectrum - analiza IQ recordings
- ✅ Universal Radio Hacker - RE protokołów

---

## 🚀 Rozwój projektu - Co dalej?

### 🔮 Planowane funkcje

**1. Czujnik poziomu paliwa ⛽**
```
- Odczyt analogowy z czujnika paliwa
- Monitoring poziomu w czasie rzeczywistym
- Alerty MQTT gdy paliwo < 20%
- Szacowanie czasu pracy do wyczerpania
- Integracja z HA (fuel level sensor)
```

**2. Fake Heater Simulator 🎭**
```
- Symulator ogrzewacza do testowania pilotów
- Odpowiada jak prawdziwy heater
- Testowanie reverse engineering
- Bez potrzeby prawdziwego urządzenia
- Wkrótce w repo!
```

### 🤝 Jak pomóc w rozwoju?

1. **Testowanie** - wypróbuj z różnymi modelami ogrzewacza
2. **Bug raporty** - zgłaszaj problemy na GitHub Issues
3. **Pull requesty** - dziel się swoimi ulepszeniami
4. **Dokumentacja** - pomóż tłumaczyć na inne języki
5. **Hardware** - testuj z różnymi CC1101 modules
---

### 🙏 Podziękowania

- **[merbanan/rtl_433](https://github.com/merbanan/rtl_433)** - To narzędzie do reverse engineeringu protokołów RF! Bez tego projektu analiza protokołu byłaby niemożliwa. Gigantyczne dzięki za rtl_433! 📡
- **[DieselHeaterRF](https://github.com/jakkik/DieselHeaterRF)** - inspiracja dla części protokołu i biblioteki CC1101 od tego projektu wszystko się zaczęło.
- **RTL-SDR community** - za dostępne i tanie narzędzia SDR (DVB-T dongles)
- **SDR#** - za świetny software do wizualizacji widma RF
- **Społeczność Home Assistant** - za motywację do stworzenia integracji MQTT

**Narzędzia wykorzystane w projekcie:**
- rtl_433 (merbanan) - dekodowanie transmisji RF
- SDR# / GQRX - analiza widma
- DVB-T R820T2 dongle - tani odbiornik SDR
- Arduino IDE, Python (PyCharm) - development

### 📸 Zdjęcia i materiały

<img width="882" height="776" alt="Zrzut ekranu 2026-01-04 022007" src="https://github.com/user-attachments/assets/c2cef19e-e951-46b3-9e6f-20fe6c7df986" />

<img width="707" height="933" alt="image" src="https://github.com/user-attachments/assets/3e8de1fa-c899-4364-b561-8d86b8930d4d" />

![IMG_20260104_011052](https://github.com/user-attachments/assets/754c2dc5-4aaf-4fa1-8733-226128dfb8b9)

![IMG_20260104_011710_1](https://github.com/user-attachments/assets/a967f4d3-b575-4444-8526-6d1ad2ea6515)

<img width="1646" height="809" alt="Zrzut ekranu 2026-01-03 223648" src="https://github.com/user-attachments/assets/e891a126-e5c3-4028-9862-442967b66352" />

