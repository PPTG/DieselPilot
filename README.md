# 🌐 Language / Język

**[🇬🇧 English](#english)** | **[🇵🇱 Polski](#polish)**

---

<a name="english"></a>
## 🇬🇧 ENGLISH VERSION

### 📖 Description

**Diesel Pilot** is a fully-featured ESP32 controller for Chinese diesel heaters communicating via 433 MHz RF. The project enables full heater control through a web browser, MQTT, and integration with Home Assistant.

⚠️ **IMPORTANT:** 
Use at your own risk!!!!

<img width="874" height="730" alt="WEB" src="https://github.com/user-attachments/assets/a3715ef1-9ef1-4257-a28f-77bb7ff2645d" />


### 🔧 Compatibility:

- I tested two controllers with 🔧 as the upper left button,one had a red remote control, the other a black one, both works.
- There is also a version of the controller with a ☀️ symbol.
- It is possible to add support as soon as I manage to buy one and map the data frames from the radio.
- However, I currently only support versions with the 🔧 symbol!!!
- You can find out more in the Wiki page.

![edited](https://github.com/user-attachments/assets/3b78064b-d00a-4f14-a39b-06667b446803)


### ✨ Features

- 🌐 **Web GUI** - elegant dark theme interface
- 📟 **OLED Display SH1106** - real-time status and IP
- 📡 **WiFi** - AP mode (default) + configurable STA mode
- 📨 **MQTT** - full Home Assistant integration
- 🔗 **Pairing** - automatic and manual
- 🎮 **Control** - POWER, UP, DOWN, MODE
- 💾 **NVS Memory** - configuration survives reset
- ☁︎ **OTA UPDATE** - Since the release of V1.2
  

Files:
- **DieselPilot.ino** - Application code

- **HomeAssistantMQTT.txt** - MQTT configuration file for HA
  
- Protocol documentation and compatibility moved to the wiki page

- **tools** - The “tools” folder contains helpful programs that allow you to determine the correct connection of the cc1101 module,
 detect the current frequency of the remote control, and tune to the required frequency. 

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

** Pairing with the stove or setting up WIFI and MQTT takes a while after clicking the button.
Wait for the pop-up window to appear confirming the operation.
This is due to the need to save this data to memory :)

#### Automatic Pairing

1. Press **AUTO PAIR** in GUI
2. ESP32 listens for 60 seconds
3. **Press and hold pairing button on heater panel** (usually ~5-10 seconds)
   - Heater enters discovery mode
   - Sends STATUS frame with address
4. ESP32 catches address and saves in NVS memory
5. Done - heater paired!

- Video showing the pairing process: https://youtu.be/xmEbU_qbN60

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

~~**0. Reading errors ❌**~~ ✅ 

- ~~Mapping error code to message~~ ✅
- ~~Forcing/scanning possible controller errors~~ ✅
- ~~Adding error field in GUI~~ ✅
- ~~Adding error field in MQTT~~ ✅


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

**3. Support for controller version ☀️**
```
- Driver version detection
- Pairing mode adjustment
- Data frame mapping
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

<img width="874" height="730" alt="WEB" src="https://github.com/user-attachments/assets/a3715ef1-9ef1-4257-a28f-77bb7ff2645d" />

### 🔧 Kompatybilność:

- Przetestowałem dwa kontrolery zawierające 🔧 jako górny lewy przycisk, jeden miał pilot czerwony drugi czarny oba działają.
- Jest jeszcze wersja sterownika z symbolem ☀️ jest możliwe dodanie wsparcia jak tylko uda mi się taki kupić i zmapować ramki danych  z radia,
- Natomiast aktualnie wspieram tylko wersje z symbolem 🔧!!!
- Możesz dowiedzieć się więcej w zakładce Wiki

![edited](https://github.com/user-attachments/assets/b8a33a3f-0c65-4451-8a56-ab2ca61467db)

### ✨ Funkcje

- 🌐 **Web GUI** - elegancki interfejs w ciemnym motywie
- 📟 **Wyświetlacz OLED SH1106** - status i IP w czasie rzeczywistym
- 📡 **WiFi** - tryb AP (domyślnie) + konfigurowalny tryb STA
- 📨 **MQTT** - pełna integracja z Home Assistant
- 🔗 **Parowanie** - automatyczne i ręczne
- 🎮 **Sterowanie** - POWER, UP, DOWN, MODE
- 💾 **Pamięć NVS** - konfiguracja przetrwa reset
- ☁︎ **OTA UPDATE** - Od wydania V1.2

Pliki:
- **DieselPilot.ino** - Kod aplikacji

- **HomeAsistantMQTT.txt** - Plik konfiguracji MQTT dla HA

- **Dokumentacja protokołu oraz kompatybilność przeniesiona do strony wiki**

- **tools** -  Folder "tools" zawiera pomocne programy pozwalające ustalić poprawne podłączenie modułu cc1101 oraz wykryć aktualną częstotliwość pilota i dostroić się do wymaganej częstotliwości. 

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

** Parowanie z piecykiem czy ustawianie WIFI, MQTT trochę trwa po kliknięciu przycisku poczekaj na wyskakujący popup który potwierdzi operację.
Jest to związane z potrzebą zapisu tych danych do pamięci :)  



#### Automatyczne parowanie

1. Wciśnij **AUTO PAIR** w GUI
2. ESP32 nasłuchuje przez 60 sekund
3. **Przytrzymaj przycisk parowania na panelu ogrzewacza** (zwykle ~5-10 sekund)
   - Ogrzewacz wejdzie w tryb discovery
   - Wyśle ramkę STATUS z adresem
4. ESP32 wyłapie adres i zapisze w pamięci NVS
5. Gotowe - ogrzewacz sparowany!
   
- Wideo pokazujące proces parowania: https://youtu.be/xmEbU_qbN60

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

**~~0. Odczytywanie błędów ❌~~**✅

- ~~Mapowanie kodu błędu do komunikatu~~✅
- ~~Wymuszenie/zeskanowanie możliwych błędów sterownika~~✅
- ~~Dodanie pola błędu w GUI~~✅
- ~~Dodanie pola błędu w MQTT~~✅


**1. Czujnik poziomu paliwa ⛽**
```
- Odczyt analogowy z czujnika paliwa
- Monitoring poziomu w czasie rzeczywistym
- Alerty MQTT gdy paliwo < 20%
- Szacowanie czasu pracy do wyczerpania
- Integracja z HA (fuel level sensor)
```

**2. Symulator sterownika ogrzewacza 🎭**
```
- Symulator ogrzewacza do testowania pilotów
- Odpowiada jak prawdziwy heater
- Testowanie reverse engineering
- Bez potrzeby prawdziwego urządzenia
- Wkrótce w repo!
```

**3. Wsparcie dla kontrolera w wersji ☀️**
```
- Detekcja wersji sterownika
- Dostosowanie trybu parowania
- Zmapowanie ramek danych
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

<img width="874" height="730" alt="WEB" src="https://github.com/user-attachments/assets/a4e0e552-14da-4d78-8f53-31c4da614f80" />
<img width="300" height="200" alt="Main" src="https://github.com/user-attachments/assets/60ad6659-44c4-4aa2-b6ba-24122151368d" />
<img width="300" height="200" alt="OTA" src="https://github.com/user-attachments/assets/144348e6-a0b6-4a15-8c61-f9c249082756" />
<img width="300" height="200" alt="Auto" src="https://github.com/user-attachments/assets/b692eaf9-2e64-407b-a2cd-5df985432718" />

<img width="643" height="944" alt="Zrzut ekranu 2026-01-05 151325" src="https://github.com/user-attachments/assets/e2bd8273-1ace-4bec-9c46-a75536e3ab33" />

![IMG_20260104_011052](https://github.com/user-attachments/assets/754c2dc5-4aaf-4fa1-8733-226128dfb8b9)

<img width="2574" height="3227" alt="Device" src="https://github.com/user-attachments/assets/eea2903f-88ae-41e3-b676-d00306fc08db" />

<img width="1646" height="809" alt="Zrzut ekranu 2026-01-03 223648" src="https://github.com/user-attachments/assets/e891a126-e5c3-4028-9862-442967b66352" />

