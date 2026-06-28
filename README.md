# 🌐 Language / Język

**[🇬🇧 English](#english)** | **[🇵🇱 Polski](#polish)**

---

<a name="english"></a>
## 🇬🇧 ENGLISH VERSION

### 📖 Description

**Diesel Pilot — Free** is a fully-featured, **open-source (no cloud)** ESP32 controller
for Chinese diesel heaters communicating via 433 MHz RF. It gives you full control of the
heater through a web browser, MQTT and Home Assistant. Remote access is handled **locally**:
Web GUI, MQTT and on-network **OTA** firmware updates.

> 🛠️ **This edition is built with [PlatformIO](https://platformio.org/)** (the project
> migrated away from the Arduino IDE). See [Build & Upload](#-build--upload-platformio) below.

⚠️ **IMPORTANT:** 
Use at your own risk!!!!

<img width="874" height="730" alt="WEB" src="https://github.com/user-attachments/assets/a3715ef1-9ef1-4257-a28f-77bb7ff2645d" />


### 🔧 Compatibility:

- I tested two controllers with 🔧 as the upper left button, one had a red remote control and the other a black one, both work.
- There is also a version of the controller with the symbol ☀️/⚙️. Versions with the same display as the 🔧 version should work right away.
- The ☀️/⚙️ version with an older display type without a menu and with the option to adjust the power levels from H1 to H6 does work(as of version 1.3) with this project.
- You can find out more in the Wiki tab.

| Version 1.0 | Version 1.5 | Version 2.0 |
|-----------|-------------|-------------|
| <img src="https://github.com/user-attachments/assets/b8a03480-bc09-4f54-aad0-e6de703ac34e" width="200" /> | <img src="https://github.com/user-attachments/assets/c592318d-d72a-4915-9d4b-620a8a11268a" width="200" />  | <img src="https://github.com/user-attachments/assets/5ea85b0c-e52e-4975-95cd-75102e8717f1" width="200" /> |

![edited](https://github.com/user-attachments/assets/3b78064b-d00a-4f14-a39b-06667b446803)


### ✨ Features

- 🌐 **Web GUI** - elegant dark theme interface (served from LittleFS)
- 📟 **OLED Display SH1106** - real-time status and IP
- 📡 **WiFi** - AP mode (default) + configurable STA mode
- 📨 **MQTT** - full Home Assistant integration
- 🔗 **Pairing** - automatic and manual (V1 & V2 protocols)
- 🎮 **Control** - POWER, UP, DOWN, MODE
- ❌ **Error code decoding** - BYTE[7] mapping + error history
- 💾 **NVS Memory** - configuration survives reset
- ☁︎ **OTA UPDATE** - on-network firmware updates (ArduinoOTA / espota, password protected)


### 📂 Project structure

```
platformio.ini      configuration (board, partitions, libraries)
src/                firmware (.ino)
data/               LittleFS files (index.html – Web GUI)
MQTT-Example/       Home Assistant MQTT configuration example
```

> Protocol documentation, compatibility details and **manual pairing (ForNerds)** live on the
> project **Wiki**. The Wiki also hosts the helper **tools** (CC1101 wiring check, frequency
> detection / tuning).

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

### 🚀 Build & Upload (PlatformIO)

Requirements: **[PlatformIO](https://platformio.org/)** (VS Code extension or `pip install platformio`).
Libraries (`U8g2`, `PubSubClient`) are **downloaded automatically** by PlatformIO; OTA uses the
ESP32 core's built-in `ArduinoOTA`, and the CC1101 is driven by custom SPI functions (no external lib).

```bash
# 1) Firmware (over USB)
pio run -t upload

# 2) LittleFS partition (contents of data/, incl. index.html)
pio run -t uploadfs

# Serial monitor
pio device monitor
```

> After every change to `data/index.html` you must run `uploadfs` again.
> `upload` flashes only the firmware and does **not** overwrite the filesystem.

**Partitions:** the built-in `default.csv` table provides two app partitions
(`app0`/`app1`, required for OTA) + ~1.5 MB filesystem (LittleFS), so both OTA and
serving the Web GUI from LittleFS work.

#### OTA firmware updates (over WiFi)

1. In the Web GUI open the **⬆ OTA** tab, enable OTA and (optionally) set a password — the device reboots.
2. Flash firmware over the network:

```bash
pio run -t upload --upload-port <device-ip>
# with a password:
pio run -t upload --upload-port <device-ip> --upload-flags --auth=<password>
```

You can also permanently set `upload_protocol = espota` / `upload_port` / `upload_flags`
in `platformio.ini` (commented-out section at the bottom of the file).

### ⬆️ Updating without PlatformIO — DieselPilotTool

For quick flashing of ready-made `firmware.bin` + `littlefs.bin` (over OTA **or** USB,
**without PlatformIO**) there is a separate desktop tool:
**[DieselPilotTool](https://github.com/PPTG/DieselPilotTool)**.

> ℹ️ [DieselPilotTool](https://github.com/PPTG/DieselPilotTool) is a **separate repository
> with its own development and license (GNU GPL v3)**. The DieselPilot firmware itself stays MIT.

- **Shared `.bin` selection** (firmware + filesystem) for both modes.
- **📡 OTA (WiFi)** — automatic device discovery via mDNS, manual IP, OTA password,
  separate flashing of firmware / filesystem (espota).
- **🔌 USB (Serial)** — COM port selection (auto-list + manual entry), baud, editable
  offsets, flashing firmware / filesystem / both at once (esptool, bundled into the exe).
- Progress bar + live log.

**Offsets** (matching the `default.csv` table):

| Image        | Offset     |
|--------------|------------|
| firmware.bin | `0x10000`  |
| littlefs.bin | `0x290000` |

> **Note:** the very first flash onto a blank chip (bootloader + partition table) must be
> done with **PlatformIO**. DieselPilotTool only updates the firmware/filesystem partitions
> (offset ≥ `0x10000`), never the bootloader (`0x0`).

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
Read: ForNerds.md (Wiki)


**No communication with heater:**
- Verify frequency (433.937 MHz)
- Check if heater is paired
- Make sure heater supports OLED remote 
- Check CC1101 power voltage (must be 3.3V!)

**OLED not working:**
- Check I2C address (default 0x3C)
- Verify SDA/SCL connections

### 📜 License

**MIT License** - use as you wish, at your own risk! See [LICENSE](LICENSE).
Third-party libraries keep their own licenses (U8g2 BSD-2, PubSubClient MIT,
ESP32 core / ArduinoOTA LGPL 2.1).

> The separate **DieselPilotTool** is licensed under **GNU GPL v3** (its `esptool` is bundled
> into the exe). It is an independent project — the firmware here remains MIT.

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
- PlatformIO, Python (PyCharm) - development


<a name="polish"></a>
## 🇵🇱 WERSJA POLSKA

### 📖 Opis

**Diesel Pilot — Free** to pełnoprawny, **otwartoźródłowy (bez chmury)** kontroler ESP32 dla
chińskich ogrzewaczy diesla komunikujących się przez RF 433 MHz. Umożliwia pełną kontrolę
ogrzewacza przez przeglądarkę, MQTT oraz integrację z Home Assistant. Zdalny dostęp realizowany
jest **lokalnie**: Web GUI, MQTT oraz aktualizacje firmware przez **OTA** w sieci.

> 🛠️ **Ta wersja budowana jest w [PlatformIO](https://platformio.org/)** (projekt przeszedł
> z Arduino IDE). Zobacz [Budowanie i wgrywanie](#-budowanie-i-wgrywanie-platformio) poniżej.

⚠️ **WAŻNE:** 
Używasz na własne ryzyko !!!!

<img width="874" height="730" alt="WEB" src="https://github.com/user-attachments/assets/a3715ef1-9ef1-4257-a28f-77bb7ff2645d" />

### 🔧 Kompatybilność:

- Przetestowałem 2 kontrolery zawierające 🔧 jako górny lewy przycisk, jeden miał pilot czerwony drugi czarny oba działają.
- Jest jeszcze wersja sterownika z symbolem ☀️/⚙️ wersje posiadające ten sam wyświetlacz co wersja 🔧 powinny działać od razu.
- Wersja ☀️/⚙️ posiadająca wyświetlacz starszego typu bez menu i z opcją regulacji stopni mocy od H1 do H6 działa(od wersji 1.3) z tym projektem.
- Możesz dowiedzieć się więcej w zakładce Wiki

| Wersja 1.0 | Wersja 1.5 | Wersja 2.0 |
|-----------|-------------|-------------|
| <img src="https://github.com/user-attachments/assets/b8a03480-bc09-4f54-aad0-e6de703ac34e" width="200" /> | <img src="https://github.com/user-attachments/assets/c592318d-d72a-4915-9d4b-620a8a11268a" width="200" />  | <img src="https://github.com/user-attachments/assets/5ea85b0c-e52e-4975-95cd-75102e8717f1" width="200" /> |

![edited](https://github.com/user-attachments/assets/b8a33a3f-0c65-4451-8a56-ab2ca61467db)



### ✨ Funkcje

- 🌐 **Web GUI** - elegancki interfejs w ciemnym motywie (serwowany z LittleFS)
- 📟 **Wyświetlacz OLED SH1106** - status i IP w czasie rzeczywistym
- 📡 **WiFi** - tryb AP (domyślnie) + konfigurowalny tryb STA
- 📨 **MQTT** - pełna integracja z Home Assistant
- 🔗 **Parowanie** - automatyczne i ręczne (protokoły V1 i V2)
- 🎮 **Sterowanie** - POWER, UP, DOWN, MODE
- ❌ **Dekodowanie kodów błędów** - mapowanie BYTE[7] + historia błędów
- 💾 **Pamięć NVS** - konfiguracja przetrwa reset
- ☁︎ **OTA UPDATE** - aktualizacje firmware w sieci (ArduinoOTA / espota, chronione hasłem)

### 📂 Struktura projektu

```
platformio.ini      konfiguracja (board, partycje, biblioteki)
src/                firmware (.ino)
data/               pliki LittleFS (index.html – Web GUI)
MQTT-Example/       przykładowa konfiguracja MQTT dla Home Assistant
```

> Dokumentacja protokołu, szczegóły kompatybilności oraz **parowanie ręczne (ForNerds)**
> znajdują się na **Wiki** projektu. Na Wiki są też pomocne **narzędzia** (sprawdzenie
> podłączenia CC1101, wykrywanie / dostrajanie częstotliwości).

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

### 🚀 Budowanie i wgrywanie (PlatformIO)

Wymagania: **[PlatformIO](https://platformio.org/)** (rozszerzenie VS Code lub `pip install platformio`).
Biblioteki (`U8g2`, `PubSubClient`) **pobierane są automatycznie** przez PlatformIO; OTA korzysta
z wbudowanego w rdzeń ESP32 `ArduinoOTA`, a CC1101 sterowany jest własnymi funkcjami SPI (bez
zewnętrznej biblioteki).

```bash
# 1) Firmware (przez USB)
pio run -t upload

# 2) Partycja LittleFS (zawartość katalogu data/, m.in. index.html)
pio run -t uploadfs

# Podgląd portu szeregowego
pio device monitor
```

> Po każdej zmianie `data/index.html` trzeba ponownie wykonać `uploadfs`.
> `upload` wgrywa tylko firmware i **nie** nadpisuje filesystemu.

**Partycje:** wbudowana tablica `default.csv` daje dwie partycje app
(`app0`/`app1`, wymagane przez OTA) + ok. 1.5 MB filesystem (LittleFS), dzięki czemu
działa zarówno OTA, jak i serwowanie Web GUI z LittleFS.

#### Aktualizacje OTA (przez WiFi)

1. W Web GUI otwórz zakładkę **⬆ OTA**, włącz OTA i (opcjonalnie) ustaw hasło — urządzenie się zrestartuje.
2. Wgraj firmware przez sieć:

```bash
pio run -t upload --upload-port <ip-urzadzenia>
# z hasłem:
pio run -t upload --upload-port <ip-urzadzenia> --upload-flags --auth=<haslo>
```

Można też na stałe ustawić `upload_protocol = espota` / `upload_port` / `upload_flags`
w `platformio.ini` (zakomentowana sekcja na dole pliku).

### ⬆️ Aktualizacja bez PlatformIO — DieselPilotTool

Do szybkiego wgrania gotowych `firmware.bin` + `littlefs.bin` (przez OTA **lub** USB,
**bez PlatformIO**) służy osobne narzędzie desktopowe:
**[DieselPilotTool](https://github.com/PPTG/DieselPilotTool)**.

> ℹ️ [DieselPilotTool](https://github.com/PPTG/DieselPilotTool) to **osobne repozytorium
> z własnym rozwojem i licencją (GNU GPL v3)**. Samo firmware DieselPilot pozostaje na licencji MIT.

- **Wspólny wybór plików `.bin`** (firmware + filesystem) dla obu trybów.
- **📡 OTA (WiFi)** — automatyczne wykrywanie urządzeń przez mDNS, ręczne IP, hasło OTA,
  osobne wgrywanie firmware / filesystemu (espota).
- **🔌 USB (Serial)** — wybór portu COM (auto-lista + wpis ręczny), baud, edytowalne
  offsety, wgrywanie firmware / filesystemu / obu naraz (esptool, wbudowany w exe).
- Pasek postępu + log na żywo.

**Offsety** (zgodne z tablicą `default.csv`):

| Obraz        | Offset     |
|--------------|------------|
| firmware.bin | `0x10000`  |
| littlefs.bin | `0x290000` |

> **Uwaga:** pierwsze wgranie na czysty układ (bootloader + tablica partycji) wykonaj
> **PlatformIO**. DieselPilotTool aktualizuje wyłącznie partycje firmware/filesystem
> (offset ≥ `0x10000`), nigdy bootloadera (`0x0`).

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

Przeczytaj: ForNerds.md (Wiki)


**Brak komunikacji z ogrzewaczem:**
- Zweryfikuj częstotliwość (433.937 MHz)
- Sprawdź czy ogrzewacz jest sparowany
- Upewnij się że ogrzewacz wspiera pilot OLED
- Sprawdź napięcie zasilania CC1101 (musi być 3.3V!)

**OLED nie działa:**
- Sprawdź adres I2C (domyślnie 0x3C)
- Zweryfikuj połączenia SDA/SCL

### 📜 Licencja

**MIT License** - użyj jak chcesz, na własną odpowiedzialność! Zobacz [LICENSE](LICENSE).
Biblioteki third-party na własnych licencjach (U8g2 BSD-2, PubSubClient MIT,
rdzeń ESP32 / ArduinoOTA LGPL 2.1).

> Osobne **DieselPilotTool** jest na licencji **GNU GPL v3** (wbudowany `esptool`).
> To niezależny projekt — firmware tutaj pozostaje na MIT.

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
- PlatformIO, Python (PyCharm) - development

### 📸 Zdjęcia i materiały

<img width="874" height="730" alt="WEB" src="https://github.com/user-attachments/assets/a4e0e552-14da-4d78-8f53-31c4da614f80" />
<img width="300" height="200" alt="Main" src="https://github.com/user-attachments/assets/60ad6659-44c4-4aa2-b6ba-24122151368d" />
<img width="300" height="200" alt="OTA" src="https://github.com/user-attachments/assets/144348e6-a0b6-4a15-8c61-f9c249082756" />
<img width="300" height="200" alt="Auto" src="https://github.com/user-attachments/assets/b692eaf9-2e64-407b-a2cd-5df985432718" />

<img width="643" height="944" alt="Zrzut ekranu 2026-01-05 151325" src="https://github.com/user-attachments/assets/e2bd8273-1ace-4bec-9c46-a75536e3ab33" />

![IMG_20260104_011052](https://github.com/user-attachments/assets/754c2dc5-4aaf-4fa1-8733-226128dfb8b9)

<img width="2574" height="3227" alt="Device" src="https://github.com/user-attachments/assets/eea2903f-88ae-41e3-b676-d00306fc08db" />

<img width="1657" height="863" alt="Zrzut ekranu 2026-01-17 212157" src="https://github.com/user-attachments/assets/0b88ff20-092f-4746-a39b-671355b59cc9" />
