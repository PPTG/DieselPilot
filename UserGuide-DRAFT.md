# 📘 DieselPilot — User Guide (DRAFT)

> **Draft do wklejenia na Wiki (zakładka „DieselPilot"). / Draft for the Wiki ("DieselPilot" tab).**
> Miejsca na screeny / screenshot slots: `![…](TODO: …)`. Lista zrzutów na końcu /
> checklist of screenshots at the bottom.

**[🇬🇧 English](#-english)** | **[🇵🇱 Polski](#-polski)**

---

<a name="-english"></a>
## 🇬🇧 English

Quick, practical guide: flashing, first run, pairing, control, MQTT and updates.
Assumes basic familiarity (you can open VS Code and run a Windows program).

⚠️ **Use at your own risk** — 433 MHz radio and a fuel-burning heater.

### What you need

- **ESP32** board + **CC1101** (433 MHz) + **OLED SH1106** (optional)
- USB cable, a compatible diesel heater (see the **Compatibility** wiki tab)
- For the first flash: **[VS Code](https://code.visualstudio.com/) + [PlatformIO](https://platformio.org/)**
- For later updates: **[DieselPilotTool](https://github.com/PPTG/DieselPilotTool)** (Windows app — no PlatformIO needed)

### Wiring

| CC1101 | ESP32 | | OLED | ESP32 |
|--------|-------|---|------|-------|
| GDO2 | GPIO4 | | SDA | GPIO21 |
| SCK | GPIO18 | | SCL | GPIO22 |
| MISO | GPIO19 | | VCC | 3.3V |
| MOSI | GPIO23 | | GND | GND |
| CSn | GPIO5 | | | |
| VCC | **3.3V** | | | |
| GND | GND | | | |

> ⚠️ Power the CC1101 from **3.3V only** — 5V will damage it.

![Wiring](TODO: photo/diagram ESP32 + CC1101 + OLED)

### 1. Flash the firmware (first time)

1. Open the project folder in **VS Code** (with the **PlatformIO** extension).
2. Connect the ESP32 over USB.
3. In the PlatformIO toolbar click **Upload** (firmware), then **Upload Filesystem Image** (the Web GUI).

That's it — the device reboots and starts.

![PlatformIO](TODO: screen of PlatformIO Upload / Upload Filesystem Image)

### 2. First run & connect

The ESP32 starts as a WiFi access point:

1. Connect to WiFi **`Diesel-Pilot`** (password **`12345678`**).
2. Open **http://192.168.4.1** in a browser.

![Web GUI](TODO: screen of the main Web GUI)

### 3. The Web GUI

Tabs: **Status/Control** (state + POWER/UP/DOWN/MODE), **Pairing**, **WiFi**,
**MQTT**, **⬆ OTA**, **System** (reboot / factory reset).

### 4. Pair the heater

In **Pairing**, pick your protocol version (**V1 / V2**), then:

1. Click **AUTO PAIR** — the device listens for 60 s.
2. **Press and hold the pairing button on the heater panel** until it's caught.
3. Wait for the confirmation popup. Done — the address is saved.

Don't know your version, or want manual pairing? See the **ForNerds** wiki tab.
🎥 Pairing video: https://youtu.be/xmEbU_qbN60

![Pairing](TODO: screen of the Pairing tab + confirmation popup)

### 5. Control

In **Status/Control**: **POWER** (on/off), **UP/DOWN** (power or target temp),
**MODE** (AUTO ↔ MANUAL). Live readouts: state, voltage, temperatures, pump, error.

### 6. Home WiFi (optional)

In **WiFi**: set a device name, your home SSID + password, save. The device reboots
and joins your network (new IP shown on the OLED). If it fails, it falls back to the AP.

### 7. MQTT & Home Assistant (optional)

In **MQTT**: broker address + port (default `1883`), base topic (default `diesel`),
optional user/password. A ready `configuration.yaml` example is in the repo:
**`MQTT-Example/HomeAsistantMQTT.txt`**.

![MQTT in HA](TODO: screen of DieselPilot entities in Home Assistant)

### 8. Updating the firmware

Easiest way — the **[DieselPilotTool](https://github.com/PPTG/DieselPilotTool)** Windows app:

- **OTA (WiFi):** enable OTA in the **⬆ OTA** tab, then in the tool pick the device
  (or enter its IP), select `firmware.bin` / `littlefs.bin`, enter the OTA password and flash.
- **USB:** plug in the ESP32, pick the COM port, select the `.bin` files and flash.

> The tool handles partition offsets for you. The very first flash on a blank chip must be
> done with PlatformIO; the tool only updates firmware/filesystem afterwards.

![DieselPilotTool](TODO: screen of DieselPilotTool, OTA and USB tabs)

### Error codes

`UNDERVOLTAGE` · `OVERVOLTAGE` · `SPARK PLUG` · `OIL PUMP` · `OVERHEAT` · `MOTOR` ·
`DISCONNECT` · `EXTINGUISHED` · `SENSOR` · `IGNITION` · `STANDBY`. Shown in the GUI, on
the OLED and published to `diesel/error` over MQTT.

### Troubleshooting

- **No comms with heater:** check pairing, CC1101 on **3.3V**, heater supports the OLED remote, frequency (433.937 MHz).
- **OLED blank:** check I2C address `0x3C` and SDA/SCL.
- **Device not found in DieselPilotTool (OTA):** same network + firewall allows UDP 5353; or add the IP manually.
- **Start over:** **System → Factory reset**.

---

<a name="-polski"></a>
## 🇵🇱 Polski

Szybki, praktyczny przewodnik: wgranie, pierwsze uruchomienie, parowanie, sterowanie,
MQTT i aktualizacje. Zakładamy podstawową wiedzę (umiesz odpalić VS Code i program na Windows).

⚠️ **Używasz na własne ryzyko** — radio 433 MHz i urządzenie spalające paliwo.

### Co będzie potrzebne

- Płytka **ESP32** + **CC1101** (433 MHz) + **OLED SH1106** (opcjonalnie)
- Kabel USB, kompatybilna nagrzewnica (patrz zakładka **Kompatybilność** na Wiki)
- Do pierwszego wgrania: **[VS Code](https://code.visualstudio.com/) + [PlatformIO](https://platformio.org/)**
- Do późniejszych aktualizacji: **[DieselPilotTool](https://github.com/PPTG/DieselPilotTool)** (program na Windows — bez PlatformIO)

### Podłączenie

| CC1101 | ESP32 | | OLED | ESP32 |
|--------|-------|---|------|-------|
| GDO2 | GPIO4 | | SDA | GPIO21 |
| SCK | GPIO18 | | SCL | GPIO22 |
| MISO | GPIO19 | | VCC | 3.3V |
| MOSI | GPIO23 | | GND | GND |
| CSn | GPIO5 | | | |
| VCC | **3.3V** | | | |
| GND | GND | | | |

> ⚠️ CC1101 zasilaj **wyłącznie 3.3V** — 5V uszkodzi moduł.

![Podłączenie](TODO: zdjęcie/schemat ESP32 + CC1101 + OLED)

### 1. Wgranie firmware (pierwszy raz)

1. Otwórz folder projektu w **VS Code** (z rozszerzeniem **PlatformIO**).
2. Podłącz ESP32 przez USB.
3. Na pasku PlatformIO kliknij **Upload** (firmware), a potem **Upload Filesystem Image** (Web GUI).

I tyle — urządzenie zrestartuje się i wystartuje.

![PlatformIO](TODO: screen PlatformIO Upload / Upload Filesystem Image)

### 2. Pierwsze uruchomienie i połączenie

ESP32 startuje jako punkt dostępowy WiFi:

1. Połącz się z siecią **`Diesel-Pilot`** (hasło **`12345678`**).
2. Otwórz w przeglądarce **http://192.168.4.1**.

![Web GUI](TODO: screen głównego Web GUI)

### 3. Web GUI

Zakładki: **Status/Control** (stan + POWER/UP/DOWN/MODE), **Pairing**, **WiFi**,
**MQTT**, **⬆ OTA**, **System** (restart / reset fabryczny).

### 4. Parowanie z nagrzewnicą

W **Pairing** wybierz wersję protokołu (**V1 / V2**), potem:

1. Kliknij **AUTO PAIR** — urządzenie nasłuchuje przez 60 s.
2. **Przytrzymaj przycisk parowania na panelu nagrzewnicy**, aż adres zostanie złapany.
3. Poczekaj na popup z potwierdzeniem. Gotowe — adres zapisany.

Nie wiesz, którą masz wersję, albo chcesz parować ręcznie? Zajrzyj do zakładki **ForNerds** na Wiki.
🎥 Wideo z parowania: https://youtu.be/xmEbU_qbN60

![Parowanie](TODO: screen zakładki Pairing + popup potwierdzenia)

### 5. Sterowanie

W **Status/Control**: **POWER** (wł/wył), **UP/DOWN** (moc lub temp. zadana),
**MODE** (AUTO ↔ MANUAL). Na żywo: stan, napięcie, temperatury, pompa, błąd.

### 6. Domowe WiFi (opcjonalnie)

W **WiFi**: ustaw nazwę urządzenia, SSID i hasło domowej sieci, zapisz. Urządzenie
zrestartuje się i połączy z siecią (nowe IP na OLED). Jeśli się nie uda — wróci do AP.

### 7. MQTT i Home Assistant (opcjonalnie)

W **MQTT**: adres brokera + port (domyślnie `1883`), temat bazowy (domyślnie `diesel`),
opcjonalnie user/hasło. Gotowy przykład do `configuration.yaml` jest w repo:
**`MQTT-Example/HomeAsistantMQTT.txt`**.

![MQTT w HA](TODO: screen encji DieselPilot w Home Assistant)

### 8. Aktualizacja firmware

Najprościej — program na Windows **[DieselPilotTool](https://github.com/PPTG/DieselPilotTool)**:

- **OTA (WiFi):** włącz OTA w zakładce **⬆ OTA**, w narzędziu wybierz urządzenie
  (lub podaj IP), wskaż `firmware.bin` / `littlefs.bin`, podaj hasło OTA i flashuj.
- **USB:** podłącz ESP32, wybierz port COM, wskaż pliki `.bin` i flashuj.

> Narzędzie samo pilnuje offsetów partycji. Pierwsze wgranie na czysty układ rób
> PlatformIO; narzędzie aktualizuje tylko firmware/filesystem później.

![DieselPilotTool](TODO: screen DieselPilotTool, zakładki OTA i USB)

### Kody błędów

`UNDERVOLTAGE` · `OVERVOLTAGE` · `SPARK PLUG` (świeca) · `OIL PUMP` (pompa) · `OVERHEAT`
(przegrzanie) · `MOTOR` · `DISCONNECT` · `EXTINGUISHED` (zgaśnięcie) · `SENSOR` ·
`IGNITION` (zapłon) · `STANDBY`. Widoczne w GUI, na OLED i na temacie `diesel/error`.

### Rozwiązywanie problemów

- **Brak komunikacji:** sprawdź sparowanie, CC1101 na **3.3V**, czy nagrzewnica ma pilot OLED, częstotliwość (433.937 MHz).
- **OLED nie działa:** adres I2C `0x3C` i połączenia SDA/SCL.
- **Brak urządzenia w DieselPilotTool (OTA):** ta sama sieć + zapora przepuszcza UDP 5353; albo dodaj IP ręcznie.
- **Reset do zera:** **System → Factory reset**.

---

## Screeny do zrobienia / Screenshots to capture

- [ ] Podłączenie / Wiring (ESP32 + CC1101 + OLED)
- [ ] PlatformIO — Upload / Upload Filesystem Image
- [ ] Web GUI — ekran główny / main screen
- [ ] Zakładka Pairing + popup / Pairing tab + popup
- [ ] Status/Control (aktywna nagrzewnica / heater running)
- [ ] Zakładka MQTT + encje w HA / MQTT tab + HA entities
- [ ] Zakładka ⬆ OTA
- [ ] DieselPilotTool — OTA i USB
