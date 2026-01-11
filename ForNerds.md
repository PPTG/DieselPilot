# 🌐 Language / Język

**[🇬🇧 English](#english)** | **[🇵🇱 Polski](#polish)**

---

<a name="english"></a>
## 🇬🇧 ENGLISH VERSION

### 📡 RF Protocol

### ⚠️ The protocol documentation is being created, may be inaccurate, and may contain errors! Updates coming soon :) ### 

#### Parameters
- **Frequency:** 433.937 MHz
- **Modulation:** 2-FSK
- **Encoding:** FSK_PCM (Frequency Shift Keying)
- **Sync Word:** `0x7E3C7E3C`
- **CRC:** CRC-16/MODBUS

#### STATUS Packet Structure (23 bytes)
```
[0]     = 0x17 (length)
[1]     = 0x00 (type: STATUS)
[2-5]   = Heater address (32-bit)
[6]     = State (0=OFF, 5=RUNNING, etc.)
[7]     = Power (%)
[8-9]   = Voltage (big-endian, /10 = V)
[10]    = Ambient temperature (°C, signed)
[11]    = Error (0x00 = OK)
[12]    = Heat exchanger temperature (°C)
[13]    = Target temperature (°C, signed)
[14]    = Mode (0x32=AUTO, 0xCD=MANUAL)
[15]    = Pump frequency (/10 = Hz)
[16-20] = Internal data
[21-22] = CRC-16/MODBUS
```

#### COMMAND Packet Structure (9 bytes)
```
[0]     = 0x09 (length)
[1]     = Command
[2-5]   = Heater address
[6]     = Sequence
[7-8]   = CRC-16/MODBUS
```

#### Commands

| Code | Name | Description |
|------|------|-------------|
| `0x23` | WAKEUP | Wakes heater (sent every 3s) |
| `0x2B` | POWER | Turn heater on/off |
| `0x24` | MODE | Switch AUTO ↔ MANUAL |
| `0x3C` | UP | Increase temp (AUTO) or pump (MANUAL) |
| `0x3E` | DOWN | Decrease temp (AUTO) or pump (MANUAL) |

#### Operating Modes

**AUTO (Thermostat):**
- Heater controls pump automatically
- UP/DOWN adjusts target temperature
- Indicator: `[14] = 0x32`

**MANUAL (Hz):**
- User controls pump frequency
- UP/DOWN adjusts pump Hz
- Indicator: `[14] = 0xCD`

#### Heater States

| Code | Name | Description |
|------|------|-------------|
| `0x00` | OFF | Turned off |
| `0x01` | STARTUP | Starting up |
| `0x02` | WARMING | Warming up |
| `0x03` | WARMING_WAIT | Waiting |
| `0x04` | PRE_RUN | Pre-run |
| `0x05` | RUNNING | Running |
| `0x06` | SHUTDOWN | Shutting down |
| `0x07` | SHUTTING_DOWN | Shutting down |
| `0x08` | COOLING | Cooling |

### 🔬 Advanced Protocol Details

#### Authentication and Security

**Verification Mechanism:**
- Bytes `[16-20]` in STATUS packet contain "signature"
- Counter in `[6]` increments with each packet
- Remote checks if counter increases (replay attack protection??)
- Only heaters know the correct signature generation algorithm

**Why WAKEUP every 3 seconds?**
- Allows heater to detect remote presence
- Enables quick response to state changes

#### FSK Modulation Details

**How Decoding Works:**
- Heater uses **2-FSK (Frequency Shift Keying)**
- CC1101 decodes FSK automatically in hardware
- We receive ready bytes for parsing
- Format in rtl_433: `m=FSK_PCM`

**Raw Signal Structure:**
```
Preamble: AA AA AA AA AA AA AA (7+ bytes)
Sync:     7E 3C 7E 3C
Payload:  [Data packet - ready bytes]
```

#### Timing and Retransmissions

**Sending Commands:**
- Each command is sent **10 times** (burst)
- Delay between transmissions: ~10-15ms
- Heater accepts first correct one - rest ignored

**Receiving STATUS:**
- GDO2 timeout: 2 seconds (sufficient for 3-second cycle)
- FIFO can contain 24-26 bytes (packet + RSSI/LQI)
- CRC verified ALWAYS before accepting packet

### 🔗 Pairing

#### How Does Pairing Work?

Pairing is a special communication mode where:

1. **Heater sends special STATUS frame:**
   - Address: `0x00000000` (broadcast) OR special pairing address
   - STATUS packet is sent in "discovery" mode
   - Original remote listens for these frames for ~60 seconds

2. **Remote responds with verification:**
   - After receiving STATUS frame, remote verifies "authentication signature"
   - Checks if packet comes from real heater (not fake)
   - If verification OK → saves address and switches to normal operation

3. **Authentication:**
   - Heater uses special signature bytes in STATUS packet
   - Remote checks bytes `[16-20]` + counter in `[6]`
   - Only real heaters can generate correct signature

#### 💡 Interesting Fact - Original Remote Behavior

**Original remote has security mechanism:**

If something doesn't match (e.g., no communication with heater), POWER button may send DOWN command (0x3E) instead of POWER (0x2B).

**For our ESP32:**
```cpp
// Simply always send:
sendCommand(HEATER_CMD_POWER);  // 0x2B

// Good practice:
// - Send WAKEUP every 3-4s
// - Check STATUS
```

**What happens behind the scenes:**
```
1. ESP32 → Listen on 433.92 MHz (60s timeout)
2. Heater → Sends STATUS packet (23 bytes)
   Format: [7E 3C 7E 3C] [17] [00] [XX YY ZZ WW] [data...]
            └─sync─┘     │    │    └──address──┘
                         │    └─packet type (0x00 = STATUS)
                         └─length (23 bytes)
                         
3. ESP32 → Decodes address from bytes [2-5] of packet
   Example: [CA 00 44 5B] = 0xCA00445B
   
4. ESP32 → Saves address in NVS (Preferences)

5. ESP32 → Switches to normal operation with this address
```

**Technical Details:**
- Frequency: **433.92 MHz** (nominal)
  - ⚠️ **In practice:** Each CC1101 module may have ±10-30 kHz deviation
  - 💡 **Example:** Tested module worked best at 433.937 MHz
  - 🔧 **Calibration:** Use SDR# to find optimal frequency
- Modulation: 2-FSK (Frequency Shift Keying)
- Sync word: `0x7E3C` (repeated 2x)
- Packet type STATUS: `0x00` (type), `0x17` (23 bytes)
- Address: 32-bit in bytes [2-5] after sync word

#### Manual Pairing

1. **Find heater address:**
   - Use RTL-SDR + rtl_433
   - Listen for STATUS packets
   - Extract bytes `[2-5]` (big-endian)
   
   **Example:**
   ```
   STATUS: 17 00 CA 00 44 5B 05 01 ...
                ^  ^  ^  ^
                |  |  |  └─ [5] = 0x5B
                |  |  └──── [4] = 0x44
                |  └─────── [3] = 0x00
                └────────── [2] = 0xCA
   
   Address = 0xCA00445B
   ```

2. **Enter in Manual Pair field:** `0xCA00445B`
3. **Click MANUAL PAIR**
4. Address will be saved

#### ⚠️ IMPORTANT - Discovery Mode During Pairing

When you **press and hold the pairing button** on the heater, **two types of packets** may appear:

**1. Discovery Packet (type 0xAA)** - appears ONLY during pairing:
```
7e3c7e3c 17 AA CA 00 44 5B 00 00 00 85 05 00 04 ...
         │  │  └─remote addr─┘ └─special─┘
         │  └─type 0xAA (discovery!)
         └─23 bytes

```
- Heater "echoes" the address of the remote that triggered it
- In bytes [6-9] is code **0x00000085** (pairing mode)
- **DO NOT use this address for configuration!**

**2. Normal STATUS (type 0x00)** - this is the correct packet:
```
7e3c7e3c 17 00 CA 00 44 5B 05 01 00 83 0A ...
         │  │  └──your address──┘
         │  └─type 0x00 (normal STATUS)
         └─23 bytes
```
- Address in bytes [2-5] is **correct heater address**
- Use this address for configuration!

**How to verify?**
```bash
# Run rtl_433 and wait 10 seconds
# STATUS packets (type 0x00) appear every ~3 seconds
# Look for sequence: 7e3c7e3c 17 00 ...
#                                 ^^ type 0x00!
```

**Note:** Each heater has **unique address** - don't copy address from this README!


## 🛠️ Reverse Engineering and Debugging Tools

### 📡 RTL_433 - RF Transmission Decoding

For the entire protocol reverse engineering I used **rtl_433** - universal tool for decoding RF transmissions.


**Usage Example - listening to heater:**
```bash
rtl_433 -f 433920000 -s 250000 -R 0 \
  -X "n=heater,m=FSK_PCM,s=100,l=100,r=10000,preamble=aa" \
  -F json
```

**Parameters:**
- `-f 433920000` - frequency 433.92 MHz
- `-s 250000` - sample rate 250 kHz
- `-R 0` - disable all built-in decoders
- `-X` - custom decoder:
  - `m=FSK_PCM` - FSK modulation with PCM
  - `s=100` - short pulse 100 µs
  - `l=100` - long pulse 100 µs
  - `r=10000` - reset 10 ms
  - `preamble=aa` - look for 0xAA preamble
- `-F json` - output in JSON format

**What you'll see:**
```json
{
  "time": "2026-01-04 12:34:56",
  "model": "heater",
  "data": "7e3c7e3c1700ca00445b050100830a00981ecd1a..."
}
```

### 📻 SDR# + DVB-T Tuner - Spectrum Visualization

**Hardware:**
- **DVB-T USB dongle with R820T2 chip**
  - Cost: ~$15-25
  - Example: NooElec NESDR Smart, RTL-DVBT
  - Range: 25 MHz - 1.7 GHz
  - Perfect for 433 MHz!

**Software - SDR#:**
- Windows: [SDR#](https://airspy.com/download/)
- Linux: GQRX, CubicSDR

**How to use:**
```
1. Connect DVB-T dongle
2. Start SDR#
3. Set frequency: 433.920 MHz
4. Set mode: NFM or RAW
5. Adjust Gain (15-30 dB)
6. Observe spectrum during transmission
```

**How spectrum should look:**
```
Correct signal:
- Center frequency: 433.92 MHz
- Bandwidth: ~100-150 kHz
- FSK modulation visible as "two peaks"
- Burst every ~3 seconds (STATUS packets)
```
<img width="91" height="329" alt="Zrzut ekranu 2026-01-03 005132" src="https://github.com/user-attachments/assets/c2d3ec52-ae92-4357-b547-9d5e65a7bd0b" />

<img width="314" height="276" alt="Zrzut ekranu 2025-12-28 010813" src="https://github.com/user-attachments/assets/ec14aafc-8ced-4ec2-92ba-68daa24c2d98" />

---

<a name="polish"></a>
## 🇵🇱 WERSJA POLSKA

### 📡 Protokół RF

### ⚠️ Dokumentacja protokołu powstaje, może być nie precyzyjna i zawierać błędy ! aktualizacje niebawem :) ### 

#### Parametry
- **Częstotliwość:** 433.937 MHz
- **Modulacja:** 2-FSK
- **Kodowanie:** FSK_PCM (Frequency Shift Keying)
- **Sync Word:** `0x7E3C7E3C`
- **CRC:** CRC-16/MODBUS

#### Struktura pakietu STATUS (23 bajty)
```
[0]     = 0x17 (długość)
[1]     = 0x00 (typ: STATUS)
[2-5]   = Adres ogrzewacza (32-bit)
[6]     = Stan (0=OFF, 5=RUNNING, itd.)
[7]     = Moc (%)
[8-9]   = Napięcie (big-endian, /10 = V)
[10]    = Temperatura otoczenia (°C, signed)
[11]    = Błąd (0x00 = OK)
[12]    = Temperatura wymiennika (°C)
[13]    = Temperatura zadana (°C, signed)
[14]    = Tryb (0x32=AUTO, 0xCD=MANUAL)
[15]    = Częstotliwość pompy (/10 = Hz)
[16-20] = Dane wewnętrzne
[21-22] = CRC-16/MODBUS
```

#### Struktura pakietu COMMAND (9 bajtów)
```
[0]     = 0x09 (długość)
[1]     = Komenda
[2-5]   = Adres ogrzewacza
[6]     = Sekwencja
[7-8]   = CRC-16/MODBUS
```

#### Komendy

| Kod | Nazwa | Opis |
|-----|-------|------|
| `0x23` | WAKEUP | Budzi ogrzewacz (wysyłany co 3s) |
| `0x2B` | POWER | Włącz/wyłącz ogrzewacz |
| `0x24` | MODE | Przełącz AUTO ↔ MANUAL |
| `0x3C` | UP | Zwiększ temp (AUTO) lub pompę (MANUAL) |
| `0x3E` | DOWN | Zmniejsz temp (AUTO) lub pompę (MANUAL) |

#### Tryby pracy

**AUTO (Termostat):**
- Ogrzewacz kontroluje pompę automatycznie
- UP/DOWN reguluje temperaturę docelową
- Wskaźnik: `[14] = 0x32`

**MANUAL (Hz):**
- Użytkownik kontroluje częstotliwość pompy
- UP/DOWN reguluje Hz pompy
- Wskaźnik: `[14] = 0xCD`

#### Stany ogrzewacza

| Kod | Nazwa | Opis |
|-----|-------|------|
| `0x00` | OFF | Wyłączony |
| `0x01` | STARTUP | Uruchamianie |
| `0x02` | WARMING | Rozgrzewanie |
| `0x03` | WARMING_WAIT | Oczekiwanie |
| `0x04` | PRE_RUN | Pre-run |
| `0x05` | RUNNING | Pracuje |
| `0x06` | SHUTDOWN | Zamykanie |
| `0x07` | SHUTTING_DOWN | Wyłączanie |
| `0x08` | COOLING | Chłodzenie |

### 🔬 Zaawansowane szczegóły protokołu

#### Autentykacja i bezpieczeństwo

**Mechanizm weryfikacji:**
- Bajty `[16-20]` w pakiecie STATUS zawierają "podpis"
- Counter w `[6]` inkrementuje się z każdym pakietem
- Pilot sprawdza czy counter rośnie (ochrona przed replay attack??)
- Tylko ogrzewacze znają prawidłowy algorytm generowania podpisu

**Dlaczego WAKEUP co 3 sekundy?**
- Pozwala heaterowi wykryć obecność pilota
- Umożliwia szybką reakcję na zmiany stanu

#### Szczegóły modulacji FSK

**Jak działa dekodowanie:**
- Ogrzewacz używa **2-FSK (Frequency Shift Keying)**
- CC1101 dekoduje FSK automatycznie w hardware
- Otrzymujemy gotowe bajty do parsowania
- Format w rtl_433: `m=FSK_PCM`

**Struktura surowego sygnału:**
```
Preamble: AA AA AA AA AA AA AA (7+ bajtów)
Sync:     7E 3C 7E 3C
Payload:  [Data packet - gotowe bajty]
```

#### Timing i retransmisje

**Wysyłanie komend:**
- Każda komenda jest wysyłana **10 razy** (burst)
- Delay między transmisjami: ~10-15ms
- Heater akceptuje pierwszą poprawną - reszta ignorowana

**Odbieranie STATUS:**
- Timeout GDO2: 2 sekundy (wystarczy dla 3-sekundowego cyklu)
- FIFO może zawierać 24-26 bajtów (pakiet + RSSI/LQI)
- CRC weryfikowany ZAWSZE przed przyjęciem pakietu

### 🔗 Parowanie

#### Jak działa parowanie?

Parowanie to specjalny tryb komunikacji, w którym:

1. **Ogrzewacz wysyła specjalną ramkę STATUS:**
   - Adres: `0x00000000` (broadcast) LUB specjalny adres parowania
   - Pakiet STATUS jest wysyłany w trybie "discovery"
   - Oryginalny pilot nasłuchuje tych ramek przez ~60 sekund

2. **Pilot odpowiada weryfikacją:**
   - Po odebraniu ramki STATUS, pilot weryfikuje "sygnaturę autentyczności"
   - Sprawdza czy pakiet pochodzi z prawdziwego ogrzewacza (nie fejk)
   - Jeśli weryfikacja OK → zapisuje adres i przechodzi do normalnej pracy

3. **Uwierzytelnienie:**
   - Ogrzewacz używa specjalnych bajtów sygnatury w pakiecie STATUS
   - Pilot sprawdza bajty `[16-20]` + counter w `[6]`
   - Tylko prawdziwe ogrzewacze mogą wygenerować poprawną sygnaturę

#### 💡 Ciekawostka - Zachowanie oryginalnego pilota

**Oryginalny pilot ma mechanizm bezpieczeństwa:**

Jeśli coś się nie zgadza (np. brak komunikacji z ogrzewaczem), przycisk POWER może wysyłać komendę DOWN (0x3E) zamiast POWER (0x2B).

**Dla naszego ESP32:**
```cpp
// Wystarczy zawsze wysyłać:
sendCommand(HEATER_CMD_POWER);  // 0x2B

// Dobra praktyka:
// - Wysyłaj WAKEUP co 3-4s
// - Sprawdzaj STATUS
```


**Co się dzieje w tle:**
```
1. ESP32 → Nasłuch na 433.92 MHz (60s timeout)
2. Heater → Wysyła STATUS packet (23 bajty)
   Format: [7E 3C 7E 3C] [17] [00] [XX YY ZZ WW] [dane...]
            └─sync─┘     │    │    └──adres──┘
                         │    └─typ pakietu (0x00 = STATUS)
                         └─długość (23 bajty)
                         
3. ESP32 → Dekoduje adres z bajtów [2-5] pakietu
   Przykład: [CA 00 44 5B] = 0xCA00445B
   
4. ESP32 → Zapisuje adres w NVS (Preferences)

5. ESP32 → Przechodzi do normalnej pracy z tym adresem
```

**Szczegóły techniczne:**
- Częstotliwość: **433.92 MHz** (nominalnie)
  - ⚠️ **W praktyce:** Każdy moduł CC1101 może mieć odstępstwa ±10-30 kHz
  - 💡 **Przykład:** Testowany moduł pracował najlepiej na 433.937 MHz
  - 🔧 **Kalibracja:** Użyj SDR# do znalezienia optymalnej częstotliwości
- Modulacja: 2-FSK (Frequency Shift Keying)
- Sync word: `0x7E3C` (powtórzony 2x)
- Packet type STATUS: `0x00` (typ), `0x17` (23 bajty)
- Adres: 32-bit w bajtach [2-5] po sync word

#### Ręczne parowanie

1. **Znajdź adres ogrzewacza:**
   - Użyj RTL-SDR + rtl_433
   - Nasłuchuj pakietów STATUS
   - Wyciągnij bajty `[2-5]` (big-endian)
   
   **Przykład:**
   ```
   STATUS: 17 00 CA 00 44 5B 05 01 ...
                ^  ^  ^  ^
                |  |  |  └─ [5] = 0x5B
                |  |  └──── [4] = 0x44
                |  └─────── [3] = 0x00
                └────────── [2] = 0xCA
   
   Adres = 0xCA00445B
   ```

2. **Wpisz w pole Manual Pair:** `0xCA00445B`
3. **Kliknij MANUAL PAIR**
4. Adres zostanie zapisany

#### ⚠️ WAŻNE - Discovery Mode podczas parowania

Gdy **przytrzymujesz przycisk parowania** na ogrzewaczu, mogą pojawić się **dwa typy pakietów**:

**1. Discovery Packet (typ 0xAA)** - pojawia się TYLKO podczas parowania:
```
7e3c7e3c 17 AA CA 00 44 5B 00 00 00 85 05 00 04 ...
         │  │  └─pilot addr─┘ └─special─┘
         │  └─typ 0xAA (discovery!)
         └─23 bajty

```
- Ogrzewacz "echo-uje" adres pilota który go wywołał
- W bajtach [6-9] jest kod **0x00000085** (tryb parowania)
- **NIE używaj tego adresu do konfiguracji!**

**2. Normal STATUS (typ 0x00)** - to jest właściwy pakiet:
```
7e3c7e3c 17 00 CA 00 44 5B 05 01 00 83 0A ...
         │  │  └──twój adres──┘
         │  └─typ 0x00 (normalny STATUS)
         └─23 bajty
```
- Adres w bajtach [2-5] to **właściwy adres ogrzewacza**
- Tego adresu używaj do konfiguracji!

**Jak to sprawdzić?**
```bash
# Uruchom rtl_433 i poczekaj 10 sekund
# Pakiety STATUS (typ 0x00) pojawiają się co ~3 sekundy
# Szukaj sekwencji: 7e3c7e3c 17 00 ...
#                                 ^^ typ 0x00!
```

**Uwaga:** Każdy ogrzewacz ma **unikalny adres** - nie kopiuj adresu z tego README!


## 🛠️ Narzędzia do reverse engineering i debugowania

### 📡 RTL_433 - Dekodowanie transmisji RF

Do całego reverse engineering protokołu wykorzystałem **rtl_433** - uniwersalne narzędzie do dekodowania transmisji RF.


**Przykład użycia - nasłuchiwanie ogrzewacza:**
```bash
rtl_433 -f 433920000 -s 250000 -R 0 \
  -X "n=heater,m=FSK_PCM,s=100,l=100,r=10000,preamble=aa" \
  -F json
```

**Parametry:**
- `-f 433920000` - częstotliwość 433.92 MHz
- `-s 250000` - sample rate 250 kHz
- `-R 0` - wyłącz wszystkie wbudowane dekodery
- `-X` - custom decoder:
  - `m=FSK_PCM` - modulacja FSK z PCM
  - `s=100` - short pulse 100 µs
  - `l=100` - long pulse 100 µs
  - `r=10000` - reset 10 ms
  - `preamble=aa` - szukaj preambuły 0xAA
- `-F json` - output w formacie JSON

**Co zobaczysz:**
```json
{
  "time": "2026-01-04 12:34:56",
  "model": "heater",
  "data": "7e3c7e3c1700ca00445b050100830a00981ecd1a..."
}
```

### 📻 SDR# + DVB-T Tuner - Wizualizacja widma

**Hardware:**
- **DVB-T USB dongle z chipem R820T2**
  - Koszt: ~$15-25
  - Przykład: NooElec NESDR Smart, RTL-DVBT
  - Zakres: 25 MHz - 1.7 GHz
  - Idealny do 433 MHz!

**Software - SDR#:**
- Windows: [SDR#](https://airspy.com/download/)
- Linux: GQRX, CubicSDR

**Jak użyć:**
```
1. Podłącz DVB-T dongle
2. Uruchom SDR#
3. Ustaw częstotliwość: 433.920 MHz
4. Ustaw mode: NFM lub RAW
5. Dostosuj Gain (15-30 dB)
6. Obserwuj widmo podczas transmisji
```

**Jak powinno wyglądać widmo:**
```
Prawidłowy sygnał:
- Centralna częstotliwość: 433.92 MHz
- Szerokość pasma: ~100-150 kHz
- Modulacja FSK widoczna jako "dwa szczyty"
- Burst co ~3 sekundy (STATUS packets)
```


<img width="91" height="329" alt="Zrzut ekranu 2026-01-03 005132" src="https://github.com/user-attachments/assets/c2d3ec52-ae92-4357-b547-9d5e65a7bd0b" />

<img width="314" height="276" alt="Zrzut ekranu 2025-12-28 010813" src="https://github.com/user-attachments/assets/ec14aafc-8ced-4ec2-92ba-68daa24c2d98" />
