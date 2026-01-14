# 🌐 Language / Język

**[🇬🇧 English](#english)** | **[🇵🇱 Polski](#polish)**

---

<a name="english"></a>
## 🇬🇧 ENGLISH VERSION

### 📡 RF Protocol

### ⚠️ The protocol documentation is being created, may be inaccurate ### 

### 🔑 Communication Fundamentals (Read First!)

This is the key to understanding the entire protocol:

* **The heater controller does NOT have its own address** in RF communication
* **Address in bytes [2-5] of every packet** is ALWAYS the **REMOTE CONTROL address**
* **Controller uses WHITELIST mechanism** for authorized remote addresses
* **Commands are accepted ONLY if** address = whitelist
* **DieselPilot works as a clone** (impersonation) of a paired remote

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
[7]     = Error codes
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
**Packet types:**
- **0x00** = Normal STATUS (during operation)
- **0xAA** = DISCOVERY/PAIRING mode (only when holding PAIRING button)

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

#### Operating Modes [14]

**AUTO (Thermostat):**
- Heater controls pump automatically
- UP/DOWN adjusts target temperature
- Indicator: `[14] = 0x32`

**MANUAL (Hz):**
- User controls pump frequency
- UP/DOWN adjusts pump Hz
- Indicator: `[14] = 0xCD`

#### Heater States [6]

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

### Complete Error Map [7]

| CODE | ERROR DESCRIPTION                               |
|------|-------------------------------------------------|
| 0x00 | ✅ NO ERROR - normal operation                 |
| 0x01 | ⚡ ON - Starting / Preparation phase           |
| 0x02 | 🔋 UNDERVOLTAGE - Supply voltage too low       |
| 0x03 | ⚡ OVERVOLTAGE - Supply voltage too high       |
| 0x04 | 🔌 SPARK PLUG ERROR - Glow plug failure        |
| 0x05 | 🛢️ OIL PUMP ERROR - Fuel pump malfunction      |
| 0x06 | 🌡️ OVERHEAT ERROR - Overheating condition      |
| 0x07 | ⚙️ MOTOR ERROR - Motor / fan failure           |
| 0x08 | 🔌 DISCONNECT ERROR - Connection error         |
| 0x09 | 🔥 EXTINGUISHED - Flame out / loss of combustion |
| 0x0A | 🌡️ SENSOR ERROR - Temperature sensor failure   |
| 0x0B | 🔥 IGNITION ERROR - Ignition failure           |
| 0x0C | ⏸️ STANDBY - Standby mode / Ready state        |

---


### 🔐 Whitelist & Controller Versions

| Feature | V2 "Wrench" | V1 "SUN" |
|---------|-------------|----------|
| **Whitelist slots** | 1 | 2 |
| **New pairing** | Overwrites | Fills available slot |
| **Address cloning** | Yes | Optional |
| **Support status** | ✅ Stable | ⚠️ Development |

**DieselPilot compatibility:**
- **V2:** Clones existing remote address → both work simultaneously ✅
- **V1:** support in development ⚠️

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

#### 🔄 Auto-Pairing Process

```
1. Hold PAIRING button on heater panel (5 seconds)
2. Heater broadcasts: [17][AA][CA00445B][00000085]... 
                            └─ Type 0xAA = pairing/discovery mode
3. DieselPilot captures remote address from bytes [2-5]
4. Saves as "own" address in memory
5. Sends commands: [09][2B][CA00445B][seq][CRC]
6. Controller accepts ✓ (address matches whitelist)
```

**Why both DieselPilot and original remote work:**
```
Original Remote    DieselPilot       Heater Controller
(0xCA00445B)       (0xCA00445B)      (Whitelist: 0xCA00445B)
     │                  │                    │
     │    ──[POWER]──>  │     ──[POWER]──>   │  ✓ Accepted
     │   <──[STATUS]──  │    <──[STATUS]──   │
     └── Both use SAME address → both work! 
```
Controller cannot distinguish the source — it only checks the address!

#### ✏️ Manual Pairing

**Step 1: Find the remote address**
```bash
# Listen with RTL-SDR during normal operation
rtl_433 -f 433920000 -s 250000 -R 0 \
  -X "n=heater,m=FSK_PCM,s=100,l=100,r=10000,preamble=aa"

# Find STATUS packet (type 0x00):
7e3c7e3c 17 00 CA 00 44 5B 05 01 ...
               ^─────────^ remote address!
```

**Step 2: Enter in DieselPilot**
- Manual Pair field: `0xCA00445B`
- Click MANUAL PAIR
- Done! DieselPilot will impersonate this remote

**⚠️ Important:**
- This is the **remote address**, not heater address
- You're cloning/impersonating existing paired remote
- Original remote + DieselPilot will both work (same address)
- If you pair a different remote with heater, this address stops working

---

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

### 📊 Normal STATUS Packet (type 0x00)

**Raw packet:**
```
7e3c7e3c 17 00 ca00445b 05 01 00 83 0a 00 98 1e cd 1a 00 00 e2 9c 44 3e ae 00 ed
```

**Step-by-step decoding:**

| Bytes | Hex | Meaning | Value |
|-------|-----|---------|-------|
| **Sync** | `7e 3c 7e 3c` | Sync Word | ✓ Valid |
| **[0]** | `17` | Packet length | 23 bytes |
| **[1]** | `00` | Packet type | STATUS (operation) |
| **[2-5]** | `ca 00 44 5b` | Remote address | 0xCA00445B |
| **[6]** | `05` | State | RUNNING (5) |
| **[7]** | `01` | Power | 1% |
| **[8-9]** | `00 83` | Voltage (BE) | 13.1V (131/10) |
| **[10]** | `0a` | Ambient temp | 10°C |
| **[11]** | `00` | Error | OK (0) |
| **[12]** | `98` | Exchanger temp | 152°C |
| **[13]** | `1e` | Target temp | 30°C |
| **[14]** | `cd` | Mode | MANUAL (0xCD) |
| **[15]** | `1a` | Pump frequency | 2.6 Hz (26/10) |
| **[16-20]** | `00 00 e2 9c 44` | Signature | [internal data] |
| **[21-22]** | `3e ae` | CRC | ✓ Valid |

**Interpretation:**
- 🟢 Heater is **RUNNING**
- 🔥 Exchanger temperature: **152°C**
- 🌡️ Ambient temperature: **10°C**
- 🎯 Target temperature: **30°C**
- ⚡ Voltage: **13.1V**
- 🔧 Mode: **MANUAL**, pump **2.6 Hz**


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

### ⚠️ Dokumentacja protokołu powstaje, może być nie precyzyjna ### 

### 🔑 Fundamenty komunikacji (czytaj najpierw!)

To jest klucz do zrozumienia całego protokołu:

* **Sterownik ogrzewacza NIE posiada własnego adresu** w komunikacji RF
* **Adres w bajtach [2-5] każdego pakietu** to ZAWSZE adres **PILOTA**
* **Sterownik używa mechanizmu WHITELIST** autoryzowanych adresów pilotów
* **Komendy są akceptowane tylko**, jeśli adres = whitelist
* **DieselPilot działa jako klon** (impersonacja) sparowanego pilota

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
[7]     = Kody błędu 
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

**Typy pakietów:**
- **0x00** = Normalny STATUS (podczas pracy)
- **0xAA** = Tryb DISCOVERY/PAIRING (tylko przy przytrzymaniu PAIRING)

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

#### Tryby pracy [14]

**AUTO (Termostat):**
- Ogrzewacz kontroluje pompę automatycznie
- UP/DOWN reguluje temperaturę docelową
- Wskaźnik: `[14] = 0x32`

**MANUAL (Hz):**
- Użytkownik kontroluje częstotliwość pompy
- UP/DOWN reguluje Hz pompy
- Wskaźnik: `[14] = 0xCD`

#### Stany ogrzewacza [6]

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


### Kompletna Mapa Błędów [7]

| KOD  | OPIS BŁĘDU                                      |
|------|-------------------------------------------------|
| 0x00 | ✅ BRAK BŁĘDU - normalna praca                 |
| 0x01 | ⚡ ON - Uruchamianie / Przygotowanie           |
| 0x02 | 🔋 UNDERVOLTAGE - Zbyt niskie napięcie         |
| 0x03 | ⚡ OVERVOLTAGE - Zbyt wysokie napięcie         |
| 0x04 | 🔌 SPARK PLUG ERROR - Błąd świecy żarowej     |
| 0x05 | 🛢️ OIL PUMP ERROR - Awaria pompy paliwa        |
| 0x06 | 🌡️ OVERHEAT ERROR - Przegrzanie                |
| 0x07 | ⚙️ MOTOR ERROR - Awaria silnika / wentylatora  |
| 0x08 | 🔌 DISCONNECT ERROR - Błąd połączenia          |
| 0x09 | 🔥 EXTINGUISHED - Zgaszony / Utrata płomienia  |
| 0x0A | 🌡️ SENSOR ERROR - Błąd czujnika temperatury    |
| 0x0B | 🔥 IGNITION ERROR - Błąd zapłonu               |
| 0x0C | ⏸️ STANDBY - Tryb oczekiwania / Gotowość       |

---

### 🔐 Whitelist i wersje sterowników

| Cecha | V2 "Wrench" | V1 "SUN" |
|-------|-------------|----------|
| **Sloty whitelist** | 1 | 2 |
| **Nowe parowanie** | Nadpisuje | naspisuje wybrany slot |
| **Klonowanie adresu** | Tak | Opcjonalne |
| **Status wsparcia** | ✅ Stabilne | ⚠️ w rozwoju |

**Kompatybilność DieselPilot:**
- **V2:** Klonuje adres istniejącego pilota → oba działają jednocześnie ✅
- **V1:** Wsparcie w rozwoju ⚠️

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

#### 🔄 Proces Auto-Parowania

```
1. Przytrzymaj przycisk PAIRING na panelu ogrzewacza (5 sekund)
2. Ogrzewacz wysyła: [17][AA][CA00445B][00000085]... 
                           └─ Typ 0xAA = tryb parowania/discovery
3. DieselPilot przechwytuje adres pilota z bajtów [2-5]
4. Zapisuje jako adres w pamięci
5. Wysyła komendy: [09][2B][CA00445B][seq][CRC]
6. Sterownik akceptuje ✓ (adres pasuje do whitelist)
```

**Dlaczego DieselPilot i oryginalny pilot działają jednocześnie:**
```
Oryginalny Pilot   DieselPilot       Sterownik Ogrzewacza
(0xCA00445B)       (0xCA00445B)      (Whitelist: 0xCA00445B)
     │                  │                    │
     │──[POWER]──>      │──[POWER]──>        │  ✓ Zaakceptowane
     │                  │                    │
     └── Oba używają TEGO SAMEGO adresu → oba działają! ──┘
```
Sterownik nie potrafi rozróżnić źródła — sprawdza tylko adres!

#### ✏️ Ręczne Parowanie

**Krok 1: Znajdź adres pilota**
```bash
# Nasłuchuj RTL-SDR podczas normalnej pracy
rtl_433 -f 433920000 -s 250000 -R 0 \
  -X "n=heater,m=FSK_PCM,s=100,l=100,r=10000,preamble=aa"

# Znajdź pakiet STATUS (typ 0x00):
7e3c7e3c 17 00 CA 00 44 5B 05 01 ...
               ^─────────^ adres pilota!
```

**Krok 2: Wpisz w DieselPilot**
- Pole Manual Pair: `0xCA00445B`
- Kliknij MANUAL PAIR
- Gotowe! DieselPilot będzie się podszywał pod ten pilot

**⚠️ Ważne:**
- To jest **adres pilota**, nie ogrzewacza
- Klonujesz/podszywasz się pod istniejący sparowany pilot
- Oryginalny pilot + DieselPilot będą oba działać (ten sam adres)
- Jeśli sparujesz inny pilot z ogrzewaczem, ten adres przestanie działać


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

### 📊 Normalny pakiet STATUS (typ 0x00)

**Surowy pakiet:**
```
7e3c7e3c 17 00 ca00445b 05 01 00 83 0a 00 98 1e cd 1a 00 00 e2 9c 44 3e ae 00 ed
```

**Dekodowanie krok po kroku:**

| Bajty | Hex | Znaczenie | Wartość |
|-------|-----|-----------|---------|
| **Sync** | `7e 3c 7e 3c` | Sync Word | ✓ Poprawny |
| **[0]** | `17` | Długość pakietu | 23 bajty |
| **[1]** | `00` | Typ pakietu | STATUS (praca) |
| **[2-5]** | `ca 00 44 5b` | Adres pilota | 0xCA00445B |
| **[6]** | `05` | Stan | RUNNING (5) |
| **[7]** | `01` | Kod błędu | |
| **[8-9]** | `00 83` | Napięcie (BE) | 13.1V (131/10) |
| **[10]** | `0a` | Temp. otoczenia | 10°C |
| **[11]** | `00` | Błąd | OK (0) |
| **[12]** | `98` | Temp. wymiennika | 152°C |
| **[13]** | `1e` | Temp. zadana | 30°C |
| **[14]** | `cd` | Tryb | MANUAL (0xCD) |
| **[15]** | `1a` | Częst. pompy | 2.6 Hz (26/10) |
| **[16-20]** | `00 00 e2 9c 44` | Sygnatura | [dane wewnętrzne] |
| **[21-22]** | `3e ae` | CRC | ✓ Poprawny |

**Interpretacja:**
- 🟢 Ogrzewacz **PRACUJE** (RUNNING)
- 🔥 Temperatura wymiennika: **152°C**
- 🌡️ Temperatura otoczenia: **10°C**
- 🎯 Temperatura zadana: **30°C**
- ⚡ Napięcie: **13.1V**
- 🔧 Tryb **MANUAL**, pompa **2.6 Hz**

---

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
