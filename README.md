# DieselPilot — Free (open source)

Sterownik ESP32 do chińskich nagrzewnic Diesla (CC1101 433.937 MHz, OLED SH1106,
WebGUI, MQTT, OTA). Wersja **open source — bez chmury**. Zdalny dostęp realizowany
lokalnie: WebGUI, MQTT (Home Assistant) oraz aktualizacje firmware przez OTA.

## Struktura

```
platformio.ini      konfiguracja (board, partycje, biblioteki)
src/                kod firmware (.ino)
data/               pliki LittleFS (index.html – WebGUI)
```

## Wymagania

- [PlatformIO](https://platformio.org/) (rozszerzenie VS Code lub `pip install platformio`)

## Budowanie i wgrywanie

```bash
# 1) Firmware (przez USB)
pio run -t upload

# 2) Partycja LittleFS (zawartość katalogu data/, m.in. index.html)
pio run -t uploadfs

# Podgląd portu szeregowego
pio device monitor
```

> Po każdej zmianie `data/index.html` trzeba ponownie wykonać `uploadfs`.
> `upload` wgrywa tylko firmware i nie nadpisuje filesystemu.

## Aktualizacje OTA

1. W WebGUI otwórz zakładkę **⬆ OTA**, włącz OTA i (opcjonalnie) ustaw hasło — urządzenie się zrestartuje.
2. Wgraj firmware przez sieć:

```bash
pio run -t upload --upload-port <ip-urzadzenia>
# z hasłem:
pio run -t upload --upload-port <ip-urzadzenia> --upload-flags --auth=<haslo>
```

Można też na stałe ustawić `upload_protocol = espota` / `upload_port` / `upload_flags`
w `platformio.ini` (zakomentowana sekcja na dole pliku).

## Partycje

Wbudowana tablica `default.csv`: dwie partycje app (`app0`/`app1`, wymagane przez OTA)
+ ok. 1.5 MB na filesystem (LittleFS). Dzięki temu działa zarówno OTA, jak i
serwowanie WebGUI z LittleFS.

## Biblioteki (pobierane automatycznie przez PlatformIO)

- olikraus/U8g2 — OLED SH1106
- knolleary/PubSubClient — MQTT

OTA korzysta z `ArduinoOTA` (wbudowane w rdzeń ESP32).
CC1101 sterowany jest własnymi funkcjami SPI (bez zewnętrznej biblioteki).
