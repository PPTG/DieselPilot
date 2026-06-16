/*
 * ═══════════════════════════════════════════════════════════════════════════
 *                        DIESEL PILOT — FREE (open source)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Full-featured ESP32 controller for Chinese diesel heaters
 *
 * Features:
 * - WiFi AP mode (default) + STA mode
 * - Web GUI (dark theme)
 * - OLED SH1106 display (IP + status)
 * - Auto/Manual pairing
 * - Real-time heater control
 * - MQTT integration (Home Assistant ready)
 * - ERROR CODE DECODING (BYTE[7])
 * - OTA firmware updates (ArduinoOTA / espota — password protected)
 *
 * Hardware:
 * - ESP32
 * - CC1101 @ 433.937 MHz
 * - SH1106 OLED (I2C)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *                  Version: V1.3 - FREE
 * ═══════════════════════════════════════════════════════════════════════════
 * This is the open-source edition (no cloud). Remote access is provided via
 * local Web GUI, MQTT and on-network OTA updates.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>          // OTA firmware updates (espota protocol)

// ═══════════════════════════════════════════════════════════════════════════
// HARDWARE CONFIG
// ═══════════════════════════════════════════════════════════════════════════

// CC1101 Pins
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_SS    5
#define PIN_GDO2  4

// I2C Pins (OLED)
#define PIN_SDA   21
#define PIN_SCL   22

// OLED Configuration
#define USE_OLED  true

// Commands
#define CMD_WAKEUP 0x23
#define CMD_MODE   0x24
#define CMD_POWER  0x2B
#define CMD_UP     0x3C
#define CMD_DOWN   0x3E

// Heater States
#define STATE_OFF            0
#define STATE_STARTUP        1
#define STATE_WARMING        2
#define STATE_WARMING_WAIT   3
#define STATE_PRE_RUN        4
#define STATE_RUNNING        5
#define STATE_SHUTDOWN       6
#define STATE_SHUTTING_DOWN  7
#define STATE_COOLING        8

// Error Codes (BYTE[7])
#define ERR_NONE           0x00
#define ERR_ON             0x01
#define ERR_UNDERVOLTAGE   0x02
#define ERR_OVERVOLTAGE    0x03
#define ERR_SPARK_PLUG     0x04
#define ERR_OIL_PUMP       0x05
#define ERR_OVERHEAT       0x06
#define ERR_MOTOR          0x07
#define ERR_DISCONNECT     0x08
#define ERR_EXTINGUISHED   0x09
#define ERR_SENSOR         0x0A
#define ERR_IGNITION       0x0B
#define ERR_STANDBY        0x0C

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

WebServer server(80);
Preferences prefs;
WiFiClient espClient;
PubSubClient mqtt(espClient);

// VERSION CONTROL
String heaterVersion = "V2";
uint32_t customFrequency = 0;
bool autoMenuV1 = true;
unsigned long lastMenuChangeV1 = 0;
const unsigned long menuIntervalV1 = 3500;
const unsigned long commandLockTimeV1 = 5000;
unsigned long lastCommandTimeV1 = 0;
byte myAddrV1[3] = {0x19, 0x52, 0x4B};
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════════════

String version = "1.3";

// WiFi
String apSSID = "Diesel-Pilot";
String apPassword = "12345678";
String staSSID = "";
String staPassword = "";
bool useAP = true;

// MQTT
String deviceName = "DieselPilot";
String mqttServer = "";
int mqttPort = 1883;
String mqttTopic = "diesel";
String mqttUser = "";
String mqttPassword = "";
bool mqttAuthEnabled = false;
bool mqttEnabled = false;

// Heater
uint32_t heaterAddress = 0x00000000;
uint8_t packetSeq = 0;
bool heaterPaired = false;

// Status
struct {
    uint8_t state = 0;
    uint8_t power = 0;
    uint16_t voltage = 0;
    int8_t ambientTemp = 0;
    uint8_t caseTemp = 0;
    int8_t setpoint = 0;
    uint8_t pumpFreq = 0;
    bool autoMode = false;
    int16_t rssi = 0;
    uint8_t errorCode = 0;
    unsigned long lastUpdate = 0;
} heaterStatus;

// Error History
struct ErrorHistoryEntry {
    uint8_t errorCode;
    unsigned long timestamp;
};
ErrorHistoryEntry errorHistory[10];
int errorHistoryIndex = 0;

// Display
String displayLine1 = "Diesel Pilot";
String displayLine2 = "Initializing...";
String displayLine3 = "";
String displayLine4 = "";

// Timing
unsigned long lastUpdate = 0;
unsigned long lastDisplay = 0;
unsigned long lastMQTTRetry = 0;
const unsigned long mqttRetryInterval = 30000;

// ═══════════════════════════════════════════════════════════════════════════
// OTA VARIABLES
// ═══════════════════════════════════════════════════════════════════════════

bool   otaEnabled  = false;             // Master ON/OFF switch (saved in NVS)
String otaPassword = "";                // Password required to flash (saved in NVS)
bool   otaRunning  = false;             // ArduinoOTA.begin() has been called

// ═══════════════════════════════════════════════════════════════════════════
// ERROR CODE DECODER
// ═══════════════════════════════════════════════════════════════════════════

const char* getErrorName(uint8_t code) {
    switch(code) {
        case ERR_NONE:         return "NORMAL";
        case ERR_ON:           return "NORMAL";
        case ERR_UNDERVOLTAGE: return "UNDERVOLTAGE";
        case ERR_OVERVOLTAGE:  return "OVERVOLTAGE";
        case ERR_SPARK_PLUG:   return "SPARK PLUG";
        case ERR_OIL_PUMP:     return "OIL PUMP";
        case ERR_OVERHEAT:     return "OVERHEAT";
        case ERR_MOTOR:        return "MOTOR";
        case ERR_DISCONNECT:   return "DISCONNECT";
        case ERR_EXTINGUISHED: return "EXTINGUISHED";
        case ERR_SENSOR:       return "SENSOR";
        case ERR_IGNITION:     return "IGNITION";
        case ERR_STANDBY:      return "STANDBY";
        default:               return "UNKNOWN";
    }
}

void addErrorToHistory(uint8_t errorCode) {
    if(errorCode == ERR_NONE) return;
    if(errorHistoryIndex > 0 && errorHistory[(errorHistoryIndex - 1) % 10].errorCode == errorCode) return;
    errorHistory[errorHistoryIndex % 10].errorCode = errorCode;
    errorHistory[errorHistoryIndex % 10].timestamp = millis();
    errorHistoryIndex++;
}

// ═══════════════════════════════════════════════════════════════════════════
// CC1101 LOW-LEVEL FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void cc1101_writeReg(uint8_t addr, uint8_t val) {
    digitalWrite(PIN_SS, LOW);
    while(digitalRead(PIN_MISO));
    SPI.transfer(addr);
    SPI.transfer(val);
    digitalWrite(PIN_SS, HIGH);
}

void cc1101_writeBurst(uint8_t addr, uint8_t len, uint8_t* bytes) {
    digitalWrite(PIN_SS, LOW);
    while(digitalRead(PIN_MISO));
    SPI.transfer(addr);
    for(int i = 0; i < len; i++) SPI.transfer(bytes[i]);
    digitalWrite(PIN_SS, HIGH);
}

void cc1101_strobe(uint8_t addr) {
    digitalWrite(PIN_SS, LOW);
    while(digitalRead(PIN_MISO));
    SPI.transfer(addr);
    digitalWrite(PIN_SS, HIGH);
}

uint8_t cc1101_readReg(uint8_t addr) {
    digitalWrite(PIN_SS, LOW);
    while(digitalRead(PIN_MISO));
    SPI.transfer(addr);
    uint8_t val = SPI.transfer(0xFF);
    digitalWrite(PIN_SS, HIGH);
    return val;
}

// ═══════════════════════════════════════════════════════════════════════════
// CRC-16/MODBUS
// ═══════════════════════════════════════════════════════════════════════════

uint16_t crc16_modbus(uint8_t* buf, int len) {
    uint16_t crc = 0xFFFF;
    for(int pos = 0; pos < len; pos++) {
        crc ^= (uint8_t)buf[pos];
        for(int i = 8; i != 0; i--) {
            if((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; }
            else { crc >>= 1; }
        }
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════════════════
// CC1101 INIT
// ═══════════════════════════════════════════════════════════════════════════

void cc1101_init() {
    cc1101_strobe(0x30); delay(100);
    cc1101_writeReg(0x00, 0x07); cc1101_writeReg(0x02, 0x06);
    cc1101_writeReg(0x03, 0x47); cc1101_writeReg(0x07, 0x04);
    cc1101_writeReg(0x08, 0x05); cc1101_writeReg(0x0A, 0x00);
    cc1101_writeReg(0x0B, 0x06); cc1101_writeReg(0x0C, 0x00);
    cc1101_writeReg(0x0D, 0x10); cc1101_writeReg(0x0E, 0xB0);
    cc1101_writeReg(0x0F, 0x9C); cc1101_writeReg(0x10, 0xF8);
    cc1101_writeReg(0x11, 0x93); cc1101_writeReg(0x12, 0x13);
    cc1101_writeReg(0x13, 0x22); cc1101_writeReg(0x14, 0xF8);
    cc1101_writeReg(0x15, 0x26); cc1101_writeReg(0x17, 0x30);
    cc1101_writeReg(0x18, 0x18); cc1101_writeReg(0x19, 0x16);
    cc1101_writeReg(0x1A, 0x6C); cc1101_writeReg(0x1B, 0x03);
    cc1101_writeReg(0x1C, 0x40); cc1101_writeReg(0x1D, 0x91);
    cc1101_writeReg(0x20, 0xFB); cc1101_writeReg(0x21, 0x56);
    cc1101_writeReg(0x22, 0x17); cc1101_writeReg(0x23, 0xE9);
    cc1101_writeReg(0x24, 0x2A); cc1101_writeReg(0x25, 0x00);
    cc1101_writeReg(0x26, 0x1F); cc1101_writeReg(0x2C, 0x81);
    cc1101_writeReg(0x2D, 0x35); cc1101_writeReg(0x2E, 0x09);
    cc1101_writeReg(0x09, 0x00); cc1101_writeReg(0x04, 0x7E);
    cc1101_writeReg(0x05, 0x3C);
    uint8_t paTable[8] = {0x00, 0x12, 0x0E, 0x34, 0x60, 0xC5, 0xC1, 0xC0};
    cc1101_writeBurst(0x7E, 8, paTable);
    cc1101_strobe(0x31); cc1101_strobe(0x36); cc1101_strobe(0x3B);
    cc1101_strobe(0x36); cc1101_strobe(0x3A); delay(136);
    Serial.println("✅ CC1101 initialized @ 433.937 MHz (V2)");
}

void cc1101_init_V1() {
    cc1101_strobe(0x30); delay(100);
    const byte configRegsV1[] = {
        0x0B, 0x06, 0x0D, 0x10, 0x0E, 0xB0, 0x0F, 0x71,
        0x10, 0x86, 0x11, 0x83, 0x12, 0x12, 0x13, 0x22,
        0x04, 0x09, 0x05, 0x1A, 0x06, 0x0A, 0x08, 0x00,
        0x15, 0x40, 0x18, 0x18, 0x23, 0xFF
    };
    for(int i = 0; i < sizeof(configRegsV1); i += 2)
        cc1101_writeReg(configRegsV1[i], configRegsV1[i+1]);
    cc1101_strobe(0x36); delay(5); cc1101_strobe(0x34);
    Serial.println("✅ CC1101 initialized @ 433.892 MHz (V1)");
}

void cc1101_setFrequency(uint32_t freqHz) {
    uint32_t freq = ((uint64_t)freqHz * 65536) / 26000000;
    cc1101_writeReg(0x0D, (freq >> 16) & 0xFF);
    cc1101_writeReg(0x0E, (freq >> 8) & 0xFF);
    cc1101_writeReg(0x0F, freq & 0xFF);
    Serial.printf("✅ CC1101 custom frequency set: %lu Hz\n", freqHz);
}

// ═══════════════════════════════════════════════════════════════════════════
// V1 SPECIFIC FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void rxFlushV1() { cc1101_strobe(0x36); cc1101_strobe(0x3A); delay(5); }
void rxEnableV1() { cc1101_strobe(0x36); delay(5); cc1101_strobe(0x34); }

void sendFrame_V1(byte cmd, byte sub, byte d1, byte d2) {
    byte frame[10] = {0x2B, 0x01, myAddrV1[0], myAddrV1[1], myAddrV1[2], cmd, sub, d1, d2, 0x00};
    cc1101_strobe(0x36); cc1101_strobe(0x3B);
    for(int i = 0; i < 10; i++) cc1101_writeReg(0x3F, frame[i]);
    cc1101_strobe(0x35); delay(100);
    rxFlushV1(); rxEnableV1();
}

void decodePacket_V1(byte* data) {
    if(data[2] != myAddrV1[0] || data[3] != myAddrV1[1] || data[4] != myAddrV1[2]) return;
    uint8_t cmd = data[5], sub = data[6], b7 = data[7], b8 = data[8];
    Serial.printf("V1 PKT: CMD=%02X SUB=%02X B7=%02X B8=%02X\n", cmd, sub, b7, b8);

    if(cmd == 0x32 && sub == 0x20) {
        heaterStatus.state = STATE_RUNNING;
        heaterStatus.voltage = (b7 * 2 + (b8 & 0x01)) * 10;
        heaterStatus.errorCode = ERR_NONE;
    } else if(cmd == 0x31 && sub == 0x60) {
        heaterStatus.state = STATE_COOLING;
        heaterStatus.errorCode = (b7 == 0x64) ? ERR_EXTINGUISHED : ERR_NONE;
    } else if(cmd == 0x31 && sub == 0x00) {
        heaterStatus.state = STATE_OFF;
        if(b7 == 0x06)       heaterStatus.errorCode = ERR_NONE;
        else if(b7 == 0x61)  heaterStatus.errorCode = ERR_OIL_PUMP;
        else if(b7 == 0x62)  heaterStatus.errorCode = ERR_SPARK_PLUG;
        else if(b7 == 0x63)  heaterStatus.errorCode = ERR_SENSOR;
        else                  heaterStatus.errorCode = ERR_NONE;
    } else if(cmd == 0x31 && sub == 0x20) {
        heaterStatus.state = STATE_RUNNING;
        heaterStatus.errorCode = ERR_NONE;
        if(b7 <= 0x03) {
            heaterStatus.autoMode = false;
            if(b7 == 0x00 && (b8 & 0xF0) == 0x90) heaterStatus.power = 1;
            else if(b7 == 0x01 && (b8 & 0xF0) == 0x10) heaterStatus.power = 2;
            else if(b7 == 0x01 && (b8 & 0xF0) == 0x90) heaterStatus.power = 3;
            else if(b7 == 0x02 && (b8 & 0xF0) == 0x10) heaterStatus.power = 4;
            else if(b7 == 0x02 && (b8 & 0xF0) == 0x90) heaterStatus.power = 5;
            else if(b7 == 0x03 && (b8 & 0xF0) == 0x10) heaterStatus.power = 6;
            heaterStatus.pumpFreq = heaterStatus.power * 10;
            heaterStatus.setpoint = 0;
        } else {
            heaterStatus.autoMode = true;
            heaterStatus.setpoint = (b7 - 32) * 2 + (b8 >> 7);
            heaterStatus.power = 0;
        }
        heaterStatus.ambientTemp = (b8 & 0x0F) + 10;
    } else if(cmd == 0x31 && sub == 0xA0) {
        heaterStatus.caseTemp = b7 * 2;
    }
    heaterStatus.lastUpdate = millis();
}

void updateHeaterStatus_V1() {
    if(!heaterPaired) return;
    unsigned long now = millis();
    static unsigned long lastPoll = 0;
    if(now - lastPoll > 100) {
        lastPoll = now;
        uint8_t rxBytes = cc1101_readReg(0xBB) & 0x7F;
        if(rxBytes >= 10) {
            byte buf[10];
            for(int i = 0; i < 10; i++) buf[i] = cc1101_readReg(0xBF);
            decodePacket_V1(buf);
            rxFlushV1(); rxEnableV1();
        }
    }
    if(autoMenuV1 && heaterPaired) {
        if(now - lastCommandTimeV1 > commandLockTimeV1) {
            if(now - lastMenuChangeV1 >= menuIntervalV1) {
                lastMenuChangeV1 = now;
                sendFrame_V1(0x29, 0x00, 0x00, 0x00);
                delay(150);
                sendFrame_V1(0x2B, 0xAA, 0x80, 0x00);
            }
        }
    }
    if(mqttEnabled && mqtt.connected()) publishMQTT();
}

// ═══════════════════════════════════════════════════════════════════════════
// TX/RX FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void txFlush() { cc1101_strobe(0x36); cc1101_strobe(0x3B); delay(16); }

void txBurst(uint8_t len, uint8_t* bytes) {
    txFlush();
    cc1101_writeBurst(0x7F, len, bytes);
    cc1101_strobe(0x35);
}

void sendCommand(uint8_t cmd) {
    if(!heaterPaired) return;
    if(heaterVersion == "V1") {
        lastCommandTimeV1 = millis();
        if(cmd == CMD_POWER) {
            if(heaterStatus.state == STATE_OFF || heaterStatus.state == 0)
                sendFrame_V1(0x2B, 0xAD, 0x80, 0x00);
            else
                sendFrame_V1(0x2B, 0xAE, 0x00, 0x00);
        } else if(cmd == CMD_UP)   sendFrame_V1(0x2B, 0xA9, 0x00, 0x00);
        else if(cmd == CMD_DOWN)   sendFrame_V1(0x2B, 0xA9, 0x80, 0x00);
        else if(cmd == CMD_MODE)   sendFrame_V1(0x2B, 0xAA, 0x00, 0x00);
        return;
    }
    uint8_t buf[10];
    buf[0] = 9; buf[1] = cmd;
    buf[2] = (heaterAddress >> 24) & 0xFF;
    buf[3] = (heaterAddress >> 16) & 0xFF;
    buf[4] = (heaterAddress >> 8) & 0xFF;
    buf[5] = heaterAddress & 0xFF;
    buf[6] = packetSeq++; buf[9] = 0;
    uint16_t crc = crc16_modbus(buf, 7);
    buf[7] = (crc >> 8) & 0xFF; buf[8] = crc & 0xFF;
    for(int i = 0; i < 10; i++) {
        txBurst(10, buf);
        unsigned long t = millis();
        while(cc1101_readReg(0xF5) != 0x01) {
            delay(1);
            if(millis() - t > 100) return;
        }
    }
}

void rxFlush() { cc1101_strobe(0x36); cc1101_readReg(0xBF); cc1101_strobe(0x3A); delay(16); }
void rxEnable() { cc1101_strobe(0x34); }

bool receivePacket(uint8_t* bytes, uint16_t timeout) {
    unsigned long t = millis();
    uint8_t rxLen;
    rxFlush(); rxEnable();
    while(1) {
        yield();
        if(millis() - t > timeout) return false;
        while(!digitalRead(PIN_GDO2)) { yield(); if(millis() - t > timeout) return false; }
        delay(5);
        rxLen = cc1101_readReg(0xFB);
        if(rxLen >= 23 && rxLen <= 26) break;
        rxFlush(); rxEnable();
    }
    for(int i = 0; i < rxLen; i++) bytes[i] = cc1101_readReg(0xBF);
    rxFlush();
    uint16_t crc = crc16_modbus(bytes, 21);
    uint16_t rxCrc = (bytes[21] << 8) | bytes[22];
    return (crc == rxCrc);
}

// ═══════════════════════════════════════════════════════════════════════════
// HEATER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void updateHeaterStatus() {
    if(!heaterPaired) return;
    sendCommand(CMD_WAKEUP);
    uint8_t buf[32];
    if(receivePacket(buf, 2000)) {
        uint32_t addr = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) |
                        ((uint32_t)buf[4] << 8) | buf[5];
        if(addr == heaterAddress) {
            heaterStatus.state = buf[6];
            heaterStatus.power = buf[7];
            heaterStatus.errorCode = buf[7];
            heaterStatus.voltage = buf[9];
            heaterStatus.ambientTemp = (int8_t)buf[10];
            heaterStatus.caseTemp = buf[12];
            heaterStatus.setpoint = (int8_t)buf[13];
            heaterStatus.autoMode = (buf[14] == 0x32);
            heaterStatus.pumpFreq = buf[15];
            heaterStatus.rssi = (buf[23] - (buf[23] >= 128 ? 256 : 0)) / 2 - 74;
            heaterStatus.lastUpdate = millis();
            addErrorToHistory(heaterStatus.errorCode);
            if(mqttEnabled && mqtt.connected()) publishMQTT();
        }
    }
}

uint32_t findHeater(uint16_t timeout) {
    Serial.println("Searching for heater...");
    displayLine2 = "Pairing..."; updateDisplay();
    uint8_t buf[32];
    if(receivePacket(buf, timeout)) {
        return ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) |
               ((uint32_t)buf[4] << 8) | buf[5];
    }
    return 0;
}

const char* getStateName(uint8_t state) {
    switch(state) {
        case STATE_OFF:         return "OFF";
        case STATE_STARTUP:     return "STARTUP";
        case STATE_WARMING:     return "WARMING";
        case STATE_WARMING_WAIT:return "WARM WAIT";
        case STATE_PRE_RUN:     return "PRE-RUN";
        case STATE_RUNNING:     return "RUNNING";
        case STATE_SHUTDOWN:    return "SHUTDOWN";
        case STATE_SHUTTING_DOWN:return "SHUTTING";
        case STATE_COOLING:     return "COOLING";
        default:                return "UNKNOWN";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// OTA FUNCTIONS  (ArduinoOTA / espota — local network firmware updates)
// ═══════════════════════════════════════════════════════════════════════════

// Initialize OTA subsystem (called from setup after WiFi, and on enable)
void setupOTA() {
    if(!otaEnabled || otaRunning) return;

    ArduinoOTA.setHostname(deviceName.c_str());
    if(otaPassword.length() > 0) ArduinoOTA.setPassword(otaPassword.c_str());

    ArduinoOTA.onStart([]() {
        displayLine1 = "OTA UPDATE";
        displayLine2 = "Starting...";
        displayLine3 = "";
        displayLine4 = "";
        updateDisplay();
        Serial.println("OTA: Update started");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        int pct = total ? (progress * 100) / total : 0;
        displayLine2 = "Flashing " + String(pct) + "%";
        updateDisplay();
    });
    ArduinoOTA.onEnd([]() {
        displayLine2 = "Done!";
        displayLine3 = "Rebooting...";
        updateDisplay();
        Serial.println("\nOTA: Update complete");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        displayLine2 = "OTA FAILED!";
        displayLine3 = "Err " + String(error);
        updateDisplay();
        Serial.printf("OTA: Error[%u]\n", error);
    });

    ArduinoOTA.begin();
    otaRunning = true;
    Serial.println("OTA: Enabled — hostname=" + deviceName +
                   ", password=" + String(otaPassword.length() > 0 ? "[set]" : "[none]"));
}

// Called from loop() — services OTA requests
void loopOTA() {
    if(otaEnabled && otaRunning) ArduinoOTA.handle();
}

// ═══════════════════════════════════════════════════════════════════════════
// MQTT FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for(int i = 0; i < length; i++) message += (char)payload[i];
    String topicStr = String(topic);
    if(topicStr == mqttTopic + "/cmd/power")      sendCommand(CMD_POWER);
    else if(topicStr == mqttTopic + "/cmd/up")    sendCommand(CMD_UP);
    else if(topicStr == mqttTopic + "/cmd/down")  sendCommand(CMD_DOWN);
    else if(topicStr == mqttTopic + "/cmd/mode")  sendCommand(CMD_MODE);
}

void connectMQTT() {
    if(!mqttEnabled || mqttServer.length() == 0) return;
    if(mqtt.connected()) { mqtt.disconnect(); delay(100); }
    mqtt.setServer(mqttServer.c_str(), mqttPort);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(512);
    for(int attempt = 1; attempt <= 3; attempt++) {
        bool ok = mqttAuthEnabled ?
            mqtt.connect(deviceName.c_str(), mqttUser.c_str(), mqttPassword.c_str()) :
            mqtt.connect(deviceName.c_str());
        if(ok) { mqtt.subscribe((mqttTopic + "/cmd/#").c_str()); return; }
        if(attempt < 3) delay(2000);
    }
}

void publishMQTT() {
    if(!mqtt.connected()) return;
    mqtt.publish((mqttTopic + "/state").c_str(),   getStateName(heaterStatus.state));
    mqtt.publish((mqttTopic + "/voltage").c_str(), String(heaterStatus.voltage / 10.0, 1).c_str());
    mqtt.publish((mqttTopic + "/ambient").c_str(), String(heaterStatus.ambientTemp).c_str());
    mqtt.publish((mqttTopic + "/case").c_str(),    String(heaterStatus.caseTemp).c_str());
    mqtt.publish((mqttTopic + "/setpoint").c_str(),String(heaterStatus.setpoint).c_str());
    mqtt.publish((mqttTopic + "/pump").c_str(),    String(heaterStatus.pumpFreq / 10.0, 1).c_str());
    mqtt.publish((mqttTopic + "/mode").c_str(),    heaterStatus.autoMode ? "AUTO" : "MANUAL");
    mqtt.publish((mqttTopic + "/rssi").c_str(),    String(heaterStatus.rssi).c_str());
    mqtt.publish((mqttTopic + "/error").c_str(),   getErrorName(heaterStatus.errorCode));
}

// ═══════════════════════════════════════════════════════════════════════════
// DISPLAY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void updateDisplay() {
#if USE_OLED
    display.clearBuffer();
    display.drawFrame(0, 0, 128, 12);
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(4, 9, displayLine1.c_str());
    display.setFont(u8g2_font_7x13_tf);
    display.drawStr(2, 26, displayLine2.c_str());
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(2, 40, displayLine3.c_str());
    display.setFont(u8g2_font_5x8_tf);
    display.drawStr(2, 52, displayLine4.c_str());

    if(heaterPaired && heaterStatus.lastUpdate > 0) {
        String footer = useAP ? "AP " : "WiFi ";
        if(mqttEnabled && mqtt.connected())   footer += "| MQTT ";
        if(otaEnabled)                        footer += "| OTA";
        display.drawStr(2, 63, footer.c_str());
    }
    display.sendBuffer();
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// WEB SERVER HANDLERS
// ═══════════════════════════════════════════════════════════════════════════

void handleRoot() {
    File file = LittleFS.open("/index.html", "r");
    if(!file) { server.send(404, "text/plain", "File not found!"); return; }
    server.streamFile(file, "text/html");
    file.close();
}

void handleAPI_Status() {
    String json = "{";
    json += "\"state\":\"" + String(getStateName(heaterStatus.state)) + "\",";
    json += "\"voltage\":" + String(heaterStatus.voltage / 10.0, 1) + ",";
    json += "\"ambient\":" + String(heaterStatus.ambientTemp) + ",";
    json += "\"case\":" + String(heaterStatus.caseTemp) + ",";
    json += "\"setpoint\":" + String(heaterStatus.setpoint) + ",";
    json += "\"pump\":" + String(heaterStatus.pumpFreq / 10.0, 1) + ",";
    json += "\"mode\":\"" + String(heaterStatus.autoMode ? "AUTO" : "MANUAL") + "\",";
    json += "\"rssi\":" + String(heaterStatus.rssi) + ",";
    json += "\"addr\":\"" + (heaterPaired ? String(heaterAddress, HEX) : "Not paired") + "\",";
    json += "\"version\":\"" + heaterVersion + "\",";
    json += "\"errorCode\":" + String(heaterStatus.errorCode) + ",";
    json += "\"errorName\":\"" + String(getErrorName(heaterStatus.errorCode)) + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleAPI_OTAStatus() {
    // Returns OTA state for GUI
    String json = "{";
    json += "\"enabled\":" + String(otaEnabled ? "true" : "false") + ",";
    json += "\"running\":" + String(otaRunning ? "true" : "false") + ",";
    json += "\"hasPassword\":" + String(otaPassword.length() > 0 ? "true" : "false") + ",";
    json += "\"hostname\":\"" + deviceName + "\",";
    json += "\"ip\":\"" + (useAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleAPI_OTAConfig() {
    // Toggle OTA ON/OFF and (optionally) set the flashing password.
    otaEnabled = (server.arg("enabled") == "1");

    // Password is authoritative when supplied; empty value clears it (no auth).
    if(server.hasArg("password")) otaPassword = server.arg("password");

    prefs.putBool("otaEnabled", otaEnabled);
    prefs.putString("otaPass", otaPassword);

    // Reboot so ArduinoOTA starts/stops cleanly with the new settings.
    server.send(200, "text/plain", "OTA saved! Rebooting...");
    delay(1000); ESP.restart();
}

void handleAPI_Command() {
    String cmd = server.arg("c");
    if(cmd == "power")      sendCommand(CMD_POWER);
    else if(cmd == "up")    sendCommand(CMD_UP);
    else if(cmd == "down")  sendCommand(CMD_DOWN);
    else if(cmd == "mode")  sendCommand(CMD_MODE);
    server.send(200, "text/plain", "OK");
}

void handleAPI_PairAuto() {
    String ver = server.arg("version");
    String customFreqStr = server.arg("customFreq");
    if(ver.length() == 0) ver = "V2";

    uint32_t newCustomFreq = customFreqStr.length() > 0 ? customFreqStr.toInt() : 0;

    if(ver != heaterVersion || newCustomFreq != customFrequency) {
        heaterVersion = ver; customFrequency = newCustomFreq;
        prefs.putString("heaterVer", heaterVersion);
        prefs.putULong("customFreq", customFrequency);
        if(customFrequency > 0) { cc1101_init(); cc1101_setFrequency(customFrequency); }
        else if(heaterVersion == "V1") cc1101_init_V1();
        else cc1101_init();
    }

    if(heaterVersion == "V1") {
        myAddrV1[0] = random(0x01, 0xFE); myAddrV1[1] = random(0x01, 0xFE); myAddrV1[2] = random(0x01, 0xFE);
        for(int i = 0; i < 20; i++) { sendFrame_V1(0x21, 0x00, 0x00, 0x00); delay(80); }
        prefs.putBytes("addrV1", myAddrV1, 3);
        heaterPaired = true; heaterAddress = 1; prefs.putUInt("heaterAddr", 1);
        char addrStr[16]; sprintf(addrStr, "%02X%02X%02X", myAddrV1[0], myAddrV1[1], myAddrV1[2]);
        server.send(200, "text/plain", "V1 Paired! ID: " + String(addrStr));
    } else {
        uint32_t addr = findHeater(60000);
        if(addr != 0) {
            heaterAddress = addr; heaterPaired = true; prefs.putUInt("heaterAddr", heaterAddress);
            server.send(200, "text/plain", "V2 Paired: 0x" + String(heaterAddress, HEX));
        } else {
            server.send(200, "text/plain", "V2 Pairing failed!");
        }
    }
}

void handleAPI_PairManual() {
    String addrStr = server.arg("addr");
    String ver = server.arg("version");
    String customFreqStr = server.arg("customFreq");
    if(ver.length() == 0) ver = "V2";
    uint32_t newCustomFreq = customFreqStr.length() > 0 ? customFreqStr.toInt() : 0;

    if(ver != heaterVersion || newCustomFreq != customFrequency) {
        heaterVersion = ver; customFrequency = newCustomFreq;
        prefs.putString("heaterVer", heaterVersion);
        prefs.putULong("customFreq", customFrequency);
        if(customFrequency > 0) { cc1101_init(); cc1101_setFrequency(customFrequency); }
        else if(heaterVersion == "V1") cc1101_init_V1();
        else cc1101_init();
    }

    if(heaterVersion == "V1") {
        unsigned long addr = strtoul(addrStr.c_str(), NULL, 0);
        myAddrV1[0] = (addr >> 16) & 0xFF; myAddrV1[1] = (addr >> 8) & 0xFF; myAddrV1[2] = addr & 0xFF;
        prefs.putBytes("addrV1", myAddrV1, 3);
        heaterPaired = true; heaterAddress = 1; prefs.putUInt("heaterAddr", 1);
        char addrOut[16]; sprintf(addrOut, "%02X%02X%02X", myAddrV1[0], myAddrV1[1], myAddrV1[2]);
        server.send(200, "text/plain", "V1 Paired! ID: " + String(addrOut));
    } else {
        heaterAddress = strtoul(addrStr.c_str(), NULL, 0);
        heaterPaired = true; prefs.putUInt("heaterAddr", heaterAddress);
        server.send(200, "text/plain", "V2 Paired: 0x" + String(heaterAddress, HEX));
    }
}

void handleAPI_WiFi() {
    deviceName = server.arg("deviceName");
    if(deviceName.length() == 0) deviceName = "DieselPilot";
    staSSID = server.arg("ssid"); staPassword = server.arg("pass");
    prefs.putString("deviceName", deviceName);
    prefs.putString("staSSID", staSSID); prefs.putString("staPass", staPassword);
    server.send(200, "text/plain", "WiFi saved! Rebooting...");
    delay(1000); ESP.restart();
}

void handleAPI_MQTT() {
    mqttServer = server.arg("server");
    mqttPort = server.arg("port").toInt();
    mqttTopic = server.arg("topic");
    mqttAuthEnabled = (server.arg("authEnabled") == "1");
    mqttUser = server.arg("user"); mqttPassword = server.arg("pass");
    mqttEnabled = (mqttServer.length() > 0);
    prefs.putString("mqttServer", mqttServer); prefs.putInt("mqttPort", mqttPort);
    prefs.putString("mqttTopic", mqttTopic); prefs.putBool("mqttAuthEn", mqttAuthEnabled);
    prefs.putString("mqttUser", mqttUser); prefs.putString("mqttPass", mqttPassword);
    prefs.putBool("mqttEnabled", mqttEnabled);
    server.send(200, "text/plain", "MQTT saved!");
    if(mqttEnabled) connectMQTT();
}

void handleAPI_Info() {
    String json = "{";
    json += "\"hostname\":\"" + deviceName + "\",";
    json += "\"wifiMode\":\"" + String(useAP ? "AP" : "STA") + "\",";
    json += "\"ip\":\"" + (useAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\",";
    json += "\"mqtt\":\"" + String(mqttEnabled && mqtt.connected() ? "Connected" : "Disconnected") + "\",";
    json += "\"ota\":\"" + String(otaEnabled ? "Enabled" : "Disabled") + "\",";
    json += "\"uptime\":\"" + String(millis() / 1000 / 60) + " min\",";
    json += "\"version\":\"" + version + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleAPI_Reboot() {
    server.send(200, "text/plain", "Rebooting...");
    delay(500); ESP.restart();
}

void handleAPI_Factory() {
    server.send(200, "text/plain", "Factory reset in progress...");
    Serial.println("\n⚠️ FACTORY RESET");
    prefs.clear();
    displayLine1 = "FACTORY RESET"; displayLine2 = "Clearing...";
    displayLine3 = "All settings"; displayLine4 = "erased!";
    updateDisplay(); delay(2000);
    ESP.restart();
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n═══════════════════════════════════════");
    Serial.println("    DIESEL PILOT V" + (String)version + " FREE");
    Serial.println("═══════════════════════════════════════\n");

    if(!LittleFS.begin(true)) {
        Serial.println("❌ LittleFS Mount Failed!");
    } else {
        Serial.println("✅ LittleFS mounted");
    }

#if USE_OLED
    Wire.begin(PIN_SDA, PIN_SCL);
    display.begin(); display.setContrast(255);
    displayLine1 = "Diesel Pilot";
    displayLine2 = "V" + version + " Starting";
    displayLine3 = "Made by PPTG";
    displayLine4 = "Happy Heating :)";
    updateDisplay();
    Serial.println("✅ OLED initialized");
    delay(2000);
#endif

    // Load all preferences
    prefs.begin("diesel", false);
    heaterAddress   = prefs.getUInt("heaterAddr", 0);
    heaterPaired    = (heaterAddress != 0);
    heaterVersion   = prefs.getString("heaterVer", "V2");
    customFrequency = prefs.getULong("customFreq", 0);
    deviceName      = prefs.getString("deviceName", "DieselPilot");
    staSSID         = prefs.getString("staSSID", "");
    staPassword     = prefs.getString("staPass", "");
    mqttServer      = prefs.getString("mqttServer", "");
    mqttPort        = prefs.getInt("mqttPort", 1883);
    mqttTopic       = prefs.getString("mqttTopic", "diesel");
    mqttAuthEnabled = prefs.getBool("mqttAuthEn", false);
    mqttUser        = prefs.getString("mqttUser", "");
    mqttPassword    = prefs.getString("mqttPass", "");
    mqttEnabled     = prefs.getBool("mqttEnabled", false);
    // OTA preferences
    otaEnabled      = prefs.getBool("otaEnabled", false);
    otaPassword     = prefs.getString("otaPass", "");

    if(heaterVersion == "V1") {
        if(prefs.getBytes("addrV1", myAddrV1, 3) != 3) {
            myAddrV1[0] = 0x19; myAddrV1[1] = 0x52; myAddrV1[2] = 0x4B;
        }
    }

    // SPI + CC1101
    pinMode(PIN_SCK, OUTPUT); pinMode(PIN_MOSI, OUTPUT);
    pinMode(PIN_MISO, INPUT); pinMode(PIN_SS, OUTPUT); pinMode(PIN_GDO2, INPUT);
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);

    if(customFrequency > 0) { cc1101_init(); cc1101_setFrequency(customFrequency); }
    else if(heaterVersion == "V1") cc1101_init_V1();
    else cc1101_init();

    WiFi.setHostname(deviceName.c_str());

    // WiFi connect
    if(staSSID.length() > 0) {
        Serial.println("Connecting to WiFi: " + staSSID);
        displayLine2 = "WiFi: " + staSSID; updateDisplay();
        WiFi.begin(staSSID.c_str(), staPassword.c_str());
        int attempts = 0;
        while(WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); Serial.print("."); attempts++; }
        if(WiFi.status() == WL_CONNECTED) {
            useAP = false;
            Serial.println("\n✅ WiFi: " + WiFi.localIP().toString());
            displayLine2 = "IP:"; displayLine3 = WiFi.localIP().toString();
        } else {
            Serial.println("\n❌ WiFi failed, starting AP");
            useAP = true;
        }
    }

    if(useAP) {
        WiFi.softAP(apSSID.c_str(), apPassword.c_str());
        Serial.println("✅ AP: " + apSSID + " / " + WiFi.softAPIP().toString());
        displayLine2 = "AP: " + apSSID;
        displayLine3 = WiFi.softAPIP().toString();
    }

    displayLine4 = heaterPaired ? (heaterVersion + " Paired!") : (heaterVersion + " Not paired");
    updateDisplay();

    if(mqttEnabled) connectMQTT();

    // OTA init (after WiFi — works in both AP and STA)
    setupOTA();

    // Register HTTP routes
    server.on("/", handleRoot);
    server.on("/api/status",       handleAPI_Status);
    server.on("/api/info",         handleAPI_Info);
    server.on("/api/cmd",          handleAPI_Command);
    server.on("/api/pair/auto",    handleAPI_PairAuto);
    server.on("/api/pair/manual",  handleAPI_PairManual);
    server.on("/api/wifi",         handleAPI_WiFi);
    server.on("/api/mqtt",         handleAPI_MQTT);
    server.on("/api/factory",      handleAPI_Factory);
    server.on("/api/reboot",       handleAPI_Reboot);
    // OTA endpoints
    server.on("/api/ota/status",   handleAPI_OTAStatus);
    server.on("/api/ota/config",   handleAPI_OTAConfig);
    server.begin();

    Serial.println("\n✅ Ready! V" + version);
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    static unsigned long lastHeaterUpdate = 0;
    static unsigned long lastDisplayUpdate = 0;

    yield();
    server.handleClient();
    // MQTT
    if(mqttEnabled && !mqtt.connected()) {
        if(millis() - lastMQTTRetry > mqttRetryInterval) {
            lastMQTTRetry = millis();
            connectMQTT();
        }
    }
    if(mqttEnabled && mqtt.connected()) mqtt.loop();

    // OTA
    loopOTA();

    // Heater status update
    if(millis() - lastHeaterUpdate > 3000 && heaterPaired) {
        lastHeaterUpdate = millis();
        if(heaterVersion == "V1") updateHeaterStatus_V1();
        else updateHeaterStatus();
    }

    // Display update
    if(millis() - lastDisplayUpdate > 1000) {
        lastDisplayUpdate = millis();

        if(heaterPaired && heaterStatus.lastUpdate > 0) {
            displayLine1 = "DIESEL PILOT " + version;
            displayLine2 = String(getStateName(heaterStatus.state));

            if(heaterStatus.errorCode > 1 && heaterStatus.errorCode != 12) {
                displayLine3 = "! " + String(getErrorName(heaterStatus.errorCode)) + " !";
            } else {
                if(heaterStatus.autoMode) {
                    displayLine3 = String(heaterStatus.ambientTemp) + "C -> " + String(heaterStatus.setpoint) + "C";
                } else {
                    displayLine3 = String(heaterStatus.ambientTemp) + "C  P:" + String(heaterStatus.pumpFreq / 10.0, 1) + "Hz";
                }
            }

            String modeIcon = heaterStatus.autoMode ? "A" : "M";
            displayLine4 = String(heaterStatus.voltage / 10.0, 1) + "V  [" + modeIcon + "]  " + String(heaterStatus.caseTemp) + "C";

        } else if(heaterPaired) {
            displayLine1 = "DIESEL PILOT";
            displayLine2 = "Waiting...";
            displayLine3 = "No data";
            displayLine4 = "";
        }

        updateDisplay();
    }
}
