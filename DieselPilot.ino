/*
 * ═══════════════════════════════════════════════════════════════════════════
 *                        DIESEL PILOT WEB
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
 * 
 * Hardware:
 * - ESP32
 * - CC1101 @ 433.937 MHz
 * - SH1106 OLED (I2C)
 * 
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>

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

// OLED Configuration - set to false if you don't have OLED display
#define USE_OLED  true  // Change to false to disable OLED

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

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

WebServer server(80);
Preferences prefs;
WiFiClient espClient;
PubSubClient mqtt(espClient);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════════════

// WiFi
String apSSID = "Diesel-Pilot";
String apPassword = "12345678";
String staSSID = "";
String staPassword = "";
bool useAP = true;

// MQTT
String mqttServer = "";
int mqttPort = 1883;
String mqttTopic = "diesel";
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
    unsigned long lastUpdate = 0;
} heaterStatus;

// Display
String displayLine1 = "Diesel Pilot";
String displayLine2 = "Initializing...";
String displayLine3 = "";
String displayLine4 = "";

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
    for(int i = 0; i < len; i++) {
        SPI.transfer(bytes[i]);
    }
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
            if((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════════════════
// CC1101 INIT
// ═══════════════════════════════════════════════════════════════════════════

void cc1101_init() {
    cc1101_strobe(0x30); // SRES
    delay(100);
    
    cc1101_writeReg(0x00, 0x07); // IOCFG2
    cc1101_writeReg(0x02, 0x06); // IOCFG0
    cc1101_writeReg(0x03, 0x47); // FIFOTHR
    cc1101_writeReg(0x07, 0x04); // PKTCTRL1
    cc1101_writeReg(0x08, 0x05); // PKTCTRL0
    cc1101_writeReg(0x0A, 0x00); // CHANNR
    cc1101_writeReg(0x0B, 0x06); // FSCTRL1
    cc1101_writeReg(0x0C, 0x00); // FSCTRL0
    
    // 433.937 MHz
    cc1101_writeReg(0x0D, 0x10); // FREQ2
    cc1101_writeReg(0x0E, 0xB0); // FREQ1
    cc1101_writeReg(0x0F, 0x9C); // FREQ0
    
    cc1101_writeReg(0x10, 0xF8); // MDMCFG4
    cc1101_writeReg(0x11, 0x93); // MDMCFG3
    cc1101_writeReg(0x12, 0x13); // MDMCFG2
    cc1101_writeReg(0x13, 0x22); // MDMCFG1
    cc1101_writeReg(0x14, 0xF8); // MDMCFG0
    cc1101_writeReg(0x15, 0x26); // DEVIATN
    cc1101_writeReg(0x17, 0x30); // MCSM1
    cc1101_writeReg(0x18, 0x18); // MCSM0
    cc1101_writeReg(0x19, 0x16); // FOCCFG
    cc1101_writeReg(0x1A, 0x6C); // BSCFG
    cc1101_writeReg(0x1B, 0x03); // AGCTRL2
    cc1101_writeReg(0x1C, 0x40); // AGCTRL1
    cc1101_writeReg(0x1D, 0x91); // AGCTRL0
    cc1101_writeReg(0x20, 0xFB); // WORCTRL
    cc1101_writeReg(0x21, 0x56); // FREND1
    cc1101_writeReg(0x22, 0x17); // FREND0
    cc1101_writeReg(0x23, 0xE9); // FSCAL3
    cc1101_writeReg(0x24, 0x2A); // FSCAL2
    cc1101_writeReg(0x25, 0x00); // FSCAL1
    cc1101_writeReg(0x26, 0x1F); // FSCAL0
    cc1101_writeReg(0x2C, 0x81); // TEST2
    cc1101_writeReg(0x2D, 0x35); // TEST1
    cc1101_writeReg(0x2E, 0x09); // TEST0
    cc1101_writeReg(0x09, 0x00); // ADDR
    cc1101_writeReg(0x04, 0x7E); // SYNC1
    cc1101_writeReg(0x05, 0x3C); // SYNC0
    
    uint8_t paTable[8] = {0x00, 0x12, 0x0E, 0x34, 0x60, 0xC5, 0xC1, 0xC0};
    cc1101_writeBurst(0x7E, 8, paTable);
    
    cc1101_strobe(0x31); // SFSTXON
    cc1101_strobe(0x36); // SIDLE
    cc1101_strobe(0x3B); // SFTX
    cc1101_strobe(0x36); // SIDLE
    cc1101_strobe(0x3A); // SFRX
    delay(136);
    
    Serial.println("✅ CC1101 initialized");
}

// ═══════════════════════════════════════════════════════════════════════════
// TX/RX FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void txFlush() {
    cc1101_strobe(0x36); // SIDLE
    cc1101_strobe(0x3B); // SFTX
    delay(16);
}

void txBurst(uint8_t len, uint8_t* bytes) {
    txFlush();
    cc1101_writeBurst(0x7F, len, bytes);
    cc1101_strobe(0x35); // STX
}

void sendCommand(uint8_t cmd) {
    if(!heaterPaired) return;
    
    uint8_t buf[10];
    buf[0] = 9;
    buf[1] = cmd;
    buf[2] = (heaterAddress >> 24) & 0xFF;
    buf[3] = (heaterAddress >> 16) & 0xFF;
    buf[4] = (heaterAddress >> 8) & 0xFF;
    buf[5] = heaterAddress & 0xFF;
    buf[6] = packetSeq++;
    buf[9] = 0;
    
    uint16_t crc = crc16_modbus(buf, 7);
    buf[7] = (crc >> 8) & 0xFF;
    buf[8] = crc & 0xFF;
    
    for(int i = 0; i < 10; i++) {
        txBurst(10, buf);
        unsigned long t = millis();
        while(cc1101_readReg(0xF5) != 0x01) {
            delay(1);
            if(millis() - t > 100) return;
        }
    }
}

void rxFlush() {
    cc1101_strobe(0x36); // SIDLE
    cc1101_readReg(0xBF); // Dummy read
    cc1101_strobe(0x3A); // SFRX
    delay(16);
}

void rxEnable() {
    cc1101_strobe(0x34); // SRX
}

bool receivePacket(uint8_t* bytes, uint16_t timeout) {
    unsigned long t = millis();
    uint8_t rxLen;
    
    rxFlush();
    rxEnable();
    
    while(1) {
        yield();
        if(millis() - t > timeout) return false;
        
        while(!digitalRead(PIN_GDO2)) {
            yield();
            if(millis() - t > timeout) return false;
        }
        
        delay(5);
        rxLen = cc1101_readReg(0xFB);
        
        if(rxLen >= 23 && rxLen <= 26) break;
        
        rxFlush();
        rxEnable();
    }
    
    for(int i = 0; i < rxLen; i++) {
        bytes[i] = cc1101_readReg(0xBF);
    }
    
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
            heaterStatus.voltage = buf[9];
            heaterStatus.ambientTemp = (int8_t)buf[10];
            heaterStatus.caseTemp = buf[12];
            heaterStatus.setpoint = (int8_t)buf[13];
            heaterStatus.autoMode = (buf[14] == 0x32);
            heaterStatus.pumpFreq = buf[15];
            heaterStatus.rssi = (buf[23] - (buf[23] >= 128 ? 256 : 0)) / 2 - 74;
            heaterStatus.lastUpdate = millis();
            
            // Publish to MQTT
            if(mqttEnabled && mqtt.connected()) {
                publishMQTT();
            }
        }
    }
}

uint32_t findHeater(uint16_t timeout) {
    Serial.println("Searching for heater...");
    displayLine2 = "Pairing...";
    updateDisplay();
    
    uint8_t buf[32];
    if(receivePacket(buf, timeout)) {
        uint32_t addr = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) | 
                        ((uint32_t)buf[4] << 8) | buf[5];
        return addr;
    }
    return 0;
}

const char* getStateName(uint8_t state) {
    switch(state) {
        case STATE_OFF: return "OFF";
        case STATE_STARTUP: return "STARTUP";
        case STATE_WARMING: return "WARMING";
        case STATE_WARMING_WAIT: return "WARM WAIT";
        case STATE_PRE_RUN: return "PRE-RUN";
        case STATE_RUNNING: return "RUNNING";
        case STATE_SHUTDOWN: return "SHUTDOWN";
        case STATE_SHUTTING_DOWN: return "SHUTTING";
        case STATE_COOLING: return "COOLING";
        default: return "UNKNOWN";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// MQTT FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for(int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    String topicStr = String(topic);
    
    if(topicStr == mqttTopic + "/cmd/power") {
        sendCommand(CMD_POWER);
    } else if(topicStr == mqttTopic + "/cmd/up") {
        sendCommand(CMD_UP);
    } else if(topicStr == mqttTopic + "/cmd/down") {
        sendCommand(CMD_DOWN);
    } else if(topicStr == mqttTopic + "/cmd/mode") {
        sendCommand(CMD_MODE);
    }
}

void connectMQTT() {
    if(!mqttEnabled || mqttServer.length() == 0) return;
    
    mqtt.setServer(mqttServer.c_str(), mqttPort);
    mqtt.setCallback(mqttCallback);
    
    if(mqtt.connect("DieselPilot")) {
        Serial.println("✅ MQTT connected");
        mqtt.subscribe((mqttTopic + "/cmd/#").c_str());
    }
}

void publishMQTT() {
    if(!mqtt.connected()) return;
    
    mqtt.publish((mqttTopic + "/state").c_str(), getStateName(heaterStatus.state));
    mqtt.publish((mqttTopic + "/power").c_str(), String(heaterStatus.power).c_str());
    mqtt.publish((mqttTopic + "/voltage").c_str(), String(heaterStatus.voltage / 10.0, 1).c_str());
    mqtt.publish((mqttTopic + "/ambient").c_str(), String(heaterStatus.ambientTemp).c_str());
    mqtt.publish((mqttTopic + "/case").c_str(), String(heaterStatus.caseTemp).c_str());
    mqtt.publish((mqttTopic + "/setpoint").c_str(), String(heaterStatus.setpoint).c_str());
    mqtt.publish((mqttTopic + "/pump").c_str(), String(heaterStatus.pumpFreq / 10.0, 1).c_str());
    mqtt.publish((mqttTopic + "/mode").c_str(), heaterStatus.autoMode ? "AUTO" : "MANUAL");
    mqtt.publish((mqttTopic + "/rssi").c_str(), String(heaterStatus.rssi).c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
// DISPLAY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void updateDisplay() {
#if USE_OLED
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);
    
    display.drawStr(0, 10, displayLine1.c_str());
    display.drawStr(0, 25, displayLine2.c_str());
    display.drawStr(0, 40, displayLine3.c_str());
    display.drawStr(0, 55, displayLine4.c_str());
    
    display.sendBuffer();
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// WEB SERVER HANDLERS  
// ═══════════════════════════════════════════════════════════════════════════

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Diesel Pilot</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: #0a0a0a;
            color: #e0e0e0;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            padding: 20px;
        }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { 
            color: #ff6b00; 
            margin-bottom: 30px;
            text-align: center;
            text-shadow: 0 0 10px rgba(255, 107, 0, 0.5);
        }
        
        /* TABS */
        .tabs {
            display: flex;
            gap: 10px;
            margin-bottom: 20px;
            border-bottom: 2px solid #333;
        }
        .tab {
            background: #1a1a1a;
            border: 1px solid #333;
            border-bottom: none;
            padding: 12px 24px;
            cursor: pointer;
            border-radius: 8px 8px 0 0;
            transition: all 0.3s;
        }
        .tab:hover { background: #252525; }
        .tab.active {
            background: #ff6b00;
            color: white;
            border-color: #ff6b00;
        }
        .tab-content { display: none; }
        .tab-content.active { display: block; }
        
        .card {
            background: #1a1a1a;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
            border: 1px solid #333;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        .card h2 {
            color: #ff6b00;
            margin-bottom: 15px;
            font-size: 1.3em;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        .status-item {
            background: #0f0f0f;
            padding: 15px;
            border-radius: 8px;
            border: 1px solid #2a2a2a;
        }
        .status-label {
            color: #888;
            font-size: 0.85em;
            margin-bottom: 5px;
        }
        .status-value {
            color: #fff;
            font-size: 1.5em;
            font-weight: bold;
        }
        .state-running { color: #4CAF50; }
        .state-off { color: #f44336; }
        .state-other { color: #ff9800; }
        .mode-auto { color: #2196F3; }
        .mode-manual { color: #9C27B0; }
        .btn {
            background: #ff6b00;
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 6px;
            cursor: pointer;
            font-size: 1em;
            margin: 5px;
            transition: all 0.3s;
        }
        .btn:hover {
            background: #ff8533;
            transform: translateY(-2px);
            box-shadow: 0 4px 8px rgba(255, 107, 0, 0.3);
        }
        .btn:active { transform: translateY(0); }
        .btn-group {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
            margin-top: 15px;
        }
        input, select {
            background: #0f0f0f;
            border: 1px solid #333;
            color: #e0e0e0;
            padding: 10px;
            border-radius: 6px;
            width: 100%;
            margin: 5px 0;
        }
        input:focus, select:focus {
            outline: none;
            border-color: #ff6b00;
        }
        .form-row {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin: 10px 0;
        }
        .info-box {
            margin-top:15px; 
            padding:10px; 
            background:#0f0f0f; 
            border-radius:6px; 
            border:1px solid #2a2a2a; 
            font-size:0.9em; 
            color:#888;
        }
        .info-box strong { color:#ff6b00; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔥 DIESEL PILOT</h1>
        
        <!-- TABS -->
        <div class="tabs">
            <div class="tab active" onclick="switchTab('dashboard')">📊 Dashboard</div>
            <div class="tab" onclick="switchTab('config')">⚙️ Config</div>
        </div>
        
        <!-- DASHBOARD TAB -->
        <div id="dashboard" class="tab-content active">
            
            <!-- STATUS -->
            <div class="card">
                <h2>📊 Status</h2>
                <div class="status-grid">
                    <div class="status-item">
                        <div class="status-label">State</div>
                        <div class="status-value" id="state">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Mode</div>
                        <div class="status-value" id="mode">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Voltage</div>
                        <div class="status-value" id="voltage">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Ambient</div>
                        <div class="status-value" id="ambient">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Case</div>
                        <div class="status-value" id="case">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label" id="setpointLabel">Setpoint</div>
                        <div class="status-value" id="setpoint">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label" id="pumpLabel">Pump</div>
                        <div class="status-value" id="pump">-</div>
                    </div>
                </div>
            </div>
            
            <!-- CONTROLS -->
            <div class="card">
                <h2>🎮 Controls</h2>
                <div class="btn-group">
                    <button class="btn" onclick="sendCmd('power')">⚡ POWER</button>
                    <button class="btn" onclick="sendCmd('up')">⬆️ UP</button>
                    <button class="btn" onclick="sendCmd('down')">⬇️ DOWN</button>
                    <button class="btn" onclick="sendCmd('mode')">🔄 MODE</button>
                </div>
                <div class="info-box">
                    <strong>💡 Info:</strong><br>
                    • <strong>AUTO mode</strong>: UP/DOWN adjust <strong>temperature</strong> setpoint<br>
                    • <strong>MANUAL mode</strong>: UP/DOWN adjust <strong>pump frequency</strong><br>
                    • Press MODE to switch between AUTO ↔ MANUAL
                </div>
            </div>
            
        </div>
        
        <!-- CONFIG TAB -->
        <div id="config" class="tab-content">
            
            <!-- PAIRING -->
            <div class="card">
                <h2>🔗 Pairing</h2>
                <button class="btn" onclick="autoPair()">🔍 AUTO PAIR</button>
                <div class="form-row">
                    <input type="text" id="manualAddr" placeholder="0xCA00445B">
                    <button class="btn" onclick="manualPair()">✏️ MANUAL PAIR</button>
                </div>
                <div style="margin-top:10px; color:#888;">
                    Current: <span id="currentAddr">Not paired</span>
                </div>
                <div class="info-box">
                    <strong>📡 Auto Pairing:</strong> Hold pairing button on heater panel for 60s<br>
                    <strong>✏️ Manual Pairing:</strong> Enter heater address (find with RTL-SDR)
                </div>
            </div>
            
            <!-- WIFI CONFIG -->
            <div class="card">
                <h2>📡 WiFi Configuration</h2>
                <input type="text" id="wifiSSID" placeholder="Your WiFi SSID">
                <input type="password" id="wifiPass" placeholder="WiFi Password">
                <button class="btn" onclick="saveWiFi()">💾 SAVE & REBOOT</button>
                <div class="info-box">
                    <strong>ℹ️ Note:</strong> After saving, ESP32 will reboot and connect to your WiFi.<br>
                    If connection fails, it will fall back to AP mode.
                </div>
            </div>
            
            <!-- MQTT CONFIG -->
            <div class="card">
                <h2>📨 MQTT Configuration</h2>
                <input type="text" id="mqttServer" placeholder="MQTT Broker IP (e.g., 192.168.1.100)">
                <div class="form-row">
                    <input type="number" id="mqttPort" placeholder="Port" value="1883">
                    <input type="text" id="mqttTopic" placeholder="Topic (e.g., diesel)">
                </div>
                <button class="btn" onclick="saveMQTT()">💾 SAVE MQTT</button>
                <div class="info-box">
                    <strong>🏠 Home Assistant:</strong> Configure MQTT broker IP and topic.<br>
                    Topics: <code>topic/state</code>, <code>topic/cmd/power</code>, etc.
                </div>
            </div>
            
            <!-- SYSTEM INFO -->
            <div class="card">
                <h2>ℹ️ System Info</h2>
                <div style="font-family: monospace; font-size: 0.9em; line-height: 1.6;">
                    <div>WiFi Mode: <span id="wifiMode">-</span></div>
                    <div>IP Address: <span id="ipAddr">-</span></div>
                    <div>MQTT: <span id="mqttStatus">-</span></div>
                    <div>Uptime: <span id="uptime">-</span></div>
                </div>
            </div>
            
        </div>
    </div>
    
    <script>
        // TAB SWITCHING
        function switchTab(tabName) {
            // Hide all tabs
            document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
            
            // Show selected tab
            document.getElementById(tabName).classList.add('active');
            event.target.classList.add('active');
        }
        
        // STATUS UPDATE
        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(d => {
                    // State with color
                    document.getElementById('state').innerText = d.state;
                    let stateEl = document.getElementById('state');
                    stateEl.className = 'status-value ' + 
                        (d.state === 'RUNNING' ? 'state-running' : 
                         d.state === 'OFF' ? 'state-off' : 'state-other');
                    
                    // Mode with color
                    document.getElementById('mode').innerText = d.mode;
                    let modeEl = document.getElementById('mode');
                    modeEl.className = 'status-value ' + 
                        (d.mode === 'AUTO' ? 'mode-auto' : 'mode-manual');
                    
                    // Basic values
                    document.getElementById('voltage').innerText = d.voltage + 'V';
                    document.getElementById('ambient').innerText = d.ambient + '°C';
                    document.getElementById('case').innerText = d.case + '°C';
                    document.getElementById('currentAddr').innerText = d.addr;
                    
                    // Smart labels based on mode
                    if(d.mode === 'AUTO') {
                        document.getElementById('setpointLabel').innerText = 'Setpoint (°C)';
                        document.getElementById('pumpLabel').innerText = 'Pump (actual)';
                        document.getElementById('setpoint').innerText = d.setpoint + '°C';
                        document.getElementById('pump').innerText = d.pump + 'Hz';
                    } else {
                        document.getElementById('setpointLabel').innerText = 'Last Setpoint';
                        document.getElementById('pumpLabel').innerText = 'Pump (set)';
                        document.getElementById('setpoint').innerText = d.setpoint + '°C';
                        document.getElementById('pump').innerText = d.pump + 'Hz';
                    }
                });
        }
        
        // SYSTEM INFO UPDATE
        function updateSystemInfo() {
            fetch('/api/info')
                .then(r => r.json())
                .then(d => {
                    document.getElementById('wifiMode').innerText = d.wifiMode;
                    document.getElementById('ipAddr').innerText = d.ip;
                    document.getElementById('mqttStatus').innerText = d.mqtt;
                    document.getElementById('uptime').innerText = d.uptime;
                });
        }
        
        function sendCmd(cmd) {
            fetch('/api/cmd?c=' + cmd).then(() => setTimeout(updateStatus, 500));
        }
        
        function autoPair() {
            if(confirm('Start auto pairing? Hold pairing button on heater!')) {
                fetch('/api/pair/auto').then(r => r.text()).then(alert);
            }
        }
        
        function manualPair() {
            let addr = document.getElementById('manualAddr').value;
            fetch('/api/pair/manual?addr=' + addr).then(r => r.text()).then(alert);
        }
        
        function saveWiFi() {
            let ssid = document.getElementById('wifiSSID').value;
            let pass = document.getElementById('wifiPass').value;
            if(confirm('Save WiFi config and reboot?')) {
                fetch('/api/wifi?ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass))
                    .then(r => r.text()).then(msg => {
                        alert(msg);
                        setTimeout(() => location.reload(), 2000);
                    });
            }
        }
        
        function saveMQTT() {
            let server = document.getElementById('mqttServer').value;
            let port = document.getElementById('mqttPort').value;
            let topic = document.getElementById('mqttTopic').value;
            fetch('/api/mqtt?server=' + server + '&port=' + port + '&topic=' + topic)
                .then(r => r.text()).then(alert);
        }
        
        // Auto-refresh
        setInterval(updateStatus, 2000);
        setInterval(updateSystemInfo, 5000);
        updateStatus();
        updateSystemInfo();
    </script>
</body>
</html>
)rawliteral";
    
    server.send(200, "text/html", html);
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
    json += "\"addr\":\"" + (heaterPaired ? String(heaterAddress, HEX) : "Not paired") + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleAPI_Command() {
    String cmd = server.arg("c");
    if(cmd == "power") sendCommand(CMD_POWER);
    else if(cmd == "up") sendCommand(CMD_UP);
    else if(cmd == "down") sendCommand(CMD_DOWN);
    else if(cmd == "mode") sendCommand(CMD_MODE);
    server.send(200, "text/plain", "OK");
}

void handleAPI_PairAuto() {
    uint32_t addr = findHeater(60000);
    if(addr != 0) {
        heaterAddress = addr;
        heaterPaired = true;
        prefs.putUInt("heaterAddr", heaterAddress);
        server.send(200, "text/plain", "Paired: 0x" + String(heaterAddress, HEX));
    } else {
        server.send(200, "text/plain", "Pairing failed!");
    }
}

void handleAPI_PairManual() {
    String addrStr = server.arg("addr");
    heaterAddress = strtoul(addrStr.c_str(), NULL, 0);
    heaterPaired = true;
    prefs.putUInt("heaterAddr", heaterAddress);
    server.send(200, "text/plain", "Paired: 0x" + String(heaterAddress, HEX));
}

void handleAPI_WiFi() {
    staSSID = server.arg("ssid");
    staPassword = server.arg("pass");
    prefs.putString("staSSID", staSSID);
    prefs.putString("staPass", staPassword);
    server.send(200, "text/plain", "WiFi saved! Reboot to connect.");
}

void handleAPI_MQTT() {
    mqttServer = server.arg("server");
    mqttPort = server.arg("port").toInt();
    mqttTopic = server.arg("topic");
    mqttEnabled = (mqttServer.length() > 0);
    
    prefs.putString("mqttServer", mqttServer);
    prefs.putInt("mqttPort", mqttPort);
    prefs.putString("mqttTopic", mqttTopic);
    prefs.putBool("mqttEnabled", mqttEnabled);
    
    server.send(200, "text/plain", "MQTT saved! Reconnecting...");
    
    if(mqttEnabled) {
        connectMQTT();
    }
}

void handleAPI_Info() {
    String json = "{";
    json += "\"wifiMode\":\"" + String(useAP ? "AP" : "STA") + "\",";
    json += "\"ip\":\"" + (useAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\",";
    json += "\"mqtt\":\"" + String(mqttEnabled && mqtt.connected() ? "Connected" : "Disconnected") + "\",";
    json += "\"uptime\":\"" + String(millis() / 1000 / 60) + " min\"";
    json += "}";
    server.send(200, "application/json", json);
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n═══════════════════════════════════════");
    Serial.println("    DIESEL PILOT WEB");
    Serial.println("═══════════════════════════════════════\n");
    
    // Init OLED (if enabled)
#if USE_OLED
    Wire.begin(PIN_SDA, PIN_SCL);
    display.begin();
    display.setContrast(255);
    displayLine1 = "Diesel Pilot";
    displayLine2 = "Starting...";
    updateDisplay();
    Serial.println("✅ OLED initialized");
#else
    Serial.println("ℹ️  OLED disabled");
#endif
    
    // Init Preferences
    prefs.begin("diesel", false);
    heaterAddress = prefs.getUInt("heaterAddr", 0);
    heaterPaired = (heaterAddress != 0);
    staSSID = prefs.getString("staSSID", "");
    staPassword = prefs.getString("staPass", "");
    mqttServer = prefs.getString("mqttServer", "");
    mqttPort = prefs.getInt("mqttPort", 1883);
    mqttTopic = prefs.getString("mqttTopic", "diesel");
    mqttEnabled = prefs.getBool("mqttEnabled", false);
    
    // Init CC1101
    pinMode(PIN_SCK, OUTPUT);
    pinMode(PIN_MOSI, OUTPUT);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_SS, OUTPUT);
    pinMode(PIN_GDO2, INPUT);
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
    cc1101_init();
    
    // Init WiFi
    if(staSSID.length() > 0) {
        Serial.println("Connecting to WiFi: " + staSSID);
        displayLine2 = "WiFi: " + staSSID;
        updateDisplay();
        
        WiFi.begin(staSSID.c_str(), staPassword.c_str());
        int attempts = 0;
        while(WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        if(WiFi.status() == WL_CONNECTED) {
            useAP = false;
            Serial.println("\n✅ WiFi connected!");
            Serial.println("IP: " + WiFi.localIP().toString());
            displayLine2 = "IP: ";
            displayLine3 = WiFi.localIP().toString();
        } else {
            Serial.println("\n❌ WiFi failed, starting AP");
            useAP = true;
        }
    }
    
    if(useAP) {
        WiFi.softAP(apSSID.c_str(), apPassword.c_str());
        Serial.println("✅ AP started");
        Serial.println("SSID: " + apSSID);
        Serial.println("Pass: " + apPassword);
        Serial.println("IP: " + WiFi.softAPIP().toString());
        displayLine2 = "AP: " + apSSID;
        displayLine3 = WiFi.softAPIP().toString();
    }
    
    displayLine4 = heaterPaired ? "Paired!" : "Not paired";
    updateDisplay();
    
    // Init MQTT
    if(mqttEnabled) {
        connectMQTT();
    }
    
    // Init Web Server
    server.on("/", handleRoot);
    server.on("/api/status", handleAPI_Status);
    server.on("/api/info", handleAPI_Info);
    server.on("/api/cmd", handleAPI_Command);
    server.on("/api/pair/auto", handleAPI_PairAuto);
    server.on("/api/pair/manual", handleAPI_PairManual);
    server.on("/api/wifi", handleAPI_WiFi);
    server.on("/api/mqtt", handleAPI_MQTT);
    server.begin();
    
    Serial.println("\n✅ Web server started");
    Serial.println("Ready!");
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastDisplay = 0;
    
    yield();
    server.handleClient();
    
    if(mqttEnabled && !mqtt.connected()) {
        connectMQTT();
    }
    if(mqttEnabled) {
        mqtt.loop();
    }
    
    // Update heater status every 3 seconds
    if(millis() - lastUpdate > 3000 && heaterPaired) {
        lastUpdate = millis();
        updateHeaterStatus();
    }
    
    // Update display every 1 second
    if(millis() - lastDisplay > 1000) {
        lastDisplay = millis();
        
        if(heaterPaired && heaterStatus.lastUpdate > 0) {
            displayLine2 = String(getStateName(heaterStatus.state));
            displayLine4 = String(heaterStatus.ambientTemp) + "C/" + 
                          String(heaterStatus.setpoint) + "C " +
                          String(heaterStatus.voltage / 10.0, 1) + "V";
        }
        
        updateDisplay();
    }
}

