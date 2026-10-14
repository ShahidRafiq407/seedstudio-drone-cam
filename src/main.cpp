#include <Arduino.h>
#include <driver/i2s.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_sleep.h>

// ==========================================
// 1. WI-FI CREDENTIALS
// ==========================================
const char* WIFI_SSID     = "Robotics";
const char* WIFI_PASSWORD = "09876541";

// Standalone SoftAP for remote/field access anywhere in the world
const char* AP_SSID       = "XIAO-Audio-Surveillance";
const char* AP_PASSWORD   = "12345678";

// ==========================================
// 2. PIN DEFINITIONS (Seeed Studio XIAO ESP32S3 Sense)
// ==========================================
// Built-in PDM Digital Microphone Pins
#define I2S_MIC_CHANNEL   I2S_NUM_0
#define I2S_MIC_CLK_PIN   42
#define I2S_MIC_DATA_PIN  41
#define AUDIO_SAMPLE_RATE 16000

// MicroSD Card Pins (SPI)
#define SD_CS_PIN         21
#define SD_SCK_PIN        7
#define SD_MISO_PIN       8
#define SD_MOSI_PIN       9

// Battery Voltage ADC Pin (A0 / GPIO 1)
#define BATTERY_ADC_PIN   1

// ==========================================
// 3. SYSTEM STATE VARIABLES
// ==========================================
enum SystemMode {
  MODE_STANDBY,     // Low power / Light Sleep (80MHz, I2S paused)
  MODE_MIC          // Live High-Fidelity Audio Listening (240MHz, I2S active)
};

volatile SystemMode currentMode = MODE_STANDBY;
volatile bool isRecordingSD = false;
bool sdCardAvailable = false;
bool micInitialized = false;

File sdAudioFile;
uint32_t sdBytesWritten = 0;
unsigned long sdRecordStartTime = 0;

WebServer server(80);

// ==========================================
// 4. AUDIO WAV HEADER STRUCT (Standard 44 bytes)
// ==========================================
struct WavHeader {
  char riff[4] = {'R', 'I', 'F', 'F'};
  uint32_t chunkSize;
  char wave[4] = {'W', 'A', 'V', 'E'};
  char fmt[4] = {'f', 'm', 't', ' '};
  uint32_t subchunk1Size = 16;
  uint16_t audioFormat = 1; // PCM
  uint16_t numChannels = 1; // Mono
  uint32_t sampleRate = AUDIO_SAMPLE_RATE;
  uint32_t byteRate = AUDIO_SAMPLE_RATE * 1 * 2;
  uint16_t blockAlign = 2;
  uint16_t bitsPerSample = 16;
  char data[4] = {'d', 'a', 't', 'a'};
  uint32_t dataSize;
};

// ==========================================
// 5. HARDWARE INITIALIZATION
// ==========================================
bool initMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = -1,
    .ws_io_num = I2S_MIC_CLK_PIN,
    .data_out_num = -1,
    .data_in_num = I2S_MIC_DATA_PIN
  };

  esp_err_t err = i2s_driver_install(I2S_MIC_CHANNEL, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("[MIC] I2S install failed: 0x%x\n", err);
    return false;
  }
  err = i2s_set_pin(I2S_MIC_CHANNEL, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("[MIC] I2S pin set failed: 0x%x\n", err);
    return false;
  }
  Serial.println("[MIC] PDM Microphone Initialized Successfully (16kHz Mono)");
  return true;
}

bool initSD() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[SD] MicroSD Card Mount Failed (Not inserted or FAT32 needed)");
    return false;
  }
  Serial.printf("[SD] MicroSD Initialized. Total Space: %llu MB\n", SD.totalBytes() / (1024 * 1024));
  if (!SD.exists("/rec")) {
    SD.mkdir("/rec");
  }
  return true;
}

// ==========================================
// 6. POWER & RECORDING CONTROL
// ==========================================
void setPowerMode(SystemMode mode) {
  currentMode = mode;
  if (mode == MODE_STANDBY) {
    // Light Sleep / Standby: Throttle CPU to 80MHz, enable Wi-Fi modem sleep, pause I2S DMA
    setCpuFrequencyMhz(80);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    i2s_stop(I2S_MIC_CHANNEL);
    Serial.println("[POWER] Entered Light Sleep / Standby (80MHz, Modem Sleep, I2S Paused)");
  } else {
    // Active Listening: Boost CPU to 240MHz, disable Wi-Fi power save, start I2S DMA
    setCpuFrequencyMhz(240);
    esp_wifi_set_ps(WIFI_PS_NONE);
    i2s_start(I2S_MIC_CHANNEL);
    Serial.println("[POWER] Boosted to High Performance Listening (240MHz, I2S Active)");
  }
}

void closeSDRecording() {
  if (!isRecordingSD || !sdAudioFile) return;

  WavHeader finalHeader;
  finalHeader.dataSize = sdBytesWritten;
  finalHeader.chunkSize = sdBytesWritten + sizeof(WavHeader) - 8;

  sdAudioFile.seek(0);
  sdAudioFile.write((const uint8_t*)&finalHeader, sizeof(WavHeader));
  sdAudioFile.close();
  isRecordingSD = false;
  Serial.printf("[SD] Audio recording saved! Size: %u bytes\n", sdBytesWritten);
}

void startSDRecording() {
  if (!sdCardAvailable) return;
  if (!SD.exists("/rec")) SD.mkdir("/rec");

  // Find next available filename
  char filename[32];
  for (uint32_t i = 1; i < 9999; i++) {
    snprintf(filename, sizeof(filename), "/rec/AUD_%04u.WAV", i);
    if (!SD.exists(filename)) break;
  }

  sdAudioFile = SD.open(filename, FILE_WRITE);
  if (!sdAudioFile) {
    Serial.println("[SD] Failed to create WAV file");
    return;
  }

  WavHeader dummyHeader;
  sdAudioFile.write((const uint8_t*)&dummyHeader, sizeof(WavHeader));
  sdBytesWritten = 0;
  sdRecordStartTime = millis();
  isRecordingSD = true;
  Serial.printf("[SD] Recording audio to %s\n", filename);
}

void triggerDeepSleep(uint32_t sleepMinutes = 10) {
  Serial.printf("[SLEEP] Entering Deep Sleep for %u minutes...\n", sleepMinutes);
  if (isRecordingSD) {
    closeSDRecording();
  }
  i2s_stop(I2S_MIC_CHANNEL);
  i2s_driver_uninstall(I2S_MIC_CHANNEL);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  uint64_t sleepUs = (uint64_t)sleepMinutes * 60ULL * 1000000ULL;
  esp_sleep_enable_timer_wakeup(sleepUs);
  esp_deep_sleep_start();
}

// Battery ADC Measurement
struct BatteryInfo {
  float voltage;
  int percentage;
  bool isConnected;
  String status;
};

BatteryInfo readBattery() {
  BatteryInfo b;
  uint32_t mv = analogReadMilliVolts(BATTERY_ADC_PIN);
  uint32_t batMv = mv * 2; // 2:1 divider ratio

  if (batMv >= 2700 && batMv <= 4400) {
    b.isConnected = true;
    b.voltage = batMv / 1000.0f;
    if (b.voltage >= 4.15f) b.percentage = 100;
    else if (b.voltage >= 4.05f) b.percentage = 90;
    else if (b.voltage >= 3.95f) b.percentage = 78;
    else if (b.voltage >= 3.85f) b.percentage = 60;
    else if (b.voltage >= 3.75f) b.percentage = 40;
    else if (b.voltage >= 3.65f) b.percentage = 20;
    else if (b.voltage >= 3.50f) b.percentage = 10;
    else b.percentage = 2;

    if (b.percentage > 50) b.status = "Good";
    else if (b.percentage > 20) b.status = "Moderate";
    else b.status = "Low Battery";
  } else {
    b.isConnected = false;
    b.voltage = 0.0f;
    b.percentage = -1;
    b.status = "No Divider on A0";
  }
  return b;
}

// ==========================================
// 7. DASHBOARD HTML & UI
// ==========================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>XIAO Audio Surveillance Station</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@500;700&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg: #090d16;
      --card-bg: rgba(18, 24, 38, 0.85);
      --card-border: rgba(255, 255, 255, 0.08);
      --accent: #00f2fe;
      --accent-glow: rgba(0, 242, 254, 0.35);
      --green: #0aff96;
      --green-glow: rgba(10, 255, 150, 0.35);
      --red: #ff3366;
      --text: #f0f4f8;
      --text-muted: #8e9bb0;
      --yellow: #ffd166;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background-color: var(--bg);
      background-image: radial-gradient(circle at 50% 0%, rgba(0, 242, 254, 0.12) 0%, transparent 60%);
      color: var(--text);
      font-family: 'Inter', sans-serif;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 16px;
    }
    .container {
      width: 100%;
      max-width: 540px;
      display: flex;
      flex-direction: column;
      gap: 14px;
    }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 10px 4px;
    }
    .title-box h1 {
      font-size: 1.18rem;
      font-weight: 700;
      letter-spacing: -0.5px;
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .badge {
      font-size: 0.72rem;
      font-weight: 700;
      padding: 4px 10px;
      border-radius: 20px;
      border: 1px solid var(--accent);
      color: var(--accent);
      background: var(--accent-glow);
      text-transform: uppercase;
      letter-spacing: 0.8px;
    }
    .badge-online {
      background: rgba(10, 255, 150, 0.15);
      color: var(--green);
      border: 1px solid var(--green);
    }
    .badge-offline {
      background: rgba(255, 51, 102, 0.15);
      color: var(--red);
      border: 1px solid var(--red);
    }
    .badge-rec {
      background: rgba(255, 51, 102, 0.25);
      color: var(--red);
      border-color: var(--red);
      animation: pulse 1.5s infinite;
    }
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.4; }
    }

    .audio-card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 20px;
      padding: 18px;
      backdrop-filter: blur(16px);
      box-shadow: 0 12px 30px rgba(0,0,0,0.4);
      display: flex;
      flex-direction: column;
      gap: 14px;
    }
    .audio-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.88rem;
      font-weight: 600;
    }
    .audio-indicator {
      font-family: 'JetBrains Mono', monospace;
      font-size: 0.75rem;
      padding: 3px 8px;
      border-radius: 6px;
      background: rgba(255,255,255,0.06);
      color: var(--text-muted);
    }
    canvas {
      width: 100%;
      height: 70px;
      background: rgba(0, 0, 0, 0.4);
      border-radius: 12px;
      border: 1px solid rgba(255,255,255,0.05);
    }

    .vu-meter-box {
      display: flex;
      flex-direction: column;
      gap: 6px;
    }
    .vu-bar-bg {
      width: 100%;
      height: 8px;
      background: rgba(255,255,255,0.08);
      border-radius: 4px;
      overflow: hidden;
      position: relative;
    }
    .vu-bar-fill {
      width: 0%;
      height: 100%;
      background: linear-gradient(90deg, #0aff96, #ffd166, #ff3366);
      border-radius: 4px;
      transition: width 0.08s ease;
    }
    .vu-labels {
      display: flex;
      justify-content: space-between;
      font-size: 0.72rem;
      color: var(--text-muted);
    }

    .control-row {
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    .slider-box {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.8rem;
      color: var(--text-muted);
    }
    input[type=range] {
      width: 100%;
      accent-color: var(--accent);
      cursor: pointer;
    }

    .filter-pills {
      display: flex;
      gap: 6px;
      flex-wrap: wrap;
    }
    .pill-btn {
      flex: 1;
      padding: 6px 10px;
      font-size: 0.75rem;
      font-weight: 600;
      border-radius: 10px;
      border: 1px solid var(--card-border);
      background: rgba(255,255,255,0.05);
      color: var(--text);
      cursor: pointer;
      text-align: center;
      transition: all 0.2s;
    }
    .pill-btn.active {
      border-color: var(--accent);
      background: var(--accent-glow);
      color: #fff;
    }

    .phone-rec-btn {
      background: linear-gradient(135deg, rgba(255, 51, 102, 0.2), rgba(255, 51, 102, 0.4));
      border: 1px solid var(--red);
      color: #fff;
      padding: 12px;
      border-radius: 14px;
      font-size: 0.88rem;
      font-weight: 700;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      transition: all 0.2s;
      width: 100%;
    }
    .phone-rec-btn.recording {
      background: var(--red);
      color: #fff;
      box-shadow: 0 0 20px rgba(255, 51, 102, 0.5);
      animation: pulse 1s infinite;
    }

    .controls-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 8px;
    }
    .btn {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      color: var(--text);
      padding: 12px 6px;
      border-radius: 14px;
      font-size: 0.78rem;
      font-weight: 600;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 6px;
      cursor: pointer;
      transition: all 0.2s ease;
    }
    .btn span { font-size: 1.3rem; }
    .btn:hover { background: rgba(255, 255, 255, 0.08); }
    .btn.active {
      border-color: var(--accent);
      background: var(--accent-glow);
      color: #fff;
      box-shadow: 0 0 15px var(--accent-glow);
    }
    .btn.rec-btn.active {
      border-color: var(--red);
      background: rgba(255, 51, 102, 0.25);
      color: var(--red);
      animation: pulse 1.2s infinite;
    }

    .telemetry {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
    }
    .tele-item {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      padding: 10px 14px;
      border-radius: 12px;
      display: flex;
      flex-direction: column;
      gap: 2px;
    }
    .tele-label {
      font-size: 0.68rem;
      text-transform: uppercase;
      letter-spacing: 0.6px;
      color: var(--text-muted);
    }
    .tele-val {
      font-family: 'JetBrains Mono', monospace;
      font-size: 0.85rem;
      font-weight: 700;
    }

    .sd-card-panel {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 16px;
      padding: 14px;
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    .sd-card-panel h3 {
      font-size: 0.85rem;
      display: flex;
      justify-content: space-between;
      align-items: center;
      color: var(--text);
    }
    .file-list {
      max-height: 180px;
      overflow-y: auto;
      display: flex;
      flex-direction: column;
      gap: 6px;
    }
    .file-item {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 8px 10px;
      background: rgba(0,0,0,0.3);
      border-radius: 8px;
      font-size: 0.78rem;
      font-family: 'JetBrains Mono', monospace;
    }
    .file-actions { display: flex; gap: 8px; }
    .file-actions a, .file-actions button {
      background: rgba(255,255,255,0.08);
      border: none;
      color: var(--accent);
      padding: 4px 8px;
      border-radius: 6px;
      font-size: 0.72rem;
      cursor: pointer;
      text-decoration: none;
    }
    .file-actions button.del-btn { color: var(--red); }

    .modal-overlay {
      position: fixed;
      top: 0; left: 0; width: 100%; height: 100%;
      background: rgba(0,0,0,0.8);
      display: flex;
      align-items: center;
      justify-content: center;
      z-index: 100;
    }
    .modal-content {
      background: #161e2e;
      border: 1px solid var(--card-border);
      border-radius: 16px;
      padding: 20px;
      width: 90%;
      max-width: 380px;
    }
    .sleep-opts {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
      margin: 12px 0;
    }
    .opt-btn {
      background: rgba(255,255,255,0.06);
      border: 1px solid var(--card-border);
      color: #fff;
      padding: 10px;
      border-radius: 10px;
      font-size: 0.82rem;
      font-weight: 600;
      cursor: pointer;
    }
    .opt-btn:hover, .opt-btn.active {
      background: var(--accent);
      color: #000;
      border-color: var(--accent);
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <div class="title-box">
        <h1>🎙️ XIAO Audio Surveillance</h1>
      </div>
      <div style="display: flex; gap: 8px; align-items: center;">
        <div id="connBadge" class="badge badge-online">🟢 ONLINE</div>
        <div id="statusBadge" class="badge">Standby</div>
      </div>
    </header>

    <div class="audio-card">
      <div class="audio-header">
        <span>🎙️ 16kHz PDM Microphone Stream</span>
        <span id="audioStatus" class="audio-indicator">Standby</span>
      </div>

      <canvas id="audioCanvas"></canvas>

      <div class="vu-meter-box">
        <div class="vu-bar-bg">
          <div id="vuFill" class="vu-bar-fill"></div>
        </div>
        <div class="vu-labels">
          <span id="vuLevelTxt">Level: Silent</span>
          <span id="vuDbtxt">-- dB</span>
        </div>
      </div>

      <div class="control-row">
        <div class="slider-box">
          <span>🔊 Whisper Sensitivity: <strong id="volVal" style="color:var(--accent);">3.5x</strong></span>
          <span id="whisperTag" style="color:var(--yellow); font-size:0.75rem;">Whisper Ready</span>
        </div>
        <input type="range" id="volBoost" min="1" max="8" step="0.5" value="3.5" oninput="updateVolume(this.value)">
      </div>

      <div class="control-row">
        <div class="slider-box">
          <span>🍃 Noise & Frequency Filter:</span>
        </div>
        <div class="filter-pills">
          <button id="pillWhisper" class="pill-btn active" onclick="setFilterMode('whisper')">🍃 Whisper-Safe (140Hz HPF)</button>
          <button id="pillVoice" class="pill-btn" onclick="setFilterMode('voice')">🎯 Voice Focus (Bandpass)</button>
          <button id="pillOff" class="pill-btn" onclick="setFilterMode('off')">🔊 Raw (No Filter)</button>
        </div>
      </div>

      <div class="control-row">
        <button id="btnCompressor" class="pill-btn active" style="padding:8px;" onclick="toggleCompressor()">
          🎛️ Broadcast Speech Compressor: ON (Boosts Whispers, Tames Shouts)
        </button>
      </div>

      <button id="btnPhoneRec" class="phone-rec-btn" onclick="togglePhoneRecording()">
        <span id="phoneRecIcon">⏺️</span>
        <span id="phoneRecText">Record Audio to Phone (Lossless .WAV)</span>
      </button>
    </div>

    <div class="controls-grid">
      <button id="btnListen" class="btn" onclick="toggleListening()">
        <span>🎙️</span>
        Listen
      </button>
      <button id="btnStandby" class="btn active" onclick="setDeviceMode('standby')">
        <span>🍃</span>
        Light Sleep
      </button>
      <button id="btnRecSD" class="btn rec-btn" onclick="toggleSDRecording()">
        <span>💾</span>
        SD Record
      </button>
      <button id="btnDeepSleep" class="btn" onclick="openDeepSleepModal()">
        <span>🌙</span>
        Deep Sleep
      </button>
    </div>

    <div class="telemetry">
      <div class="tele-item">
        <div class="tele-label">Wi-Fi Signal</div>
        <div id="teleWifi" class="tele-val">-- dBm</div>
      </div>
      <div class="tele-item">
        <div class="tele-label">Active Power State</div>
        <div id="teleMode" class="tele-val">STANDBY (80MHz)</div>
      </div>
      <div class="tele-item" id="batTeleBox">
        <div class="tele-label">Battery Level</div>
        <div id="teleBat" class="tele-val" style="color: #ffd166;">Checking...</div>
      </div>
      <div class="tele-item" id="sdTeleBox">
        <div class="tele-label">MicroSD Storage</div>
        <div id="teleSD" class="tele-val" style="color: #ff3366;">Checking...</div>
      </div>
    </div>

    <div class="sd-card-panel">
      <h3>
        <span>📁 MicroSD Storage WAV Files</span>
        <button class="pill-btn" style="flex:none; padding:4px 10px;" onclick="loadFiles()">Refresh</button>
      </h3>
      <div id="fileList" class="file-list">
        <div style="text-align:center; color:var(--text-muted); padding:10px;">Click refresh to list SD files</div>
      </div>
    </div>
  </div>

  <div id="sleepModal" class="modal-overlay" style="display: none;">
    <div class="modal-content">
      <h3 style="font-size:1rem; margin-bottom:8px;">🌙 Enter Deep Sleep Mode</h3>
      <p style="font-size:0.82rem; color:#8e9bb0; margin-bottom:12px;">CPU and Wi-Fi will shut down (<15µA). It will auto-wake after:</p>
      <div class="sleep-opts">
        <button class="opt-btn" onclick="confirmDeepSleep(5)">5 Minutes</button>
        <button class="opt-btn active" onclick="confirmDeepSleep(10)">10 Minutes</button>
        <button class="opt-btn" onclick="confirmDeepSleep(30)">30 Minutes</button>
        <button class="opt-btn" onclick="confirmDeepSleep(60)">1 Hour</button>
      </div>
      <div style="display: flex; justify-content: flex-end; gap: 8px;">
        <button class="opt-btn" onclick="closeDeepSleepModal()">Cancel</button>
      </div>
    </div>
  </div>

  <script>
    let currentMode = 'standby';
    let isRecordingSD = false;
    let sdMounted = false;

    let audioCtx = null;
    let gainNode = null;
    let compressorNode = null;
    let biquadFilter = null;
    let compressorEnabled = true;
    let filterMode = 'whisper'; // 'whisper', 'voice', 'off'
    let userVolume = 3.5;

    let isListening = false;
    let audioFetchInFlight = false;
    let audioTimer = null;
    let nextPlayTime = 0;

    let isPhoneRecording = false;
    let phoneRecordedChunks = [];
    let phoneRecStartTime = 0;
    let phoneRecTimer = null;

    const canvas = document.getElementById('audioCanvas');
    const ctx = canvas.getContext('2d');

    function resizeCanvas() {
      canvas.width = canvas.parentElement.clientWidth - 36;
      canvas.height = 70;
    }
    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();

    function drawWave(data) {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#00f2fe';
      ctx.beginPath();
      const sliceWidth = canvas.width * 1.0 / data.length;
      let x = 0;
      for (let i = 0; i < data.length; i++) {
        const v = data[i] / 128.0;
        const y = v * (canvas.height / 2);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
        x += sliceWidth;
      }
      ctx.lineTo(canvas.width, canvas.height / 2);
      ctx.stroke();
    }

    function initAudioPipeline() {
      if (!audioCtx) {
        audioCtx = new (window.AudioContext || window.webkitAudioContext)({ sampleRate: 16000 });
      }
      if (!gainNode) {
        gainNode = audioCtx.createGain();
        gainNode.gain.value = userVolume;
      }
      if (!compressorNode) {
        compressorNode = audioCtx.createDynamicsCompressor();
        compressorNode.threshold.value = -38;
        compressorNode.knee.value = 12;
        compressorNode.ratio.value = 6;
        compressorNode.attack.value = 0.003;
        compressorNode.release.value = 0.25;
      }
      if (!biquadFilter) {
        biquadFilter = audioCtx.createBiquadFilter();
        applyFilterSettings();

        // Audio Chain: Source -> Biquad Filter -> Compressor -> Gain -> Speakers
        biquadFilter.connect(compressorEnabled ? compressorNode : gainNode);
        if (compressorEnabled) compressorNode.connect(gainNode);
        gainNode.connect(audioCtx.destination);
      }
    }

    function applyFilterSettings() {
      if (!biquadFilter) return;
      if (filterMode === 'whisper') {
        biquadFilter.type = 'highpass';
        biquadFilter.frequency.value = 140; // Cuts fan/motor hum, preserves 100% of whisper fricatives
        biquadFilter.Q.value = 0.7;
      } else if (filterMode === 'voice') {
        biquadFilter.type = 'bandpass';
        biquadFilter.frequency.value = 1500;
        biquadFilter.Q.value = 0.8;
      } else {
        biquadFilter.type = 'allpass';
      }
    }

    function setFilterMode(mode) {
      filterMode = mode;
      document.getElementById('pillWhisper').classList.toggle('active', mode === 'whisper');
      document.getElementById('pillVoice').classList.toggle('active', mode === 'voice');
      document.getElementById('pillOff').classList.toggle('active', mode === 'off');
      applyFilterSettings();
    }

    function toggleCompressor() {
      compressorEnabled = !compressorEnabled;
      const btn = document.getElementById('btnCompressor');
      btn.classList.toggle('active', compressorEnabled);
      btn.innerText = compressorEnabled ? 
        '🎛️ Broadcast Speech Compressor: ON (Boosts Whispers, Tames Shouts)' : 
        '🎛️ Broadcast Speech Compressor: OFF (Raw Dynamics)';

      if (biquadFilter && compressorNode && gainNode) {
        biquadFilter.disconnect();
        compressorNode.disconnect();
        if (compressorEnabled) {
          biquadFilter.connect(compressorNode);
          compressorNode.connect(gainNode);
        } else {
          biquadFilter.connect(gainNode);
        }
      }
    }

    function updateVolume(val) {
      userVolume = parseFloat(val);
      document.getElementById('volVal').innerText = userVolume.toFixed(1) + 'x';
      if (gainNode) gainNode.gain.value = userVolume;
    }

    function resumeAudio() {
      initAudioPipeline();
      if (audioCtx && audioCtx.state === 'suspended') audioCtx.resume();
    }

    function toggleListening() {
      if (currentMode === 'mic') {
        setDeviceMode('standby');
      } else {
        setDeviceMode('mic');
      }
    }

    function setDeviceMode(mode) {
      fetch('/set_mode?mode=' + mode)
        .then(r => r.json())
        .then(d => updateUI(d))
        .catch(e => console.error(e));
    }

    function toggleSDRecording() {
      if (!sdMounted) {
        alert("⚠️ No MicroSD Card detected! Please insert an SD card formatted as FAT32.");
        return;
      }
      const act = isRecordingSD ? 'stop' : 'start';
      fetch('/record_sd?action=' + act)
        .then(r => r.json())
        .then(d => updateUI(d));
    }

    function openDeepSleepModal() {
      document.getElementById('sleepModal').style.display = 'flex';
    }
    function closeDeepSleepModal() {
      document.getElementById('sleepModal').style.display = 'none';
    }
    function confirmDeepSleep(minutes) {
      closeDeepSleepModal();
      if (confirm(`ESP32 will sleep for ${minutes} minutes to save battery.\nWiFi will be OFF.\nContinue?`)) {
        stopAudioStream();
        fetch('/deep_sleep?minutes=' + minutes);
        alert(`ESP32 is now in Deep Sleep for ${minutes} minutes.`);
        document.getElementById('statusBadge').innerText = "DEEP SLEEP";
      }
    }

    function startAudioStream() {
      if (isListening) return;
      isListening = true;
      audioFetchInFlight = false;
      nextPlayTime = 0;
      resumeAudio();
      document.getElementById('audioStatus').innerText = "Active (Listening)";
      fetchAudioLoop();
    }

    function stopAudioStream() {
      isListening = false;
      audioFetchInFlight = false;
      if (audioTimer) {
        clearTimeout(audioTimer);
        audioTimer = null;
      }
      document.getElementById('audioStatus').innerText = "Standby (Paused)";
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      document.getElementById('vuFill').style.width = '0%';
      document.getElementById('vuLevelTxt').innerText = "Level: Silent";
      document.getElementById('vuDbtxt').innerText = "-- dB";
    }

    function fetchAudioLoop() {
      if (!isListening) return;
      if (currentMode !== 'mic') {
        stopAudioStream();
        return;
      }
      if (audioFetchInFlight) return;

      audioFetchInFlight = true;
      const gainParam = Math.round(userVolume * 10);

      fetch('/audio_chunk?gain=' + gainParam + '&filter=' + filterMode + '&t=' + Date.now())
        .then(res => {
          const peak = res.headers.get('X-Peak-Volume');
          if (peak) {
            updateVUMeter(parseInt(peak, 10));
          }
          return res.arrayBuffer();
        })
        .then(buf => {
          audioFetchInFlight = false;
          if (!isListening) return;

          if (buf && buf.byteLength > 44) {
            resumeAudio();

            if (isPhoneRecording) {
              phoneRecordedChunks.push(new Int16Array(buf.slice(44)));
            }

            audioCtx.decodeAudioData(buf.slice(0), (audioBuffer) => {
              if (!isListening) return;
              const src = audioCtx.createBufferSource();
              src.buffer = audioBuffer;
              src.connect(biquadFilter);

              const now = audioCtx.currentTime;
              if (nextPlayTime < now) nextPlayTime = now;
              src.start(nextPlayTime);
              nextPlayTime += audioBuffer.duration;

              const pcm = audioBuffer.getChannelData(0);
              const sampleWave = [];
              const step = Math.floor(pcm.length / 80) || 1;
              for (let i = 0; i < pcm.length; i += step) {
                sampleWave.push((pcm[i] + 1) * 128);
              }
              drawWave(sampleWave);
            }, (err) => {});
          }

          if (isListening) {
            audioTimer = setTimeout(fetchAudioLoop, 20);
          }
        })
        .catch(err => {
          audioFetchInFlight = false;
          if (isListening) {
            audioTimer = setTimeout(fetchAudioLoop, 150);
          }
        });
    }

    function updateVUMeter(peak) {
      const pct = Math.min(100, Math.round((peak / 24000.0) * 100));
      document.getElementById('vuFill').style.width = pct + '%';

      const vuTxt = document.getElementById('vuLevelTxt');
      const vuDb = document.getElementById('vuDbtxt');

      const db = peak > 10 ? Math.round(20 * Math.log10(peak / 32767.0)) : -60;
      vuDb.innerText = db + ' dB';

      if (peak < 700) {
        vuTxt.innerText = "Level: Quiet / Ambient";
        vuTxt.style.color = "var(--text-muted)";
      } else if (peak < 3500) {
        vuTxt.innerText = "Level: 🎙️ Whisper Detected";
        vuTxt.style.color = "var(--yellow)";
      } else if (peak < 16000) {
        vuTxt.innerText = "Level: 🗣️ Normal Speech";
        vuTxt.style.color = "var(--green)";
      } else {
        vuTxt.innerText = "Level: 🔊 Loud Audio";
        vuTxt.style.color = "var(--red)";
      }
    }

    // Phone Lossless WAV Audio Recording
    function togglePhoneRecording() {
      resumeAudio();
      const btn = document.getElementById('btnPhoneRec');
      const icon = document.getElementById('phoneRecIcon');
      const txt = document.getElementById('phoneRecText');

      if (!isPhoneRecording) {
        if (currentMode !== 'mic') {
          setDeviceMode('mic');
        }
        isPhoneRecording = true;
        phoneRecordedChunks = [];
        phoneRecStartTime = Date.now();
        btn.classList.add('recording');

        phoneRecTimer = setInterval(() => {
          const elapsed = Math.floor((Date.now() - phoneRecStartTime) / 1000);
          const m = String(Math.floor(elapsed / 60)).padStart(2, '0');
          const s = String(elapsed % 60).padStart(2, '0');
          icon.innerText = '⏹️';
          txt.innerText = `Stop & Save WAV (${m}:${s})`;
        }, 1000);
      } else {
        isPhoneRecording = false;
        clearInterval(phoneRecTimer);
        btn.classList.remove('recording');
        icon.innerText = '⏺️';
        txt.innerText = "Record Audio to Phone (Lossless .WAV)";

        if (phoneRecordedChunks.length === 0) {
          alert("No audio received from microphone!");
          return;
        }
        savePhoneWavFile(phoneRecordedChunks);
      }
    }

    function savePhoneWavFile(chunks) {
      let totalSamples = 0;
      for (let c of chunks) totalSamples += c.length;

      const buffer = new ArrayBuffer(44 + totalSamples * 2);
      const view = new DataView(buffer);

      writeString(view, 0, 'RIFF');
      view.setUint32(4, 36 + totalSamples * 2, true);
      writeString(view, 8, 'WAVE');
      writeString(view, 12, 'fmt ');
      view.setUint32(16, 16, true);
      view.setUint16(20, 1, true);
      view.setUint16(22, 1, true);
      view.setUint32(24, 16000, true);
      view.setUint32(28, 32000, true);
      view.setUint16(32, 2, true);
      view.setUint16(34, 16, true);
      writeString(view, 36, 'data');
      view.setUint32(40, totalSamples * 2, true);

      let offset = 44;
      for (let c of chunks) {
        for (let i = 0; i < c.length; i++) {
          view.setInt16(offset, c[i], true);
          offset += 2;
        }
      }

      const blob = new Blob([buffer], { type: 'audio/wav' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      const now = new Date();
      const ts = now.toISOString().replace(/[-:T]/g, '_').slice(0, 15);
      a.href = url;
      a.download = `Audio_Voice_${ts}.wav`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      setTimeout(() => URL.revokeObjectURL(url), 2000);
    }

    function writeString(view, offset, string) {
      for (let i = 0; i < string.length; i++) {
        view.setUint8(offset + i, string.charCodeAt(i));
      }
    }

    function updateUI(status) {
      currentMode = status.mode;
      isRecordingSD = status.recording_sd;
      sdMounted = !!status.sd;

      const wifiEl = document.getElementById('teleWifi');
      if (wifiEl) {
        const rssi = status.wifi_rssi;
        let quality = "Weak (Attach Antenna!)";
        let color = "var(--red)";
        if (rssi > -60) { quality = "Excellent"; color = "var(--green)"; }
        else if (rssi > -70) { quality = "Good"; color = "#ffd166"; }
        else if (rssi > -80) { quality = "Fair"; color = "#ff9f43"; }
        wifiEl.innerText = `${rssi} dBm (${quality})`;
        wifiEl.style.color = color;
      }

      document.getElementById('teleMode').innerText = 
        currentMode === 'mic' ? 'LISTENING (240MHz)' : 'LIGHT SLEEP (80MHz)';

      const batEl = document.getElementById('teleBat');
      if (batEl) {
        if (status.battery_connected) {
          batEl.innerText = `${status.battery_v.toFixed(2)}V (${status.battery_pct}%) - ${status.battery_status}`;
          if (status.battery_pct > 50) batEl.style.color = 'var(--green)';
          else if (status.battery_pct > 20) batEl.style.color = '#ffd166';
          else batEl.style.color = 'var(--red)';
        } else {
          batEl.innerText = status.battery_status || "No Divider on A0";
          batEl.style.color = '#a0aec0';
        }
      }

      const sdEl = document.getElementById('teleSD');
      if (sdMounted) {
        const freeGb = (status.sd_free_mb / 1024).toFixed(1);
        sdEl.innerText = `Ready (${freeGb}GB Free)`;
        sdEl.style.color = 'var(--green)';
      } else {
        sdEl.innerText = "No SD Card";
        sdEl.style.color = 'var(--red)';
      }

      document.getElementById('btnListen').classList.toggle('active', currentMode === 'mic');
      document.getElementById('btnStandby').classList.toggle('active', currentMode === 'standby');
      document.getElementById('btnRecSD').classList.toggle('active', isRecordingSD);

      const statusBadge = document.getElementById('statusBadge');
      if (isRecordingSD) {
        statusBadge.innerText = "REC SD ACTIVE";
        statusBadge.className = "badge badge-rec";
      } else if (currentMode === 'mic') {
        statusBadge.innerText = "LIVE LISTENING";
        statusBadge.className = "badge";
      } else {
        statusBadge.innerText = "STANDBY (SLEEP)";
        statusBadge.className = "badge";
      }

      if (currentMode === 'mic') {
        startAudioStream();
      } else {
        stopAudioStream();
      }
    }

    function loadFiles() {
      fetch('/files').then(r => r.json()).then(files => {
        const fl = document.getElementById('fileList');
        if (!files || files.length === 0) {
          fl.innerHTML = '<div style="text-align:center; color:var(--text-muted); padding:10px;">No WAV files on SD card</div>';
          return;
        }
        fl.innerHTML = files.map(f => `
          <div class="file-item">
            <span>🎵 ${f.name} (${Math.round(f.size/1024)} KB)</span>
            <div class="file-actions">
              <a href="/file?name=${f.name}" target="_blank" download>Download</a>
              <button class="del-btn" onclick="deleteFile('${f.name}')">Delete</button>
            </div>
          </div>
        `).join('');
      }).catch(e => console.error(e));
    }

    function deleteFile(name) {
      if (confirm("Delete " + name + "?")) {
        fetch('/delete?name=' + name).then(() => loadFiles());
      }
    }

    function pollStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(d => {
          updateUI(d);
          const connBadge = document.getElementById('connBadge');
          if (connBadge) {
            connBadge.innerText = "🟢 ONLINE";
            connBadge.className = "badge badge-online";
          }
        })
        .catch(e => {
          const connBadge = document.getElementById('connBadge');
          if (connBadge) {
            connBadge.innerText = "🔴 OFFLINE";
            connBadge.className = "badge badge-offline";
          }
        });
    }

    setInterval(pollStatus, 2500);
    pollStatus();
  </script>
</body>
</html>
)rawliteral";

// ==========================================
// 8. HTTP ROUTE HANDLERS
// ==========================================
void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
}

void handleRoot() {
  addCorsHeaders();
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  addCorsHeaders();
  String modeStr = (currentMode == MODE_MIC) ? "mic" : "standby";

  uint64_t sdTotalMB = 0;
  uint64_t sdFreeMB = 0;
  if (sdCardAvailable && SD.cardType() != CARD_NONE) {
    sdTotalMB = SD.totalBytes() / (1024ULL * 1024ULL);
    uint64_t used = SD.usedBytes() / (1024ULL * 1024ULL);
    sdFreeMB = (sdTotalMB > used) ? (sdTotalMB - used) : 0;
  } else {
    sdCardAvailable = false;
  }

  BatteryInfo bat = readBattery();

  String json = "{";
  json += "\"mode\":\"" + modeStr + "\",";
  json += "\"recording_sd\":" + String(isRecordingSD ? "true" : "false") + ",";
  json += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"sd\":" + String(sdCardAvailable ? "true" : "false") + ",";
  json += "\"sd_total_mb\":" + String((unsigned long)sdTotalMB) + ",";
  json += "\"sd_free_mb\":" + String((unsigned long)sdFreeMB) + ",";
  json += "\"battery_v\":" + String(bat.voltage, 2) + ",";
  json += "\"battery_pct\":" + String(bat.percentage) + ",";
  json += "\"battery_connected\":" + String(bat.isConnected ? "true" : "false") + ",";
  json += "\"battery_status\":\"" + bat.status + "\",";
  json += "\"free_psram\":" + String(ESP.getFreePsram()) + ",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap());
  json += "}";

  server.send(200, "application/json", json);
}

void handleSetMode() {
  addCorsHeaders();
  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    if (m == "mic") setPowerMode(MODE_MIC);
    else setPowerMode(MODE_STANDBY);
  }
  handleStatus();
}

void handleRecordSD() {
  addCorsHeaders();
  if (server.hasArg("action")) {
    String act = server.arg("action");
    if (act == "start" && sdCardAvailable && !isRecordingSD) {
      startSDRecording();
    } else if (act == "stop" && isRecordingSD) {
      closeSDRecording();
    }
  }
  handleStatus();
}

void handleDeepSleep() {
  addCorsHeaders();
  uint32_t mins = 10;
  if (server.hasArg("minutes")) {
    mins = server.arg("minutes").toInt();
    if (mins == 0) mins = 10;
  }
  server.send(200, "text/plain", "ESP32 entering Deep Sleep for " + String(mins) + " minutes.");
  delay(500);
  triggerDeepSleep(mins);
}

// 16kHz High-Fidelity Audio Chunk Streamer with Whisper-Safe HPF & Soft Limiting
void handleAudioChunk() {
  addCorsHeaders();
  const size_t sampleCount = 3200; // 0.20s chunk
  const size_t pcmBytes = sampleCount * sizeof(int16_t);

  int16_t *pcmBuffer = (int16_t*)malloc(pcmBytes);
  if (!pcmBuffer) {
    server.send(500, "text/plain", "Internal RAM allocation failed");
    return;
  }

  size_t bytesRead = 0;
  esp_err_t r = i2s_read(I2S_MIC_CHANNEL, (void*)pcmBuffer, pcmBytes, &bytesRead, pdMS_TO_TICKS(220));

  int16_t peakVal = 0;
  if (r == ESP_OK && bytesRead > 0) {
    size_t samples = bytesRead / sizeof(int16_t);

    float targetGain = 3.5f;
    if (server.hasArg("gain")) {
      float userGain = server.arg("gain").toFloat();
      if (userGain >= 5.0f && userGain <= 80.0f) {
        targetGain = userGain / 8.0f;
      }
    }

    String filterMode = server.hasArg("filter") ? server.arg("filter") : "whisper";

    // 140Hz Whisper-Safe High-Pass Filter (Alpha = 0.947f at 16000Hz)
    // Eliminates fan hum, motor drone, and AC hum without touching speech/whispers
    static float x_prev = 0.0f;
    static float y_prev = 0.0f;
    const float alpha_whisper = 0.947f;

    for (size_t i = 0; i < samples; i++) {
      float x = (float)pcmBuffer[i];
      float y;
      if (filterMode == "whisper") {
        y = x - x_prev + alpha_whisper * y_prev;
        x_prev = x;
        y_prev = y;
      } else {
        y = x;
      }

      float amplified = y * targetGain;

      // Soft limiting curve: boosts quiet whispers while taming sudden loud sounds
      if (amplified > 25000.0f) amplified = 25000.0f + (amplified - 25000.0f) * 0.12f;
      else if (amplified < -25000.0f) amplified = -25000.0f + (amplified + 25000.0f) * 0.12f;

      if (amplified > 32767.0f) amplified = 32767.0f;
      else if (amplified < -32768.0f) amplified = -32768.0f;

      pcmBuffer[i] = (int16_t)amplified;
      int16_t absVal = abs(pcmBuffer[i]);
      if (absVal > peakVal) peakVal = absVal;
    }
  }

  WavHeader header;
  header.dataSize = bytesRead;
  header.chunkSize = bytesRead + sizeof(WavHeader) - 8;

  WiFiClient client = server.client();
  client.setNoDelay(true);
  client.print("HTTP/1.1 200 OK\r\n"
               "Content-Type: audio/wav\r\n"
               "Content-Length: " + String(sizeof(WavHeader) + bytesRead) + "\r\n"
               "X-Peak-Volume: " + String(peakVal) + "\r\n"
               "Access-Control-Allow-Origin: *\r\n\r\n");

  client.write((const uint8_t*)&header, sizeof(WavHeader));
  if (bytesRead > 0) {
    client.write((const uint8_t*)pcmBuffer, bytesRead);
  }

  free(pcmBuffer);
}

// SD File Browser Handlers
void handleListFiles() {
  addCorsHeaders();
  if (!sdCardAvailable) {
    server.send(200, "application/json", "[]");
    return;
  }

  File root = SD.open("/rec");
  if (!root || !root.isDirectory()) {
    server.send(200, "application/json", "[]");
    return;
  }

  String json = "[";
  bool first = true;
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      if (!first) json += ",";
      first = false;
      json += "{\"name\":\"" + String(file.name()) + "\",\"size\":" + String(file.size()) + "}";
    }
    file = root.openNextFile();
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleGetFile() {
  addCorsHeaders();
  if (!server.hasArg("name") || !sdCardAvailable) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  String filename = "/rec/" + server.arg("name");
  File file = SD.open(filename, FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "File Not Found");
    return;
  }
  server.streamFile(file, "audio/wav");
  file.close();
}

void handleDeleteFile() {
  addCorsHeaders();
  if (!server.hasArg("name") || !sdCardAvailable) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  String filename = "/rec/" + server.arg("name");
  SD.remove(filename);
  server.send(200, "text/plain", "Deleted");
}

// ==========================================
// 9. SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==========================================");
  Serial.println("  XIAO ESP32-S3 Audio Surveillance Station");
  Serial.println("==========================================");

  micInitialized  = initMic();
  sdCardAvailable = initSD();

  // Connect to Wi-Fi with MAX transmit power
  Serial.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected Successfully!");
    Serial.print("[WIFI] Dashboard IP: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WIFI] Could not connect to AP. Starting SoftAP Fallback...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("[WIFI] SoftAP Active: %s (Pass: %s)\n", AP_SSID, AP_PASSWORD);
    Serial.print("[WIFI] Dashboard IP: http://");
    Serial.println(WiFi.softAPIP());
  }

  // Register HTTP Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/set_mode", HTTP_GET, handleSetMode);
  server.on("/record_sd", HTTP_GET, handleRecordSD);
  server.on("/deep_sleep", HTTP_GET, handleDeepSleep);
  server.on("/audio_chunk", HTTP_GET, handleAudioChunk);
  server.on("/files", HTTP_GET, handleListFiles);
  server.on("/file", HTTP_GET, handleGetFile);
  server.on("/delete", HTTP_GET, handleDeleteFile);

  server.begin();
  Serial.println("[HTTP] Web Server Started on Port 80");

  // Default to Standby / Light Sleep mode
  setPowerMode(MODE_STANDBY);
}

void loop() {
  server.handleClient();

  // Background SD Audio Recorder
  if (isRecordingSD && sdCardAvailable && sdAudioFile) {
    int16_t tempBuf[512];
    size_t bytesRead = 0;
    esp_err_t r = i2s_read(I2S_MIC_CHANNEL, (void*)tempBuf, sizeof(tempBuf), &bytesRead, 0);
    if (r == ESP_OK && bytesRead > 0) {
      sdAudioFile.write((const uint8_t*)tempBuf, bytesRead);
      sdBytesWritten += bytesRead;
    }
  }

  yield();
}