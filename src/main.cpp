#include <Arduino.h>
#include "esp_camera.h"
#include <driver/i2s.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_sleep.h>

// ==========================================
// 1. WI-FI CREDENTIALS (CHANGE THESE)
// ==========================================
const char* WIFI_SSID     = "Shahid";
const char* WIFI_PASSWORD = "Rafiq@5219786";

// Fallback SoftAP if Wi-Fi fails to connect
const char* AP_SSID       = "XIAO-DroneCam-AP";
const char* AP_PASSWORD   = "12345678";

// ==========================================
// 2. PIN DEFINITIONS (Seeed Studio XIAO ESP32S3 Sense)
// ==========================================
// Camera OV2640 Pins
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// PDM Digital Microphone Pins
#define I2S_MIC_CHANNEL   I2S_NUM_0
#define I2S_MIC_CLK_PIN   42
#define I2S_MIC_DATA_PIN  41
#define AUDIO_SAMPLE_RATE 16000

// MicroSD Card Pins (SPI)
#define SD_CS_PIN         21
#define SD_SCK_PIN        7
#define SD_MISO_PIN       8
#define SD_MOSI_PIN       9

// ==========================================
// 3. SYSTEM STATE VARIABLES
// ==========================================
enum SystemMode {
  MODE_STANDBY,     // Low power / Light Sleep
  MODE_CAMERA,      // Video stream active
  MODE_MIC,         // Audio stream active
  MODE_TURBO        // Camera + Mic active simultaneously
};

volatile SystemMode currentMode = MODE_STANDBY;
volatile bool isRecording = false;
bool sdCardAvailable = false;
bool cameraInitialized = false;
bool micInitialized = false;
unsigned long lastRecordTime = 0;
uint32_t recordFileIndex = 1;

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
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA;    // 640x480
  config.jpeg_quality = 12;               // 10-63 (lower = higher quality)
  config.fb_count     = 2;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Camera init failed with error 0x%x\n", err);
    return false;
  }
  Serial.println("[CAM] Camera Initialized Successfully");
  return true;
}

bool initMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
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
    Serial.printf("[MIC] I2S driver install failed: 0x%x\n", err);
    return false;
  }
  err = i2s_set_pin(I2S_MIC_CHANNEL, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("[MIC] I2S pin set failed: 0x%x\n", err);
    return false;
  }
  Serial.println("[MIC] PDM Microphone Initialized Successfully");
  return true;
}

bool initSD() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[SD] MicroSD Card Mount Failed (Card not inserted or wrong format)");
    return false;
  }
  Serial.printf("[SD] MicroSD Initialized. Total Space: %llu MB\n", SD.totalBytes() / (1024 * 1024));
  if (!SD.exists("/rec")) {
    SD.mkdir("/rec");
  }
  return true;
}

// ==========================================
// 6. HELPER FUNCTIONS: POWER & RECORDING
// ==========================================
void setPowerMode(SystemMode mode) {
  currentMode = mode;
  if (mode == MODE_STANDBY) {
    // Light sleep / Standby: Throttle down CPU, pause peripherals
    setCpuFrequencyMhz(80);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    Serial.println("[POWER] Entered Light Sleep / Standby Mode (80MHz, Modem Sleep)");
  } else {
    // Active modes: Boost CPU to 240MHz
    setCpuFrequencyMhz(240);
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.println("[POWER] Boosted to High Performance (240MHz)");
  }
}

void triggerDeepSleep(uint32_t sleepMinutes = 10) {
  Serial.printf("[SLEEP] Entering Deep Sleep for %u minutes...\n", sleepMinutes);
  if (isRecording) {
    isRecording = false;
  }
  // Turn off Wi-Fi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  // Configure timer wakeup (microseconds)
  uint64_t sleepUs = (uint64_t)sleepMinutes * 60ULL * 1000000ULL;
  esp_sleep_enable_timer_wakeup(sleepUs);
  
  // Power down into deep sleep
  esp_deep_sleep_start();
}

void saveSnapshotToSD() {
  if (!sdCardAvailable || !cameraInitialized) return;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[REC] Failed to get camera frame");
    return;
  }
  char path[32];
  snprintf(path, sizeof(path), "/rec/SNAP_%04lu.jpg", (unsigned long)recordFileIndex++);
  File file = SD.open(path, FILE_WRITE);
  if (file) {
    file.write(fb->buf, fb->len);
    file.close();
    Serial.printf("[REC] Saved snapshot: %s (%u bytes)\n", path, fb->len);
  }
  esp_camera_fb_return(fb);
}

// ==========================================
// 7. EMBEDDED DASHBOARD (HTML/CSS/JS)
// ==========================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>XIAO ESP32-S3 Drone Surveillance Hub</title>
  <style>
    :root {
      --bg: #090d16;
      --card: rgba(22, 30, 49, 0.85);
      --card-border: rgba(255, 255, 255, 0.09);
      --accent: #00f2fe;
      --accent-glow: rgba(0, 242, 254, 0.35);
      --primary: #4facfe;
      --red: #ff3366;
      --red-glow: rgba(255, 51, 102, 0.4);
      --green: #00e676;
      --text: #f0f4f8;
      --text-muted: #8e9bb0;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body {
      background: radial-gradient(circle at top, #141f38 0%, var(--bg) 100%);
      color: var(--text);
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 12px;
    }
    .container { width: 100%; max-width: 820px; display: flex; flex-direction: column; gap: 14px; }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 12px 18px;
      background: var(--card);
      border: 1px solid var(--card-border);
      border-radius: 14px;
      backdrop-filter: blur(12px);
    }
    .title-box h1 { font-size: 1.15rem; font-weight: 700; color: #fff; }
    .badge {
      font-size: 0.72rem;
      padding: 4px 10px;
      border-radius: 20px;
      background: rgba(0, 242, 254, 0.15);
      color: var(--accent);
      border: 1px solid var(--accent);
      text-transform: uppercase;
      font-weight: 600;
      transition: all 0.3s ease;
    }
    .badge.rec-active {
      background: var(--red);
      color: #fff;
      border-color: var(--red);
      animation: pulse 1.2s infinite;
    }
    @keyframes pulse {
      0%, 100% { opacity: 1; transform: scale(1); }
      50% { opacity: 0.75; transform: scale(0.97); }
    }
    .stream-card {
      position: relative;
      width: 100%;
      height: 380px;
      background: #04060a;
      border-radius: 16px;
      overflow: hidden;
      display: flex;
      align-items: center;
      justify-content: center;
      border: 1px solid var(--card-border);
      box-shadow: 0 10px 30px rgba(0,0,0,0.6);
    }
    #videoFeed {
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: none;
    }
    .no-feed-msg {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 8px;
      color: var(--text-muted);
      text-align: center;
    }
    .audio-card {
      background: var(--card);
      border: 1px solid var(--card-border);
      border-radius: 14px;
      padding: 14px 18px;
      backdrop-filter: blur(12px);
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    .audio-header { display: flex; justify-content: space-between; align-items: center; font-size: 0.85rem; color: var(--text-muted); }
    .audio-indicator { font-weight: 600; color: var(--accent); }
    #audioCanvas { width: 100%; height: 50px; background: rgba(0,0,0,0.4); border-radius: 8px; }
    .audio-controls-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      gap: 10px;
      padding-top: 4px;
      border-top: 1px solid rgba(255,255,255,0.05);
    }
    .slider-group {
      display: flex;
      align-items: center;
      gap: 10px;
      font-size: 0.82rem;
      color: var(--text);
    }
    .slider-group input[type="range"] {
      accent-color: var(--accent);
      cursor: pointer;
      width: 120px;
    }
    .chip-btn {
      background: rgba(255,255,255,0.06);
      border: 1px solid var(--card-border);
      color: var(--text-muted);
      padding: 5px 12px;
      border-radius: 20px;
      font-size: 0.78rem;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.2s ease;
    }
    .chip-btn.active {
      background: rgba(0, 230, 118, 0.15);
      color: var(--green);
      border-color: var(--green);
    }
    .audio-rec-bar { display: flex; justify-content: center; margin-top: 2px; }
    .phone-rec-btn {
      width: 100%;
      background: rgba(255, 51, 102, 0.12);
      border: 1px solid rgba(255, 51, 102, 0.4);
      color: #ff5577;
      padding: 8px 14px;
      border-radius: 10px;
      font-size: 0.82rem;
      font-weight: 600;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      transition: all 0.2s ease;
    }
    .phone-rec-btn.recording {
      background: var(--red);
      color: #fff;
      border-color: var(--red);
      animation: pulse 1s infinite;
    }
    .controls-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(105px, 1fr));
      gap: 10px;
    }
    .btn {
      background: var(--card);
      border: 1px solid var(--card-border);
      color: var(--text);
      padding: 12px 10px;
      border-radius: 12px;
      font-size: 0.85rem;
      font-weight: 600;
      cursor: pointer;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 6px;
      transition: all 0.2s cubic-bezier(0.4, 0, 0.2, 1);
      backdrop-filter: blur(8px);
    }
    .btn span { font-size: 1.35rem; }
    .btn:hover { border-color: var(--accent); transform: translateY(-2px); }
    .btn:active { transform: scale(0.96); }
    .btn.active {
      background: linear-gradient(135deg, var(--primary), var(--accent));
      color: #000;
      border-color: transparent;
      box-shadow: 0 0 16px var(--accent-glow);
    }
    .btn.rec-btn.active {
      background: var(--red);
      color: #fff;
      box-shadow: 0 0 16px var(--red-glow);
    }
    .btn.sleep-btn:hover { border-color: #ffd166; color: #ffd166; }
    .telemetry {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 8px;
    }
    .tele-item {
      background: var(--card);
      border: 1px solid var(--card-border);
      padding: 10px 8px;
      border-radius: 10px;
      text-align: center;
    }
    .tele-label { font-size: 0.68rem; color: var(--text-muted); text-transform: uppercase; margin-bottom: 2px; }
    .tele-val { font-size: 0.88rem; font-weight: 700; color: #fff; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .sd-card-panel {
      background: var(--card);
      border: 1px solid var(--card-border);
      border-radius: 14px;
      padding: 14px 18px;
    }
    .sd-card-panel h3 { font-size: 0.95rem; margin-bottom: 10px; color: var(--text); display: flex; justify-content: space-between; align-items: center; }
    .file-list { max-height: 140px; overflow-y: auto; display: flex; flex-direction: column; gap: 6px; }
    .file-item {
      display: flex;
      justify-content: space-between;
      align-items: center;
      background: rgba(0,0,0,0.35);
      padding: 7px 12px;
      border-radius: 8px;
      font-size: 0.8rem;
    }
    .file-actions a, .file-actions button {
      color: var(--accent);
      text-decoration: none;
      background: transparent;
      border: none;
      cursor: pointer;
      margin-left: 8px;
      font-size: 0.8rem;
    }
    .file-actions button.del-btn { color: var(--red); }
    .modal-overlay {
      position: fixed;
      top: 0; left: 0; width: 100%; height: 100%;
      background: rgba(0,0,0,0.7);
      display: flex;
      align-items: center;
      justify-content: center;
      z-index: 1000;
      backdrop-filter: blur(6px);
    }
    .modal-content {
      background: #111827;
      border: 1px solid var(--card-border);
      border-radius: 16px;
      padding: 20px;
      width: 90%;
      max-width: 400px;
      box-shadow: 0 20px 40px rgba(0,0,0,0.6);
    }
    .sleep-opts {
      display: grid;
      grid-template-columns: 1fr 1fr;
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
      transition: all 0.2s ease;
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
        <h1>🚁 XIAO Drone Surveillance</h1>
      </div>
      <div id="statusBadge" class="badge">Standby</div>
    </header>

    <div class="stream-card">
      <img id="videoFeed" alt="Camera Feed">
      <div id="noFeedMsg" class="no-feed-msg">
        <span style="font-size: 2.8rem;">📷</span>
        <span style="font-weight: 600; font-size: 1rem;">Camera in Standby</span>
        <span style="font-size: 0.78rem; color: #8e9bb0;">Tap 'Camera' or 'Turbo Mode' to start live video stream</span>
      </div>
    </div>

    <div class="audio-card">
      <div class="audio-header">
        <span>🎙️ PDM Microphone (16kHz Audio)</span>
        <span id="audioStatus" class="audio-indicator">Muted</span>
      </div>
      <canvas id="audioCanvas"></canvas>
      
      <div class="audio-controls-row">
        <div class="slider-group">
          <label for="volBoost">🔊 Whisper Boost: <span id="volVal">2.5x</span></label>
          <input type="range" id="volBoost" min="1" max="5" step="0.5" value="2.5" oninput="updateVolume(this.value)">
        </div>
        <button id="btnNoiseFilter" class="chip-btn active" onclick="toggleNoiseFilter()">
          🍃 Fan Filter: ON
        </button>
      </div>

      <div class="audio-rec-bar">
        <button id="btnPhoneRec" class="phone-rec-btn" onclick="togglePhoneAudioRecording()">
          <span>🔴</span>
          <span id="phoneRecText">Record Audio to Phone</span>
        </button>
      </div>
    </div>

    <div class="controls-grid">
      <button id="btnCam" class="btn" onclick="handleModeClick('camera')">
        <span>📷</span>
        Camera
      </button>
      <button id="btnMic" class="btn" onclick="handleModeClick('mic')">
        <span>🎙️</span>
        Mic
      </button>
      <button id="btnTurbo" class="btn" onclick="handleModeClick('turbo')">
        <span>⚡</span>
        Turbo Mode
      </button>
      <button id="btnRec" class="btn rec-btn" onclick="handleRecordClick()">
        <span>⏺️</span>
        SD Record
      </button>
      <button id="btnSnap" class="btn" onclick="takeSnapshot()">
        <span>📸</span>
        Snapshot
      </button>
      <button id="btnStandby" class="btn sleep-btn" onclick="handleModeClick('standby')">
        <span>🍃</span>
        Light Sleep
      </button>
      <button id="btnDeepSleep" class="btn sleep-btn" onclick="openDeepSleepModal()">
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
        <div class="tele-label">Active Mode</div>
        <div id="teleMode" class="tele-val">Standby</div>
      </div>
      <div class="tele-item" id="sdTeleBox">
        <div class="tele-label">MicroSD Card</div>
        <div id="teleSD" class="tele-val" style="color: #ff3366;">Checking...</div>
      </div>
      <div class="tele-item">
        <div class="tele-label">Free PSRAM</div>
        <div id="teleRam" class="tele-val">-- KB</div>
      </div>
    </div>

    <div class="sd-card-panel">
      <h3>
        <span>📁 MicroSD Storage Recordings</span>
        <button class="chip-btn" onclick="loadFiles()">Refresh</button>
      </h3>
      <div id="fileList" class="file-list">
        <div style="text-align:center; color:var(--text-muted); padding:10px;">Click refresh to list SD files</div>
      </div>
    </div>
  </div>

  <div id="sleepModal" class="modal-overlay" style="display: none;">
    <div class="modal-content">
      <h3>🌙 Enter Deep Sleep Mode</h3>
      <p style="font-size: 0.85rem; color: #a0aec0; margin: 10px 0;">ESP32 Wi-Fi and CPU will turn OFF to preserve battery. It will auto-wake after:</p>
      <div class="sleep-opts">
        <button class="opt-btn" onclick="confirmDeepSleep(5)">5 Minutes</button>
        <button class="opt-btn active" onclick="confirmDeepSleep(10)">10 Minutes</button>
        <button class="opt-btn" onclick="confirmDeepSleep(30)">30 Minutes</button>
        <button class="opt-btn" onclick="confirmDeepSleep(60)">1 Hour</button>
      </div>
      <div style="display: flex; justify-content: flex-end; gap: 8px; margin-top: 15px;">
        <button class="opt-btn" onclick="closeDeepSleepModal()">Cancel</button>
      </div>
    </div>
  </div>

  <script>
    let currentMode = 'standby';
    let isRecording = false;
    let isSwitching = false;
    let sdMounted = false;

    let audioCtx = null;
    let gainNode = null;
    let filterNode = null;
    let noiseFilterEnabled = true;
    let userVolume = 2.5;
    let audioInterval = null;

    let isPhoneRecording = false;
    let phoneRecordedChunks = [];
    let phoneRecStartTime = 0;
    let phoneRecTimer = null;

    const canvas = document.getElementById('audioCanvas');
    const ctx = canvas.getContext('2d');

    function resizeCanvas() {
      canvas.width = canvas.parentElement.clientWidth - 36;
      canvas.height = 50;
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
        audioCtx = new (window.AudioContext || window.webkitAudioContext)();
      }
      if (!gainNode) {
        gainNode = audioCtx.createGain();
        gainNode.gain.value = userVolume;
      }
      if (!filterNode) {
        filterNode = audioCtx.createBiquadFilter();
        filterNode.type = 'highpass';
        filterNode.frequency.value = noiseFilterEnabled ? 180 : 20;
        filterNode.Q.value = 0.7;

        filterNode.connect(gainNode);
        gainNode.connect(audioCtx.destination);
      }
    }

    function updateVolume(val) {
      userVolume = parseFloat(val);
      document.getElementById('volVal').innerText = userVolume.toFixed(1) + 'x';
      if (gainNode) gainNode.gain.value = userVolume;
    }

    function toggleNoiseFilter() {
      noiseFilterEnabled = !noiseFilterEnabled;
      const btn = document.getElementById('btnNoiseFilter');
      btn.classList.toggle('active', noiseFilterEnabled);
      btn.innerText = noiseFilterEnabled ? '🍃 Fan Filter: ON' : '🍃 Fan Filter: OFF';
      if (filterNode) filterNode.frequency.value = noiseFilterEnabled ? 180 : 20;
    }

    function resumeAudio() {
      initAudioPipeline();
      if (audioCtx && audioCtx.state === 'suspended') audioCtx.resume();
    }

    async function handleModeClick(requestedMode) {
      if (isSwitching) return;
      isSwitching = true;

      let targetMode = requestedMode;
      if (currentMode === requestedMode && requestedMode !== 'standby') {
        targetMode = 'standby';
      }

      const videoFeed = document.getElementById('videoFeed');
      if (currentMode === 'camera' || currentMode === 'turbo') {
        videoFeed.src = '';
        videoFeed.style.display = 'none';
        document.getElementById('noFeedMsg').style.display = 'flex';
      }
      stopAudioStream();

      await new Promise(r => setTimeout(r, 60));

      try {
        const res = await fetch('/set_mode?mode=' + targetMode);
        const data = await res.json();
        updateUI(data);
      } catch (err) {
        console.error(err);
      } finally {
        isSwitching = false;
      }
    }

    function handleRecordClick() {
      if (!sdMounted) {
        alert("⚠️ No MicroSD Card detected! Please insert an SD card formatted as FAT32.");
        return;
      }
      const action = isRecording ? 'stop' : 'start';
      fetch('/record?action=' + action).then(r => r.json()).then(d => updateUI(d));
    }

    function takeSnapshot() {
      if (!sdMounted) {
        alert("⚠️ No MicroSD Card detected! Please insert an SD card to save snapshots.");
        return;
      }
      fetch('/take_snap').then(r => r.text()).then(t => { alert(t); loadFiles(); });
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
        document.getElementById('videoFeed').src = '';
        stopAudioStream();
        fetch('/deep_sleep?minutes=' + minutes);
        alert(`ESP32 is now in Deep Sleep for ${minutes} minutes.`);
        document.getElementById('statusBadge').innerText = "DEEP SLEEP";
      }
    }

    function updateUI(status) {
      currentMode = status.mode;
      isRecording = status.recording;
      sdMounted = !!status.sd;

      document.getElementById('teleWifi').innerText = (status.wifi_rssi || '--') + " dBm";
      document.getElementById('teleMode').innerText = status.mode.toUpperCase();
      document.getElementById('teleRam').innerText = Math.round((status.free_psram || 0) / 1024) + " KB";

      const sdEl = document.getElementById('teleSD');
      if (sdMounted) {
        const freeGb = (status.sd_free_mb / 1024).toFixed(1);
        sdEl.innerText = `Ready (${freeGb}GB Free)`;
        sdEl.style.color = 'var(--green)';
      } else {
        sdEl.innerText = "No SD Card";
        sdEl.style.color = 'var(--red)';
      }

      document.getElementById('btnCam').classList.toggle('active', currentMode === 'camera');
      document.getElementById('btnMic').classList.toggle('active', currentMode === 'mic');
      document.getElementById('btnTurbo').classList.toggle('active', currentMode === 'turbo');
      document.getElementById('btnStandby').classList.toggle('active', currentMode === 'standby');
      document.getElementById('btnRec').classList.toggle('active', isRecording);

      const statusBadge = document.getElementById('statusBadge');
      if (isRecording) {
        statusBadge.innerText = "REC " + currentMode.toUpperCase();
        statusBadge.className = "badge rec-active";
      } else {
        statusBadge.innerText = currentMode.toUpperCase();
        statusBadge.className = "badge";
      }

      const videoFeed = document.getElementById('videoFeed');
      const noFeedMsg = document.getElementById('noFeedMsg');
      if (currentMode === 'camera' || currentMode === 'turbo') {
        const streamUrl = '/stream?t=' + Date.now();
        if (videoFeed.src !== streamUrl) videoFeed.src = streamUrl;
        videoFeed.style.display = 'block';
        noFeedMsg.style.display = 'none';
      } else {
        videoFeed.src = '';
        videoFeed.style.display = 'none';
        noFeedMsg.style.display = 'flex';
      }

      if (currentMode === 'mic' || currentMode === 'turbo') {
        startAudioStream();
      } else {
        stopAudioStream();
      }
    }

    function startAudioStream() {
      if (audioInterval) return;
      resumeAudio();
      document.getElementById('audioStatus').innerText = "Active (Listening)";

      let isFetching = false;
      audioInterval = setInterval(() => {
        if (isFetching) return;
        isFetching = true;

        const gainParam = Math.round(userVolume * 10);
        fetch('/audio_chunk?gain=' + gainParam + '&t=' + Date.now())
          .then(res => {
            const peak = res.headers.get('X-Peak-Volume');
            if (peak) {
              const peakNum = parseInt(peak, 10);
              if (peakNum > 20000) document.getElementById('audioStatus').innerText = `Loud (${peak})`;
              else if (peakNum > 1000) document.getElementById('audioStatus').innerText = `Speech (${peak})`;
              else document.getElementById('audioStatus').innerText = `Quiet (${peak})`;
            }
            return res.arrayBuffer();
          })
          .then(buf => {
            isFetching = false;
            if (!buf || buf.byteLength <= 44) return;
            resumeAudio();

            if (isPhoneRecording) {
              phoneRecordedChunks.push(new Int16Array(buf.slice(44)));
            }

            audioCtx.decodeAudioData(buf.slice(0), (audioBuffer) => {
              const src = audioCtx.createBufferSource();
              src.buffer = audioBuffer;
              src.connect(filterNode);
              src.start();

              const pcm = audioBuffer.getChannelData(0);
              const sampleWave = [];
              const step = Math.floor(pcm.length / 80) || 1;
              for (let i = 0; i < pcm.length; i += step) {
                sampleWave.push((pcm[i] + 1) * 128);
              }
              drawWave(sampleWave);
            }, (err) => {});
          }).catch(e => { isFetching = false; });
      }, 450);
    }

    function stopAudioStream() {
      if (audioInterval) {
        clearInterval(audioInterval);
        audioInterval = null;
      }
      document.getElementById('audioStatus').innerText = "Muted";
      ctx.clearRect(0, 0, canvas.width, canvas.height);
    }

    function togglePhoneAudioRecording() {
      resumeAudio();
      const btn = document.getElementById('btnPhoneRec');
      const txt = document.getElementById('phoneRecText');

      if (!isPhoneRecording) {
        isPhoneRecording = true;
        phoneRecordedChunks = [];
        phoneRecStartTime = Date.now();
        btn.classList.add('recording');
        if (currentMode !== 'mic' && currentMode !== 'turbo') handleModeClick('mic');

        phoneRecTimer = setInterval(() => {
          const elapsed = Math.floor((Date.now() - phoneRecStartTime) / 1000);
          const m = String(Math.floor(elapsed / 60)).padStart(2, '0');
          const s = String(elapsed % 60).padStart(2, '0');
          txt.innerText = `⏹️ Stop & Save (${m}:${s})`;
        }, 1000);
      } else {
        isPhoneRecording = false;
        clearInterval(phoneRecTimer);
        btn.classList.remove('recording');
        txt.innerText = "Record Audio to Phone";
        if (phoneRecordedChunks.length === 0) { alert("No audio data!"); return; }
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
      URL.revokeObjectURL(url);
    }

    function writeString(view, offset, string) {
      for (let i = 0; i < string.length; i++) {
        view.setUint8(offset + i, string.charCodeAt(i));
      }
    }

    function loadFiles() {
      fetch('/files').then(r => r.json()).then(files => {
        const fl = document.getElementById('fileList');
        if (!files || files.length === 0) {
          fl.innerHTML = '<div style="text-align:center; color:var(--text-muted); padding:10px;">No files on SD card</div>';
          return;
        }
        fl.innerHTML = files.map(f => `
          <div class="file-item">
            <span>📄 ${f.name} (${Math.round(f.size/1024)} KB)</span>
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

    setInterval(() => {
      fetch('/status').then(r => r.json()).then(d => updateUI(d)).catch(e => {});
    }, 3000);

    fetch('/status').then(r => r.json()).then(d => updateUI(d));
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
  String modeStr;
  switch (currentMode) {
    case MODE_CAMERA:  modeStr = "camera"; break;
    case MODE_MIC:     modeStr = "mic"; break;
    case MODE_TURBO:   modeStr = "turbo"; break;
    default:           modeStr = "standby"; break;
  }

  // Dynamically verify SD card status and space
  uint64_t sdTotalMB = 0;
  uint64_t sdFreeMB = 0;
  if (sdCardAvailable && SD.cardType() != CARD_NONE) {
    sdTotalMB = SD.totalBytes() / (1024ULL * 1024ULL);
    uint64_t used = SD.usedBytes() / (1024ULL * 1024ULL);
    sdFreeMB = (sdTotalMB > used) ? (sdTotalMB - used) : 0;
  } else {
    sdCardAvailable = false;
  }

  String json = "{";
  json += "\"mode\":\"" + modeStr + "\",";
  json += "\"recording\":" + String(isRecording ? "true" : "false") + ",";
  json += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"sd\":" + String(sdCardAvailable ? "true" : "false") + ",";
  json += "\"sd_total_mb\":" + String((unsigned long)sdTotalMB) + ",";
  json += "\"sd_free_mb\":" + String((unsigned long)sdFreeMB) + ",";
  json += "\"free_psram\":" + String(ESP.getFreePsram()) + ",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap());
  json += "}";

  server.send(200, "application/json", json);
}

void handleSetMode() {
  addCorsHeaders();
  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    if (m == "camera") setPowerMode(MODE_CAMERA);
    else if (m == "mic") setPowerMode(MODE_MIC);
    else if (m == "turbo") setPowerMode(MODE_TURBO);
    else setPowerMode(MODE_STANDBY);
  }
  handleStatus();
}

void handleRecord() {
  addCorsHeaders();
  if (server.hasArg("action")) {
    String act = server.arg("action");
    if (act == "start" && sdCardAvailable) {
      isRecording = true;
    } else {
      isRecording = false;
    }
  }
  handleStatus();
}

void handleTakeSnap() {
  addCorsHeaders();
  if (!sdCardAvailable) {
    server.send(200, "text/plain", "Error: No SD Card Inserted");
    return;
  }
  saveSnapshotToSD();
  server.send(200, "text/plain", "Snapshot saved successfully to SD card!");
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

// MJPEG Stream Handler with clean timeout & rapid exit
void handleStream() {
  addCorsHeaders();
  if (!cameraInitialized) {
    server.send(503, "text/plain", "Camera not initialized");
    return;
  }

  WiFiClient client = server.client();
  client.setTimeout(1); // 1 second network timeout
  client.print("HTTP/1.1 200 OK\r\n"
               "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
               "Access-Control-Allow-Origin: *\r\n\r\n");

  while (client.connected()) {
    // If mode changed away from video, exit stream immediately
    if (currentMode != MODE_CAMERA && currentMode != MODE_TURBO) {
      break;
    }
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      delay(10);
      continue;
    }

    size_t w = client.print("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + String(fb->len) + "\r\n\r\n");
    if (w == 0) { esp_camera_fb_return(fb); break; }
    w = client.write(fb->buf, fb->len);
    if (w == 0) { esp_camera_fb_return(fb); break; }
    client.print("\r\n");
    esp_camera_fb_return(fb);

    yield();
  }
  client.stop();
}

// Single Photo Capture
void handleCapture() {
  addCorsHeaders();
  if (!cameraInitialized) {
    server.send(503, "text/plain", "Camera not initialized");
    return;
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Failed to capture frame");
    return;
  }
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Length", String(fb->len));
  WiFiClient client = server.client();
  client.print("HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: " + String(fb->len) + "\r\nAccess-Control-Allow-Origin: *\r\n\r\n");
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// Audio Chunk Handler with DSP Fan Filter & Whisper AGC Boost
void handleAudioChunk() {
  addCorsHeaders();
  const size_t sampleCount = 8000; // 0.5 second at 16000Hz
  const size_t pcmBytes = sampleCount * sizeof(int16_t);
  
  // Allocate in internal SRAM for I2S DMA
  int16_t *pcmBuffer = (int16_t*)malloc(pcmBytes);
  if (!pcmBuffer) {
    server.send(500, "text/plain", "Audio internal RAM allocation failed");
    return;
  }

  size_t bytesRead = 0;
  esp_err_t r = i2s_read(I2S_MIC_CHANNEL, (void*)pcmBuffer, pcmBytes, &bytesRead, pdMS_TO_TICKS(500));

  int16_t peakVal = 0;
  if (r == ESP_OK && bytesRead > 0) {
    size_t samples = bytesRead / sizeof(int16_t);

    // 1. First pass: Measure ambient level to auto-detect whispers
    float maxAmp = 0.0f;
    for (size_t i = 0; i < samples; i++) {
      float a = fabsf((float)pcmBuffer[i]);
      if (a > maxAmp) maxAmp = a;
    }

    // 2. Adaptive Gain Control (AGC)
    // If whisper detected (maxAmp < 250), boost up to 36x!
    // If user provided gain in query parameter (?gain=X), use that.
    float targetGain = 24.0f;
    if (server.hasArg("gain")) {
      float userGain = server.arg("gain").toFloat();
      if (userGain >= 1.0f && userGain <= 60.0f) targetGain = userGain;
    } else {
      if (maxAmp < 200.0f) targetGain = 36.0f;     // Extreme whisper boost
      else if (maxAmp < 500.0f) targetGain = 28.0f; // Soft speech boost
      else if (maxAmp > 3000.0f) targetGain = 8.0f; // Loud speech limiter
    }

    // 3. DSP High-Pass IIR Filter: Cut frequencies below ~160Hz
    // Kills fan motor hum, wind noise, and DC bias without muffling speech
    static float x_prev = 0.0f;
    static float y_prev = 0.0f;
    const float alpha = 0.94f; // 150-180Hz cutoff

    for (size_t i = 0; i < samples; i++) {
      float x = (float)pcmBuffer[i];
      float y = x - x_prev + alpha * y_prev;
      x_prev = x;
      y_prev = y;

      float amplified = y * targetGain;
      // Soft saturation clamp
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
  String contentType = "application/octet-stream";
  if (filename.endsWith(".jpg")) contentType = "image/jpeg";
  else if (filename.endsWith(".wav")) contentType = "audio/wav";
  
  server.streamFile(file, contentType);
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
  Serial.println("\n======================================");
  Serial.println("  Seeed Studio XIAO ESP32-S3 Sense");
  Serial.println("  Surveillance & Power Management");
  Serial.println("======================================");

  // Initialize hardware components
  cameraInitialized = initCamera();
  micInitialized    = initMic();
  sdCardAvailable   = initSD();

  // Connect to Wi-Fi
  Serial.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
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

  // Register WebServer routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/set_mode", HTTP_GET, handleSetMode);
  server.on("/record", HTTP_GET, handleRecord);
  server.on("/take_snap", HTTP_GET, handleTakeSnap);
  server.on("/deep_sleep", HTTP_GET, handleDeepSleep);
  server.on("/stream", HTTP_GET, handleStream);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/audio_chunk", HTTP_GET, handleAudioChunk);
  server.on("/files", HTTP_GET, handleListFiles);
  server.on("/file", HTTP_GET, handleGetFile);
  server.on("/delete", HTTP_GET, handleDeleteFile);

  server.begin();
  Serial.println("[HTTP] Web Server Started on Port 80");

  // Default to Standby (Light Sleep) mode to preserve battery
  setPowerMode(MODE_STANDBY);
}

void loop() {
  server.handleClient();

  // Continuous background recording logic (if enabled)
  if (isRecording && sdCardAvailable && cameraInitialized) {
    if (millis() - lastRecordTime > 1000) { // 1 frame per second
      lastRecordTime = millis();
      saveSnapshotToSD();
    }
  }

  yield();
}