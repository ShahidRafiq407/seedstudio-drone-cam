let espBase = localStorage.getItem('esp32_base_url') || '';
let currentMode = 'standby';
let isRecording = false;
let isSwitching = false;
let sdMounted = false;

// Audio processing nodes
let audioCtx = null;
let gainNode = null;
let filterNode = null;
let noiseFilterEnabled = true;
let userVolume = 2.5;
let audioInterval = null;

// Phone Audio Recording variables
let isPhoneRecording = false;
let phoneRecordedChunks = [];
let phoneRecStartTime = 0;
let phoneRecTimer = null;

const inputEl = document.getElementById('espUrl');
if (espBase) {
  inputEl.value = espBase;
}

function saveEspUrl() {
  let val = inputEl.value.trim();
  if (val && !val.startsWith('http://') && !val.startsWith('https://')) {
    val = 'http://' + val;
  }
  if (val.endsWith('/')) {
    val = val.slice(0, -1);
  }
  espBase = val;
  localStorage.setItem('esp32_base_url', espBase);
  alert('Connected to: ' + espBase);
  fetchStatus();
}

function getUrl(path) {
  if (!espBase) return path;
  return espBase + path;
}

// Waveform Canvas Setup
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

// Web Audio System & DSP Filters
function initAudioPipeline() {
  if (!audioCtx) {
    audioCtx = new (window.AudioContext || window.webkitAudioContext)();
  }
  if (!gainNode) {
    gainNode = audioCtx.createGain();
    gainNode.gain.value = userVolume;
  }
  if (!filterNode) {
    // High-pass filter at 180Hz to eliminate fan rumble and AC hum
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
  if (gainNode) {
    gainNode.gain.value = userVolume;
  }
}

function toggleNoiseFilter() {
  noiseFilterEnabled = !noiseFilterEnabled;
  const btn = document.getElementById('btnNoiseFilter');
  btn.classList.toggle('active', noiseFilterEnabled);
  btn.innerText = noiseFilterEnabled ? '🍃 Fan Filter: ON' : '🍃 Fan Filter: OFF';
  if (filterNode) {
    filterNode.frequency.value = noiseFilterEnabled ? 180 : 20;
  }
}

function resumeAudio() {
  initAudioPipeline();
  if (audioCtx && audioCtx.state === 'suspended') {
    audioCtx.resume();
  }
}

// Mode Switching Logic
async function handleModeClick(requestedMode) {
  if (isSwitching) return;
  isSwitching = true;

  // Toggle behavior: If active button is clicked again, return to standby
  let targetMode = requestedMode;
  if (currentMode === requestedMode && requestedMode !== 'standby') {
    targetMode = 'standby';
  }

  // 1. Immediately terminate active video & audio streams on client
  const videoFeed = document.getElementById('videoFeed');
  if (currentMode === 'camera' || currentMode === 'turbo') {
    videoFeed.src = ''; // Drops TCP socket to ESP32 stream handler immediately
    videoFeed.style.display = 'none';
    document.getElementById('noFeedMsg').style.display = 'flex';
  }
  stopAudioStream();

  // Allow a tiny 60ms gap for socket FIN handshake
  await new Promise(r => setTimeout(r, 60));

  // 2. Send mode change to ESP32
  try {
    const res = await fetch(getUrl('/set_mode?mode=' + targetMode));
    const data = await res.json();
    updateUI(data);
  } catch (err) {
    console.error('Mode switch failed:', err);
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
  fetch(getUrl('/record?action=' + action))
    .then(r => r.json())
    .then(d => updateUI(d))
    .catch(e => console.error(e));
}

function takeSnapshot() {
  if (!sdMounted) {
    alert("⚠️ No MicroSD Card detected! Please insert an SD card to save snapshots.");
    return;
  }
  fetch(getUrl('/take_snap'))
    .then(r => r.text())
    .then(t => {
      alert(t);
      loadFiles();
    })
    .catch(e => alert('Failed to capture snapshot'));
}

// Deep Sleep Modal
function openDeepSleepModal() {
  document.getElementById('sleepModal').style.display = 'flex';
}

function closeDeepSleepModal() {
  document.getElementById('sleepModal').style.display = 'none';
}

function confirmDeepSleep(minutes) {
  closeDeepSleepModal();
  if (confirm(`ESP32 will sleep for ${minutes} minutes to save battery.\nWiFi will be OFF.\nContinue?`)) {
    // Stop video and audio
    document.getElementById('videoFeed').src = '';
    stopAudioStream();
    fetch(getUrl('/deep_sleep?minutes=' + minutes));
    alert(`ESP32 is now in Deep Sleep for ${minutes} minutes.`);
    document.getElementById('statusBadge').innerText = "DEEP SLEEP";
    document.getElementById('teleMode').innerText = "DEEP SLEEP";
  }
}

// Telemetry & UI Updates
function updateUI(status) {
  currentMode = status.mode;
  isRecording = status.recording;
  sdMounted = !!status.sd;

  document.getElementById('teleWifi').innerText = (status.wifi_rssi || '--') + " dBm";
  document.getElementById('teleMode').innerText = status.mode.toUpperCase();
  document.getElementById('teleRam').innerText = Math.round((status.free_psram || 0) / 1024) + " KB";

  // SD Card status badge & text
  const sdEl = document.getElementById('teleSD');
  if (sdMounted) {
    const freeGb = (status.sd_free_mb / 1024).toFixed(1);
    sdEl.innerText = `Ready (${freeGb}GB Free)`;
    sdEl.style.color = 'var(--green)';
  } else {
    sdEl.innerText = "No SD Card";
    sdEl.style.color = 'var(--red)';
  }

  // Battery Status
  const batEl = document.getElementById('teleBat');
  if (batEl) {
    if (status.battery_connected) {
      batEl.innerText = `${status.battery_v}V (${status.battery_pct}%)`;
      if (status.battery_pct > 50) batEl.style.color = 'var(--green)';
      else if (status.battery_pct > 20) batEl.style.color = '#ffd166';
      else batEl.style.color = 'var(--red)';
    } else {
      batEl.innerText = "Needs Divider on A0";
      batEl.style.color = 'var(--text-muted)';
    }
  }

  // Active button styling
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

  // Manage Video Stream
  const videoFeed = document.getElementById('videoFeed');
  const noFeedMsg = document.getElementById('noFeedMsg');
  if (currentMode === 'camera' || currentMode === 'turbo') {
    const streamUrl = getUrl('/stream?t=' + Date.now());
    if (videoFeed.src !== streamUrl) {
      videoFeed.src = streamUrl;
    }
    videoFeed.style.display = 'block';
    noFeedMsg.style.display = 'none';
  } else {
    videoFeed.src = '';
    videoFeed.style.display = 'none';
    noFeedMsg.style.display = 'flex';
  }

  // Manage Audio Stream
  if (currentMode === 'mic' || currentMode === 'turbo') {
    startAudioStream();
  } else {
    stopAudioStream();
  }
}

// Live Audio Streaming & Playback
function startAudioStream() {
  if (audioInterval) return;
  resumeAudio();
  document.getElementById('audioStatus').innerText = "Active (Listening)";

  let isFetching = false;
  audioInterval = setInterval(() => {
    if (isFetching) return;
    isFetching = true;

    // Send gain parameter to firmware AGC
    const gainParam = Math.round(userVolume * 10);
    fetch(getUrl('/audio_chunk?gain=' + gainParam + '&t=' + Date.now()))
      .then(res => {
        const peak = res.headers.get('X-Peak-Volume');
        if (peak) {
          const peakNum = parseInt(peak, 10);
          if (peakNum > 20000) {
            document.getElementById('audioStatus').innerText = `Loud (${peak})`;
          } else if (peakNum > 1000) {
            document.getElementById('audioStatus').innerText = `Speech (${peak})`;
          } else {
            document.getElementById('audioStatus').innerText = `Quiet (${peak})`;
          }
        }
        return res.arrayBuffer();
      })
      .then(buf => {
        isFetching = false;
        if (!buf || buf.byteLength <= 44) return;
        resumeAudio();

        // If user is recording to phone, store raw PCM
        if (isPhoneRecording) {
          phoneRecordedChunks.push(new Int16Array(buf.slice(44)));
        }

        audioCtx.decodeAudioData(buf.slice(0), (audioBuffer) => {
          const src = audioCtx.createBufferSource();
          src.buffer = audioBuffer;
          // Connect through DSP Filter -> Gain Booster -> Destination
          src.connect(filterNode);
          src.start();

          // Render Waveform Canvas
          const pcm = audioBuffer.getChannelData(0);
          const sampleWave = [];
          const step = Math.floor(pcm.length / 80) || 1;
          for (let i = 0; i < pcm.length; i += step) {
            sampleWave.push((pcm[i] + 1) * 128);
          }
          drawWave(sampleWave);
        }, (err) => {});
      }).catch(e => {
        isFetching = false;
      });
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

// Record Audio Locally to Mobile Phone
function togglePhoneAudioRecording() {
  resumeAudio();
  const btn = document.getElementById('btnPhoneRec');
  const txt = document.getElementById('phoneRecText');

  if (!isPhoneRecording) {
    // Start recording
    isPhoneRecording = true;
    phoneRecordedChunks = [];
    phoneRecStartTime = Date.now();
    btn.classList.add('recording');

    // Make sure audio is playing/fetching
    if (currentMode !== 'mic' && currentMode !== 'turbo') {
      handleModeClick('mic');
    }

    phoneRecTimer = setInterval(() => {
      const elapsed = Math.floor((Date.now() - phoneRecStartTime) / 1000);
      const m = String(Math.floor(elapsed / 60)).padStart(2, '0');
      const s = String(elapsed % 60).padStart(2, '0');
      txt.innerText = `⏹️ Stop & Save (${m}:${s})`;
    }, 1000);
  } else {
    // Stop recording and download WAV
    isPhoneRecording = false;
    clearInterval(phoneRecTimer);
    btn.classList.remove('recording');
    txt.innerText = "Record Audio to Phone";

    if (phoneRecordedChunks.length === 0) {
      alert("No audio data recorded!");
      return;
    }

    // Build standard WAV file in browser
    savePhoneWavFile(phoneRecordedChunks);
  }
}

function savePhoneWavFile(chunks) {
  let totalSamples = 0;
  for (let c of chunks) totalSamples += c.length;

  const buffer = new ArrayBuffer(44 + totalSamples * 2);
  const view = new DataView(buffer);

  // RIFF identifier
  writeString(view, 0, 'RIFF');
  view.setUint32(4, 36 + totalSamples * 2, true);
  writeString(view, 8, 'WAVE');
  writeString(view, 12, 'fmt ');
  view.setUint32(16, 16, true); // SubChunk1Size
  view.setUint16(20, 1, true);  // PCM format
  view.setUint16(22, 1, true);  // Mono
  view.setUint32(24, 16000, true); // Sample rate
  view.setUint32(28, 32000, true); // Byte rate
  view.setUint16(32, 2, true);  // Block align
  view.setUint16(34, 16, true); // Bits per sample
  writeString(view, 36, 'data');
  view.setUint32(40, totalSamples * 2, true);

  // Write PCM audio data
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
  const timestamp = now.toISOString().replace(/[-:T]/g, '_').slice(0, 15);
  a.href = url;
  a.download = `Audio_Voice_${timestamp}.wav`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

// Record Live Video Directly on Mobile Phone (HD Canvas + MediaRecorder)
let isPhoneVideoRecording = false;
let mediaRecorder = null;
let recordedVideoChunks = [];
let videoRecTimer = null;
let videoRecStartTime = 0;
let recCanvas = null;
let recDrawInterval = null;

function togglePhoneVideoRecording() {
  if (currentMode !== 'camera' && currentMode !== 'turbo') {
    alert("Please turn on Camera or Turbo Mode first before recording video!");
    return;
  }

  const btn = document.getElementById('btnPhoneVideoRec');
  const txt = document.getElementById('phoneVideoRecText');
  const videoFeed = document.getElementById('videoFeed');

  if (!isPhoneVideoRecording) {
    // Start recording video
    isPhoneVideoRecording = true;
    recordedVideoChunks = [];
    videoRecStartTime = Date.now();
    btn.classList.add('recording');

    if (!recCanvas) {
      recCanvas = document.createElement('canvas');
      recCanvas.width = 640;
      recCanvas.height = 480;
    }
    const recCtx = recCanvas.getContext('2d');

    recDrawInterval = setInterval(() => {
      if (videoFeed && videoFeed.naturalWidth) {
        recCtx.drawImage(videoFeed, 0, 0, 640, 480);
      }
    }, 66); // ~15 FPS capture

    const stream = recCanvas.captureStream(15);

    try {
      mediaRecorder = new MediaRecorder(stream, { mimeType: 'video/webm' });
    } catch(e) {
      try {
        mediaRecorder = new MediaRecorder(stream);
      } catch(err2) {
        alert("Video recording not supported on this browser: " + err2.message);
        isPhoneVideoRecording = false;
        clearInterval(recDrawInterval);
        btn.classList.remove('recording');
        return;
      }
    }

    mediaRecorder.ondataavailable = (e) => {
      if (e.data && e.data.size > 0) {
        recordedVideoChunks.push(e.data);
      }
    };

    mediaRecorder.onstop = () => {
      const blob = new Blob(recordedVideoChunks, { type: 'video/webm' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      const now = new Date();
      const ts = now.toISOString().replace(/[-:T]/g, '_').slice(0, 15);
      a.href = url;
      a.download = `Drone_Video_${ts}.webm`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    };

    mediaRecorder.start(250);

    videoRecTimer = setInterval(() => {
      const elapsed = Math.floor((Date.now() - videoRecStartTime) / 1000);
      const m = String(Math.floor(elapsed / 60)).padStart(2, '0');
      const s = String(elapsed % 60).padStart(2, '0');
      txt.innerText = `⏹️ Stop Video (${m}:${s})`;
    }, 1000);

  } else {
    // Stop recording
    isPhoneVideoRecording = false;
    clearInterval(videoRecTimer);
    clearInterval(recDrawInterval);
    btn.classList.remove('recording');
    txt.innerText = "Record Video to Phone";

    if (mediaRecorder && mediaRecorder.state !== 'inactive') {
      mediaRecorder.stop();
    }
  }
}

function writeString(view, offset, string) {
  for (let i = 0; i < string.length; i++) {
    view.setUint8(offset + i, string.charCodeAt(i));
  }
}

// MicroSD File Management
function loadFiles() {
  if (!espBase) return;
  fetch(getUrl('/files'))
    .then(r => r.json())
    .then(files => {
      const fl = document.getElementById('fileList');
      if (!files || files.length === 0) {
        fl.innerHTML = '<div style="text-align:center; color:var(--text-muted); padding:10px;">No files on SD card</div>';
        return;
      }
      fl.innerHTML = files.map(f => `
        <div class="file-item">
          <span>📄 ${f.name} (${Math.round(f.size/1024)} KB)</span>
          <div class="file-actions">
            <a href="${getUrl('/file?name=' + f.name)}" target="_blank" download>Download</a>
            <button class="del-btn" onclick="deleteFile('${f.name}')">Delete</button>
          </div>
        </div>
      `).join('');
    }).catch(e => console.error(e));
}

function deleteFile(name) {
  if (confirm("Delete " + name + "?")) {
    fetch(getUrl('/delete?name=' + name)).then(() => loadFiles());
  }
}

// Telemetry Polling
function fetchStatus() {
  if (!espBase) return;
  fetch(getUrl('/status'))
    .then(r => r.json())
    .then(d => updateUI(d))
    .catch(e => {});
}

setInterval(fetchStatus, 3000);
if (espBase) fetchStatus();
