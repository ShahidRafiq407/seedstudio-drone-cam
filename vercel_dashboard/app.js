let espBase = localStorage.getItem('esp32_base_url') || '';
let currentMode = 'standby';
let isRecording = false;
let audioCtx = null;
let audioInterval = null;

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
  alert('Connecting to ' + espBase);
  fetchStatus();
}

function getUrl(path) {
  if (!espBase) return path;
  return espBase + path;
}

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

function toggleMode(mode) {
  if (!espBase) return alert('Please enter ESP32 IP or URL first!');
  const target = (currentMode === mode && mode !== 'standby') ? 'standby' : mode;
  fetch(getUrl('/set_mode?mode=' + target))
    .then(r => r.json())
    .then(d => { updateUI(d); })
    .catch(e => console.error(e));
}

function toggleRecord() {
  if (!espBase) return alert('Please enter ESP32 IP or URL first!');
  const action = isRecording ? 'stop' : 'start';
  fetch(getUrl('/record?action=' + action))
    .then(r => r.json())
    .then(d => { updateUI(d); })
    .catch(e => console.error(e));
}

function takeSnapshot() {
  if (!espBase) return alert('Please enter ESP32 IP or URL first!');
  fetch(getUrl('/take_snap'))
    .then(r => r.text())
    .then(t => {
      alert(t);
      loadFiles();
    });
}

function triggerDeepSleep() {
  if (!espBase) return alert('Please enter ESP32 IP or URL first!');
  if (confirm("ESP32 will enter Deep Sleep for 10 minutes to preserve battery.\nWiFi will be OFF.\nContinue?")) {
    fetch(getUrl('/deep_sleep?minutes=10'));
    alert("ESP32 is now in Deep Sleep. It will wake up after 10 minutes or upon pressing Reset.");
    document.getElementById('statusBadge').innerText = "Deep Sleep";
  }
}

function updateUI(status) {
  currentMode = status.mode;
  isRecording = status.recording;

  document.getElementById('teleWifi').innerText = status.wifi_rssi + " dBm";
  document.getElementById('teleMode').innerText = status.mode.toUpperCase();
  document.getElementById('teleSD').innerText = status.sd ? "Ready" : "No SD";
  document.getElementById('teleRam').innerText = Math.round(status.free_psram / 1024) + " KB";

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

  // Video feed
  const videoFeed = document.getElementById('videoFeed');
  const noFeedMsg = document.getElementById('noFeedMsg');
  if (currentMode === 'camera' || currentMode === 'turbo') {
    const streamSrc = getUrl('/stream?t=' + Date.now());
    if (videoFeed.src !== streamSrc) {
      videoFeed.src = streamSrc;
    }
    videoFeed.style.display = 'block';
    noFeedMsg.style.display = 'none';
  } else {
    videoFeed.src = '';
    videoFeed.style.display = 'none';
    noFeedMsg.style.display = 'flex';
  }

  // Audio stream
  if (currentMode === 'mic' || currentMode === 'turbo') {
    startAudioStream();
  } else {
    stopAudioStream();
  }
}

function resumeAudioContext() {
  if (!audioCtx) {
    audioCtx = new (window.AudioContext || window.webkitAudioContext)();
  }
  if (audioCtx.state === 'suspended') {
    audioCtx.resume();
  }
}

function startAudioStream() {
  if (audioInterval) return;
  resumeAudioContext();
  document.getElementById('audioStatus').innerText = "Streaming (16kHz Active)";

  let isFetching = false;
  audioInterval = setInterval(() => {
    if (isFetching) return;
    isFetching = true;
    fetch(getUrl('/audio_chunk?t=' + Date.now()))
      .then(res => {
        const peak = res.headers.get('X-Peak-Volume');
        if (peak) {
          document.getElementById('audioStatus').innerText = `Active (Peak: ${peak})`;
        }
        return res.arrayBuffer();
      })
      .then(buf => {
        isFetching = false;
        if (!buf || buf.byteLength <= 44) return;
        resumeAudioContext();
        audioCtx.decodeAudioData(buf.slice(0), (audioBuffer) => {
          const src = audioCtx.createBufferSource();
          src.buffer = audioBuffer;
          src.connect(audioCtx.destination);
          src.start();

          const pcm = audioBuffer.getChannelData(0);
          const sampleWave = [];
          const step = Math.floor(pcm.length / 80) || 1;
          for (let i = 0; i < pcm.length; i += step) {
            sampleWave.push((pcm[i] + 1) * 128);
          }
          drawWave(sampleWave);
        }, (err) => {
          console.warn("Audio decode err:", err);
        });
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

function loadFiles() {
  if (!espBase) return;
  fetch(getUrl('/files'))
    .then(r => r.json())
    .then(files => {
      const fl = document.getElementById('fileList');
      if (!files || files.length === 0) {
        fl.innerHTML = '<div style="text-align:center; color:var(--text-muted); padding:10px;">No recordings found on SD card</div>';
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

function fetchStatus() {
  if (!espBase) return;
  fetch(getUrl('/status'))
    .then(r => r.json())
    .then(d => updateUI(d))
    .catch(e => {});
}

setInterval(fetchStatus, 3000);
if (espBase) fetchStatus();
