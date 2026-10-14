@echo off
title XIAO Audio Surveillance - Permanent Cloudflare Tunnel
echo ===================================================
echo   XIAO ESP32-S3 Audio Surveillance Live Tunnel
echo ===================================================
echo.
cd /d "%~dp0"
echo Starting Cloudflare Tunnel for http://10.49.247.48 ...
echo Press Ctrl+C to stop.
echo.
cloudflared.exe tunnel --edge-ip-version 4 --url http://10.49.247.48
pause
