# WebSocket Server Hub for Waveshare ESP32-S3 AprilTag System

This directory contains the Python WebSocket server that receives AprilTag telemetry data from the Waveshare ESP32-S3 Touch LCD 2.0 device and forwards it to game engines (GDevelop, etc.).

## 📋 System Architecture

```
Waveshare ESP32-S3 (Camera + AprilTag Detection)
    ↓ WiFi: Chan-Comcast
    ↓ WebSocket: ws://76.102.42.17:5000/websocket
Ubuntu Server Hub (Python WebSocket Server) ← YOU ARE HERE
    ↓ WebSocket
GDevelop Game Engine (Receives vehicle/telemetry data)
```

## 🚀 Quick Start

### Windows Instructions (Recommended: Virtual Environment)

#### 1. Install Python (if not already installed)
- Download Python 3.8+ from: https://www.python.org/downloads/
- During installation, **check "Add Python to PATH"**
- Verify installation: Open Command Prompt and type `python --version`

#### 2. Create Virtual Environment (Recommended)

Open Command Prompt (cmd.exe) or PowerShell:

```cmd
cd c:\12i-Db\Dropbox\09k-E32-SM\25-0517-1900-E32--OPENED\26-0105-0500-Waveshare-E32S3_TouchDisplay2p0In_Cam-PioarduinoIde-NOW\src\09_ZA-Stage_02-ServerHub_WebSockets

REM Create virtual environment
python -m venv venv

REM Activate virtual environment
venv\Scripts\activate
```

**Why use venv?**
- ✅ Isolates dependencies from system Python
- ✅ Prevents version conflicts with other projects
- ✅ Easy to delete/recreate if issues occur
- ✅ Professional best practice

#### 3. Install Dependencies (inside venv)

```cmd
REM Make sure venv is activated (you should see "(venv)" in prompt)
pip install -r 0a-requirements.txt
```

#### 4. Run Server (inside venv)

```cmd
REM Make sure venv is activated
python 02-ubuntu_server_websocket-NOW.py
```

**Note:** Use `python` (not `python3`) on Windows.

#### 5. Verify Server is Running

Open browser to: `http://localhost:5000/`

You should see the server status dashboard.

#### 6. Find Your Windows IP Address (for ESP32 connection)

In Command Prompt:
```cmd
ipconfig
```

Look for "IPv4 Address" under your active network adapter (WiFi or Ethernet).
Example: `192.168.1.100`

**Important:** Update ESP32 code with your Windows IP:
```cpp
const char* ws_host = "192.168.1.100";  // Replace with YOUR Windows IP
```

#### 7. Deactivate Virtual Environment (when done)

```cmd
deactivate
```

### Linux/Ubuntu Instructions (Recommended: Virtual Environment)

#### 1. Create Virtual Environment (Recommended)

```bash
cd src/09_ZA-Stage_02-ServerHub_WebSockets

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate
```

**Why use venv?**
- ✅ Isolates dependencies from system Python
- ✅ Prevents version conflicts with other projects
- ✅ Easy to delete/recreate if issues occur
- ✅ Professional best practice

#### 2. Install Dependencies (inside venv)

```bash
# Make sure venv is activated (you should see "(venv)" in prompt)
pip install -r 0a-requirements.txt
```

#### 3. Run Server (inside venv)

```bash
# Make sure venv is activated
python3 02-ubuntu_server_websocket-NOW.py
```

#### 4. Verify Server is Running

Open browser to: `http://localhost:5000/`

You should see the server status dashboard.

#### 5. Deactivate Virtual Environment (when done)

```bash
deactivate
```

## 📡 Server Endpoints

### WebSocket Endpoint
- **URL:** `ws://[SERVER_IP]:5000/websocket`
- **Protocol:** WebSocket (flask-sock)
- **Clients:** ESP32 cameras, GDevelop game engines

### HTTP Endpoints
- **`/`** - Server status dashboard (auto-refresh)
- **`/video`** - Live camera video viewer (auto-refresh)
- **`/photo`** - Latest camera photo (JPEG)
- **`/status`** - JSON status API
- **`/video_fps`** - Video FPS and resolution data
- **`/video_stats`** - Comprehensive performance metrics
- **`/connection_uptime`** - ESP32 connection uptime
- **`/connection_history`** - ESP32 connection history

## 🔧 Configuration

### Server Settings (in Python file)
```python
SERVER_PORT = 5000
SERVER_HOST = '0.0.0.0'  # Listen on all interfaces
AUTH_TOKEN = "Jesus333!!!"  # Must match ESP32
```

### ESP32 Settings (in .ino file)
```cpp
const char* ws_host = "76.102.42.17";  // Server IP
const uint16_t ws_port = 5000;
const char* ws_path = "/websocket";
```

## 📨 WebSocket Message Format

### ESP32 → Server (AprilTag Data)
```json
{
  "event": "apriltag_data",
  "smartcam_ip": "192.168.1.100",
  "tag_id": 5,
  "yaw": -15.2,
  "pitch": 8.7,
  "roll": 2.1,
  "x_cm": 10.5,
  "y_cm": -5.3,
  "z_cm": 42.0,
  "tag_size_percent": 15.5,
  "distance_cm": 42.0,
  "timestamp": 12345678
}
```

### Server → GDevelop (Forwarded Data)
Same format as above - server forwards ESP32 data to all connected GDevelop clients.

### ESP32 → Server (Identify)
```json
{
  "event": "identify",
  "data": {
    "type": "esp32",
    "device": "Waveshare-ESP32-S3",
    "auth_token": "Jesus333!!!"
  }
}
```

### GDevelop → Server (Identify)
```json
{
  "event": "identify",
  "data": {
    "type": "gdevelop",
    "name": "MyGame"
  }
}
```

## 🔒 Security

- **Authentication:** ESP32 must provide `auth_token` in identify message
- **Token:** Change `AUTH_TOKEN` in Python file for production
- **Network:** Currently uses `ws://` (plain text). For production, use VPN or upgrade to `wss://` (SSL)

## 📊 Performance Monitoring

The server tracks detailed performance metrics:
- **FPS:** Actual vs target frame rate
- **Jitter:** Timing consistency between frames
- **Efficiency:** How well actual performance matches target
- **Processing Time:** Server-side frame processing latency

View metrics at: `http://[SERVER_IP]:5000/video`

## 🐛 Troubleshooting

### ESP32 Won't Connect
1. Check WiFi credentials in ESP32 code
2. Verify server IP address matches `ws_host`
3. Check firewall allows port 5000
4. Verify server is running: `http://[SERVER_IP]:5000/status`

### No AprilTag Data
1. Check ESP32 Serial Monitor for detection messages
2. Verify WebSocket connection: Check `/status` endpoint
3. Ensure AprilTag is visible to camera (proper size/distance)

### Video Not Updating
1. Check `/video_fps` endpoint for actual FPS
2. Verify ESP32 is sending frames (check Serial Monitor)
3. Check network bandwidth (reduce JPEG quality if needed)

## 📚 Related Files

- **ESP32 Code:** `../09_lvgl_camera-Z01-SimpleTagDetect_April-NOW/`
- **GDevelop Project:** `../09_ZB-Stage_03-DigitalGameGraphicEngine/`

## 🔗 Source Reference

This server hub was migrated from:
```
C:\12i-Db\Dropbox\09k-E32-SM\25-0517-1900-E32--OPENED\
13i-T-CameraPlus-S3-NOW-Ubuntu22_BmaxB1Pro--25-0505-0730-NOW\
25-1123-1700-E32_TCameraPlusS3-AprilTag-SerialToMicrobit-HttpsCorsToGDevelop\
examples\Camera_Screen_AprilTag__Serial_With_Microbit--Esp32_Client_Websocket-NOW\
02-Ubuntu-Server_Hub\
```

## 📝 Version History

- **26-0124-1130:** Initial migration to Waveshare ESP32-S3 project
- **25-1202-0340:** Switched from simple-websocket to flask-sock
- **25-1209-1600:** Added dynamic FPS auto-detection
- **25-1215-1700:** Added connection history tracking
