# Migration Guide: Lilygo T-CameraPlus-S3 → Waveshare ESP32-S3-Touch-LCD-2

**Date:** 2026-01-24  
**Author:** jwc  
**Purpose:** Migrate AprilTag detection + WebSocket streaming from Lilygo to Waveshare platform

---

## 📊 Overview

This guide documents the migration of major features from the **Lilygo T-CameraPlus-S3** to the **Waveshare ESP32-S3-Touch-LCD-2** platform, enabling real-time AprilTag detection and streaming to game engines (GDevelop, Unreal, Unity).

### **Source System (Lilygo T-CameraPlus-S3)**
- **Display:** 1.3" TFT (240x240 pixels)
- **Camera:** OV2640 at 240x240 grayscale
- **Communication:** WiFi + WebSocket + Serial (GPIO 50/49)
- **Features:** AprilTag detection, pose estimation, WebSocket streaming

### **Target System (Waveshare ESP32-S3-Touch-LCD-2)**
- **Display:** 2.0" Touch LCD (240x320 pixels) with LVGL
- **Camera:** OV2640 at 480x320 (cropped to 240x320 for display)
- **Communication:** Serial (GPIO 44/18) → **Adding WiFi + WebSocket**
- **Features:** AprilTag detection (existing) → **Adding WebSocket streaming**

---

## 🎯 Migration Difficulty: MODERATE (6/10)

**Estimated Time:** 6-14 hours (most likely 8-10 hours)

**Why Moderate?**
- ✅ Waveshare already has 60% of features (camera, AprilTag, display, serial)
- ✅ Lilygo code is extremely well-documented
- ⚠️ Need to add WiFi + WebSocket libraries (~500 lines)
- ⚠️ Need to adapt camera calibration for different resolution

---

## 📋 Major Features to Migrate

### ✅ **Already Working on Waveshare**
1. AprilTag detection (Tag36h11 family)
2. Pose estimation (6DOF: x, y, z, yaw, pitch, roll)
3. Camera display (GFX rendering)
4. Serial communication with micro:bit
5. Touch UI with LVGL

### 🔧 **To Be Added**
1. WiFi connectivity
2. WebSocket client (real-time streaming)
3. AprilTag data queue (FIFO buffer)
4. Rate limiting (capture vs send)
5. Performance monitoring (FPS, queue stats)
6. Display feedback (connection status, counters)

---

## 🚀 Implementation Plan: 7 Phases

### **Phase 1: Add WiFi & WebSocket Libraries**

**Files to modify:** `09_lvgl_camera-Z01-SimpleTagDetect_April.ino`

**Add at top of file (after existing includes):**
```cpp
//// jwc 26-0124-0000 NEW: WiFi and WebSocket libraries for server communication
#include <WiFi.h>
#include <WebSocketsClient.h>  // Links2004 library
#include <ArduinoJson.h>        // JSON serialization
```

**Add WiFi credentials (after includes):**
```cpp
//// jwc 26-0124-0000 NEW: WiFi configuration
const char* WIFI_SSID = "Chan-Comcast";      // Your WiFi network name
const char* WIFI_PASSWORD = "Jesus333!";     // Your WiFi password
```

**Add WebSocket configuration:**
```cpp
//// jwc 26-0124-0000 NEW: WebSocket server configuration
const char* WS_HOST = "76.102.42.17";        // Server IP (public or private)
const uint16_t WS_PORT = 5000;               // Server port
const char* WS_PATH = "/websocket";          // WebSocket endpoint
const char* AUTH_TOKEN = "Jesus333!!!";      // Authentication token (must match server)
const bool WS_USE_SSL = false;               // false = ws://, true = wss://
```

**Add global WebSocket objects:**
```cpp
//// jwc 26-0124-0000 NEW: WebSocket client globals
WebSocketsClient webSocket;                  // WebSocket client instance
bool webSocketConnected = false;             // Connection status flag
unsigned long totalMessagesSent = 0;         // Statistics: messages sent
unsigned long totalMessagesCaptured = 0;     // Statistics: tags captured
```

---

### **Phase 2: Add AprilTag Data Queue & Timing**

**Add timing constants (after WebSocket config):**
```cpp
//// jwc 26-0124-0000 NEW: Rate limiting configuration
// These constants control how fast AprilTags are captured and transmitted
// Adjust these values to balance responsiveness vs network load

// How often to ADD detected tags to queue (capture rate)
const unsigned long AprilTag_Capture_INTERVAL_MS = 1000;  // 1.0 FPS (1 tag/second)

// How often to SEND queued tags via WebSocket (transmission rate)
const unsigned long AprilTag_Send_INTERVAL_MS = 1000;     // 1.0 FPS (1 msg/second)

// Timing trackers
unsigned long list_add_time_last = 0;        // Last time tag was added to queue
unsigned long http_send_time_last = 0;       // Last time tag was sent via WebSocket
```

**Add queue structure (after timing constants):**
```cpp
//// jwc 26-0124-0000 NEW: AprilTag event queue (FIFO buffer)
// This queue decouples detection (fast) from transmission (rate-limited)
// Benefits:
//   - Camera processing never blocks on network delays
//   - Smooth transmission rate (no bursts)
//   - Can handle multiple tags in one frame
//   - Absorbs temporary network hiccups

const int tagData_MAX = 50;  // Buffer size (50 events = ~1.6KB RAM)

struct tagData_Struct {
    int tag_id;                  // AprilTag ID (0-255)
    float yaw;                   // Camera orientation (degrees)
    float pitch;                 // Camera orientation (degrees)
    float roll;                  // Camera orientation (degrees)
    float x_cm;                  // Tag position X (centimeters)
    float y_cm;                  // Tag position Y (centimeters)
    float z_cm;                  // Tag position Z (centimeters)
    float tag_size_percent;      // Tag size as % of screen
    float distance_cm;           // Distance from camera (centimeters)
    unsigned long timestamp;     // Capture time (milliseconds)
};

tagData_Struct tagData_Queue[tagData_MAX];  // Circular buffer
int list_head = 0;                          // Write position (0 to tagData_MAX-1)
int list_count = 0;                         // Number of events in queue
```

**Add queue management functions (before `setup()`):**
```cpp
//// jwc 26-0124-0000 NEW: Queue management functions

// Add tag detection to queue (called when tag is detected)
void listTagEvent_Add(int id, float yaw, float pitch, float roll,
                      float x_cm, float y_cm, float z_cm,
                      float tag_size_percent, float distance_cm) {
    if (list_count < tagData_MAX) {
        // Add to queue
        tagData_Queue[list_head] = {id, yaw, pitch, roll, x_cm, y_cm, z_cm,
                                    tag_size_percent, distance_cm, millis()};
        list_head = (list_head + 1) % tagData_MAX;  // Circular buffer wrap
        list_count++;
        totalMessagesCaptured++;
        
        #if DEBUG >= 1
        Serial.printf("*** ADDED TO QUEUE: Tag ID=%d, Queue=%d/%d, Total=%lu\n",
                     id, list_count, tagData_MAX, totalMessagesCaptured);
        #endif
    } else {
        Serial.printf("*** QUEUE FULL! Dropped Tag ID=%d\n", id);
    }
}

// Remove oldest tag from queue (called when sending via WebSocket)
bool listTagEvent_Remove(tagData_Struct* out_tag) {
    if (list_count > 0) {
        // Calculate read position (oldest event = FIFO)
        int read_pos = (list_head - list_count + tagData_MAX) % tagData_MAX;
        *out_tag = tagData_Queue[read_pos];
        list_count--;
        return true;
    }
    return false;
}
```

---

### **Phase 3: Update Camera Calibration Parameters**

**Find existing calibration section (search for `#define FX`):**
```cpp
//// jwc 26-0124-0000 OLD: Original calibration (for different camera/resolution)
//// These values were for a larger resolution and different camera module
//// #define FX 924.713610878  // fx (focal length in pixels)
//// #define FY 924.713610878  // fy (focal length in pixels)
//// #define CX 403.801748132  // cx (principal point X in pixels)
//// #define CY 305.082642826  // cy (principal point Y in pixels)

//// jwc 26-0124-0000 NEW: Scaled calibration for 240x320 display
// These parameters are scaled from Lilygo's proven 240x240 calibration
// Lilygo: FX=200, FY=200, CX=120, CY=120 (for 240x240)
// Waveshare: Scale FY for 320px height, adjust CY for center
//
// CALCULATION:
//   FX = 200.0 (same, width is still 240px)
//   FY = 200.0 * (320/240) = 266.7 (scaled for taller display)
//   CX = 240/2 = 120.0 (center of 240px width)
//   CY = 320/2 = 160.0 (center of 320px height)
//
// NOTE: For production use, perform proper camera calibration using:
//   - OpenCV calibration tools with checkerboard pattern
//   - Multiple images at different angles/distances
//   - Calculate distortion coefficients if needed
//
#define FX 200.0   // Focal length X (pixels) - same as Lilygo
#define FY 266.7   // Focal length Y (pixels) - scaled for 320px height
#define CX 120.0   // Principal point X (pixels) - center of 240px width
#define CY 160.0   // Principal point Y (pixels) - center of 320px height
```

**Explanation:**
- **FX/FY:** Focal length in pixels (how "zoomed in" the camera is)
- **CX/CY:** Principal point (optical center of the image)
- These values convert 2D pixel coordinates → 3D world coordinates
- Scaling maintains aspect ratio from Lilygo's proven calibration

---

### **Phase 4: Add WiFi Initialization Function**

**Add before `setup()` function:**
```cpp
//// jwc 26-0124-0000 NEW: WiFi initialization function
void initWiFi() {
    Serial.printf("\n*** WiFi Init: Starting...\n");
    Serial.printf("*** WiFi SSID: %s\n", WIFI_SSID);
    Serial.printf("*** WiFi Band: 2.4GHz ONLY (ESP32-S3 hardware limitation)\n");
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n*** WiFi Connected! SmartCam-IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("*** WebSocket Server: ws://%s:%d%s\n", WS_HOST, WS_PORT, WS_PATH);
    } else {
        Serial.println("\n*** WiFi Failed to connect");
        Serial.println("*** Check SSID/password and ensure 2.4GHz band is enabled");
    }
}
```

---

### **Phase 5: Add WebSocket Event Handler**

**Add before `setup()` function:**
```cpp
//// jwc 26-0124-0000 NEW: WebSocket event handler
// This function is called automatically by the WebSocket library
// when connection state changes or messages are received
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("\n❌ WebSocket DISCONNECTED");
            webSocketConnected = false;
            break;
            
        case WStype_CONNECTED: {
            Serial.printf("\n✅ WebSocket CONNECTED | URL=ws://%s:%d%s\n", 
                         WS_HOST, WS_PORT, WS_PATH);
            webSocketConnected = true;
            
            // Send identify message with authentication
            StaticJsonDocument<256> identifyDoc;
            identifyDoc["event"] = "identify";
            JsonObject identifyData = identifyDoc.createNestedObject("data");
            identifyData["type"] = "esp32";
            identifyData["device"] = "Waveshare-ESP32-S3-Touch-LCD-2";
            identifyData["version"] = "1.0.0";
            identifyData["auth_token"] = AUTH_TOKEN;
            
            String identifyJson;
            serializeJson(identifyDoc, identifyJson);
            webSocket.sendTXT(identifyJson);
            
            Serial.printf("🔵<<-- SEND: identify | Auth=%s\n", AUTH_TOKEN);
            break;
        }
            
        case WStype_TEXT:
            Serial.printf("🟢-->> RECV: %s\n", (char*)payload);
            
            // Parse server responses (optional - for future features)
            StaticJsonDocument<512> doc;
            DeserializationError error = deserializeJson(doc, payload, length);
            
            if (!error) {
                const char* event = doc["event"];
                
                if (strcmp(event, "connection_success") == 0) {
                    Serial.println("      ✅ Server: connection acknowledged");
                }
                else if (strcmp(event, "identify_success") == 0) {
                    Serial.println("      ✅ Server: identified & authenticated");
                }
                else if (strcmp(event, "apriltag_ack") == 0) {
                    Serial.println("      ✅ Server: apriltag_data acknowledged");
                }
            }
            break;
            
        case WStype_ERROR:
            Serial.println("❌ WebSocket error occurred");
            break;
    }
}
```

---

### **Phase 6: Add AprilTag Data Transmission Function**

**Add before `setup()` function:**
```cpp
//// jwc 26-0124-0000 NEW: Send AprilTag data via WebSocket
// This function sends tag detection data to the server in a flat JSON format
// that works with GDevelop, Unreal, Unity, or any WebSocket client
bool sendAprilTagDataWebSocket(int tag_id, float yaw, float pitch, float roll,
                                float x_cm, float y_cm, float z_cm,
                                float tag_size_percent, float distance_cm) {
    if (!webSocketConnected) {
        Serial.println("⚠️  WebSocket not connected - data not sent");
        return false;
    }
    
    unsigned long send_start = millis();
    
    // Create flat JSON message (game-engine agnostic format)
    // This format works with GDevelop, Unreal, Unity, and any WebSocket client
    StaticJsonDocument<512> doc;
    doc["event"] = "apriltag_data";
    doc["smartcam_ip"] = WiFi.localIP().toString();
    doc["tag_id"] = tag_id;
    doc["camera_name"] = "Waveshare-ESP32-S3";
    doc["x_cm"] = round(x_cm * 10) / 10.0;      // Round to 1 decimal place
    doc["y_cm"] = round(y_cm * 10) / 10.0;
    doc["z_cm"] = round(z_cm * 10) / 10.0;
    doc["yaw"] = round(yaw * 10) / 10.0;
    doc["pitch"] = round(pitch * 10) / 10.0;
    doc["roll"] = round(roll * 10) / 10.0;
    doc["tag_size_percent"] = round(tag_size_percent * 10) / 10.0;
    doc["distance_cm"] = round(distance_cm * 10) / 10.0;
    doc["timestamp"] = millis();
    
    String json;
    serializeJson(doc, json);
    
    // Send via WebSocket
    bool sent = webSocket.sendTXT(json);
    
    unsigned long send_duration = millis() - send_start;
    
    if (sent) {
        totalMessagesSent++;
        
        Serial.printf("🔵<<-- SEND: apriltag_data (#%lu) | ID=%d Pos=(%.1f,%.1f,%.1f)cm Yaw=%.1f° [%lums]\n",
                     totalMessagesSent, tag_id, x_cm, y_cm, z_cm, yaw, send_duration);
        
        return true;
    } else {
        Serial.println("❌ WebSocket SEND FAILED");
        return false;
    }
}
```

---

### **Phase 7: Integrate into Existing Code**

#### **7A: Initialize WiFi/WebSocket in `setup()`**

**Find the section in `setup()` after AprilTag detector initialization, add:**
```cpp
//// jwc 26-0124-0000 NEW: Initialize WiFi and WebSocket
Serial.println("\n*** Starting WiFi and WebSocket initialization...");

initWiFi();

if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("🔌 WebSocket Init | Server=ws://%s:%d%s\n", WS_HOST, WS_PORT, WS_PATH);
    
    // Set event handler
    webSocket.onEvent(webSocketEvent);
    
    // Connect to WebSocket server
    webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
    
    // Configure automatic reconnection
    webSocket.setReconnectInterval(5000);  // Retry every 5 seconds if disconnected
    webSocket.enableHeartbeat(15000, 3000, 2);  // Ping every 15s, timeout 3s, disconnect after 2 missed pongs
    
    Serial.println("✅ WebSocket client initialized (connecting...)");
} else {
    Serial.println("❌ WiFi not connected - WebSocket initialization skipped");
}
```

#### **7B: Add Queue Logic in `task()` Function**

**Find the section in `task()` where pose is calculated (after `camera_position` is computed), add:**
```cpp
//// jwc 26-0124-0000 NEW: Queue tag data with rate limiting
// This prevents queue overflow by only adding tags at a controlled rate
unsigned long current_time_for_list = millis();

if (current_time_for_list - list_add_time_last >= AprilTag_Capture_INTERVAL_MS) {
    // Calculate all values BEFORE freeing matrices
    float x_cm = MATD_EL(camera_position, 0, 0) * 100.0;
    float y_cm = MATD_EL(camera_position, 1, 0) * 100.0;
    float z_cm = MATD_EL(camera_position, 2, 0) * 100.0;
    float distance_cm = fabs(z_cm);
    
    // Calculate tag size percentage of display
    float tag_width_px = sqrt(pow(det->p[2][0] - det->p[0][0], 2) +
                              pow(det->p[2][1] - det->p[0][1], 2));
    float screen_diagonal_px = sqrt(pow(240, 2) + pow(320, 2));
    float tag_size_percent = (tag_width_px / screen_diagonal_px) * 100.0;
    
    // Add to queue
    listTagEvent_Add(det->id, yaw, pitch, roll, x_cm, y_cm, z_cm,
                    tag_size_percent, distance_cm);
    list_add_time_last = current_time_for_list;
    
    #if DEBUG >= 1
    Serial.printf("*** QUEUE: Added tag (rate-limited to every %lums)\n", 
                 AprilTag_Capture_INTERVAL_MS);
    #endif
}
```

#### **7C: Add Sender Logic at End of `task()` Loop**

**Find the end of the `task()` while loop (after `esp_camera_fb_return()`), add:**
```cpp
//// jwc 26-0124-0000 NEW: Periodic WebSocket sender
// This sends queued tags at a controlled rate (separate from capture rate)
webSocket.loop();  // MUST call every loop iteration for WebSocket to work

unsigned long current_time = millis();
if (list_count > 0 &&
    (current_time - http_send_time_last >= AprilTag_Send_INTERVAL_MS)) {
    
    // Remove oldest tag from queue
    tagData_Struct tag_to_send;
    if (listTagEvent_Remove(&tag_to_send)) {
        #if DEBUG >= 1
        Serial.printf("\n*** WebSocket: Sending tag from queue (Queue: %d remaining)...\n", list_count);
        #endif
        
        // Send via WebSocket
        sendAprilTagDataWebSocket(tag_to_send.tag_id,
                                 tag_to_send.yaw, tag_to_send.pitch, tag_to_send.roll,
                                 tag_to_send.x_cm, tag_to_send.y_cm, tag_to_send.z_cm,
                                 tag_to_send.tag_size_percent, tag_to_send.distance_cm);
        
        // Update last send time (whether success or failure)
        http_send_time_last = current_time;
    }
}
```

#### **7D: Add Display Feedback (Optional)**

**Find the section in `task()` where camera image is drawn to screen, add after:**
```cpp
//// jwc 26-0124-0000 NEW: Display WebSocket status overlay
// Shows real-time stats on the display for visual confirmation
gfx->setTextSize(2);
gfx->setTextColor(YELLOW);

// Top-right: Captured/Sent ratio
gfx->setCursor(120, 1);
gfx->printf("%lu/%lu", totalMessagesCaptured, totalMessagesSent);

// Bottom-left: Capture rate
float capture_rate = 1000.0 / AprilTag_Capture_INTERVAL_MS;
gfx->setCursor(1, 300);
gfx->printf("C:%.1f/s", capture_rate);

// Bottom-right: Send rate
float send_rate = 1000.0 / AprilTag_Send_INTERVAL_MS;
gfx->setCursor(130, 300);
gfx->printf("S:%.1f/s", send_rate);
```

---

## 📦 Required Libraries (platformio.ini)

Add these to your `platformio.ini` file:
```ini
lib_deps = 
    bblanchon/ArduinoJson@^6.21.3
    links2004/WebSockets@^2.4.1
    # ... keep existing libraries ...
```

---

## 🎮 Game Engine Compatibility

The WebSocket protocol is **game-engine agnostic** and works with:

### **GDevelop** (JavaScript)
```javascript
const ws = new WebSocket("ws://76.102.42.17:5000/websocket");
ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.event === "apriltag_data") {
        // Use data.x_cm, data.y_cm, data.z_cm, data.yaw, etc.
        console.log(`Tag ${data.tag_id} at (${data.x_cm}, ${data.y_cm}, ${data.z_cm})`);
    }
};
```

### **Unreal Engine** (C++)
```cpp
#include "IWebSocket.h"
TSharedPtr<IWebSocket> WebSocket = FWebSocketsModule::Get().CreateWebSocket(
    "ws://76.102.42.17:5000/websocket");
WebSocket->OnMessage().AddLambda([](const FString& Message) {
    TSharedPtr<FJsonObject> JsonObject;
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Message);
    if (FJsonSerializer::Deserialize(Reader, JsonObject)) {
        FString Event = JsonObject->GetStringField("event");
        if (Event == "apriltag_data") {
            float X = JsonObject->GetNumberField("x_cm");
            float Y = JsonObject->GetNumberField("y_cm");
            float Z = JsonObject->GetNumberField("z_cm");
            // Use X, Y, Z for game object positioning
        }
    }
});
```

### **Unity** (C#)
```csharp
using NativeWebSocket;
WebSocket ws = new WebSocket("ws://76.102.42.17:5000/websocket");
ws.OnMessage += (bytes) => {
    string message = System.Text.Encoding.UTF8.GetString(bytes);
    var data = JsonUtility.FromJson<AprilTagData>(message);
    if (data.@event == "apriltag_data") {
        Vector3 position = new Vector3(data.x_cm, data.y_cm, data.z_cm);
        // Use position for game object
    }
};
```

---

## 🧪 Testing Checklist

### **Phase 1: WiFi Connection**
- [ ] ESP32 connects to WiFi successfully
- [ ] IP address displayed in serial monitor
- [ ] Can ping ESP32 from computer

### **Phase 2: WebSocket Connection**
- [ ] WebSocket connects to server
- [ ] "identify" message sent successfully
- [ ] Server acknowledges connection

### **Phase 3: AprilTag Detection**
- [ ] Tags detected and displayed on screen
- [ ] Pose estimation working (x, y, z, yaw, pitch, roll)
- [ ] Tags added to queue (check serial output)

### **Phase 4: Data Transmission**
- [ ] Tags sent via WebSocket at configured rate
- [ ] Server receives data (check server logs)
- [ ] Display shows capture/send counters

### **Phase 5: Game Engine Integration**
- [ ] GDevelop/Unreal/Unity receives data
- [ ] Data format correct (flat JSON)
- [ ] Real-time updates working

### **Phase 6: Performance**
- [ ] No queue overflow (check "QUEUE FULL" messages)
- [ ] Stable FPS (capture and send rates)
- [ ] No WebSocket disconnections

---

## 🐛 Troubleshooting

### **WiFi Won't Connect**
- Check SSID/password spelling
- Ensure 2.4GHz band is enabled (ESP32-S3 doesn't support 5GHz)
- Try moving closer to router
- Check router firewall settings

### **WebSocket Won't Connect**
- Verify server is running (`python3 server.py`)
- Check server IP address (use `ip addr` on Ubuntu)
- Verify port 5000 is not blocked by firewall
- Check AUTH_TOKEN matches server

### **No AprilTag Data Received**
- Check queue is not full (serial output)
- Verify WebSocket is connected (check display)
- Ensure tags are within detection range (5-23cm for 5cm tags)
- Check camera calibration parameters

### **Queue Overflow**
- Increase `AprilTag_Send_INTERVAL_MS` (send faster)
- Decrease `AprilTag_Capture_INTERVAL_MS` (capture slower)
- Increase `tagData_MAX` buffer size

### **Compilation Errors**
- Verify libraries installed: `ArduinoJson`, `WebSockets`
- Check `platformio.ini` has correct lib_deps
- Clean build: `pio run -t clean` then `pio run`

---

## 📚 Additional Resources

### **Server Setup**
See: `../09_ZA-Stage_02-ServerHub_WebSockets/README.md`

### **Game Engine Integration**
See: `../09_ZB-Stage_03-DigitalGameGraphicEngine/README.md`

### **Camera Calibration**
For production use, perform proper calibration:
- OpenCV calibration tutorial: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Print checkerboard pattern (9x6 or 7x5)
- Capture 20+ images at different angles
- Run calibration script to get FX, FY, CX, CY values

---

## 📝 Change Log

### **2026-01-24 - Initial Migration**
- Added WiFi connectivity
- Added WebSocket client
- Added AprilTag data queue
- Added rate limiting
- Updated camera calibration for 240x320 display
- Added display feedback overlays
- Documented game engine compatibility

---

## 🎯 Next Steps

1. **Implement Phase 1-7** following this guide
2. **Test each phase** before moving to next
3. **Deploy server** (see Stage 2 README)
4. **Integrate with game engine** (see Stage 3 README)
5. **Tune performance** (adjust timing constants)
6. **Optional: Perform camera calibration** for production accuracy

---

**End of Migration Guide**
