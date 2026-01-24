//// jwc 25-0407-0000 SimpleTagDetect_April
//// * simple_tag_detect_24_1115_0530_ACa_a__Commit_24_0416_Wrover_Vid
////   * Flash_Jpg_24_1115_1040_ABC_a_Esp32_FnWroverCamToTft7735.ino
////     * TFT_graphicstest_one_lib_24_1114_0800_ACA_a_FnWrover_St7735_1p4.ino
//// * 24-1127-1850 Appears that at small 'FRAMESIZE_QVGA' to reduce April-Tag processing
////   * for 2.5x2.5cm Tft, need April-Tag min. 0.5x0.5cm for recognition.
////   * 2.5x2.5 = 6.25 || 0.5x0.5 = 0.25 || 0.25/6.25 = 0.04 = 4% of screen min
//// jwc 25-0409-1900 April-Tag Pose

//// jwc 26-0109-1730 Serial communications on GPIO 18 (TX) and GPIO 44 (RX) using UART1
//// jwc 26-0109-1730 Bidirectional UART with line buffering and timed TX
//// jwc 26-0109-1730 Pin Selection Rationale:
//// * Available GPIO range: [2,4, 6-21, 43-44, 47-48]
//// * Constraints:
////   - GPIO 2,4,6-17,21: Camera hardware (cannot repurpose)
////   - GPIO 47,48: Touch I2C (cannot repurpose)
////   - GPIO 19,20: USB D-/D+ hardware (cannot repurpose)
////   - GPIO 5: Not broken out on Waveshare board
////   - GPIO 43: UART0 TX (boot messages would flood micro:bit RX buffer)
////   - GPIO 44: UART0 RX (safe - no outgoing boot data, can be remapped)
////   - GPIO 18: Free (previously RX-only, now TX)
//// * Final Configuration: E3_RX=GPIO44 (from micro:bit), E3_TX=GPIO18 (to micro:bit)
//// * Trade-off: GPIO 44 remapped to UART1 disables UART0 RX (Serial Monitor input lost)
////   but Serial.print() debugging still works (uses GPIO 43 TX only)

//// jwc 26-0109-1730 ARCHIVED: Previous RX-only configuration
//// jwc 26-0109-1300 TESTING: RX-only mode (no TX) to verify camera works
//// * GPIO 18 (RX) should be safe, GPIO 17 (TX) disabled for camera PWDN
//// * Higher Priority for E3_Rx from MB.  Lower Priority for E3_Tx to MB.

//// jwc 26-0109-1730 ### __Configuration Summary:__
//// jwc 26-0109-1730 - __E3_RX = GPIO 44__ (receives from micro:bit)
//// jwc 26-0109-1730 - __E3_TX = GPIO 18__ (sends to micro:bit)
//// jwc 26-0109-1730 - No G or V Connection needed, as long each device powered separately
//// jwc 26-0109-1730 - __Trade-off:__ GPIO 44 remapped to UART1 disables UART0 RX (Serial Monitor input), but Serial.print() debugging still works (uses GPIO 43 TX)
//// jwc 26-0109-1730 ###


/*
 * AprilTag detector demo on AI Thinker ESP32-CAM
 * Created by gvl610
 * Based on https://github.com/AprilRobotics/apriltag
 * with some modifications (for adaption and performance)
 */

/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */
#include "esp_camera.h"
#include <lvgl.h>
#include <demos/lv_demos.h>
#include <Arduino_GFX_Library.h>
#include "bsp_cst816.h"

//// jwc 26-0109-1200 NEW: Robust serial communications on GPIO 17 (TX) / GPIO 18 (RX)
//// Replaces simple Serial1 code in loop() with proper line buffering and timing
#include "Serial_Comms.h"

//// jwc 26-0124-1030 PHASE 1: WiFi & WebSocket includes + configuration
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

//// jwc 26-0124-1030 WiFi credentials (update these for your network)
const char* ssid = "Chan-Comcast";
const char* password = "Jesus333!";

//// jwc 26-0124-1240 WebSocket authentication token (must match server)
const char* AUTH_TOKEN = "Jesus333!!!";

//// jwc 26-0124-1030 WebSocket server configuration
//// Format: ws://hostname:port/path or wss://hostname:port/path for SSL
//// jwc 26-0124-1105 UPDATED: Using same server config as Lilygo T-Camera Plus S3
//// * C:\12i-Db\Dropbox\09k-E32-SM\25-0517-1900-E32--OPENED\13i-T-CameraPlus-S3-NOW-Ubuntu22_BmaxB1Pro--25-0505-0730-NOW\25-1123-1700-E32_TCameraPlusS3-AprilTag-SerialToMicrobit-HttpsCorsToGDevelop\examples\Camera_Screen_AprilTag__Serial_With_Microbit--Esp32_Client_Websocket-NOW\01-Esp32-Client
//// jwc 26-0124-1215 o const char* ws_host = "76.102.42.17";    // Ubuntu server IP (public IP via port-forward)
const char* ws_host = "10.0.0.89";       // Python server IP (local network): Win
const uint16_t ws_port = 5000;           // WebSocket server port
const char* ws_path = "/websocket";      // WebSocket endpoint path

WebSocketsClient webSocket;

//// jwc 26-0124-1030 PHASE 2: AprilTag data queue & timing
//// Queue to store detected AprilTag data for transmission
#define MAX_QUEUE_SIZE 10
struct AprilTagData {
  int id;
  float decision_margin;
  float yaw;
  float pitch;
  float roll;
  float x;
  float y;
  float z;
  unsigned long timestamp;
};
AprilTagData tagQueue[MAX_QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;
SemaphoreHandle_t queueMutex = NULL;

//// jwc 26-0124-1030 Timing control for WebSocket transmission
unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL = 1000; // Send every 1 second

/*To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 *You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 Note that the `lv_examples` library is for LVGL v7 and you shouldn't install it for this version (since LVGL v8)
 as the examples and demos are now part of the main LVGL library. */

// #include <examples/lv_examples.h>
// #include <demos/lv_demos.h>

//// jwc 26-0109-1300 ARCHIVED: Attempted to free GPIO 17 by disabling camera PWDN
//// #define PWDN_GPIO_NUM -1   //disabled to free GPIO 17 for UART TX
//// Result: Camera task failed to start (white screen), serial worked fine
//// Conclusion: Camera REQUIRES PWDN pin even if "not used" for proper initialization

//// jwc 26-0109-1300 RESTORED: Camera needs PWDN on GPIO 17
//// Cannot use GPIO 17 for serial TX - camera hardware requires it
#define PWDN_GPIO_NUM 17   //power down is not used (but camera init requires this pin)
#define RESET_GPIO_NUM -1  //software reset will be performed
#define XCLK_GPIO_NUM 8
#define SIOD_GPIO_NUM 21
#define SIOC_GPIO_NUM 16

#define Y9_GPIO_NUM 2
#define Y8_GPIO_NUM 7
#define Y7_GPIO_NUM 10
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 11
#define Y4_GPIO_NUM 15
#define Y3_GPIO_NUM 13
#define Y2_GPIO_NUM 12
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 4
#define PCLK_GPIO_NUM 9



#define EXAMPLE_PIN_NUM_LCD_SCLK 39
#define EXAMPLE_PIN_NUM_LCD_MOSI 38
#define EXAMPLE_PIN_NUM_LCD_MISO 40
#define EXAMPLE_PIN_NUM_LCD_DC 42
#define EXAMPLE_PIN_NUM_LCD_RST -1
#define EXAMPLE_PIN_NUM_LCD_CS 45
#define EXAMPLE_PIN_NUM_LCD_BL 1
#define EXAMPLE_PIN_NUM_TP_SDA 48
#define EXAMPLE_PIN_NUM_TP_SCL 47

#define LEDC_FREQ 5000
#define LEDC_TIMER_10_BIT 10

#define EXAMPLE_LCD_ROTATION 0
#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 320

//// jwc 26-0109-1520 NEW: Comm display overlay constants
#define BUTTON_HEIGHT 40
#define BUTTON_Y (320 - BUTTON_HEIGHT)  // Bottom of screen (y=280)
#define TEXT_AREA_HEIGHT (BUTTON_Y - 0)  // Top to button (0 to 280)


/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *bus = new Arduino_ESP32SPI(
  EXAMPLE_PIN_NUM_LCD_DC /* DC */, EXAMPLE_PIN_NUM_LCD_CS /* CS */,
  EXAMPLE_PIN_NUM_LCD_SCLK /* SCK */, EXAMPLE_PIN_NUM_LCD_MOSI /* MOSI */, EXAMPLE_PIN_NUM_LCD_MISO /* MISO */);

/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
Arduino_GFX *gfx = new Arduino_ST7789(
  bus, EXAMPLE_PIN_NUM_LCD_RST /* RST */, EXAMPLE_LCD_ROTATION /* rotation */, true /* IPS */,
  EXAMPLE_LCD_H_RES /* width */, EXAMPLE_LCD_V_RES /* height */);


/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

/*******************************************************************************
 * Please config the touch panel in touch.h
 ******************************************************************************/
// #include "touch.h"

uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_disp_draw_buf_t draw_buf;
lv_color_t *disp_draw_buf;
lv_disp_drv_t disp_drv;

static SemaphoreHandle_t lvgl_api_mux = NULL;

lv_obj_t *img_camera;

//
// >> jwc 25-0407-0000 SimpleTagDetect_April
//

// Apriltag headers
// We choose 36h11 family to use in this demo, but you can use
// any family of your choice. Note that due to memory limitation,
// you might have to reduce the number of tag in `codedata` array
// in the tag family source file.
#include "apriltag.h"
#include "tag36h11.h"
#include "common/image_u8.h"
#include "common/zarray.h"


//// jwc o http://esp32.io/viewtopic.php?t=31461
/** jwc \/
"Failed to get the frame on time!" after a while working fine.
* I have a script that in the loop does some kind of "motion detection". Later I plan to take a high-res pic and send it to a server if a motion is detected. Beside this I want to have the possibility to connect via a Webserver. The part in the loop works fine already. When I connect via the Webserver it works fine at the beginning but after taking about 40-50 frames I get this error and the esp32 reboots:
* [E][camera.c:1483] esp_camera_fb_get(): Failed to get the frame on time!
Guru Meditation Error: Core 1 panic'ed (LoadProhibited). Exception was unhandled.
* The Webserver on its own seems to work fine. When I comment out everything in the loop function and insert the then missing "Config(LOW)" in the setup function the stream goes on and on without any problems.
**/
#include <img_converters.h>


//// jwc 25-0409-1600 April-Tag Pose
//
#include "apriltag_pose.h" // For pose estimation
#include "common/matd.h"

// Config for pose estimation
// Tag size (in meter). See original AprilTag readme for how to measure
// You have to put your value here. This value is of NO standard and
// is just my own tag size.
//// jwc 26-0124-1250 Renamed to avoid conflict with ESP32 ROM cache.h TAG_SIZE (value 4)
#define APRILTAG_SIZE 0.05

//// jwc 26-0124-1030 PHASE 3: Updated camera calibration for Waveshare ESP32-S3 (HVGA 480x320)
//// Original values were for different camera - these are estimates for OV2640 at HVGA
//// TODO: Calibrate your specific camera using 3DF Zephyr or similar tool for accurate pose
// Camera calibration data
// This information is obtained by calibrating your camera using software like 3DF Zephyr
// You have to calibrate and put your own values here, this value is just for my camera
// and likely not work on your camera.
//// jwc 26-0124-1030 ARCHIVED: Original calibration (different camera/resolution)
//// #define FX 924.713610878 // fx (in pixel)
//// #define FY 924.713610878 // fy (in pixel)
//// #define CX 403.801748132 // cx (in pixel)
//// #define CY 305.082642826 // cy (in pixel)
//// jwc 26-0124-1030 NEW: Estimated calibration for OV2640 at HVGA (480x320)
//// Focal length estimates based on typical OV2640 specs and HVGA resolution
#define FX 480.0 // fx (in pixel) - estimated for HVGA width
#define FY 480.0 // fy (in pixel) - estimated (square pixels)
#define CX 240.0 // cx (in pixel) - center of 480px width
#define CY 160.0 // cy (in pixel) - center of 320px height


/*
 * Define this macro to enable debug mode
 * Level 0: Absolutely no debug at all. Suitable for
 *  production use.
 * Level 1: Simple debug messages , like simple events
 *  notifications.
 * Level 2: Low level debug messages
 * Level 3: Debug messages that in a loop
 */
//// jwc o #define DEBUG 2
//// jwc \/ get as much info as possible
//// jwc y #define DEBUG 1
//// jwc y #define DEBUG 3
#define DEBUG 1

//// jwc 26-0109-1540 NEW: Camera debug prints control
//// Set to 1 to enable camera frame debug prints, 0 to disable
//// jwc 26-0109-1600 y #define DEBUG_PRINT_CAM 1
#define DEBUG_PRINT_CAM 0


//
// >> jwc 25-0407-0000 SimpleTagDetect_April
//

  // Create tag family object
  apriltag_family_t *tf = tag36h11_create();

  // Create AprilTag detector object
  apriltag_detector_t *td = apriltag_detector_create();

  //// jwc ? move to within 'setup' due to error 'error: expected constructor, destructor, or type conversion before '(' token' : // Add tag family to the detector
  //// jwc ? move to within 'setup' due to error 'error: expected constructor, destructor, or type conversion before '(' token' : apriltag_detector_add_family(td, tf);

//
// << jwc 25-0407-0000 SimpleTagDetect_April
//



bool lvgl_lock(int timeout_ms) {
  // Convert timeout in milliseconds to FreeRTOS ticks
  // If `timeout_ms` is set to -1, the program will block until the condition is met
  const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xSemaphoreTakeRecursive(lvgl_api_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void) {
  xSemaphoreGiveRecursive(lvgl_api_mux);
}


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
//// jwc oy #if (LV_COLOR_16_SWAP != 0)
//// jwc oy   gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
//// jwc oy #else
//// jwc oy   gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
//// jwc oy #endif
//// jwc 26-0105-2120 o gfx->drawGrayscaleBitmap(0, 0, (uint8_t *)disp_draw_buf, screenWidth, screenHeight);
// jwc 26-0105-2120 This function is never called in camera mode - camera draws directly to screen
// The actual display happens in the task() function via lv_img_set_src()
// We still need this for LVGL UI elements, so keep it as RGB565
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#else
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif

  lv_disp_flush_ready(disp_drv);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  uint16_t touchpad_x;
  uint16_t touchpad_y;
  bsp_touch_read();
  if (bsp_touch_get_coordinates(&touchpad_x, &touchpad_y)) {
    //// jwc 26-0109-1520 NEW: Check if touch is in button area (bottom 40px)
    if(touchpad_y >= BUTTON_Y) {
      // Toggle comm display state
      comm_display_enabled = !comm_display_enabled;
      Serial.printf("*** COMM display toggled: %s\n", comm_display_enabled ? "ON" : "OFF");
      
      // Redraw button immediately to show new state
      draw_comm_button();
      
      // Small delay to prevent double-trigger
      delay(200);
    }
    
    data->point.x = touchpad_x;
    data->point.y = touchpad_y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}


void lvgl_camera_ui_init(lv_obj_t *parent) {
  // lv_obj_t *obj = lv_obj_create(parent);
  // lv_obj_set_size(obj, lv_pct(100), lv_pct(100));
  img_camera = lv_img_create(parent);
  // lv_obj_set_size(img_camera, 240,);
  // lv_img_set_angle(img_camera, 900);
  lv_obj_align(img_camera, LV_ALIGN_CENTER, 0, 0);  // 居中显示
  lv_obj_set_pos(img_camera, -1, 0);
  lv_obj_set_scroll_dir(parent, LV_DIR_NONE);

  lv_obj_set_style_pad_top(img_camera, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(img_camera, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(img_camera, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(img_camera, 0, LV_PART_MAIN);
}

//// jwc 26-0109-1520 NEW: Draw comm text overlay (yellow text on black background)
//// jwc 26-0110-0820 UPDATED: Remove black background, decrease font size for 20 lines
void draw_comm_overlay() {
  if(!comm_display_enabled) return;
  
  //// jwc 26-0110-0820 ARCHIVED: Black background causes flickering
  //// gfx->fillRect(0, 0, 240, TEXT_AREA_HEIGHT, BLACK);
  
  //// jwc 26-0110-0820 NEW: Transparent text overlay (no background)
  // Draw yellow text from top down (newest at top)
  gfx->setTextColor(YELLOW);
  //// jwc 26-0110-0820 ARCHIVED: gfx->setTextSize(2);  // Too large for 20 lines
  //// jwc 26-0110-0820 NEW: Smaller font to fit 20 lines in 280px (14px per line)
  gfx->setTextSize(1);  // Smaller font (8px height → ~12px with spacing)
  gfx->setTextWrap(true);
  
  int y = 5;  // Start near top
  //// jwc 26-0110-0820 ARCHIVED: int line_height = 20;  // For text size 2
  //// jwc 26-0110-0820 NEW: Smaller line height for 20 lines in 280px
  int line_height = 12;  // 280px / 20 lines ≈ 14px, use 12px for tighter spacing
  
  for(int i = 0; i < comm_line_count && y < TEXT_AREA_HEIGHT; i++) {
    gfx->setCursor(5, y);
    gfx->println(comm_lines[i]);
    y += line_height;
  }
}

//// jwc 26-0109-1520 NEW: Draw toggle button at bottom of screen
void draw_comm_button() {
  // Button background (green when ON, dark gray when OFF)
  uint16_t btn_color = comm_display_enabled ? GREEN : 0x4208;  // Green or dark gray
  gfx->fillRect(0, BUTTON_Y, 240, BUTTON_HEIGHT, btn_color);
  
  // Button border
  gfx->drawRect(0, BUTTON_Y, 240, BUTTON_HEIGHT, WHITE);
  
  // Button text
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(60, BUTTON_Y + 12);
  gfx->print("COMM: ");
  gfx->print(comm_display_enabled ? "ON" : "OFF");
}

//// jwc 26-0124-1030 PHASE 4: WiFi initialization function
void initWiFi() {
  Serial.println("*** Initializing WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n*** WiFi connected!");
    Serial.print("*** IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n*** ERROR: WiFi connection failed!");
  }
}

//// jwc 26-0124-1030 PHASE 5: WebSocket event handler
//// jwc 26-0124-1240 UPDATED: Send identify message on connect
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("*** WebSocket disconnected!");
      break;
    case WStype_CONNECTED:
      {
        Serial.printf("*** WebSocket connected to: %s\n", payload);
        
        //// jwc 26-0124-1240 Send identify message with authentication
        StaticJsonDocument<256> identifyDoc;
        identifyDoc["event"] = "identify";
        JsonObject identifyData = identifyDoc.createNestedObject("data");
        identifyData["type"] = "esp32";
        identifyData["device"] = "Waveshare-ESP32-S3-Touch-LCD-2";
        identifyData["auth_token"] = AUTH_TOKEN;
        
        String identifyJson;
        serializeJson(identifyDoc, identifyJson);
        webSocket.sendTXT(identifyJson);
        
        Serial.printf("*** Sent identify message: %s\n", identifyJson.c_str());
      }
      break;
    case WStype_TEXT:
      Serial.printf("*** WebSocket RX: %s\n", payload);
      break;
    case WStype_ERROR:
      Serial.println("*** WebSocket ERROR!");
      break;
    default:
      break;
  }
}

//// jwc 26-0124-1030 PHASE 6: AprilTag queue & transmission functions
//// Enqueue detected AprilTag data (called from detection loop)
void enqueueAprilTag(int id, float decision_margin, float yaw, float pitch, float roll, float x, float y, float z) {
  if (xSemaphoreTake(queueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (queueCount < MAX_QUEUE_SIZE) {
      tagQueue[queueTail].id = id;
      tagQueue[queueTail].decision_margin = decision_margin;
      tagQueue[queueTail].yaw = yaw;
      tagQueue[queueTail].pitch = pitch;
      tagQueue[queueTail].roll = roll;
      tagQueue[queueTail].x = x;
      tagQueue[queueTail].y = y;
      tagQueue[queueTail].z = z;
      tagQueue[queueTail].timestamp = millis();
      
      queueTail = (queueTail + 1) % MAX_QUEUE_SIZE;
      queueCount++;
      
      Serial.printf("*** Enqueued tag ID %d (queue: %d/%d)\n", id, queueCount, MAX_QUEUE_SIZE);
    } else {
      Serial.println("*** WARNING: Queue full, dropping tag data!");
    }
    xSemaphoreGive(queueMutex);
  }
}

//// Transmit queued AprilTag data via WebSocket (called periodically)
void transmitAprilTags() {
  unsigned long currentTime = millis();
  
  // Check if it's time to transmit
  if (currentTime - lastTransmitTime < TRANSMIT_INTERVAL) {
    return;
  }
  
  // Check if WebSocket is connected
  if (!webSocket.isConnected()) {
    Serial.println("*** WebSocket not connected, skipping transmission");
    return;
  }
  
  // Check if queue has data
  if (xSemaphoreTake(queueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (queueCount > 0) {
      // Create JSON document (allocate enough for multiple tags)
      StaticJsonDocument<1024> doc;
      JsonArray tags = doc.createNestedArray("tags");
      
      // Dequeue all available tags
      int transmitted = 0;
      while (queueCount > 0 && transmitted < MAX_QUEUE_SIZE) {
        JsonObject tag = tags.createNestedObject();
        tag["id"] = tagQueue[queueHead].id;
        tag["decision_margin"] = tagQueue[queueHead].decision_margin;
        tag["yaw"] = tagQueue[queueHead].yaw;
        tag["pitch"] = tagQueue[queueHead].pitch;
        tag["roll"] = tagQueue[queueHead].roll;
        tag["x"] = tagQueue[queueHead].x;
        tag["y"] = tagQueue[queueHead].y;
        tag["z"] = tagQueue[queueHead].z;
        tag["timestamp"] = tagQueue[queueHead].timestamp;
        
        queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
        queueCount--;
        transmitted++;
      }
      
      // Serialize and send
      String jsonString;
      serializeJson(doc, jsonString);
      webSocket.sendTXT(jsonString);
      
      Serial.printf("*** Transmitted %d tags via WebSocket\n", transmitted);
      lastTransmitTime = currentTime;
    }
    xSemaphoreGive(queueMutex);
  }
}


static void task(void *param) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_1;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  //// jwc o 25-0408-2050: config.xclk_freq_hz = 20000000;
  //// jwc o 25-0408-2050: config.frame_size = FRAMESIZE_HVGA;
  //// jwc o 25-0408-2050: config.pixel_format = PIXFORMAT_RGB565;  // for streaming
  //// jwc o 25-0408-2050: //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  //// jwc o 25-0408-2050: config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  //// jwc o 25-0408-2050: config.fb_location = CAMERA_FB_IN_PSRAM;
  //// jwc o 25-0408-2050: config.jpeg_quality = 12;
  //// jwc o 25-0408-2050: config.fb_count = 1;


  // >> jwc from: C:\11i-GD-C\09j-E3\24-1114-0752-AAA_AAa-TYJ-AprilTag--Github-Raspiduino-Apriltag\apriltag-esp32\examples\simpletagdetect_24_1126_2101_NOW_Git_24_0416_Wrover_AprilTag\simpletagdetect_24_1126_2101_NOW_Git_24_0416_Wrover_AprilTag.ino

  //// jwc o 'sinple_tag_detect' \/

  // Set clock frequency
  config.xclk_freq_hz = 20000000;
  // Set frame config
  // • FRAMESIZE_UXGA (1600 x 1200)
  // * FRAMESIZE_SXGA (1280 x 1024)
  // • FRAMESIZE_XGA (1024 x 768)
  // • FRAMESIZE_SVGA (800 x 600)
  // • FRAMESIZE_VGA (640 x 480)
  // • FRAMESIZE_QVGA (320 x 240)
  // • FRAMESIZE_CIF (352 x 288)
  //
  
  //// jwc typedef enum {
  //// jwc   FRAMESIZE_96X96,    // 96x96
  //// jwc   FRAMESIZE_QQVGA,    // 160x120
  //// jwc   FRAMESIZE_QCIF,     // 176x144
  //// jwc   FRAMESIZE_HQVGA,    // 240x176
  //// jwc   FRAMESIZE_240X240,  // 240x240
  //// jwc   FRAMESIZE_QVGA,     // 320x240
  //// jwc   FRAMESIZE_CIF,      // 400x296
  //// jwc   FRAMESIZE_HVGA,     // 480x320
  //// jwc   FRAMESIZE_VGA,      // 640x480
  //// jwc   FRAMESIZE_SVGA,     // 800x600
  //// jwc   FRAMESIZE_XGA,      // 1024x768
  //// jwc   FRAMESIZE_HD,       // 1280x720
  //// jwc   FRAMESIZE_SXGA,     // 1280x1024
  //// jwc   FRAMESIZE_UXGA,     // 1600x1200
  //// jwc   // 3MP Sensors
  //// jwc   FRAMESIZE_FHD,      // 1920x1080
  //// jwc   FRAMESIZE_P_HD,     //  720x1280
  //// jwc   FRAMESIZE_P_3MP,    //  864x1536
  //// jwc   FRAMESIZE_QXGA,     // 2048x1536
  //// jwc   // 5MP Sensors
  //// jwc   FRAMESIZE_QHD,      // 2560x1440
  //// jwc   FRAMESIZE_WQXGA,    // 2560x1600
  //// jwc   FRAMESIZE_P_FHD,    // 1080x1920
  //// jwc   FRAMESIZE_QSXGA,    // 2560x1920
  //// jwc   FRAMESIZE_INVALID
  //// jwc } framesize_t;


  //// jwc y but maybe too big? \/ config.frame_size = FRAMESIZE_VGA;
  //// jwc o config.frame_size = FRAMESIZE_QVGA;
  config.frame_size = FRAMESIZE_HVGA;

  //// jwc ? config.pixel_format = PIXFORMAT_GRAYSCALE; // Required for AprilTag processing
  //// jwc oy config.pixel_format = PIXFORMAT_RGB565;  // for streaming
  //// jwc n DISTORTED COLORED SCREEN:  config.pixel_format = PIXFORMAT_GRAYSCALE; // Required for AprilTag processing
  //// jwc ? config.pixel_format = PIXFORMAT_RGB565;  // for streaming
  config.pixel_format = PIXFORMAT_GRAYSCALE; // Required for AprilTag processing

  config.grab_mode = CAMERA_GRAB_LATEST; // Has to be in this mode, or detection will be lag
  config.fb_location = CAMERA_FB_IN_PSRAM;
  //config.jpeg_quality = 12;  // not seem needed since not 'PIXFORMAT_JPEG'
  config.fb_count = 1; // Can't afford (and also not needed) to have 2
  
  // << jwc from: C:\11i-GD-C\09j-E3\24-1114-0752-AAA_AAa-TYJ-AprilTag--Github-Raspiduino-Apriltag\apriltag-esp32\examples\simpletagdetect_24_1126_2101_NOW_Git_24_0416_Wrover_AprilTag\simpletagdetect_24_1126_2101_NOW_Git_24_0416_Wrover_AprilTag.ino


  Serial.println("*** Camera task started!");
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("*** ERROR: Camera init failed with error 0x%x\n", err);
    vTaskDelete(NULL);
    return;
  }
  
  Serial.println("*** Camera initialized successfully!");

  sensor_t * s = esp_camera_sensor_get();
  s->set_hmirror(s, 1);

  //// jwc from: C:\11i-GD-C\09j-E3\24-1114-0752-AAA_AAa-TYJ-AprilTag--Github-Raspiduino-Apriltag\apriltag-esp32\examples\simpletagdetect_24_1126_2101_NOW_Git_24_0416_Wrover_AprilTag\simpletagdetect_24_1126_2101_NOW_Git_24_0416_Wrover_AprilTag.ino
  // Custom camera configs should go here \/
  //
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 1);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 168);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 1);        // 0 = disable , 1 = enable
  s->set_vflip(s, 1);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable


  //// jwc Waveshare Esp32s3 2.0in
  ////
  //// jwc n even with '1', April-Tag Detected: s->set_vflip(s, 1);  
  s->set_vflip(s, 0);  
  //// jwc n and with '1', April-Tag Detected-NOT: s->set_hmirror(s, 1);
  //// jwc 26-0109-2140 ARCHIVED: s->set_hmirror(s, 0);  // TEST 1: Failed - no detection
  //// jwc 26-0109-2150 Restore original setting
  s->set_hmirror(s, 1);
  
  
  lv_img_dsc_t img_dsc;
  img_dsc.header.always_zero = 0;
  img_dsc.header.w = 480;
  img_dsc.header.h = 320;
  img_dsc.data_size = 320 * 480 * 2;

  //// jwc 26-0105-2120 Changed to TRUE_COLOR for RGB565 display
  img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
  img_dsc.data = NULL;

  //// jwc o camera_fb_t *pic;
  //// jwc 'fb' >> 'camera_framebuffer_pic_ObjPtr'
  camera_fb_t *camera_framebuffer_pic_ObjPtr;
  
  //// jwc 26-0105-2120 Allocate RGB565 buffer for display conversion
  uint16_t *rgb565_buf = (uint16_t *)heap_caps_malloc(480 * 320 * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  if (!rgb565_buf) {
    Serial.println("*** ERROR: Failed to allocate RGB565 buffer!");
    vTaskDelete(NULL);
    return;
  }
  Serial.println("*** RGB565 buffer allocated successfully");
  
  //// jwc o >> replace with readable 'true':  while (1) {
  while (true) {
    #if DEBUG_PRINT_CAM
    Serial.println("*** About to get camera frame...");
    #endif
    camera_framebuffer_pic_ObjPtr = esp_camera_fb_get();
    #if DEBUG_PRINT_CAM
    Serial.printf("*** Got frame: %s\n", camera_framebuffer_pic_ObjPtr ? "SUCCESS" : "NULL");
    #endif

    //// jwc o: Reverse terms for clearer: if (NULL != camera_framebuffer_pic_ObjPtr) {
    if (camera_framebuffer_pic_ObjPtr != NULL) {
      //// jwc 26-0106-0130 KEY DIFFERENCE: Working example (01_factory) sets img_dsc.data = pic->buf DIRECTLY!
      //// It does NOT convert or copy to another buffer. It points directly to camera buffer.
      //// BUT we need grayscale→RGB565 conversion, so we MUST use our rgb565_buf.
      //// The issue might be that we're returning the camera buffer before LVGL renders it!
      
      //// jwc 26-0105-2120 Convert grayscale to RGB565 for display
      uint8_t *gray_buf = camera_framebuffer_pic_ObjPtr->buf;
      for (int i = 0; i < 480 * 320; i++) {
        uint8_t gray = gray_buf[i];
        // Convert grayscale to RGB565: R5G6B5
        rgb565_buf[i] = ((gray & 0xF8) << 8) | ((gray & 0xFC) << 3) | (gray >> 3);
      }
      
      //// jwc 26-0106-0140 ARCHIVED - LVGL approach (too complicated, didn't render):
      //// if (lvgl_lock(-1)) {
      ////   img_dsc.data = (uint8_t *)rgb565_buf;
      ////   lv_img_set_src(img_camera, &img_dsc);
      ////   Serial.println("*** Image set to LVGL");
      ////   lvgl_unlock();
      //// } else {
      ////   Serial.println("*** ERROR: Could not lock LVGL!");
      //// }
      
      //// jwc 26-0106-0140 NEW SIMPLE APPROACH: Draw directly to screen using GFX!
      //// Bypass LVGL entirely - much simpler and proven to work
      gfx->draw16bitRGBBitmap(0, 0, rgb565_buf, 480, 320);
      #if DEBUG_PRINT_CAM
      Serial.println("*** Image drawn to screen via GFX");
      #endif
      
      //// jwc 26-0109-1520 NEW: Draw comm overlay and button after camera image
      draw_comm_overlay();  // Draw yellow text if enabled
      draw_comm_button();   // Always draw button at bottom
      
      //// jwc 26-0109-1620 NEW: Direct touch polling for button (bypasses LVGL)
      //// Touch handler in LVGL doesn't sync with GFX rendering, so poll directly here
      uint16_t touch_x, touch_y;
      bsp_touch_read();
      if (bsp_touch_get_coordinates(&touch_x, &touch_y)) {
        Serial.printf("*** TOUCH detected at x=%d, y=%d (BUTTON_Y=%d)\n", touch_x, touch_y, BUTTON_Y);
        if(touch_y >= BUTTON_Y) {
          comm_display_enabled = !comm_display_enabled;
          Serial.printf("*** COMM toggled: %s\n", comm_display_enabled ? "ON" : "OFF");
          // Redraw button with new state
          draw_comm_button();
          // Debounce delay
          delay(300);
        }
      }
      
      //// jwc 26-0106-0020 ARCHIVED - Debug code that confirmed everything works:
      //// Serial.println("jwc 26-0106-0010 Got camera frame!");
      //// Serial.println("jwc 26-0106-0010 Converted to RGB565, setting image...");
      //// Serial.println("jwc 26-0106-0010 Image set to LVGL!");
    }


    // Convert our framebuffer to detector's input format
    #if DEBUG >= 3
    Serial.println("Converting frame to detector's input format... ");
#endif
    //// jwc 26-0109-2200 ARCHIVED: Process full 480x320 camera frame
    //// image_u8_t im = {
    ////   .width = camera_framebuffer_pic_ObjPtr->width,
    ////   .height = camera_framebuffer_pic_ObjPtr->height,
    ////   .stride = camera_framebuffer_pic_ObjPtr->width,
    ////   .buf = camera_framebuffer_pic_ObjPtr->buf
    //// };
    
    //// jwc 26-0109-2200 NEW: Crop to visible display area (240x320)
    //// Only process the left 240px width that's actually displayed
    //// This prevents detecting tags on the right half (240-480px) that aren't visible
    image_u8_t im = {
      .width = 240,  // Match display width (was 480)
      .height = 320,  // Keep full height
      .stride = camera_framebuffer_pic_ObjPtr->width,  // Keep original stride for proper addressing
      .buf = camera_framebuffer_pic_ObjPtr->buf
    };
#if DEBUG >= 3
    Serial.println("done");
    Serial.println("Detecting... ");
#endif


  //// jwc move to setup:  // Setup AprilTag detection
  //// jwc move to setup: if DEBUG >= 1
  //// jwc move to setup:  Serial.print("Init AprilTag detecing... ");
  //// jwc move to setup: endif

  //// jwc done earlier before 'setup()': // Create tag family object
  //// jwc done earlier before 'setup()': apriltag_family_t *tf = tag36h11_create();
  //// jwc done earlier before 'setup()': 
  //// jwc done earlier before 'setup()': // Create AprilTag detector object
  //// jwc done earlier before 'setup()': apriltag_detector_t *td = apriltag_detector_create();
  //// jwc done earlier before 'setup()': 
  //// jwc done earlier before 'setup()': // Add tag family to the detector
  //// jwc done earlier before 'setup()': apriltag_detector_add_family(td, tf);
  
  //// jwc moved to setup: // Tag detector configs
  //// jwc moved to setup: // quad_sigma is Gaussian blur's sigma
  //// jwc moved to setup: // quad_decimate: small number = faster but cannot detect small tags
  //// jwc moved to setup: //                big number = slower but can detect small tags (or tag far away)
  //// jwc moved to setup: // With quad_sigma = 1.0 and quad_decimate = 4.0, ESP32-CAM can detect 16h5 tag
  //// jwc moved to setup: // from the distance of about 1 meter (tested with tag on screen. not on paper)
  //// jwc moved to setup: td->quad_sigma = 0.0;
  //// jwc moved to setup: td->quad_decimate = 4.0;
  //// jwc moved to setup: td->refine_edges = 0;
  //// jwc moved to setup: td->decode_sharpening = 0;
  //// jwc moved to setup: td->nthreads = 1;
  //// jwc moved to setup: td->debug = 0;

  //// jwc redundant o:   // Done init AprilTag detector
  //// jwc redundant o: #if DEBUG >= 1
  //// jwc redundant o:   Serial.println("done");
  //// jwc redundant o:   //// jwc o Serial.print("Memory available in PSRAM: ");
  //// jwc redundant o:   //// jwc o Serial.println(ESP.getFreePsram());
  //// jwc redundant o:   Serial.println("Start AprilTag detecting...");
  //// jwc redundant o: #endif


    // Detect
    zarray_t *detections = apriltag_detector_detect(td, &im);
#if DEBUG >= 3
    Serial.println("done. Result:");
#endif


if(zarray_size(detections) > 0){
  //// jwc oo tft.setCursor(1,1);
  gfx->setCursor(1,1);
  //// jwc y good for nomral, but increase for VideoMeet-Cam \/: tft.setTextSize(2); tft.setTextSize(3);
  //// jwc y tft.setTextSize(4);
  //// jwc y seems just right to fit 6 max
  //// jwc oo tft.setTextSize(5);
  gfx->setTextSize(5);
  
  // Print result
  for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      //// jwc \/
      //// jwc y Serial.print(" *** ");
      
      if(det->id % 2 == 0){
        // Even #
        //// jwc oo tft.setTextColor(TFT_BLUE);      
        gfx->setTextColor(BLUE);      
      }
      else{
        // Odd #
        //// jwc oo tft.setTextColor(TFT_RED);
        gfx->setTextColor(RED);
      }
      //// jwc tft.printf(" *** %d", (det->id));
      //// jwc oo tft.printf("%d ", (det->id));
      gfx->printf("%d ", (det->id));

      //// jwc add pose: Serial.print("*** *** ");
      //// jwc add pose: Serial.print(det->id);
      //// jwc add pose: //// jwc o Serial.print(", ");


      // Print tag ID
      //// jwc o Serial.print("ID: ");
      //// jwc o Serial.println(det->id);
      //// jwc n Serial.println("*** *** *** ", det->id, det->family, det->decision_margin);
      // Print tag ID and decision margin
      //// jwc y printf("*** *** *** [DET]%d,%f,", det->id, det->decision_margin);
      printf("*** *** *** [DETECT ID:] %5.0d,%5.0f,", det->id, det->decision_margin);

      // Creating detection info object to feed into pose estimator
      apriltag_detection_info_t info;
      info.det = det;
      info.tagsize = APRILTAG_SIZE;
      info.fx = FX;
      info.fy = FY;
      info.cx = CX;
      info.cy = CY;

      // Estimate the pose
      apriltag_pose_t pose;
      double err = estimate_tag_pose(&info, &pose);
      
      // Print result (position of the tag in the camera's coordinate system)
      //matd_print(pose.R, "%15f"); // Rotation matrix
      //matd_print(pose.t, "%15f"); // Translation matrix

#if DEBUG >= 1    
      // Print result (position of the tag in the camera's coordinate system)
      //matd_print(pose.R, "%15f"); // Rotation matrix
      //matd_print(pose.t, "%15f"); // Translation matrix
      Serial.printf("\n");
      Serial.printf("    *** pose.R: \n");
      //// jwc y matd_print(pose.R, "%15f"); // Rotation matrix
      matd_print(pose.R, "%15f"); // Rotation matrix
      Serial.printf("    *** pose.t: \n");
      //// jwc y matd_print(pose.t, "%15f"); // Translation matrix
      matd_print(pose.t, "%15f"); // Translation matrix
#endif

      // Compute the yaw, pitch, and roll from the rotation matrix (and convert to degree)
      double yaw = atan2(MATD_EL(pose.R, 1, 0), MATD_EL(pose.R, 0, 0)) * RAD_TO_DEG;
      double pitch = atan2(-MATD_EL(pose.R, 2, 0), sqrt(pow(MATD_EL(pose.R, 2, 1), 2) + pow(MATD_EL(pose.R, 2, 2), 2))) * RAD_TO_DEG;
      double roll = atan2(MATD_EL(pose.R, 2, 1), MATD_EL(pose.R, 2, 2)) * RAD_TO_DEG;

      // Print the yaw, pitch, and roll of the camera
      //// jwc y printf("y,p,r: %15f, %15f, %15f\n", yaw, pitch, roll);
      printf(" *** y,p,r: %5.0f, %5.0f, %5.0f", yaw, pitch, roll);
      
      // Compute the transpose of the rotation matrix
      matd_t *R_transpose = matd_transpose(pose.R);

      // Negate the translation vector
      for (int i = 0; i < pose.t->nrows; i++) {
          MATD_EL(pose.t, i, 0) = -MATD_EL(pose.t, i, 0);
      }

      // Compute the position of the camera in the tag's coordinate system
      matd_t *camera_position = matd_multiply(R_transpose, pose.t);
      //// jwc y printf("x,y,z: %15f, %15f, %15f\n", MATD_EL(camera_position, 0, 0), MATD_EL(camera_position, 1, 0), MATD_EL(camera_position, 2, 0));
      //// jwc o printf("*** x,y,z: %5.0f, %5.0f, %5.0f\n", MATD_EL(camera_position, 0, 0), MATD_EL(camera_position, 1, 0), MATD_EL(camera_position, 2, 0));
      printf(" *** x,y,z: %5.0f, %5.0f, %5.0f", MATD_EL(camera_position, 0, 0), MATD_EL(camera_position, 1, 0), MATD_EL(camera_position, 2, 0));

      // Free the matrices
      matd_destroy(R_transpose);
      matd_destroy(camera_position);

      //// jwc 26-0124-1030 PHASE 7: Enqueue detected AprilTag for WebSocket transmission
      enqueueAprilTag(det->id, det->decision_margin, yaw, pitch, roll, 
                      MATD_EL(camera_position, 0, 0), 
                      MATD_EL(camera_position, 1, 0), 
                      MATD_EL(camera_position, 2, 0));

      //// jwc o Serial.println("");
      printf("\n");
  }
  //// jwc o Serial.println("");
  printf("\n");

} 
//// jwc y Not Needed Since Cam-Image Refreshes: else {
//// jwc y Not Needed Since Cam-Image Refreshes:   // Blank Line to Erase Residue Text: 20 Spaces
//// jwc y Not Needed Since Cam-Image Refreshes:   //// jwc maybe not needed as new cam-image refreshes: y: tft.printf("                    ");
//// jwc y Not Needed Since Cam-Image Refreshes: }


    // Cleaning up
#if DEBUG >= 3
    //// jwc o Serial.print("Memory available in PSRAM: ");
    //// jwc o Serial.println(ESP.getFreePsram());
    Serial.println("Cleaning up... ");
#endif
    // Free detection result object
    apriltag_detections_destroy(detections);


    esp_camera_fb_return(camera_framebuffer_pic_ObjPtr);
    
    //// jwc 26-0124-1030 PHASE 7: Transmit queued AprilTags via WebSocket
    transmitAprilTags();
    
    //// jwc 26-0124-1030 PHASE 7: Process WebSocket events
    webSocket.loop();
    
    vTaskDelay(pdMS_TO_TICKS(1));
    //// jwc Clear Buffer
    camera_framebuffer_pic_ObjPtr = NULL;



  }
  vTaskDelete(NULL);
}



void setup() {
  Serial.begin(115200);
  
  //// jwc 26-0109-1200 ARCHIVED: Simple Serial1 initialization (moved to Serial_Comms)
  //// Serial1.begin(115200, SERIAL_8N1, 48, 47); // RX=48, TX=47
  //// Serial.println("Serial1 initialized: TX=IO47, RX=IO48");
  
  //// jwc 26-0109-1200 NEW: Initialize robust serial communications
  //// GPIO 17 (TX) / GPIO 18 (RX) - No boot messages, proper buffering
  Serial_Comms_Init();
  
  lvgl_api_mux = xSemaphoreCreateRecursiveMutex();
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX LVGL_Arduino_v8 example ");
  String LVGL_Arduino = String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.println(LVGL_Arduino);

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

#ifdef EXAMPLE_PIN_NUM_LCD_BL
  //// jwc 26-0105-2030 PlatformIO Arduino ESP32 3.2.0 uses older LEDC API
  //// * Arduino IDE: ledcAttach() and ledcWrite() (newer API)
  //// * PlatformIO: ledcSetup(), ledcAttachPin(), and ledcWrite() (older API)
  //// ledcAttach(EXAMPLE_PIN_NUM_LCD_BL, LEDC_FREQ, LEDC_TIMER_10_BIT);
  ledcSetup(0, LEDC_FREQ, LEDC_TIMER_10_BIT);
  ledcAttachPin(EXAMPLE_PIN_NUM_LCD_BL, 0);
  ledcWrite(0, (1 << LEDC_TIMER_10_BIT) / 100 * 80);
#endif

  // Init touch device
  // touch_init(gfx->width(), gfx->height(), gfx->getRotation());
  Wire.begin(EXAMPLE_PIN_NUM_TP_SDA, EXAMPLE_PIN_NUM_TP_SCL);
  bsp_touch_init(&Wire, gfx->getRotation(), gfx->width(), gfx->height());
  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  bufSize = screenWidth * screenHeight;


  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) {
    // remove MALLOC_CAP_INTERNAL flag try again
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  }
  if (!disp_draw_buf) {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, bufSize);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.direct_mode = true;

    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);


    // jwc from '02_gfx_helloworld'
    
    gfx->setCursor(10, 10);
    gfx->setTextColor(RED);
    gfx->println("Hello World!");
    delay(5000); // 5 seconds


    /* Option 3: Or try out a demo. Don't forget to enable the demos in lv_conf.h. E.g. LV_USE_DEMOS_WIDGETS*/
    //// jwc yy lvgl_camera_ui_init(lv_scr_act());
    lvgl_camera_ui_init(lv_scr_act());
    // lv_demo_widgets();
    // lv_demo_benchmark();
    // lv_demo_keypad_encoder();
    // lv_demo_music();
    // lv_demo_stress();
  }

  //
  // jwc 25-0407-0000
  //

//// jwc moved to runtime:   // Setup AprilTag detection
//// jwc moved to runtime: #if DEBUG >= 1
//// jwc moved to runtime:   Serial.print("Init AprilTag detecing... ");
//// jwc moved to runtime: #endif
//// jwc moved to runtime: 
//// jwc moved to runtime:   // Create tag family object
//// jwc moved to runtime:   apriltag_family_t *tf = tag36h11_create();
//// jwc moved to runtime: 
//// jwc moved to runtime:   // Create AprilTag detector object
//// jwc moved to runtime:   apriltag_detector_t *td = apriltag_detector_create();
//// jwc moved to runtime: 
  // Add tag family to the detector
  apriltag_detector_add_family(td, tf);
  
  // Tag detector configs
  // quad_sigma is Gaussian blur's sigma
  // quad_decimate: small number = faster but cannot detect small tags
  //                big number = slower but can detect small tags (or tag far away)
  // With quad_sigma = 1.0 and quad_decimate = 4.0, ESP32-CAM can detect 16h5 tag
  // from the distance of about 1 meter (tested with tag on screen. not on paper)
  
  //// jwc yy 25-0408-1920 td->quad_sigma = 0.0;
  //// jwc yy 25-0408-1930 no improvement: td->quad_sigma = 1.0;
  td->quad_sigma = 0.0;

  td->quad_decimate = 4.0;
  td->refine_edges = 0;

  //// jwc T O D O (DONE) From: pose_estimate.ino
  //// jwc: td->decode_sharpening = 0.25;
  //// jwc ? td->decode_sharpening = 0;
  //// jwc no improvement seems: td->decode_sharpening = 0.25;
  td->decode_sharpening = 0;

  //// jwc TODO: td->nthreads = 2; // The optimal (after many tries) is 2 thread on 2 cores ESP32
  td->nthreads = 1;

  //// jwc o 25-0408-1930 \/ td->debug = 0;
  //// jwc ? td->debug = 1;
  td->debug = 0;

  // Done init AprilTag detector
#if DEBUG >= 1
  Serial.println("done");
  //// jwc o Serial.print("Memory available in PSRAM: ");
  //// jwc o Serial.println(ESP.getFreePsram());
  Serial.println("Start AprilTag detecting...");
#endif


  //// jwc 26-0124-1030 PHASE 7: Initialize WiFi and WebSocket
  initWiFi();
  
  //// jwc 26-0124-1030 PHASE 7: Initialize queue mutex
  queueMutex = xSemaphoreCreateMutex();
  if (queueMutex == NULL) {
    Serial.println("*** ERROR: Failed to create queue mutex!");
  } else {
    Serial.println("*** Queue mutex created successfully");
  }
  
  //// jwc 26-0124-1030 PHASE 7: Initialize WebSocket client
  webSocket.begin(ws_host, ws_port, ws_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  Serial.println("*** WebSocket client initialized");

  Serial.println("Setup done");

  //// jwc 25-0408-1920 try simpletagdetect runtime approach: xTaskCreatePinnedToCore(
  //// jwc 25-0408-1920 try simpletagdetect runtime approach:   task,
  //// jwc 25-0408-1920 try simpletagdetect runtime approach:   "lvgl_app_task",
  //// jwc 25-0408-1920 try simpletagdetect runtime approach:   1024 * 10,
  //// jwc 25-0408-1920 try simpletagdetect runtime approach:   NULL,
  //// jwc 25-0408-1920 try simpletagdetect runtime approach:   1,
  //// jwc 25-0408-1920 try simpletagdetect runtime approach:   NULL,
  //// jwc 25-0408-1920 try simpletagdetect runtime approach:   0);
  
  //// jwc 25-0408-1930 n dead: while(true){
  //// jwc 25-0408-1930 n dead:   task(NULL);
  //// jwc 25-0408-1930 n dead: }

  //// jwc 26-0109-1230 FIX: Camera task MUST be created FIRST, before serial task
  //// Issue: Serial task was created first, preventing camera task from starting
  //// Result: White screen (no camera frames), but serial worked fine
  //// Solution: Create camera task first (core 0), then serial task (core 1)
  //// Last parameter: 0=Core0, 1=Core1
  xTaskCreatePinnedToCore(
    task,
    "lvgl_app_task",
    1024 * 10,
    NULL,
    1,
    NULL,
    0);  // Core 0: Camera + AprilTag detection

  //// jwc 26-0109-1230 Serial communications task created AFTER camera task
  //// Runs independently on core 1, sends "E3>%d:%d" every 3 seconds, line-buffered RX
  xTaskCreatePinnedToCore(
    Serial_Comms_Task,
    "serial_comms_task",
    4096,
    NULL,
    1,
    NULL,
    1);  // Core 1: Serial communications

}

void loop() {
  if (lvgl_lock(-1)) {
    lv_timer_handler(); /* let the GUI do its work */
    lvgl_unlock();
  }

  //// jwc 26-0109-1200 ARCHIVED: Simple serial TX/RX (replaced by Serial_Comms task)
  //// int val1 = random(0, 5);
  //// int val2 = random(5, 10);
  //// Serial1.printf("E3>%d:%d\n", val1, val2);
  //// Serial.printf("[Serial1 TX] E3>%d:%d\n", val1, val2);
  //// while (Serial1.available()) {
  ////   char c = Serial1.read();
  ////   Serial.printf("[Serial1 RX] %c", c);
  //// }
  
  //// jwc 26-0109-1200 NEW: Serial communications now handled by Serial_Comms_Task
  //// No code needed here - task runs independently on core 1

  //// jwc y not needed: gfx->setCursor(10, 10);
  //// jwc y not needed: gfx->setTextColor(RED);
  //// jwc y not needed: gfx->println("Hello World!");
  //// jwc y not needed: //// jwc y delay(5000); // 5 seconds

  //// jwc y gfx->setCursor(random(gfx->width()), random(gfx->height()));
  //// jwc y gfx->setCursor(random(20), random(20));
  //// jwc yy Disable this text test to not interfere w/ acutal AprilCodeOutput: gfx->setCursor(random(20), 200);
  //// jwc yy Disable this text test to not interfere w/ acutal AprilCodeOutput: gfx->setTextColor(random(0xffff), random(0xffff));
  //// jwc yy Disable this text test to not interfere w/ acutal AprilCodeOutput: //// jwc y gfx->setTextSize(random(6) /* x scale */, random(6) /* y scale */, random(2) /* pixel_margin */);
  //// jwc yy Disable this text test to not interfere w/ acutal AprilCodeOutput: gfx->setTextSize(random(2) /* x scale */, random(2) /* y scale */, random(2) /* pixel_margin */);
  //// jwc yy Disable this text test to not interfere w/ acutal AprilCodeOutput: gfx->println("Hello Jesus!");


  vTaskDelay(pdMS_TO_TICKS(5));
}
