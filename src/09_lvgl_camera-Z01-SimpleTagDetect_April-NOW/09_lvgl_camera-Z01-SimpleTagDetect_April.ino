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

//// jwc 26-0128-1440 ARCHIVED: Single DEFINE_NETWORKING_BOOL flag (WebSocket only, has memory leak)
//// //// jwc 26-0128-0730 NEW: Preprocessor flag to disable networking for memory leak testing
//// //// Set to 1 to enable WiFi/WebSocket, 0 to disable (AprilTag-only mode)
//// #define DEFINE_NETWORKING_BOOL 1  // Change to 0 to test without networking
//// 
//// #if DEFINE_NETWORKING_BOOL
//// //// jwc 26-0124-1030 PHASE 1: WiFi & WebSocket includes + configuration
//// #include <WiFi.h>
//// //// jwc 26-0128-0500 ARCHIVED: Links2004 WebSocketsClient (memory leak after 2 minutes)
//// //// #include <WebSocketsClient.h>
//// //// jwc 26-0128-0500 NEW: ArduinoWebsockets by gilmaimon (modern, better memory management)
//// #include <ArduinoWebsockets.h>
//// #include <ArduinoJson.h>
//// #endif

//// jwc 26-0128-1440 NEW: Triple protocol support - HTTP vs WebSocket vs UDP
//// jwc 26-0130-0927 TESTING: Enable HTTP to test if it has memory leaks
//// jwc 26-0130-1000 TESTING: Switch to WebSocket per user request
//// jwc 26-0130-1035 NEW: UDP protocol (stateless, zero memory leaks expected!)
//// Set ONE to 1 to enable (only one protocol active at a time)
#define DEFINE_NETWORK_HTTP_BOOL 0        // HTTP POST protocol (stateless, simpler)
#define DEFINE_NETWORK_WEBSOCKET_BOOL 0   // WebSocket protocol (MEMORY LEAK: 195 bytes/detect)
#define DEFINE_NETWORK_UDP_BOOL 1         // UDP protocol (fire-and-forget, BEST for memory!)

#if DEFINE_NETWORK_HTTP_BOOL || DEFINE_NETWORK_WEBSOCKET_BOOL || DEFINE_NETWORK_UDP_BOOL
//// jwc 26-0124-1030 PHASE 1: WiFi includes + configuration
#include <WiFi.h>
#include <ArduinoJson.h>
#endif

#if DEFINE_NETWORK_UDP_BOOL
//// jwc 26-0130-1035 NEW: UDP protocol (stateless, fire-and-forget, zero memory leaks!)
#include <WiFiUdp.h>
#endif

#if DEFINE_NETWORK_HTTP_BOOL
//// jwc 26-0129-1250 MIGRATED: ESP-IDF native HTTP client (zero memory leaks!)
//// ARCHIVED: Arduino HTTPClient (memory leak: ~160 bytes/TX due to lwIP TIME_WAIT)
//// #include <HTTPClient.h>
//// NEW: ESP-IDF esp_http_client (production-grade, Espressif-maintained)
#include "esp_http_client.h"
#endif

#if DEFINE_NETWORK_WEBSOCKET_BOOL
//// jwc 26-0128-0500 ARCHIVED: Links2004 WebSocketsClient (memory leak after 2 minutes)
//// #include <WebSocketsClient.h>
//// jwc 26-0128-0500 NEW: ArduinoWebsockets by gilmaimon (modern, better memory management)
#include <ArduinoWebsockets.h>
//// jwc 26-0128-1440 #include <ArduinoJson.h>
#endif

//// jwc 26-0128-1440 ARCHIVED: WebSocket-specific server config (memory leak!)
//// #if DEFINE_NETWORKING_BOOL
//// //// jwc 26-0124-1030 WiFi credentials (update these for your network)
//// const char* ssid = "Chan-Comcast";
//// const char* password = "Jesus333!";
//// 
//// //// jwc 26-0124-1240 WebSocket authentication token (must match server)
//// const char* AUTH_TOKEN = "Jesus333!!!";
//// 
//// //// jwc 26-0124-1030 WebSocket server configuration
//// //// Format: ws://hostname:port/path or wss://hostname:port/path for SSL
//// //// jwc 26-0124-1105 UPDATED: Using same server config as Lilygo T-Camera Plus S3
//// //// * C:\12i-Db\Dropbox\09k-E32-SM\25-0517-1900-E32--OPENED\13i-T-CameraPlus-S3-NOW-Ubuntu22_BmaxB1Pro--25-0505-0730-NOW\25-1123-1700-E32_TCameraPlusS3-AprilTag-SerialToMicrobit-HttpsCorsToGDevelop\examples\Camera_Screen_AprilTag__Serial_With_Microbit--Esp32_Client_Websocket-NOW\01-Esp32-Client
//// //// jwc 26-0124-1215 o const char* ws_host = "76.102.42.17";    // Ubuntu server IP (public IP via port-forward)
//// //// jwc 26-0124-0120 y const char* ws_host = "10.0.0.89";       // Python server IP (local network): Win:Hp-Zbook
//// const char* ws_host = "10.0.0.90";       // Python server IP (local network): Win:Hp-Zbook
//// //// jwc 26-0124-0120 y const char* ws_host = "10.0.0.149";       // Python server IP (local network): Lin:Bmax-B1Pro
//// 
//// const uint16_t ws_port = 5100;           // WebSocket server port (jwc 26-0127-0710: Changed from 5000 to 5100 for async server)
//// const char* ws_path = "/";               // WebSocket endpoint path (jwc 26-0127-0710: Changed from "/websocket" to "/" for websockets library)
//// 
//// //// jwc 26-0128-0500 ARCHIVED: Links2004 WebSocketsClient object
//// //// WebSocketsClient webSocket;
//// //// jwc 26-0128-0500 NEW: ArduinoWebsockets client object
//// using namespace websockets;
//// WebsocketsClient webSocket;
//// 
//// //// jwc 26-0124-1030 PHASE 2: AprilTag data queue & timing
//// //// Queue to store detected AprilTag data for transmission
//// #define MAX_QUEUE_SIZE 10
//// struct AprilTagData {
////   int id;
////   float decision_margin;
////   float yaw;
////   float pitch;
////   float roll;
////   float x;
////   float y;
////   float z;
////   float range;  // Distance from camera to tag (cm)
////   unsigned long timestamp;
//// };
//// AprilTagData tagQueue[MAX_QUEUE_SIZE];
//// int queueHead = 0;
//// int queueTail = 0;
//// int queueCount = 0;
//// SemaphoreHandle_t queueMutex = NULL;
//// 
//// //// jwc 26-0124-1030 Timing control for WebSocket transmission
//// unsigned long lastTransmitTime = 0;
//// const unsigned long TRANSMIT_INTERVAL = 1000; // Send every 1 second
//// #endif

//// jwc 26-0128-1440 NEW: Protocol-agnostic server config (works for HTTP, WebSocket, and UDP)
#if DEFINE_NETWORK_HTTP_BOOL || DEFINE_NETWORK_WEBSOCKET_BOOL || DEFINE_NETWORK_UDP_BOOL
//// jwc 26-0124-1030 WiFi credentials (update these for your network)
const char* ssid = "Chan-Comcast";
const char* password = "Jesus333!";

//// jwc 26-0124-1240 Authentication token (must match server)
const char* AUTH_TOKEN = "Jesus333!!!";

//// jwc 26-0129-0628 CRITICAL FIX: Separate ports for HTTP vs WebSocket vs UDP
//// HTTP server (Flask) runs on port 5000
//// WebSocket server runs on port 5100
//// UDP server runs on port 5200
const char* server_host = "10.0.0.90";   // Server IP (local network): Win:Hp-Zbook

#if DEFINE_NETWORK_HTTP_BOOL
const uint16_t server_port = 5000;       // HTTP server port (Flask)
#endif

#if DEFINE_NETWORK_WEBSOCKET_BOOL
const uint16_t server_port = 5100;       // WebSocket server port
#endif

#if DEFINE_NETWORK_UDP_BOOL
const uint16_t server_port = 5200;       // UDP server port
#endif

//// jwc 26-0124-1030 PHASE 2: AprilTag data queue & timing (protocol-agnostic)
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
  float range;  // Distance from camera to tag (cm)
  unsigned long timestamp;
};
AprilTagData tagQueue[MAX_QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;
SemaphoreHandle_t queueMutex = NULL;

//// jwc 26-0124-1030 Timing control for network transmission (protocol-agnostic)
unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL = 1000; // Send every 1 second
#endif

#if DEFINE_NETWORK_HTTP_BOOL
//// jwc 26-0128-1440 NEW: HTTP-specific configuration
const char* http_endpoint = "/apriltag"; // HTTP POST endpoint
#endif

#if DEFINE_NETWORK_WEBSOCKET_BOOL
//// jwc 26-0128-1440 NEW: WebSocket-specific configuration
const char* ws_path = "/";               // WebSocket endpoint path
using namespace websockets;
WebsocketsClient webSocket;
#endif

//// jwc 26-0129-0940 CRITICAL FIX: Forward declare total_transmitted_http BEFORE all functions
//// Compiler error: "not declared in this scope" when draw_hud_overlay() tries to access it
//// Root cause: Variable declared AFTER draw_hud_overlay() function (line order issue!)
//// Solution: Declare at file scope BEFORE any functions that use it
#if DEFINE_NETWORK_HTTP_BOOL
int total_transmitted_http = 0;  // HTTP enabled: real counter
#else
int total_transmitted_http = 0;  // HTTP disabled: dummy variable
#endif

//// jwc 26-0124-1730 NEW: FPS tracking and latest tag data for HUD display
unsigned long lastFrameTime = 0;
unsigned long frameCount = 0;
float currentFPS = 0.0;
unsigned long fpsUpdateTime = 0;
const unsigned long FPS_UPDATE_INTERVAL = 1000; // Update FPS every 1 second

//// Latest detected tag data for HUD display
struct LatestTagDisplay {
  bool hasData = false;
  int id = 0;
  float yaw = 0;
  float pitch = 0;
  float roll = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  float range = 0;  // Distance to tag
} latestTag;

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
#define APRIL_TAG_SIZE 0.05

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
//// jwc 26-0125-0720 ARCHIVED: Previous calibration caused 1-screen-width offset (CX was wrong)
//// #define FX 480.0 // fx (in pixel) - estimated for HVGA width
//// #define FY 480.0 // fy (in pixel) - estimated (square pixels)
//// #define CX 240.0 // cx (in pixel) - center of 480px width
//// #define CY 160.0 // cy (in pixel) - center of 320px height
//// jwc 26-0130-1825 REVERTED: CX=720.0 caused 1 screen-width RIGHT offset (user correction)
//// User feedback: "AprilTag detected 1 screen-width extra to the RIGHT"
//// Conclusion: Original CX=240.0 was CORRECT (geometric center is optical center)
//// jwc 26-0130-1815 ARCHIVED: CX=720.0 (caused right offset - too far!)
//// #define CX 720.0 // cx (in pixel) - shifted right by 480px (WRONG - caused right offset!)
//// jwc 26-0125-0720 RESTORED: Original calibration (CX=240.0 is correct)
#define FX 400.0 // fx (in pixel) - scaled for 480px width (2× Lilygo's 200.0)
#define FY 266.7 // fy (in pixel) - scaled for 320px height (200.0 * 320/240)
#define CX 240.0 // cx (in pixel) - center of 480px width (CORRECT - geometric center)
#define CY 160.0 // cy (in pixel) - center of 320px height (correct)


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

//// jwc 26-0130-0015 NEW: Global detection counter for memory tracking
static int total_detections = 0;  // Track total AprilTag detections


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
//// jwc 26-0130-0450 UPDATED: Use char* directly (no String.c_str() conversion needed!)
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
    //// jwc 26-0130-0450 ARCHIVED: String.c_str() conversion (no longer needed!)
    //// gfx->println(comm_lines[i].c_str());
    //// jwc 26-0130-0450 NEW: comm_lines[i] is already char* (direct use!)
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

//// jwc 26-0124-2020 NEW: Draw FPS and AprilTag data HUD overlay
//// jwc 26-0124-2230 UPDATED: Added heap memory display under FPS
//// jwc 26-0125-0000 UPDATED: Yellow for rows 2+, font size 2 (1 pixel bigger)
//// jwc 26-0125-0330 UPDATED: Moved tag data from upper-right to lower-left, improved spacing
//// jwc 26-0129-0820 UPDATED: Show memory in bytes, TX#, and MemLeak per transmission
//// jwc 26-0129-0945 UPDATED: Increased font size from 1 to 2 for better readability
void draw_hud_overlay() {
  gfx->setTextSize(2);  // Bigger font for better readability (was 1, now 2)
  
  //// Draw FPS in top-left corner (semi-transparent background)
  gfx->fillRect(0, 0, 120, 80, 0x0000);  // Black background (taller for 4 lines at font size 2)
  gfx->setTextColor(GREEN);
  gfx->setCursor(2, 2);
  gfx->printf("FPS:%.1f", currentFPS);
  
  //// Draw heap memory in bytes (line 2)
  gfx->setCursor(2, 22);  // Font size 2: 16px height + 4px spacing = 20px per line
  uint32_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < 10000) {
    gfx->setTextColor(RED);  // Red if low memory
  } else if (freeHeap < 15000) {
    gfx->setTextColor(YELLOW);  // Yellow if medium
  } else {
    gfx->setTextColor(GREEN);  // Green if plenty
  }
  gfx->printf("Mem:%db", freeHeap);
  
  //// Draw TX# and MemLeak (line 3 and 4)
  gfx->setCursor(2, 42);  // Line 3: 2 + 20 + 20 = 42px
  #if DEFINE_NETWORK_HTTP_BOOL
  //// jwc 26-0129-0900 CRITICAL FIX: Move extern declaration outside function (global scope)
  //// Linker error: "undefined reference to total_transmitted_http" when extern inside function
  //// Solution: Declare at file scope (before draw_hud_overlay function)
  //// extern int total_transmitted_http;  // MOVED to line ~850 (after #if DEFINE_NETWORK_HTTP_BOOL)
  static uint32_t lastHeap = 20536;  // Initial baseline
  int memLeak = (int)lastHeap - (int)freeHeap;
  if (total_transmitted_http > 0) {
    memLeak = memLeak / total_transmitted_http;  // Average leak per TX
  }
  gfx->setTextColor(CYAN);
  gfx->printf("TX#:%03d", total_transmitted_http);
  gfx->setCursor(2, 62);  // Line 4: 2 + 20 + 20 + 20 = 62px
  if (memLeak > 200) {
    gfx->setTextColor(RED);
  } else if (memLeak > 100) {
    gfx->setTextColor(YELLOW);
  } else {
    gfx->setTextColor(GREEN);
  }
  gfx->printf("Leak:%db/TX", memLeak);
  #endif
  
  //// jwc 26-0125-0330 NEW: Draw latest tag data in LOWER-LEFT corner (if available)
  if (latestTag.hasData) {
    int x_start = 2;  // Left side, 2px margin
    int y_start = 320 - 100;  // Bottom of screen, 100px tall
    int line_h = 18;  // Improved vertical spacing (was 15)
    int y = y_start;
    
    //// Semi-transparent black background
    gfx->fillRect(0, y_start, 120, 100, 0x0000);  // Lower-left corner
    
    //// Tag ID (row 1 - cyan)
    gfx->setTextColor(CYAN);
    gfx->setCursor(x_start, y);
    gfx->printf("ID:%d", latestTag.id);
    y += line_h;
    
    //// Yaw/Pitch/Roll (row 2 - YELLOW per user request)
    gfx->setTextColor(YELLOW);
    gfx->setCursor(x_start, y);
    gfx->printf("Y:%.0f P:%.0f R:%.0f", latestTag.yaw, latestTag.pitch, latestTag.roll);
    y += line_h;
    
    //// Range (row 3 - YELLOW per user request)
    gfx->setTextColor(YELLOW);
    gfx->setCursor(x_start, y);
    gfx->printf("Rng:%.1fcm", latestTag.range);
    y += line_h;
    
    //// X,Y coordinates (row 4 - YELLOW per user request)
    gfx->setTextColor(YELLOW);
    gfx->setCursor(x_start, y);
    gfx->printf("X:%.1f Y:%.1f", latestTag.x, latestTag.y);
    y += line_h;
    
    //// Z coordinate (row 5 - YELLOW per user request)
    gfx->setTextColor(YELLOW);
    gfx->setCursor(x_start, y);
    gfx->printf("Z:%.1fcm", latestTag.z);
  }
}

//// jwc 26-0128-1440 ARCHIVED: Old DEFINE_NETWORKING_BOOL guard (WebSocket only)
//// #if DEFINE_NETWORKING_BOOL
//// //// jwc 26-0124-1030 PHASE 4: WiFi initialization function
//// void initWiFi() {
////   Serial.println("*** Initializing WiFi...");
////   WiFi.mode(WIFI_STA);
////   WiFi.begin(ssid, password);
////   
////   int attempts = 0;
////   while (WiFi.status() != WL_CONNECTED && attempts < 20) {
////     delay(500);
////     Serial.print(".");
////     attempts++;
////   }
////   
////   if (WiFi.status() == WL_CONNECTED) {
////     Serial.println("\n*** WiFi connected!");
////     Serial.print("*** IP address: ");
////     Serial.println(WiFi.localIP());
////   } else {
////     Serial.println("\n*** ERROR: WiFi connection failed!");
////   }
//// }

//// jwc 26-0128-1440 NEW: Protocol-agnostic WiFi initialization (works for HTTP, WebSocket, and UDP)
#if DEFINE_NETWORK_HTTP_BOOL || DEFINE_NETWORK_WEBSOCKET_BOOL || DEFINE_NETWORK_UDP_BOOL
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

#if DEFINE_NETWORK_WEBSOCKET_BOOL
//// jwc 26-0128-0500 ARCHIVED: Links2004 WebSocketsClient event handler (callback-based)
//// void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
////   switch(type) {
////     case WStype_DISCONNECTED:
////       Serial.println("*** WebSocket disconnected!");
////       break;
////     case WStype_CONNECTED:
////       {
////         Serial.printf("*** WebSocket connected to: %s\n", payload);
////         
////         StaticJsonDocument<256> identifyDoc;
////         identifyDoc["event"] = "identify";
////         JsonObject identifyData = identifyDoc.createNestedObject("data");
////         identifyData["type"] = "esp32";
////         identifyData["device"] = "Waveshare-ESP32-S3-Touch-LCD-2";
////         identifyData["auth_token"] = AUTH_TOKEN;
////         
////         String identifyJson;
////         serializeJson(identifyDoc, identifyJson);
////         webSocket.sendTXT(identifyJson);
////         
////         Serial.printf("*** Esp32 -->> SvHub: TX: %s\n", identifyJson.c_str());
////       }
////       break;
////     case WStype_TEXT:
////       {
////         Serial.printf("*** Esp32 <<-- SvHub: RX: %s\n", payload);
////       }
////       break;
////     case WStype_ERROR:
////       Serial.println("*** WebSocket ERROR!");
////       break;
////     default:
////       break;
////   }
//// }

//// jwc 26-0128-0500 NEW: ArduinoWebsockets event handlers (lambda-based)
void setupWebSocketHandlers() {
  // Connection opened
  //// jwc 26-0128-1420 ARCHIVED: String leak - message.data() creates String object (~100 bytes)
  //// webSocket.onMessage([](WebsocketsMessage message) {
  ////   Serial.printf("*** Esp32 <<-- SvHub: RX: %s\n", message.data().c_str());
  //// });
  
  //// jwc 26-0128-1420 NEW: Use c_str() directly to avoid String allocation
  //// message.c_str() returns const char* from internal buffer (no heap allocation)
  //// jwc 26-0130-1022 NEW: Added green color indicator (matches HTTP success format)
  webSocket.onMessage([](WebsocketsMessage message) {
    const char* data = message.c_str();
    Serial.printf("\033[32m*** ✅ Esp32 <<-- SvHub: RX: %s\033[0m\n", data);
  });
  
  webSocket.onEvent([](WebsocketsEvent event, String data) {
    if (event == WebsocketsEvent::ConnectionOpened) {
      Serial.println("*** WebSocket connected!");
      
      //// jwc 26-0128-1320 ARCHIVED: StaticJsonDocument causes memory leak (doc.clear() doesn't free internal strings)
      //// StaticJsonDocument<256> identifyDoc;
      //// jwc 26-0128-1320 NEW: DynamicJsonDocument auto-frees all internal allocations on destructor
      DynamicJsonDocument identifyDoc(256);
      
      identifyDoc["event"] = "identify";
      JsonObject identifyData = identifyDoc.createNestedObject("data");
      identifyData["type"] = "esp32";
      identifyData["device"] = "Waveshare-ESP32-S3-Touch-LCD-2";
      identifyData["auth_token"] = AUTH_TOKEN;
      
      char identifyBuffer[256];
      serializeJson(identifyDoc, identifyBuffer, sizeof(identifyBuffer));
      webSocket.send(identifyBuffer);
      
      //// jwc 26-0128-1320 ARCHIVED: doc.clear() not needed - DynamicJsonDocument destructor handles cleanup
      //// identifyDoc.clear();
      
      Serial.printf("*** Esp32 -->> SvHub: TX: %s\n", identifyBuffer);
    } else if (event == WebsocketsEvent::ConnectionClosed) {
      Serial.println("*** WebSocket disconnected!");
    } else if (event == WebsocketsEvent::GotPing) {
      Serial.println("*** WebSocket ping received");
    } else if (event == WebsocketsEvent::GotPong) {
      Serial.println("*** WebSocket pong received");
    }
  });
}
#endif

//// jwc 26-0124-1030 PHASE 6: AprilTag queue & transmission functions
//// Enqueue detected AprilTag data (called from detection loop)
//// jwc 26-0124-1800 UPDATED: Added range parameter
//// jwc 26-0125-0430 UPDATED: Only enqueue first tag per 1-second interval (drop others to reduce traffic)
void enqueueAprilTag(int id, float decision_margin, float yaw, float pitch, float roll, float x, float y, float z, float range) {
  //// jwc 26-0125-0430 NEW: Check if we're within 1-second interval since last transmission
  unsigned long currentTime = millis();
  if (currentTime - lastTransmitTime < TRANSMIT_INTERVAL) {
    //// Drop this tag - we already have one queued for this interval
    Serial.printf("*** DROPPED tag ID %d (within 1-sec interval, reducing traffic)\n", id);
    return;
  }
  
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
      tagQueue[queueTail].range = range;
      tagQueue[queueTail].timestamp = millis();
      
      queueTail = (queueTail + 1) % MAX_QUEUE_SIZE;
      queueCount++;
      
      Serial.printf("*** Enqueued tag ID %d (queue: %d/%d) - FIRST in 1-sec interval\n", id, queueCount, MAX_QUEUE_SIZE);
    } else {
      Serial.println("*** WARNING: Queue full, dropping tag data!");
    }
    xSemaphoreGive(queueMutex);
  }
}

#if DEFINE_NETWORK_WEBSOCKET_BOOL
//// Transmit queued AprilTag data via WebSocket (called periodically)
//// jwc 26-0124-1530 UPDATED: Send individual tag messages (not array) to match server format
//// jwc 26-0127-0735 NEW: Added memory monitoring and transmission counter
static int total_transmitted_ws = 0;  // Track total WebSocket messages sent

void transmitAprilTagsWebSocket() {
  unsigned long currentTime = millis();
  
  // Check if it's time to transmit
  if (currentTime - lastTransmitTime < TRANSMIT_INTERVAL) {
    return;
  }
  
  //// jwc 26-0127-0735 NEW: Memory monitoring (print every transmission)
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minFreeHeap = ESP.getMinFreeHeap();
  Serial.printf("\n");
  Serial.printf("*** *** *** [MEM} Free_Dram_Heap: %d b | Free_Psram: %d b | Total WS TX: %d\n", freeHeap, minFreeHeap, total_transmitted_ws);
  Serial.printf("\n");

  //// jwc 26-0127-2100 y //// jwc 26-0127-0735 WARNING: Low memory detection
  //// jwc 26-0127-2100 y if (freeHeap < 20000) {
  //// jwc 26-0127-2100 y   Serial.printf("*** [MEM] WARNING: Low memory! Free=%d bytes\n", freeHeap);
  //// jwc 26-0127-2100 y }
  
  // Check if WebSocket is connected
  //// jwc 26-0128-0500 ARCHIVED: Links2004 isConnected method
  //// if (!webSocket.isConnected()) {
  //// jwc 26-0128-0500 NEW: ArduinoWebsockets available method
  if (!webSocket.available()) {
    Serial.println("*** WebSocket not connected, skipping transmission");
    return;
  }
  
  // Check if queue has data
  if (xSemaphoreTake(queueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (queueCount > 0) {
      // Send each tag as individual message (server expects flat format)
      int transmitted = 0;
      while (queueCount > 0 && transmitted < MAX_QUEUE_SIZE) {
        // Create JSON document for single tag
        //// jwc 26-0125-0510 UPDATED: Round all decimals to 1 place (reduce network traffic)
        //// jwc 26-0128-1320 ARCHIVED: StaticJsonDocument causes memory leak (doc.clear() doesn't free internal strings)
        //// StaticJsonDocument<512> doc;
        //// jwc 26-0128-1320 NEW: DynamicJsonDocument auto-frees all internal allocations on destructor
        DynamicJsonDocument doc(512);
        doc["event"] = "apriltag_data";
        doc["tag_id"] = tagQueue[queueHead].id;
        doc["decision_margin"] = round(tagQueue[queueHead].decision_margin * 10.0) / 10.0;
        doc["yaw"] = round(tagQueue[queueHead].yaw * 10.0) / 10.0;
        doc["pitch"] = round(tagQueue[queueHead].pitch * 10.0) / 10.0;
        doc["roll"] = round(tagQueue[queueHead].roll * 10.0) / 10.0;
        doc["x_cm"] = round(tagQueue[queueHead].x * 10.0) / 10.0;
        doc["y_cm"] = round(tagQueue[queueHead].y * 10.0) / 10.0;
        doc["z_cm"] = round(tagQueue[queueHead].z * 10.0) / 10.0;
        doc["range_cm"] = round(tagQueue[queueHead].range * 10.0) / 10.0;
        doc["timestamp"] = tagQueue[queueHead].timestamp;
        doc["camera_name"] = "Waveshare-ESP32-S3";
        
        //// jwc 26-0128-0140 CRITICAL FIX #5: Eliminate WiFi.localIP().toString() String leak
        //// Original code: doc["smartcam_ip"] = WiFi.localIP().toString();
        //// Problem: toString() creates String object (~180 bytes) that's not freed
        //// Solution: Use stack-allocated buffer with snprintf (no heap allocation)
        char ipBuffer[16];  // Stack-allocated, auto-freed (xxx.xxx.xxx.xxx = 15 chars max)
        IPAddress ip = WiFi.localIP();
        snprintf(ipBuffer, sizeof(ipBuffer), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        doc["smartcam_ip"] = ipBuffer;
        
        //// jwc 26-0128-0120 CRITICAL FIX #4: Replace String with stack buffer to prevent DRAM leak
        //// Original code used String object which allocates heap memory (~100 bytes per TX)
        //// String was not being freed, causing 100-byte leak per transmission
        //// After 30 transmissions: 3KB leaked → system crash!
        //// NEW: Use stack-allocated buffer (auto-freed when function returns)
        char jsonBuffer[512];  // Stack-allocated, matches StaticJsonDocument size
        size_t len = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
        //// jwc 26-0128-0500 ARCHIVED: Links2004 sendTXT method
        //// webSocket.sendTXT(jsonBuffer, len);
        //// jwc 26-0128-0500 NEW: ArduinoWebsockets send method
        webSocket.send(jsonBuffer);
        
        //// jwc 26-0128-1320 ARCHIVED: doc.clear() not needed - DynamicJsonDocument destructor handles cleanup
        //// jwc 26-0127-2150 CRITICAL FIX #2: Clear JSON document to prevent DRAM leak
        //// doc.clear();
        
        //// jwc 26-0128-1350 NEW: Force garbage collection after WebSocket send
        //// Triggers ESP32 heap cleanup to prevent memory fragmentation
        ESP.getFreeHeap();  // Triggers GC
        delay(1);  // Allow cleanup to complete
        
        //// jwc 26-0124-1340 NEW: Debug print with arrow convention
        Serial.printf("*** Esp32 -->> SvHub: TX: %s\n", jsonBuffer);
        
        queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
        queueCount--;
        transmitted++;
      }
      
      //// jwc 26-0127-0735 NEW: Increment total transmission counter
      total_transmitted_ws += transmitted;
      
      Serial.printf("*** Esp32 -->> SvHub: Sent %d tags to server\n", transmitted);
      lastTransmitTime = currentTime;
      
      //// jwc 26-0128-1440 NEW: Clear queue after transmission to prevent memory accumulation
      //// Queue array may hold old data that fragments heap even after "dequeue"
      //// Explicitly zero out the entire queue to ensure clean state
      queueHead = 0;
      queueTail = 0;
      queueCount = 0;
      memset(tagQueue, 0, sizeof(tagQueue));  // Zero out entire array
      Serial.println("*** Queue cleared after transmission");
    }
    xSemaphoreGive(queueMutex);
  }
}
#endif


#if DEFINE_NETWORK_UDP_BOOL
//// jwc 26-0130-1045 NEW: UDP transmission function (stateless, zero memory leaks!)
//// UDP is fire-and-forget: no connection, no handshake, no TIME_WAIT state
//// Expected result: ZERO memory leaks (no TCP buffers, no socket state)

static int total_transmitted_udp = 0;  // Track total UDP packets sent
WiFiUDP udp;  // UDP client object

void transmitAprilTagsUDP() {
  unsigned long currentTime = millis();
  
  // Check if it's time to transmit
  if (currentTime - lastTransmitTime < TRANSMIT_INTERVAL) {
    return;
  }
  
  //// Color-coded memory monitoring
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minFreeHeap = ESP.getMinFreeHeap();
  Serial.printf("\n");
  
  //// BLUE for all memory levels (user request: print only in blue font-color)
  Serial.printf("\033[34m*** *** *** [MEM] Free_Dram_Heap: %d b | Free_Psram: %d b | Total UDP TX: %d\033[0m\n", freeHeap, minFreeHeap, total_transmitted_udp);
  Serial.printf("\n");
  
  // Check if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("*** WiFi not connected, skipping transmission");
    return;
  }
  
  // Check if queue has data
  if (xSemaphoreTake(queueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (queueCount > 0) {
      // Send each tag as individual UDP packet
      int transmitted = 0;
      while (queueCount > 0 && transmitted < MAX_QUEUE_SIZE) {
        // Create JSON document for single tag (STACK-ALLOCATED - no heap leak!)
        StaticJsonDocument<512> doc;
        doc["tag_id"] = tagQueue[queueHead].id;
        doc["decision_margin"] = round(tagQueue[queueHead].decision_margin * 10.0) / 10.0;
        doc["yaw"] = round(tagQueue[queueHead].yaw * 10.0) / 10.0;
        doc["pitch"] = round(tagQueue[queueHead].pitch * 10.0) / 10.0;
        doc["roll"] = round(tagQueue[queueHead].roll * 10.0) / 10.0;
        doc["x_cm"] = round(tagQueue[queueHead].x * 10.0) / 10.0;
        doc["y_cm"] = round(tagQueue[queueHead].y * 10.0) / 10.0;
        doc["z_cm"] = round(tagQueue[queueHead].z * 10.0) / 10.0;
        doc["range_cm"] = round(tagQueue[queueHead].range * 10.0) / 10.0;
        doc["timestamp"] = tagQueue[queueHead].timestamp;
        doc["camera_name"] = "Waveshare-ESP32-S3";
        
        // Add IP address (stack-allocated buffer)
        char ipBuffer[16];
        IPAddress ip = WiFi.localIP();
        snprintf(ipBuffer, sizeof(ipBuffer), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        doc["smartcam_ip"] = ipBuffer;
        
        // Serialize JSON to stack buffer
        char jsonBuffer[512];
        serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
        
        //// UDP transmission (fire-and-forget, no response expected)
        udp.beginPacket(server_host, server_port);
        udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
        int result = udp.endPacket();
        
        //// Color-coded status reporting
        if (result == 1) {
          //// GREEN for success (packet sent)
          Serial.printf("\033[32m*** ✅ Esp32 <<-- SvHub: UDP SUCCESS: %s\033[0m\n", jsonBuffer);
        } else {
          //// RED for failure
          Serial.printf("\033[31m*** ❌ Esp32 <<-- SvHub: UDP FAILED: %s\033[0m\n", jsonBuffer);
        }
        
        queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
        queueCount--;
        transmitted++;
      }
      
      total_transmitted_udp += transmitted;
      
      Serial.printf("*** Esp32 -->> SvHub: Sent %d tags via UDP\n", transmitted);
      lastTransmitTime = currentTime;
      
      // Clear queue after transmission
      queueHead = 0;
      queueTail = 0;
      queueCount = 0;
      memset(tagQueue, 0, sizeof(tagQueue));
      Serial.println("*** Queue cleared after transmission");
    }
    xSemaphoreGive(queueMutex);
  }
}
#endif

#if DEFINE_NETWORK_HTTP_BOOL

//// jwc 26-0129-1250 MIGRATED: ESP-IDF native HTTP client (zero memory leaks!)
//// ARCHIVED: Arduino HTTPClient approach (memory leak: ~160 bytes/TX)
//// static HTTPClient http;  // OLD: Persistent HTTP client (still leaked!)

void transmitAprilTagsHTTP() {
  unsigned long currentTime = millis();
  
  // Check if it's time to transmit
  if (currentTime - lastTransmitTime < TRANSMIT_INTERVAL) {
    return;
  }
  
  //// jwc 26-0129-0710 UPDATED: Color-coded memory monitoring
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minFreeHeap = ESP.getMinFreeHeap();
  Serial.printf("\n");
  
  //// Color-code based on memory level
  //// jwc 26-0129-0950 UPDATED: Replace green (32m) with cyan (36m) - green not working in terminal
  if (freeHeap < 10000) {
    //// RED for critical (< 10KB)
    Serial.printf("\033[31m*** *** *** [MEM] CRITICAL: Free_Dram_Heap: %d b | Free_Psram: %d b | Total HTTP TX: %d\033[0m\n", freeHeap, minFreeHeap, total_transmitted_http);
  } else if (freeHeap < 15000) {
    //// YELLOW for warning (10-15KB)
    Serial.printf("\033[33m*** *** *** [MEM] WARNING: Free_Dram_Heap: %d b | Free_Psram: %d b | Total HTTP TX: %d\033[0m\n", freeHeap, minFreeHeap, total_transmitted_http);
  } else {
    //// CYAN for OK (> 15KB) - green (32m) not working, using cyan (36m) instead
    Serial.printf("\033[36m*** *** *** [MEM] OK: Free_Dram_Heap: %d b | Free_Psram: %d b | Total HTTP TX: %d\033[0m\n", freeHeap, minFreeHeap, total_transmitted_http);
  }
  Serial.printf("\n");
  
  // Check if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("*** WiFi not connected, skipping transmission");
    return;
  }
  
  // Check if queue has data
  if (xSemaphoreTake(queueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (queueCount > 0) {
      // Send each tag as individual HTTP POST
      int transmitted = 0;
      while (queueCount > 0 && transmitted < MAX_QUEUE_SIZE) {
        //// jwc 26-0129-1740 CRITICAL FIX: Replace DynamicJsonDocument with StaticJsonDocument (prevents heap leak!)
        //// ARCHIVED (memory leak): DynamicJsonDocument doc(512); (~512 bytes leaked per TX due to heap fragmentation)
        //// Problem: DynamicJsonDocument allocates from PSRAM→DRAM heap, causing fragmentation over time
        //// After 60 TX: PSRAM exhausted (88→0 bytes), DRAM leaked (21KB→10KB), system crash imminent!
        //// Solution: StaticJsonDocument allocates on STACK (auto-freed when function returns)
        //// Result: Zero memory leaks, runs forever! ✅
        
        // Create JSON document for single tag (STACK-ALLOCATED - no heap leak!)
        StaticJsonDocument<512> doc;
        doc["tag_id"] = tagQueue[queueHead].id;
        doc["decision_margin"] = round(tagQueue[queueHead].decision_margin * 10.0) / 10.0;
        doc["yaw"] = round(tagQueue[queueHead].yaw * 10.0) / 10.0;
        doc["pitch"] = round(tagQueue[queueHead].pitch * 10.0) / 10.0;
        doc["roll"] = round(tagQueue[queueHead].roll * 10.0) / 10.0;
        doc["x_cm"] = round(tagQueue[queueHead].x * 10.0) / 10.0;
        doc["y_cm"] = round(tagQueue[queueHead].y * 10.0) / 10.0;
        doc["z_cm"] = round(tagQueue[queueHead].z * 10.0) / 10.0;
        doc["range_cm"] = round(tagQueue[queueHead].range * 10.0) / 10.0;
        doc["timestamp"] = tagQueue[queueHead].timestamp;
        doc["camera_name"] = "Waveshare-ESP32-S3";
        
        // Add IP address (stack-allocated buffer)
        char ipBuffer[16];
        IPAddress ip = WiFi.localIP();
        snprintf(ipBuffer, sizeof(ipBuffer), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        doc["smartcam_ip"] = ipBuffer;
        
        // Serialize JSON to stack buffer
        char jsonBuffer[512];
        serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
        
        //// jwc 26-0129-1200 ARCHIVED: Arduino HTTPClient implementation (memory leak: ~160 bytes/TX)
        //// // HTTP POST request
        //// HTTPClient http;
        //// char url[128];
        //// snprintf(url, sizeof(url), "http://%s:%d%s", server_host, server_port, http_endpoint);
        //// 
        //// http.begin(url);
        //// http.addHeader("Content-Type", "application/json");
        //// http.addHeader("Authorization", AUTH_TOKEN);
        //// 
        //// int httpCode = http.POST(jsonBuffer);
        //// 
        //// //// jwc 26-0129-0650 CRITICAL FIX: Eliminate ALL String leaks in HTTP error handling
        //// //// jwc 26-0129-0700 UPDATED: Added ANSI color codes for visual debugging
        //// //// ARCHIVED (memory leak): String response = http.getString(); (~200 bytes leak per TX!)
        //// //// ARCHIVED (memory leak): http.errorToString(httpCode).c_str() (~200 bytes leak per error!)
        //// //// Problem: String objects allocate heap memory that's never freed
        //// //// Solution: Skip response body entirely (stateless HTTP doesn't need it!)
        //// if (httpCode > 0) {
        ////   if (httpCode == 200) {
        ////     //// jwc 26-0129-0700 GREEN for success
        ////     Serial.printf("\033[32m*** ✅ Esp32 <<-- SvHub: HTTP POST SUCCESS: %s (code: 200)\033[0m\n", jsonBuffer);
        ////   } else {
        ////     //// jwc 26-0129-0700 YELLOW for HTTP errors (4xx, 5xx)
        ////     Serial.printf("\033[33m*** ⚠️ Esp32 <<-- SvHub: HTTP POST ERROR: %s (code: %d)\033[0m\n", jsonBuffer, httpCode);
        ////   }
        //// } else {
        ////   //// jwc 26-0129-0700 RED for connection failures
        ////   Serial.printf("\033[31m*** ❌ Esp32 <<--SvHub: ERROR: HTTP POST failed (code: %d)\033[0m\n", httpCode);
        //// }
        //// 
        //// http.end();  // Close connection (stateless!)
        
        //// jwc 26-0129-1200 NEW: ESP-IDF esp_http_client implementation (zero memory leaks!)
        //// Build full URL
        char url[128];
        snprintf(url, sizeof(url), "http://%s:%d%s", server_host, server_port, http_endpoint);
        
        //// Configure HTTP client
        esp_http_client_config_t config = {
          .url = url,
          .method = HTTP_METHOD_POST,
          .timeout_ms = 5000,
        };
        
        //// Initialize client
        esp_http_client_handle_t client = esp_http_client_init(&config);
        
        //// Set headers
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_header(client, "Authorization", AUTH_TOKEN);
        
        //// Set POST data
        esp_http_client_set_post_field(client, jsonBuffer, strlen(jsonBuffer));
        
        //// Perform HTTP POST
        esp_err_t err = esp_http_client_perform(client);
        
        //// jwc 26-0129-1750 CRITICAL FIX #3: Read response to force ESP-IDF buffer cleanup (prevents ~253 byte/TX leak!)
        //// Problem: esp_http_client_perform() allocates internal response buffer (~256 bytes) even if you don't read it
        //// ESP-IDF doesn't always free this buffer properly in cleanup(), causing memory leak
        //// Solution: Explicitly read response (even if we don't use it) to trigger proper buffer cleanup
        //// Result: Forces ESP-IDF to free internal buffers, eliminating the ~253 byte/TX leak!
        int content_length = esp_http_client_get_content_length(client);
        if (content_length > 0) {
          char dummy_buffer[64];  // Stack-allocated, auto-freed
          int read_len = esp_http_client_read(client, dummy_buffer, sizeof(dummy_buffer));
          // We don't use the response, just reading it forces buffer cleanup
        }
        
        //// Color-coded status reporting (ESP-IDF style)
        if (err == ESP_OK) {
          int status_code = esp_http_client_get_status_code(client);
          if (status_code == 200) {
            //// GREEN for success
            Serial.printf("\033[32m*** ✅ Esp32 <<-- SvHub: HTTP POST SUCCESS: %s (code: 200)\033[0m\n", jsonBuffer);
          } else {
            //// YELLOW for HTTP errors (4xx, 5xx)
            Serial.printf("\033[33m*** ⚠️ Esp32 <<-- SvHub: HTTP POST ERROR: %s (code: %d)\033[0m\n", jsonBuffer, status_code);
          }
        } else {
          //// RED for connection failures
          Serial.printf("\033[31m*** ❌ Esp32 <<--SvHub: ERROR: HTTP POST failed (err: %s)\033[0m\n", esp_err_to_name(err));
        }
        
        //// jwc 26-0129-1730 CRITICAL FIX #2: Close TCP connection before cleanup (prevents socket leak!)
        //// ESP-IDF docs: "Call esp_http_client_close() to close the connection and free the socket
        //// before calling esp_http_client_cleanup()" - https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_client.html
        //// Without close(): TCP socket stays in TIME_WAIT state (~205 bytes leaked per TX)
        //// With close(): Socket freed immediately (zero leaks!)
        esp_http_client_close(client);
        
        //// Cleanup (properly frees all buffers - no leaks!)
        esp_http_client_cleanup(client);
        
        queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
        queueCount--;
        transmitted++;
      }  // ← MISSING CLOSING BRACE FOR while() LOOP!
      
      total_transmitted_http += transmitted;
      
      Serial.printf("*** Esp32 -->> SvHub: Sent %d tags via HTTP\n", transmitted);
      lastTransmitTime = currentTime;
      
      // Clear queue after transmission
      queueHead = 0;
      queueTail = 0;
      queueCount = 0;
      memset(tagQueue, 0, sizeof(tagQueue));
      Serial.println("*** Queue cleared after transmission");
    }
    xSemaphoreGive(queueMutex);
  }
}
#endif
#endif


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
  //// jwc 26-0124-1730 OPTIMIZATION: Reduce frame size for 2× faster AprilTag detection
  //// jwc 26-0124-1900 ARCHIVED: config.frame_size = FRAMESIZE_QVGA;  // 320x240 causes cropped/duplicated display
  //// jwc 26-0124-1900 RESTORED: Keep HVGA for full display, downsample for AprilTag only
  config.frame_size = FRAMESIZE_HVGA;  // 480x320 for display (full resolution)

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
      
      //// jwc 26-0130-1900 CRITICAL FIX: Crop camera image to center 240px (pixels 120-359)
      //// Camera captures 480×320, but screen is only 240×320 (portrait)
      //// Problem: Screen was showing LEFT HALF (pixels 0-239), camera center at right edge
      //// Solution: Extract CENTER 240px (pixels 120-359) so camera center aligns with screen center
      //// Result: AprilTag coordinates will match screen position perfectly!
      uint8_t *gray_buf = camera_framebuffer_pic_ObjPtr->buf;
      for (int y = 0; y < 320; y++) {
        for (int x = 0; x < 240; x++) {
          // Source pixel: skip first 120px, take next 240px (center crop)
          int src_idx = y * 480 + (x + 120);  // Offset by 120px to get center
          uint8_t gray = gray_buf[src_idx];
          // Destination pixel: 240×320 screen
          int dst_idx = y * 240 + x;
          // Convert grayscale to RGB565: R5G6B5
          rgb565_buf[dst_idx] = ((gray & 0xF8) << 8) | ((gray & 0xFC) << 3) | (gray >> 3);
        }
      }
      
      //// jwc 26-0127-2140 REVERTED: Cannot free camera buffer here - AprilTag detector needs it!
      //// Original attempt (jwc 26-0127-2130) caused crash: LoadProhibited at 0x00000008
      //// AprilTag detector accesses camera_framebuffer_pic_ObjPtr->buf later in code
      //// NEW LOCATION: After apriltag_detections_destroy() (line ~1380)
      
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
      //// jwc 26-0130-2040 ARCHIVED: gfx->draw16bitRGBBitmap(0, 0, rgb565_buf, 480, 320);
      //// jwc 26-0130-2040 CRITICAL FIX: Width parameter must match buffer width (240px, not 480px!)
      //// Buffer is 240×320 after center crop, so draw call must use 240 as width parameter
      gfx->draw16bitRGBBitmap(0, 0, rgb565_buf, 240, 320);
      #if DEBUG_PRINT_CAM
      Serial.println("*** Image drawn to screen via GFX");
      #endif
      
      //// jwc 26-0124-2040 NEW: Update FPS tracking
      frameCount++;
      unsigned long currentTime = millis();
      if (currentTime - fpsUpdateTime >= FPS_UPDATE_INTERVAL) {
        currentFPS = frameCount * 1000.0 / (currentTime - fpsUpdateTime);
        frameCount = 0;
        fpsUpdateTime = currentTime;
      }
      
      //// jwc 26-0109-1520 NEW: Draw comm overlay and button after camera image
      draw_comm_overlay();  // Draw yellow text if enabled
      draw_comm_button();   // Always draw button at bottom
      
      //// jwc 26-0124-2040 NEW: Draw HUD overlay (FPS + latest tag data)
      draw_hud_overlay();
      
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
    //// jwc 26-0124-1900 ARCHIVED: Crop to visible display area (240x320) - caused issues
    //// image_u8_t im = {
    ////   .width = 240,  // Match display width (was 480)
    ////   .height = 320,  // Keep full height
    ////   .stride = camera_framebuffer_pic_ObjPtr->width,  // Keep original stride for proper addressing
    ////   .buf = camera_framebuffer_pic_ObjPtr->buf
    //// };
    
    //// jwc 26-0124-2200 ARCHIVED: Downsampling optimization (C struct const members unfixable)
    //// jwc 26-0124-2200 ARCHIVED: //// jwc 26-0124-1900 NEW: Downsample HVGA (480x320) to QVGA (240x160) for AprilTag processing
    //// jwc 26-0124-2200 ARCHIVED: //// Display stays full resolution, but AprilTag processes smaller image (4× faster!)
    //// jwc 26-0124-2200 ARCHIVED: //// Strategy: Skip every other pixel in both X and Y directions
    //// jwc 26-0124-2200 ARCHIVED: //// 
    //// jwc 26-0124-2200 ARCHIVED: //// jwc 26-0124-2010 OPTIMIZATION EXPLANATION:
    //// jwc 26-0124-2200 ARCHIVED: //// * Camera captures: 480×320 = 153,600 pixels (display stays full resolution!)
    //// jwc 26-0124-2200 ARCHIVED: //// * Downsample to: 240×160 = 38,400 pixels (skip every 2nd pixel)
    //// jwc 26-0124-2200 ARCHIVED: //// * AprilTag processes: ONLY 38,400 pixels (4× fewer!)
    //// jwc 26-0124-2200 ARCHIVED: //// * Processing time: ~50ms per frame (4× faster!)
    //// jwc 26-0124-2200 ARCHIVED: 
    //// jwc 26-0124-2200 ARCHIVED: //// Allocate downsampled buffer (240x160 = 38,400 bytes) - static so allocated only once
    //// jwc 26-0124-2200 ARCHIVED: static uint8_t *downsample_buf = NULL;
    //// jwc 26-0124-2200 ARCHIVED: static image_u8_t fullres_im = {0};
    //// jwc 26-0124-2200 ARCHIVED: static image_u8_t downsampled_im = {0};
    //// jwc 26-0124-2200 ARCHIVED: 
    //// jwc 26-0124-2200 ARCHIVED: //// jwc 26-0124-2030 FIX: Use pointer to struct (proper C approach, no copy needed)
    //// jwc 26-0124-2200 ARCHIVED: image_u8_t *im_ptr;
    //// jwc 26-0124-2200 ARCHIVED: 
    //// jwc 26-0124-2200 ARCHIVED: if (downsample_buf == NULL) {
    //// jwc 26-0124-2200 ARCHIVED:   downsample_buf = (uint8_t *)heap_caps_malloc(240 * 160, MALLOC_CAP_SPIRAM);
    //// jwc 26-0124-2200 ARCHIVED:   if (!downsample_buf) {
    //// jwc 26-0124-2200 ARCHIVED:     Serial.println("*** ERROR: Failed to allocate downsample buffer!");
    //// jwc 26-0124-2200 ARCHIVED:     // Fall back to full resolution processing
    //// jwc 26-0124-2200 ARCHIVED:     fullres_im.width = camera_framebuffer_pic_ObjPtr->width;
    //// jwc 26-0124-2200 ARCHIVED:     fullres_im.height = camera_framebuffer_pic_ObjPtr->height;
    //// jwc 26-0124-2200 ARCHIVED:     fullres_im.stride = camera_framebuffer_pic_ObjPtr->width;
    //// jwc 26-0124-2200 ARCHIVED:     fullres_im.buf = camera_framebuffer_pic_ObjPtr->buf;
    //// jwc 26-0124-2200 ARCHIVED:     im_ptr = &fullres_im;
    //// jwc 26-0124-2200 ARCHIVED:     goto skip_downsample;
    //// jwc 26-0124-2200 ARCHIVED:   }
    //// jwc 26-0124-2200 ARCHIVED:   Serial.println("*** Downsample buffer allocated (240x160)");
    //// jwc 26-0124-2200 ARCHIVED: }
    //// jwc 26-0124-2200 ARCHIVED: 
    //// jwc 26-0124-2200 ARCHIVED: //// Downsample: Take every 2nd pixel in X and Y (2× reduction in each dimension = 4× total)
    //// jwc 26-0124-2200 ARCHIVED: uint8_t *src = camera_framebuffer_pic_ObjPtr->buf;
    //// jwc 26-0124-2200 ARCHIVED: uint8_t *dst = downsample_buf;
    //// jwc 26-0124-2200 ARCHIVED: for (int y = 0; y < 160; y++) {
    //// jwc 26-0124-2200 ARCHIVED:   for (int x = 0; x < 240; x++) {
    //// jwc 26-0124-2200 ARCHIVED:     // Source pixel at (x*2, y*2) in 480x320 image
    //// jwc 26-0124-2200 ARCHIVED:     dst[y * 240 + x] = src[(y * 2) * 480 + (x * 2)];
    //// jwc 26-0124-2200 ARCHIVED:   }
    //// jwc 26-0124-2200 ARCHIVED: }
    //// jwc 26-0124-2200 ARCHIVED: 
    //// jwc 26-0124-2200 ARCHIVED: //// Point to downsampled image for AprilTag detector
    //// jwc 26-0124-2200 ARCHIVED: downsampled_im.width = 240;
    //// jwc 26-0124-2200 ARCHIVED: downsampled_im.height = 160;
    //// jwc 26-0124-2200 ARCHIVED: downsampled_im.stride = 240;
    //// jwc 26-0124-2200 ARCHIVED: downsampled_im.buf = downsample_buf;
    //// jwc 26-0124-2200 ARCHIVED: im_ptr = &downsampled_im;
    //// jwc 26-0124-2200 ARCHIVED: 
    //// jwc 26-0124-2200 ARCHIVED: skip_downsample:
    
    //// jwc 26-0130-2150 NEW APPROACH: Cropped AprilTag buffer (initialize at declaration!)
    //// Previous failure: Tried to declare then assign (assignment operator deleted)
    //// New solution: Initialize struct at declaration time (no assignment needed!)
    //// Result: AprilTag only detects in visible 240px screen area
    
    //// Allocate cropped buffer (240×320 = 76,800 bytes) - static so allocated only once
    static uint8_t *cropped_apriltag_buf = NULL;
    
    if (cropped_apriltag_buf == NULL) {
      cropped_apriltag_buf = (uint8_t *)heap_caps_malloc(240 * 320, MALLOC_CAP_SPIRAM);
      if (!cropped_apriltag_buf) {
        Serial.println("*** ERROR: Failed to allocate cropped AprilTag buffer!");
      } else {
        Serial.println("*** Cropped AprilTag buffer allocated (240×320)");
      }
    }
    
    //// Extract center 240px from camera's 480px width (same as display crop)
    if (cropped_apriltag_buf != NULL) {
      uint8_t *src = camera_framebuffer_pic_ObjPtr->buf;
      for (int y = 0; y < 320; y++) {
        for (int x = 0; x < 240; x++) {
          // Source: skip first 120px, take next 240px (center crop)
          int src_idx = y * 480 + (x + 120);
          cropped_apriltag_buf[y * 240 + x] = src[src_idx];
        }
      }
    }
    
    //// jwc 26-0130-2150 CRITICAL: Initialize struct at declaration (not assignment!)
    //// This avoids the deleted assignment operator issue
    image_u8_t im = (cropped_apriltag_buf != NULL) ? 
      (image_u8_t){
        .width = 240,
        .height = 320,
        .stride = 240,
        .buf = cropped_apriltag_buf
      } : 
      (image_u8_t){
        .width = camera_framebuffer_pic_ObjPtr->width,
        .height = camera_framebuffer_pic_ObjPtr->height,
        .stride = camera_framebuffer_pic_ObjPtr->width,
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


    //// jwc 26-0130-0705 CRITICAL FIX #1 (Phase 1): Enforce 1 Hz detection rate UNCONDITIONALLY
    //// ARCHIVED (memory leak): zarray_t *detections = apriltag_detector_detect(td, &im); (runs 30 FPS = 9KB/sec leak!)
    //// Problem: AprilTag library has internal memory leak (~30 bytes per detection)
    //// At 30 FPS: 30 bytes × 30 = 900 bytes/sec → system crashes in 2.5 minutes!
    //// Solution: Only detect every 1 second (1 Hz), regardless of networking state
    //// Result: 30 bytes × 1 = 30 bytes/sec → system runs indefinitely! (30× improvement)
    //// Reference: Plan-MEMORY_LEAK_ANALYSIS.md - Critical Fix #1
    
    //// jwc 26-0129-1810 ARCHIVED: Old conditional detection (only worked when networking enabled)
    //// #if DEFINE_NETWORK_HTTP_BOOL || DEFINE_NETWORK_WEBSOCKET_BOOL
    //// // Only detect when ready to transmit (reduces leak by 30×)
    //// if (currentTime - lastTransmitTime >= TRANSMIT_INTERVAL) {
    ////   detections = apriltag_detector_detect(td, &im);
    ////   total_detections++;
    //// } else {
    ////   detections = zarray_create(sizeof(apriltag_detection_t*));  // Empty array
    //// }
    //// #else
    //// // Networking disabled: always detect (for testing) ← THIS CAUSED LEAK!
    //// detections = apriltag_detector_detect(td, &im);
    //// total_detections++;
    //// #endif
    
    // NEW: Detect at 1 Hz ALWAYS (networking enabled or disabled)
    zarray_t *detections = NULL;
    unsigned long currentTime = millis();
    static unsigned long lastDetectionTime = 0;
    const unsigned long DETECTION_INTERVAL = 1000; // 1 second = 1 Hz
    
    if (currentTime - lastDetectionTime >= DETECTION_INTERVAL) {
      detections = apriltag_detector_detect(td, &im);
      //// jwc 26-0130-0716 MOVED: Increment counter ONLY when tags actually detected (not every detection attempt)
      //// total_detections++;  // OLD: Counted detection attempts, not actual tags found
      lastDetectionTime = currentTime;
    } else {
      // Skip detection this frame (just display camera image)
      detections = zarray_create(sizeof(apriltag_detection_t*));  // Empty array
    }
    
#if DEBUG >= 3
    Serial.println("done. Result:");
#endif


if(zarray_size(detections) > 0){
  //// jwc 26-0130-0716 NEW: Increment counter when actual AprilTag detected (not just detection attempts)
  total_detections++;  // Count actual tags found, not detection attempts
  
  //// jwc 26-0124-2240 NEW: Find tag closest to screen center (480x320 → center at 240,160)
  int closest_idx = -1;
  float min_dist_to_center = 999999.0;
  float screen_center_x = 240.0;
  float screen_center_y = 160.0;
  
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    
    // Calculate distance from tag center to screen center
    float dx = det->c[0] - screen_center_x;
    float dy = det->c[1] - screen_center_y;
    float dist = sqrt(dx*dx + dy*dy);
    
    if (dist < min_dist_to_center) {
      min_dist_to_center = dist;
      closest_idx = i;
    }
  }
  
  //// jwc 26-0124-2240 Process ONLY the closest tag (prevents freeze with multiple tags)
  //// jwc 26-0125-0400 UPDATED: Moved tag ID display from upper-left to upper-right
  if (closest_idx >= 0) {
    //// jwc 26-0125-0400 ARCHIVED: Original upper-left tag ID display
    //// jwc 26-0125-0400 ARCHIVED: //// jwc oo tft.setCursor(1,1);
    //// jwc 26-0125-0400 ARCHIVED: gfx->setCursor(1,1);
    //// jwc 26-0125-0400 ARCHIVED: //// jwc y good for nomral, but increase for VideoMeet-Cam \/: tft.setTextSize(2); tft.setTextSize(3);
    //// jwc 26-0125-0400 ARCHIVED: //// jwc y tft.setTextSize(4);
    //// jwc 26-0125-0400 ARCHIVED: //// jwc y seems just right to fit 6 max
    //// jwc 26-0125-0400 ARCHIVED: //// jwc oo tft.setTextSize(5);
    //// jwc 26-0125-0400 ARCHIVED: gfx->setTextSize(5);
    
    // Process only the closest tag
    int i = closest_idx;
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      //// jwc \/
      //// jwc y Serial.print(" *** ");
      
      //// jwc 26-0125-0400 NEW: Display tag ID in upper-right corner (large font)
      gfx->setTextSize(5);
      
      //// Calculate x position for right-aligned text (approximate width: 80px for 2 digits)
      int tag_id_width = 80;  // Approximate width for "XX " with size 5 font
      int x_pos = 240 - tag_id_width;  // Right-align
      gfx->setCursor(x_pos, 1);
      
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
      printf("*** *** [DETECT ID:] %5.0d,%5.0f,", det->id, det->decision_margin);

      // Creating detection info object to feed into pose estimator
      apriltag_detection_info_t info;
      info.det = det;
      info.tagsize = APRIL_TAG_SIZE;
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

#if DEBUG >= 2
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

      // Compute the transpose of the rotation matrix
      matd_t *R_transpose = matd_transpose(pose.R);

      // Negate the translation vector
      for (int i = 0; i < pose.t->nrows; i++) {
          MATD_EL(pose.t, i, 0) = -MATD_EL(pose.t, i, 0);
      }

      // Compute the position of the camera in the tag's coordinate system
      matd_t *camera_position = matd_multiply(R_transpose, pose.t);
      
      //// jwc 26-0124-1745 CRITICAL FIX: Extract x,y,z values BEFORE freeing matrices!
      //// Bug was: matrices freed, THEN we tried to read from freed memory → always 0!
      float x_cm = MATD_EL(camera_position, 0, 0) * 100.0;  // Convert meters to cm
      float y_cm = MATD_EL(camera_position, 1, 0) * 100.0;
      float z_cm = MATD_EL(camera_position, 2, 0) * 100.0;
      
      //// jwc 26-0124-1800 NEW: Calculate range (3D distance from camera to tag)
      float range_cm = sqrt(x_cm*x_cm + y_cm*y_cm + z_cm*z_cm);
      
      //// jwc 26-0124-1335 UPDATED: Single-line AprilTag detection output (keep original printf for screen display)
      printf("*** *** [DETECT ID:] %5.0d,%5.0f,", det->id, det->decision_margin);
      printf(" *** y,p,r: %5.0f, %5.0f, %5.0f", yaw, pitch, roll);
      printf(" *** x,y,z: %.1f, %.1f, %.1f cm, range: %.1f cm\n", x_cm, y_cm, z_cm, range_cm);

      // Free the matrices (AFTER extracting values!)
      matd_destroy(R_transpose);
      matd_destroy(camera_position);

      //// jwc 26-0124-2050 NEW: Update latestTag struct for HUD display
      latestTag.hasData = true;
      latestTag.id = det->id;
      latestTag.yaw = yaw;
      latestTag.pitch = pitch;
      latestTag.roll = roll;
      latestTag.x = x_cm;
      latestTag.y = y_cm;
      latestTag.z = z_cm;
      latestTag.range = range_cm;
      
      //// jwc 26-0128-1440 ARCHIVED: Old DEFINE_NETWORKING_BOOL guard
      //// #if DEFINE_NETWORKING_BOOL
      //// //// jwc 26-0124-1030 PHASE 7: Enqueue detected AprilTag for WebSocket transmission
      //// //// jwc 26-0124-1745 FIX: Use extracted values (not freed memory!)
      //// //// jwc 26-0124-1800 NEW: Pass range to enqueue function
      //// enqueueAprilTag(det->id, det->decision_margin, yaw, pitch, roll, 
      ////                 x_cm, y_cm, z_cm, range_cm);
      //// #endif
      
      //// jwc 26-0128-1440 NEW: Protocol-agnostic enqueue (works for HTTP, WebSocket, and UDP)
      #if DEFINE_NETWORK_HTTP_BOOL || DEFINE_NETWORK_WEBSOCKET_BOOL || DEFINE_NETWORK_UDP_BOOL
      //// jwc 26-0124-1030 PHASE 7: Enqueue detected AprilTag for network transmission
      //// jwc 26-0124-1745 FIX: Use extracted values (not freed memory!)
      //// jwc 26-0124-1800 NEW: Pass range to enqueue function
      enqueueAprilTag(det->id, det->decision_margin, yaw, pitch, roll, 
                      x_cm, y_cm, z_cm, range_cm);
      #endif
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

    //// jwc 26-0127-2140 CRITICAL FIX: Return camera buffer AFTER AprilTag detection
    //// AprilTag detector needs camera_framebuffer_pic_ObjPtr->buf, so we can't free it earlier
    //// This still reduces hold time from ~250ms (full loop) to ~50ms (detection only)
    //// Previous attempt (jwc 26-0127-2130) freed too early → crash at 0x00000008
    esp_camera_fb_return(camera_framebuffer_pic_ObjPtr);
    camera_framebuffer_pic_ObjPtr = NULL;
    
    //// jwc 26-0128-1440 ARCHIVED: Old DEFINE_NETWORKING_BOOL guard (WebSocket only)
    //// #if DEFINE_NETWORKING_BOOL
    //// //// jwc 26-0124-1030 PHASE 7: Transmit queued AprilTags via WebSocket
    //// transmitAprilTags();
    //// 
    //// //// jwc 26-0124-1030 PHASE 7: Process WebSocket events
    //// //// jwc 26-0128-0500 ARCHIVED: Links2004 loop method
    //// //// webSocket.loop();
    //// //// jwc 26-0128-0500 NEW: ArduinoWebsockets poll method
    //// webSocket.poll();
    //// #endif
    
    //// jwc 26-0128-1440 NEW: Protocol-specific transmission (HTTP, WebSocket, or UDP)
    #if DEFINE_NETWORK_HTTP_BOOL
    //// HTTP: Stateless POST requests (no persistent connection)
    transmitAprilTagsHTTP();
    #elif DEFINE_NETWORK_WEBSOCKET_BOOL
    //// WebSocket: Persistent connection (MEMORY LEAK!)
    transmitAprilTagsWebSocket();
    webSocket.poll();  // Process WebSocket events
    #elif DEFINE_NETWORK_UDP_BOOL
    //// UDP: Fire-and-forget packets (ZERO memory leaks expected!)
    transmitAprilTagsUDP();
    #endif
    
    vTaskDelay(pdMS_TO_TICKS(1));
    //// jwc 26-0127-2140 NOTE: camera_framebuffer_pic_ObjPtr already freed after AprilTag detection
    //// No need to clear again here - already freed and nulled at line ~1383



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

  //// jwc 26-0128-0030 REVERTED Fix #3: LVGL buffer reduction broke PSRAM initialization
  //// Attempted to reduce from 300KB to 4.8KB, but caused PSRAM = 0 bytes (watchdog reset)
  //// Root cause: Early INTERNAL DRAM allocation prevented PSRAM hardware initialization
  //// RESTORED: Original allocation (works, even if wasteful)
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

  //// jwc 26-0124-1730 OPTIMIZATION: Enable multi-threading for 1.5-2× faster AprilTag detection
  //// jwc OLD (slow): td->nthreads = 1;  // Single core only
  td->nthreads = 2;  // Use both ESP32 cores (1.5-2× faster!)

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


  //// jwc 26-0128-1440 ARCHIVED: Old DEFINE_NETWORKING_BOOL guard (WebSocket only)
  //// #if DEFINE_NETWORKING_BOOL
  //// //// jwc 26-0124-1030 PHASE 7: Initialize WiFi and WebSocket
  //// initWiFi();
  //// 
  //// //// jwc 26-0124-1030 PHASE 7: Initialize queue mutex
  //// queueMutex = xSemaphoreCreateMutex();
  //// if (queueMutex == NULL) {
  ////   Serial.println("*** ERROR: Failed to create queue mutex!");
  //// } else {
  ////   Serial.println("*** Queue mutex created successfully");
  //// }
  //// 
  //// //// jwc 26-0124-1030 PHASE 7: Initialize WebSocket client
  //// //// jwc 26-0128-0500 ARCHIVED: Links2004 WebSocketsClient initialization
  //// //// webSocket.begin(ws_host, ws_port, ws_path);
  //// //// webSocket.onEvent(webSocketEvent);
  //// //// webSocket.setReconnectInterval(5000);
  //// //// jwc 26-0128-0500 NEW: ArduinoWebsockets initialization
  //// setupWebSocketHandlers();
  //// 
  //// // Build WebSocket URL
  //// char ws_url[128];
  //// snprintf(ws_url, sizeof(ws_url), "ws://%s:%d%s", ws_host, ws_port, ws_path);
  //// 
  //// bool connected = webSocket.connect(ws_url);
  //// if (connected) {
  ////   Serial.printf("*** WebSocket client connected to: %s\n", ws_url);
  //// } else {
  ////   Serial.printf("*** ERROR: WebSocket connection failed to: %s\n", ws_url);
  //// }
  //// #else
  //// Serial.println("*** NETWORKING DISABLED - AprilTag-only mode for memory leak testing");
  //// #endif
  
  //// jwc 26-0128-1440 NEW: Protocol-specific initialization (HTTP, WebSocket, or UDP)
  #if DEFINE_NETWORK_HTTP_BOOL || DEFINE_NETWORK_WEBSOCKET_BOOL || DEFINE_NETWORK_UDP_BOOL
  //// jwc 26-0124-1030 PHASE 7: Initialize WiFi (common for all protocols)
  initWiFi();
  
  //// jwc 26-0129-0750 CRITICAL FIX #1: Disable WiFi sleep mode to prevent memory leak
  //// WiFi power management causes TCP stack to reallocate buffers on wake (~160 bytes/TX leak)
  //// Disabling sleep keeps WiFi active 24/7, preventing reallocation overhead
  //// Trade-off: Higher power consumption, but stable memory usage
  WiFi.setSleep(false);
  Serial.println("*** WiFi sleep disabled (max performance mode)");
  
  //// jwc 26-0124-1030 PHASE 7: Initialize queue mutex (common for all protocols)
  queueMutex = xSemaphoreCreateMutex();
  if (queueMutex == NULL) {
    Serial.println("*** ERROR: Failed to create queue mutex!");
  } else {
    Serial.println("*** Queue mutex created successfully");
  }
  
  #if DEFINE_NETWORK_HTTP_BOOL
  //// jwc 26-0128-1440 NEW: HTTP mode - no persistent connection setup needed!
  Serial.println("*** HTTP mode enabled - stateless POST requests");
  Serial.printf("*** Server: http://%s:%d%s\n", server_host, server_port, http_endpoint);
  #endif
  
  #if DEFINE_NETWORK_UDP_BOOL
  //// jwc 26-0130-1100 NEW: UDP mode - no persistent connection setup needed!
  Serial.println("*** UDP mode enabled - fire-and-forget packets");
  Serial.printf("*** Server: udp://%s:%d\n", server_host, server_port);
  #endif
  
  #if DEFINE_NETWORK_WEBSOCKET_BOOL
  //// jwc 26-0124-1030 PHASE 7: Initialize WebSocket client (persistent connection)
  //// jwc 26-0128-0500 ARCHIVED: Links2004 WebSocketsClient initialization
  //// webSocket.begin(ws_host, ws_port, ws_path);
  //// webSocket.onEvent(webSocketEvent);
  //// webSocket.setReconnectInterval(5000);
  //// jwc 26-0128-0500 NEW: ArduinoWebsockets initialization
  setupWebSocketHandlers();
  
  // Build WebSocket URL
  char ws_url[128];
  snprintf(ws_url, sizeof(ws_url), "ws://%s:%d%s", server_host, server_port, ws_path);
  
  bool connected = webSocket.connect(ws_url);
  if (connected) {
    Serial.printf("*** WebSocket client connected to: %s\n", ws_url);
  } else {
    Serial.printf("*** ERROR: WebSocket connection failed to: %s\n", ws_url);
  }
  #endif
  
  #else
  Serial.println("*** NETWORKING DISABLED - AprilTag-only mode for memory leak testing");
  #endif

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
  //// jwc 26-0127-2010 NEW: Memory monitoring for leak detection
  //// jwc 26-0130-0015 UPDATED: Enhanced memory tracking with MinDRAM and detection counter
  //// jwc 26-0130-0515 UPDATED: Added MemLoss per detection calculation
  //// jwc 26-0130-0823 UPDATED: Added heap defragmentation every 60 seconds
  //// jwc 26-0130-0855 CRITICAL FIX: Capture initial_heap AFTER first detection (not at boot!)
  //// Print heap and PSRAM usage every loop iteration to track memory leaks
  static unsigned long last_mem_print = 0;
  static unsigned long last_defrag = 0;  // Track last defragmentation time
  static uint32_t initial_heap = 0;  // Track initial heap AFTER first detection
  static bool first_detection_done = false;  // Flag to capture heap after first detection
  unsigned long current_time = millis();
  
  //// jwc 26-0130-0855 CRITICAL FIX: Capture initial_heap AFTER first detection
  //// Problem: Old code captured at boot, including LVGL/WiFi/camera setup (~800KB!)
  //// Result: mem_loss_per_detect was MASSIVELY inflated (18,100 bytes vs actual 93 bytes)
  //// Solution: Capture AFTER first detection to measure ONLY detection leaks
  if (total_detections == 1 && !first_detection_done) {
    initial_heap = ESP.getFreeHeap();  // Capture AFTER first detection
    first_detection_done = true;
    Serial.println("*** [MEM] Initial heap captured after first detection");
  }
  
  //// jwc 26-0130-0823 NEW: Emergency heap defragmentation every 60 seconds
  //// jwc 26-0130-0847 UPDATED: Added before/after memory measurement for effectiveness tracking
  //// Forces ESP32 to check heap integrity and consolidate free blocks
  //// May help reduce fragmentation from repeated alloc/free cycles
  //// Reference: Plan-MEMORY_LEAK_ANALYSIS_26-0130-0753.md - Fix #3
  if (current_time - last_defrag >= 60000) {  // Every 60 seconds
    uint32_t heap_before = ESP.getFreeHeap();
    uint32_t largest_before = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    
    heap_caps_check_integrity_all(true);  // Force heap cleanup
    
    uint32_t heap_after = ESP.getFreeHeap();
    uint32_t largest_after = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    int heap_recovered = (int)heap_after - (int)heap_before;
    int largest_improved = (int)largest_after - (int)largest_before;
    
    last_defrag = current_time;
    Serial.printf(">>> >>> 26-0130-0847 [DEFRAG] Heap: %d→%d (%+d b) | Largest: %d→%d (%+d b)\n", 
                  heap_before, heap_after, heap_recovered,
                  largest_before, largest_after, largest_improved);
  }
  
  if (current_time - last_mem_print >= 2000) {  // Print every 2 seconds
    uint32_t current_heap = ESP.getFreeHeap();
    int mem_loss_total = (int)initial_heap - (int)current_heap;
    //// jwc 26-0130-0855 UPDATED: Exclude first detection from average (it's the baseline)
    int mem_loss_per_detect = (total_detections > 1) ? (mem_loss_total / (total_detections - 1)) : 0;
    
    Serial.printf("\n");
    //// YELLOW for loop() memory monitoring (user request: print in yellow font-color)
    Serial.printf("\033[33m[MEM] DRAM: %d b | PSRAM: %d b | MinDRAM: %d b | Detections: %d | MemLoss PerDetect: %d b\033[0m\n", 
                  current_heap, 
                  ESP.getFreePsram(),
                  ESP.getMinFreeHeap(),
                  total_detections,
                  mem_loss_per_detect);
    Serial.printf("\n");
    last_mem_print = current_time;
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
