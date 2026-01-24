/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */

#include <lvgl.h>
#include <demos/lv_demos.h>
#include <Arduino_GFX_Library.h>
#include "bsp_cst816.h"
/*To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 *You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 Note that the `lv_examples` library is for LVGL v7 and you shouldn't install it for this version (since LVGL v8)
 as the examples and demos are now part of the main LVGL library. */

// #include <examples/lv_examples.h>
// #include <demos/lv_demos.h>


#define EXAMPLE_PIN_NUM_LCD_SCLK 39
#define EXAMPLE_PIN_NUM_LCD_MOSI 38
#define EXAMPLE_PIN_NUM_LCD_MISO 40
#define EXAMPLE_PIN_NUM_LCD_DC 42
#define EXAMPLE_PIN_NUM_LCD_RST -1
#define EXAMPLE_PIN_NUM_LCD_CS 45
#define EXAMPLE_PIN_NUM_LCD_BL 1
#define EXAMPLE_PIN_NUM_TP_SDA 48
#define EXAMPLE_PIN_NUM_TP_SCL 47

#define LEDC_FREQ             5000
#define LEDC_TIMER_10_BIT     10

#define EXAMPLE_LCD_ROTATION 0
#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 320


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

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
  lv_disp_flush_ready(disp_drv);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  static uint32_t last_touch_time = 0;
  static uint32_t last_poll_debug = 0;
  static uint32_t poll_count = 0;
  
  //// jwc 26-0105-1920 Debug: Show that this function is being called
  poll_count++;
  if (millis() - last_poll_debug > 3000) {
    Serial.printf("my_touchpad_read called %d times in last 3 sec\n", poll_count);
    poll_count = 0;
    last_poll_debug = millis();
  }
  
  uint16_t touchpad_x;
  uint16_t touchpad_y;
  bsp_touch_read();
  
  bool got_coords = bsp_touch_get_coordinates(&touchpad_x, &touchpad_y);
  
  //// jwc 26-0105-1920 Debug: Show result of bsp_touch_get_coordinates
  static uint32_t last_coord_debug = 0;
  if (millis() - last_coord_debug > 2000) {
    Serial.printf("bsp_touch_get_coordinates returned: %s\n", got_coords ? "TRUE" : "FALSE");
    last_coord_debug = millis();
  }
  
  if (got_coords) {
    data->point.x = touchpad_x;
    data->point.y = touchpad_y;
    data->state = LV_INDEV_STATE_PRESSED;
    
    //// jwc 26-0105-1900 Debug: Print touch coordinates every 500ms
    if (millis() - last_touch_time > 500) {
      Serial.printf("TOUCH DETECTED: x=%d, y=%d\n", touchpad_x, touchpad_y);
      last_touch_time = millis();
    }
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void setup() {
  Serial.begin(115200);

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
  //// jwc 26-0105-1815 PlatformIO Arduino ESP32 3.2.0 uses older LEDC API
  //// * Arduino IDE: ledcAttach() and ledcWrite() (newer API)
  //// * PlatformIO: ledcSetup(), ledcAttachPin(), and ledcWrite() (older API)
  //// jwc 26-0105-1815 ledcAttach(EXAMPLE_PIN_NUM_LCD_BL , LEDC_FREQ, LEDC_TIMER_10_BIT);
  ledcSetup(0, LEDC_FREQ, LEDC_TIMER_10_BIT);
  ledcAttachPin(EXAMPLE_PIN_NUM_LCD_BL, 0);
  ledcWrite(0, (1 << LEDC_TIMER_10_BIT) / 100 * 80);
#endif

  // Init touch device
  // touch_init(gfx->width(), gfx->height(), gfx->getRotation());
  Serial.println("Initializing I2C for touch...");
  Wire.begin(EXAMPLE_PIN_NUM_TP_SDA, EXAMPLE_PIN_NUM_TP_SCL);
  Serial.printf("I2C initialized: SDA=%d, SCL=%d\n", EXAMPLE_PIN_NUM_TP_SDA, EXAMPLE_PIN_NUM_TP_SCL);
  
  //// jwc 26-0105-1850 Fixed: Use EXAMPLE_LCD_ROTATION instead of gfx->getRotation() to match 01_factory
  //// * 01_factory uses: bsp_touch_init(&Wire, EXAMPLE_LCD_ROTATION, gfx->width(), gfx->height())
  //// * This was using: bsp_touch_init(&Wire, gfx->getRotation(), gfx->width(), gfx->height())
  //// * gfx->getRotation() returns the display rotation, but touch needs the LCD rotation constant
  Serial.printf("Calling bsp_touch_init: rotation=%d, width=%d, height=%d\n", 
                EXAMPLE_LCD_ROTATION, gfx->width(), gfx->height());
  bool touch_ok = bsp_touch_init(&Wire, EXAMPLE_LCD_ROTATION, gfx->width(), gfx->height());
  if (touch_ok) {
    Serial.println("Touch initialized successfully!");
  } else {
    Serial.println("ERROR: Touch initialization failed!");
  }
  
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
    //// jwc 26-0105-2000 Changed from direct_mode to full_refresh to match 01_factory
    //// * direct_mode = true causes touch to work but UI doesn't respond
    //// * full_refresh = 1 (like 01_factory) should fix UI responsiveness
    //// disp_drv.direct_mode = true;
    disp_drv.full_refresh = 1;

    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);


    /* Option 3: Or try out a demo. Don't forget to enable the demos in lv_conf.h. E.g. LV_USE_DEMOS_WIDGETS*/
    lv_demo_widgets();
    // lv_demo_benchmark();
    // lv_demo_keypad_encoder();
    // lv_demo_music();
    // lv_demo_stress();
  }

  Serial.println("Setup done");
}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#else
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif
  delay(5);
}
