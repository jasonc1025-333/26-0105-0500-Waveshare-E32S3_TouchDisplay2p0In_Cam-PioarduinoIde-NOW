#include <arduino.h>
#include "app_system.h"

//// jwc 26-0105-1455 Temperature sensor API differs between Arduino IDE and PlatformIO
//// * Arduino IDE may have temperature_sensor.h with newer API
//// * PlatformIO Arduino ESP32 3.2.0 does NOT have driver/temperature_sensor.h
//// * PlatformIO uses older temp_sensor.h API for all versions
//// * Solution: Always use temp_sensor.h for PlatformIO compatibility
//// jwc 26-0105-1455 Arduino ESP32 3.0+ uses different temperature sensor API
//// jwc 26-0105-1455 #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
//// jwc 26-0105-1455 #include "driver/temperature_sensor.h"
//// jwc 26-0105-1455 #else
#include "driver/temp_sensor.h"
//// jwc 26-0105-1455 #endif

#include "esp_flash.h"
//// jwc 26-0105-1450 PSRAM header discrepancy between Arduino IDE and PlatformIO
//// * Arduino IDE (esp32 3.2.0): Uses esp_psram.h (may have compatibility wrapper)
//// * PlatformIO (espressif32 @6.10.0 = Arduino ESP32 3.2.0): Uses esp32-hal-psram.h
//// * PSRAM function name also different:
//// ** Arduino IDE: esp_psram_get_size()
//// ** PlatformIO: esp_spiram_get_size() (note: spiram not psram)
//// * PlatformIO uses Arduino HAL naming convention, Arduino IDE may have additional layers
//// * Verified: esp_psram.h does NOT exist in PlatformIO's framework directory
//// * Solution: Use esp32-hal-psram.h which provides esp_spiram_get_size()
//// jwc 26-0105-1450 #include "esp_psram.h"
#include "esp32-hal-psram.h"

#include "../lvgl_ui/lvgl_ui.h"
#include "../../bsp_lv_port.h"

#include "../../bsp_spi.h"
#include "SD.h"
#include "SPI.h"

//// jwc 26-0105-1500 PlatformIO uses older temp_sensor API (no handle needed)
//// jwc 26-0105-1500 temperature_sensor_handle_t temp_sensor = NULL;




void app_system_init(void) {
  //// jwc 26-0105-1500 PlatformIO temp_sensor API: temp_sensor_set_config() and temp_sensor_start()
  //// jwc 26-0105-1500 Arduino IDE API: temperature_sensor_install() and temperature_sensor_enable()
  temp_sensor_config_t temp_sensor_config = TSENS_CONFIG_DEFAULT();
  temp_sensor_set_config(temp_sensor_config);
  temp_sensor_start();
}

void app_system_task(void *arg) {
  char str[20];
  float tsens_out;
  uint32_t flash_size;
  uint32_t psram_size;
  uint32_t sd_size;
  
  esp_flash_get_size(NULL, &flash_size);
  //// jwc 26-0105-1450 PlatformIO uses esp_spiram_get_size() not esp_psram_get_size()
  psram_size = esp_spiram_get_size();


  if (bsp_spi_lock(-1)) {
    if (SD.begin(EXAMPLE_PIN_NUM_SD_CS, bsp_spi)) {
      uint8_t cardType = SD.cardType();
      if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
      }

      Serial.print("SD Card Type: ");
      if (cardType == CARD_MMC) {
        Serial.println("MMC");
      } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
      } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
      } else {
        Serial.println("UNKNOWN");
      }
      sd_size = (uint32_t)(SD.cardSize() / (1024 * 1024));
    }
    bsp_spi_unlock();
  }
  if (lvgl_lock(-1)) {

    snprintf(str, sizeof(str), "%dM", flash_size / (uint32_t)(1024 * 1024));
    lv_label_set_text(label_flash, str);  // 初始值

    snprintf(str, sizeof(str), "%dM", psram_size / (uint32_t)(1024 * 1024));
    lv_label_set_text(label_psram, str);  // 初始值

    snprintf(str, sizeof(str), "%dM", sd_size);
    lv_label_set_text(label_sd, str);  // 初始值

    lvgl_unlock();
  }

  while (1) {
    float sensorValue = 0;
    for (int i = 0; i < 10; i++) {
      sensorValue += analogRead(EXAMPLE_PIN_NUM_BAT_ADC) / 10.0;  // Read the value from the ADC
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    float voltage = sensorValue * (3.3 / 4095.0);
    //// jwc 26-0105-1500 PlatformIO API: temp_sensor_read_celsius(&tsens_out)
    //// jwc 26-0105-1500 Arduino IDE API: temperature_sensor_get_celsius(temp_sensor, &tsens_out)
    temp_sensor_read_celsius(&tsens_out);
    if (lvgl_lock(-1)) {
      snprintf(str, sizeof(str), "%.1fC", tsens_out);
      lv_label_set_text(label_chip_temp, str);  // 初始值

      snprintf(str, sizeof(str), "%.2fV", voltage * 3);
      lv_label_set_text(label_battery, str);  // 初始值
      lvgl_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void app_system_run(void) {
  xTaskCreate(app_system_task, "system_task", 4096, NULL, 0, NULL);
}
