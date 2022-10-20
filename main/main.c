#include <stdio.h>
#include "esp_log.h"
#include "i2c.h"
#include "trill.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mixesp-example";

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    trill_identity identity = trill_cmd_identify();
    ESP_LOGI(TAG, "IDENTITY = b%X v%X", identity.board, identity.version);

    trill_mode mode = trill_cmd_mode(0);
    ESP_LOGI(TAG, "MODE = %X", mode.mode);

    trill_scan_setting setting = trill_cmd_scan_setting(0, 12);
    ESP_LOGI(TAG, "SETTING = s%X n%X", setting.speed, setting.num_bits);

    trill_baseline baseline = trill_cmd_baseline_update();
    ESP_LOGI(TAG, "BASELINE = {}");

    for (uint8_t x = 0; x < 1000; x++)
    {
        trill_touches bar = trill_read_touch();
        ESP_LOGI(TAG, "num_touches: %02X", bar.num_touches);
        // ESP_LOGI(TAG, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", bar.data[0], bar.data[1], bar.data[2], bar.data[3], bar.data[4], bar.data[5], bar.data[6], bar.data[7], bar.data[8], bar.data[9]);
        // ESP_LOGI(TAG, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", bar.data[10], bar.data[11], bar.data[12], bar.data[13], bar.data[14], bar.data[15], bar.data[16], bar.data[17], bar.data[18], bar.data[19]);

        for (uint8_t i = 0; i < bar.num_touches; i++)
        {
            trill_touch touch = bar.touches[i];
            ESP_LOGI(TAG, "i: %04X x: %04X, y: %04X, size: %04X", i, touch.x, touch.y, touch.size);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_ERROR_CHECK(i2c_master_deinit());
    ESP_LOGI(TAG, "I2C de-initialized successfully");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    };
}
