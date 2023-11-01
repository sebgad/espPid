#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "espPid.h"


extern "C" void app_main()
{
    // initialize non-volatile memory
    esp_err_t nvs_ret_init = nvs_flash_init();

    if (nvs_ret_init == ESP_OK){
        ESP_LOGI( "PID", "NVS flash initialized successfully.");
    } else if (nvs_ret_init == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret_init == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret_init = nvs_flash_init();
        ESP_LOGI( "PID", "NVS was not initialized successfully. Erased flash and reeinit.");
    }
    // Make sure that init was successfully.
    ESP_ERROR_CHECK(nvs_ret_init);

    nvs_handle_t pid_nvs_handle;
    nvs_open("PID", NVS_READWRITE, &pid_nvs_handle);
    espPid objPid(&pid_nvs_handle);

    float f_x_act = 0.F;
    float f_x_manip = 0.F;
    objPid.begin(&f_x_act, &f_x_manip);

    while(1){
        objPid.compute();
        ESP_LOGI("PID", "Actual value: %.2f, Manipulation Value: %.2f", f_x_act, f_x_manip);
        f_x_act += 1.F;
        vTaskDelay(pdMS_TO_TICKS(250));
        if (f_x_act > 100.F) {
            f_x_act = 0;
        }
    }
}
