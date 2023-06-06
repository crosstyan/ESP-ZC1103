/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <driver/gpio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "rfsystem.h"
#include "esp_spi_flash.h"
#include <stdio.h>

extern "C" void app_main();

static auto const PKT_GPIO = GPIO_NUM_25;
// other pin definitions are in rfsystem.h

void app_main() {
    // https://github.com/nkolban/esp32-snippets/blob/fe3d318acddf87c6918944f24e8b899d63c816dd/gpio/interrupts/test_intr.c#LL40C2-L40C22
    auto io_conf = gpio_config_t {
        .pin_bit_mask = 1ULL << PKT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);
    gpio_isr_handler_add(PKT_GPIO, [](void* arg) {
        printf("Interrupt!\n");
    }, nullptr);
    auto& rf = RfSystem::get();
    rf.begin();
    fflush(stdout);
    esp_restart();
}
