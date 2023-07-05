#include <driver/gpio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "EspHal.h"
#include "RadioLib.h"
#include "boring.h"
#include "spot.h"
#include "instant.h"
#include "message_wrapper.h"
#include <etl/random.h>
#include <cstdio>
#include <msd/channel.hpp>
#include <memory>
#include "utils.h"
#include "channel.h"
#include <rtc_wdt.h>

extern "C" void app_main();

static auto const MISO_PIN  = GPIO_NUM_5;
static auto const MOSI_PIN  = GPIO_NUM_17;
static auto const SCK_PIN   = GPIO_NUM_16;
static auto const NSS_PIN   = GPIO_NUM_4;
static auto const BUSY_PIN  = GPIO_NUM_19;
static auto const RESET_PIN = GPIO_NUM_18;
static auto const DIO1_PIN  = GPIO_NUM_21;
static auto const DIO2_PIN  = GPIO_NUM_33;
static auto const DIO3_PIN  = GPIO_NUM_27;
static auto const TX_EN     = GPIO_NUM_25;
static auto const RX_EN     = GPIO_NUM_26;

using MessageVector = etl::vector<uint8_t, MessageWrapper::MAX_ENCODER_OUTPUT_SIZE>;
using MessageQueue  = msd::channel<MessageVector>;

struct RfTaskParam {
  LLCC68 *rf;
  std::shared_ptr<MessageQueue> channel;
};

struct LedTaskParam {
  std::shared_ptr<MessageQueue> channel;
};

void app_main() {
  auto hal = EspHal(SCK_PIN, MISO_PIN, MOSI_PIN);
  hal.init();
  // IRQ is DIO
  // GPIO is BUSY
  // modify the source code to use DIO2 and DIO3. You know the deal
  // https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem
  // https://github.com/espressif/arduino-esp32/issues/3871#issuecomment-913186206
  auto module = Module(&hal, NSS_PIN, DIO1_PIN, RESET_PIN, BUSY_PIN);
  auto rf     = LLCC68(&module);
  auto st     = rf.begin();

  if (st != RADIOLIB_ERR_NONE) {
    ESP_LOGE("rf", "failed, code %d", st);
    esp_restart();
  }
  ESP_LOGI("rf", "RF init success!");
  auto chan          = std::make_shared<MessageQueue>();
  auto rf_task_param = RfTaskParam{
      .rf      = &rf,
      .channel = chan,
  };
  auto led_task_param = LedTaskParam{
      .channel = chan,
  };
  auto led_task = [](void *pvParameters) {
    auto &param = *static_cast<LedTaskParam *>(pvParameters);
    auto &chan  = *param.channel;
    uint8_t src[3] = {0x01, 0x02, 0x03};
    uint8_t dst[3] = {0xff, 0xff, 0xff};
    uint8_t pkt_id = 0;
    auto d         = std::chrono::milliseconds(500);
    auto rng       = etl::random_xorshift();
    rng.initialise(0);
    uint8_t rgb;
    auto encoder = MessageWrapper::Encoder(src, dst, pkt_id);
    while (true) {
      bool random_r;
      bool random_g;
      bool random_b;
      uint8_t temp;
      do {
        random_r = rng.range(0, 1);
        random_g = rng.range(0, 1);
        random_b = rng.range(0, 1);
        temp     = random_b | (random_g << 1) | (random_r << 2);
      }
      // refresh until at least one color is on and the color is different from the previous one
      while (!(temp != 0) || !(temp != rgb));
      rgb                 = temp;
      auto b              = boring::Boring();
      b.led               = rgb;
      const char *payload = "test";
      for (auto i = 0; i < strlen(payload); i++) {
        b.comments.emplace_back(payload[i]);
      }
      uint8_t buf[256];

      auto sz = boring::toBytes(b, buf);
      encoder.setPayload(buf, sz);
      auto maybe = encoder.next();
      while (maybe.has_value()) {
        auto &v = maybe.value();
        // construct a new message vector and send it to the channel
        chan << v;
        maybe = encoder.next();
      }
      ESP_LOGI("rgb", "Boring rgb=%d", rgb);
      pkt_id++;
      encoder.reset(src, dst, pkt_id);
      vTaskDelay(d.count() / portTICK_PERIOD_MS);
    }
  };

  auto rf_task = [](void *pvParameters) {
    // Channel Activity Detection
    // https://github.com/jgromes/RadioLib/blob/bea5e70d0ad4f6df74a4eb2a3d1bdb683014d6c1/examples/SX126x/SX126x_Channel_Activity_Detection_Interrupt/SX126x_Channel_Activity_Detection_Interrupt.ino#L4
    // https://github.com/jgromes/RadioLib/blob/bea5e70d0ad4f6df74a4eb2a3d1bdb683014d6c1/examples/SX128x/SX128x_Channel_Activity_Detection_Blocking/SX128x_Channel_Activity_Detection_Blocking.ino#L54
    // basic do an IRQ status check
    auto &rf_param = *static_cast<RfTaskParam *>(pvParameters);
    auto &rf       = *rf_param.rf;
    auto &chan     = *rf_param.channel;
    for (auto v : chan) {
      auto pv  = v.data();
      auto pcv = reinterpret_cast<uint8_t *>(pv);
      // NOTE: `char *` and `uint8_t *` have different semantics.
      // `char *` implies a null-terminated string, while `uint8_t *` is just a pointer to a byte array.
      // at least for RadioLib.
      auto r = rf.transmit(pcv, v.size());
      if (r != RADIOLIB_ERR_NONE) {
        ESP_LOGE("rf", "failed to transmit, code %d", r);
      }
      r = rf.startReceive();
      if (r != RADIOLIB_ERR_NONE) {
        ESP_LOGE("rf", "failed to start receive, code %d", r);
      }
    }
  };

  xTaskCreate(rf_task, "rf_task", 4096, &rf_task_param, 1, nullptr);
  xTaskCreate(led_task, "led_task", 4096, &led_task_param, 1, nullptr);
  // keep `app_main()` alive forever to prevent the stack variables from being destroyed
  while (true) {
    vTaskDelay(1);
  }
}
