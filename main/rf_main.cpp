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

/// global flag to indicate packet reception
static bool RX_FLAG = false;

using MessageVector = etl::vector<uint8_t, MessageWrapper::MAX_ENCODER_OUTPUT_SIZE>;
using MessageQueue  = msd::channel<MessageVector>;

// TODO: a mutex for RF
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

  rf.setPacketReceivedAction([]() {
    RX_FLAG = true;
  });

  ESP_LOGI("rf", "RF init success!");
  auto chan               = std::make_shared<MessageQueue>();
  auto rf_send_task_param = RfTaskParam{
      .rf      = &rf,
      .channel = chan,
  };
  auto led_task_param = LedTaskParam{
      .channel = chan,
  };
  auto led_task = [](void *pvParameters) {
    auto &param    = *static_cast<LedTaskParam *>(pvParameters);
    auto &chan     = *param.channel;
    uint8_t src[3] = {0x01, 0x02, 0x03};
    uint8_t dst[3] = {0xff, 0xff, 0xff};
    uint8_t pkt_id = 0;
    auto d         = std::chrono::milliseconds(5000);
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

  auto rf_send_task = [](void *pvParameters) {
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

  auto rf_recv_task = [](void *pvParameters) {
    auto &rf_param = *static_cast<RfTaskParam *>(pvParameters);
    auto &rf       = *rf_param.rf;
    uint8_t buf[256];
    while (true) {
      if (RX_FLAG) {
        auto sz = rf.getPacketLength(true);
        auto r  = rf.readData(buf, sz);
        RX_FLAG = false;
        if (r != RADIOLIB_ERR_NONE) {
          ESP_LOGE("rf", "failed to read data, code %d", r);
        }
        auto decoder = MessageWrapper::Decoder();
        auto res     = decoder.decode(buf, sz);
        if (res == MessageWrapper::WrapperDecodeResult::Finished) {
          auto &msg = decoder.getOutput();
          printf("[INFO] received message: ");
          utils::printWithSize(msg, true);
          printf("\n");
        } else if (res == MessageWrapper::WrapperDecodeResult::Unfinished) {
          continue;
        } else {
          auto err_str = MessageWrapper::decodeResultToString(res);
          ESP_LOGE("rf", "failed to decode message, code %s", err_str);
        }
      } else {
        vTaskDelay(10);
      }
    }
  };

  xTaskCreate(rf_recv_task, "rf_recv_task", 4096, &rf_send_task_param, 1, nullptr);
  xTaskCreate(rf_send_task, "rf_send_task", 4096, &rf_send_task_param, 2, nullptr);
  xTaskCreate(led_task, "led_task", 4096, &led_task_param, 1, nullptr);
  // keep `app_main()` alive forever to prevent the stack variables from being destroyed
  while (true) {
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}
