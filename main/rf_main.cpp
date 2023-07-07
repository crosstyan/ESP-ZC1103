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
#include <utility>
#include "utils.h"
#include <NimBLEDevice.h>
#include <rtc_wdt.h>
#include "message.h"
#include "pkt_id.h"

// https://stackoverflow.com/questions/64017982/c-equivalent-of-rust-enums
// https://thatonegamedev.com/cpp/rust-enums-in-modern-cpp/

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

static auto const BLE_NAME             = "TRACK_HUB";
static auto const BLE_SERVER_UUID      = "1f6cc873-c9d4-49c0-845c-02dbd21e4fe3";
static auto const BLE_CONFIG_CHAR_UUID = "77de9e2b-66e1-42e8-9b1f-5dae1c5f2737";
static uint8_t const HUB_SRC_ADDR[3]   = {0x00, 0x00, 0x01};

/// global flag to indicate packet reception
static bool RX_FLAG = false;

using MessageVector = etl::vector<uint8_t, MessageWrapper::MAX_ENCODER_OUTPUT_SIZE>;
using RfMessageChan = msd::channel<MessageVector>;

// TODO: a mutex for RF
struct RfTaskParam {
  LLCC68 *rf;
  std::shared_ptr<RfMessageChan> channel;
};

struct LedTaskParam {
  std::shared_ptr<RfMessageChan> channel;
};

class ConfigCharCallback : public BLECharacteristicCallbacks {
private:
  std::shared_ptr<RfMessageChan> channel_;
  MessageWrapper::Decoder decoder_;

public:
  // please note that it's the `simple wrapper` instead of `message wrapper` which has `src`, `dst` and `pkt_id` fields
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override {
    auto val  = characteristic->getValue();
    auto data = val.data();
    auto sz   = val.size();
    auto res  = decoder_.decode(data, sz, true);
    if (res == MessageWrapper::WrapperDecodeResult::Finished) {
      auto &payload      = decoder_.getOutput();
      auto maybe_ble_res = RfMessage::decodeBleMsg(payload.data(), payload.size());
      if (maybe_ble_res.has_value()) {
        auto &ble_res = maybe_ble_res.value();
        auto dst      = etl::array<uint8_t, 3>{};
        if (etl::holds_alternative<RfMessage::AddrArray>(ble_res.dest)) {
          dst = etl::get<RfMessage::AddrArray>(ble_res.dest);
        } else if (etl::holds_alternative<uint32_t>(ble_res.dest)) {
          dst = etl::array<uint8_t, 3>{0xff, 0xff, 0xff};
        } else {
          ESP_LOGE("ble", "bad BLE message");
          return;
        }
        auto &chan       = *channel_;
        auto encoder     = MessageWrapper::Encoder(HUB_SRC_ADDR, dst.data(), PacketId::next());
        auto msg_payload = RfMessage::toBytes(ble_res.msg);
        // don't have to say but the `msg_payload` should live longer than the `encoder` until the `encoder` is done
        encoder.setPayload(msg_payload.data(), msg_payload.size());
        auto r = encoder.next();
        while (r.has_value()) {
          chan << r.value();
          r = encoder.next();
        }
        decoder_.reset();
      } else {
        ESP_LOGE("ble", "bad BLE message");
      }
    } else if (res == MessageWrapper::WrapperDecodeResult::Unfinished) {
      return;
    } else {
      auto reason = MessageWrapper::decodeResultToString(res);
      ESP_LOGE("ble", "failed to decode BLE message: %s", reason);
    }
  };
  explicit ConfigCharCallback(std::shared_ptr<RfMessageChan> channel) : channel_(std::move(channel)) {
    decoder_ = MessageWrapper::Decoder();
  };
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

  BLEDevice::init(BLE_NAME);
  auto &server      = *BLEDevice::createServer();
  auto &service     = *server.createService(BLE_SERVER_UUID);
  auto &advertising = *BLEDevice::getAdvertising();
  // https://github.com/h2zero/esp-nimble-cpp/blob/1786d0ede39d33369ce8c21ff3b5de45b17594f2/examples/basic/BLE_server/main/main.cpp#L32-L41
  auto &config_char = *service.createCharacteristic(
      BLE_CONFIG_CHAR_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  auto chan = std::make_shared<RfMessageChan>();
  config_char.setCallbacks(new ConfigCharCallback(chan));
  advertising.setName(BLE_NAME);
  advertising.setScanResponse(false);
  service.start();
  server.start();

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
      rgb                = temp;
      auto b             = RfMessage::Boring();
      b.led              = rgb;
      const auto payload = "test";
      for (auto i = 0; i < strlen(payload); i++) {
        b.comments.emplace_back(payload[i]);
      }
      uint8_t buf[256];

      auto sz = RfMessage::toBytes(b, buf);
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
    auto decoder   = MessageWrapper::Decoder();
    uint8_t buf[256];
    while (true) {
      if (RX_FLAG) {
        auto sz = rf.getPacketLength(true);
        auto r  = rf.readData(buf, sz);
        RX_FLAG = false;
        if (r != RADIOLIB_ERR_NONE) {
          ESP_LOGE("rf", "failed to read data, code %d", r);
        }
        auto res = decoder.decode(buf, sz, false);
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
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
