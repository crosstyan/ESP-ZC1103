#include "rfsystem.h"

// TODO: figure out how RSSI in this chip works.
// otherwise we can't use auto detect channel before transmission.
uint8_t const RSSI_THRESHOLD = 100;

volatile bool rx_flag = false;

// for C linkage
static auto PRIV_CS_PIN = DEFAULT_CS_PIN;

void RF::setRxFlag(bool flag) { rx_flag = flag; }

bool RF::rxFlag() { return rx_flag; }

/// a utility function to check if a bit is set at shift.
/// note that shift is 0-indexed.
inline static bool shift_equal(uint8_t byte, uint8_t shift) {
  return (byte & (1 << shift)) == (1 << shift);
}

inline static void PRIV_CS_HIGH(spi_transaction_t *t) {
  gpio_set_level(PRIV_CS_PIN, 1);
}

inline static void PRIV_CS_LOW(spi_transaction_t *t) {
  gpio_set_level(PRIV_CS_PIN, 0);
}

inline void RfSystem::RST_LOW() {
  gpio_set_level(this->RST_PIN, 0);
}

inline void RfSystem::RST_HIGH() {
  gpio_set_level(this->RST_PIN, 1);
}

inline void RfSystem::SDN_LOW() {
  gpio_set_level(this->SDN_PIN, 0);
}

inline void RfSystem::SDN_HIGH() {
  gpio_set_level(this->SDN_PIN, 1);
}

inline void RfSystem::CS_HIGH() {
  PRIV_CS_HIGH(nullptr);
}

inline void RfSystem::CS_LOW() {
  PRIV_CS_LOW(nullptr);
}
inline void RfSystem::DelayMs(uint32_t ms) {
  vTaskDelay(ms / portTICK_PERIOD_MS);
}

inline void RfSystem::gpioConfigure() {
  auto io_conf = gpio_config_t{
      .pin_bit_mask = (1ULL << this->RST_PIN) | (1ULL << this->SDN_PIN) |
                      (1ULL << this->CS_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);
  // PKT_FLAG_PIN is configured elsewhere (checkout main.cpp)
}

/// 芯片的所有控制都是通 SPI 接口操作，支持的模式是时钟极性为正，相位极性可选，
/// 当 ckpha=1 时，为下降沿采样，ckpha=0 时，上升沿采样。
///
/// See also `src/inc/spi.h`
inline void RfSystem::spiConfigure() {
  // https://www.cnblogs.com/redlightASl/p/15542572.html
  auto bus_cfg = spi_bus_config_t{
      .mosi_io_num = GPIO_NUM_13,
      .miso_io_num = GPIO_NUM_12,
      .sclk_io_num = GPIO_NUM_14,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
      .flags = 0,
      .intr_flags = 0,
  };
  spi_bus_initialize(HSPI_HOST, &bus_cfg, 0);
  auto device_cfg = spi_device_interface_config_t{
      .command_bits = 0,
      .address_bits = 8,
      .dummy_bits = 0,
      // (CPOL, CPHA )= (0, 1)
      .mode = 1,
      .duty_cycle_pos = 0,
      .cs_ena_pretrans = 0,
      .cs_ena_posttrans = 0,
      .clock_speed_hz = 1000000,
      .spics_io_num = CS_PIN,
      .flags = 0,
      .queue_size = 1,
      // low enable
      .pre_cb = PRIV_CS_LOW,
      .post_cb = PRIV_CS_HIGH,
  };
  spi_bus_add_device(HSPI_HOST, &device_cfg, &_spi_handle);
}

void RfSystem::reset() {
  RST_LOW();
  DelayMs(100);
  RST_HIGH();
  SDN_LOW();
}

/// On this particular device, you transmit a 7-bit register address
/// (with the high-bit set to 1=read, 0=write),
/// then either read or write the value of the register from there.
esp_err_t RfSystem::write(const uint8_t addr, const uint8_t val) {
  // don't use cmd bit...use tx_buffer as cmd instead.
  auto trans_desc = spi_transaction_t{
      .flags = 0,
      .cmd = 0,
      .addr = static_cast<uint8_t>(addr & 0x7f),
      .length = 8,
      .rxlength = 0,
      .tx_buffer = &val,
  };
  return spi_device_transmit(_spi_handle, &trans_desc);
}

etl::expected<uint8_t, esp_err_t> RfSystem::read(uint8_t addr) {
  // 0x80 = 0b1000_0000 i.e. set the high bit to 1 to indicate read
  uint8_t val = 0;
  auto trans_desc = spi_transaction_t{
      .flags = 0,
      .cmd = 0,
      .addr = static_cast<uint8_t>(addr & 0x80),
      .length = 0,
      .rxlength = 8,
      .rx_buffer = &val,
  };
  // This function is the equivalent of calling spi_device_queue_trans()
  // followed by spi_device_get_trans_result().
  auto e = spi_device_transmit(_spi_handle, &trans_desc);
  if (e != ESP_OK) {
    return etl::expected<uint8_t, esp_err_t>(etl::unexpected<esp_err_t>(e));
  }
  return val;
}

void RfSystem::registerConfigure() {
  // r(0x0c) channel detect
  // 0x03 = 0b0000_0011
  write(0x09, 0x08);
  write(0x0c, 0x03);
  /* r(0x0e)
   0xa1 = 0b10100001
   7        1        preamble 中断使能，当接收端收到有效 的前导码 preamble 时产生中断信号
   6         0       sync word 中断使能，当接收端收到有效的 同步字 sync word 时产生中断信号
   5          1      该位选择 pkt_flag 有效 后，是否自动拉低。如果自动拉低, 脉冲宽度大约 1us
   [4:0]       00001 allowed error bit
  */
  // write(0x0e, 0xa1);
  write(0x0e, 0b00100001);
  /*
   * r(0x0a)
   * 0x1f = 0b00011111
   * 7        0          SRST_EN
   * 6         0         AUTO_ACK_EN
   * [5:0]      011111   AUTO_ACK_RX_TIME 每个步进表示增加 128bit (16 bytes) 数据的时间
   *                     (decided by the data rate?)
   */
  // enable auto acknowledge
  write(0x0a, 0b01010000);
  /*
   * r(0x0b)
   * 0x03 = 0b00000011
   * [7:4]    0000      RC32K_CAL_OFFSET
   * [3:0]        0011  RE_TX_TIMES
   */
  write(0x0b, 0x05);
  /*
   * r(0x0c)
   * 0x03 = 0b00000011
   * 7        0          TX_DATA_INVERSE 发送数据取反
   * 6         0         读取中频信号幅度值
   * 5          0        AUTO_DET_TX_CHL 在发送数据前先检测信道是否忙
   * 4           0       AUTO_DET_CHL_MODE
   *                     0: exit directly when channel is busy
   *                     1: wait until channel is idle
   * [3:0]        0011   AUTO_DET_WAIT_TIME
   *                     1bit = 256us
   */
  // disable this for now... can't figure out RSSI_THRESHOLD
  write(0x0c, 0b00010011);
  //  信道检测忙参考值，RSSI 绝对值低于该寄
  //  存器值时表示信道忙。最低 1bit 表示小数
  write(0x0d, RSSI_THRESHOLD << 1);
  /*
   r(0x0f)
   0x0a = 0b00001010
               00000 发送 FIFO 空门限，发送 FIFO
                     还剩余字节数低于门限值时会产生 fifo_flag 标志
  */
  write(0x0F, 0x0A);
  write(0x10, 0x54);
  /*
   r(0x1b) WOR
   0x25 = 0b00100101
   7        0          Reserved
   6         0         自动唤醒后执行的命令 (0: RX, 1: TX)
   5          1        内部低频 RC 振荡时钟校准使能
   4            0       WOR (Wake On Radio) 使能
   [3:0]         0101   WOR 功能计数器时钟选择 (32KHz/2^n)
                       0b0101 = 5 = 32KHz/2^5 = 1KHz
                       i.e. 1ms per tick
  */
  write(0x1b, 0b00100101);

  write(0x20, 0xa4);
  write(0x21, 0x37);
  write(0x22, 0x3a);
  write(0x23, 0x36);
  write(0x2F, 0xe0);
  write(0x2E, 0x00);

  write(0x30, 0x00);
  write(0x31, 0x00);
  write(0x32, 0x00);
  write(0x33, 0x00);
  write(0x34, 0x00);
  write(0x35, 0x00);
  write(0x36, 0x00);

  /* r(0x39)
   0x74 = 0b0111_0100
   7        0          Preamble Threshold
   6         1         该功能使能时，接收端找到谱线后一定周期内没有没有收到同步字，则进行复位
   5          1        使能在 100K 以上数据率时自动识别信号 到达时先进行软复位
   4           1       找到谱线后一定周期内没有收到有效的 preamble 则进行接收机复位
   -              -    rest trivial
  */
  write(0x39, 0x74);
  write(0x3A, 0x61);
  /*
   r(0x4a)=0x60
   r(0x4b)=0x45
   r(0x4c)=0x67
   uint32_t GPIO_SEL = 0x60 & 0b00001111 << 16 | 0x45 << 8 | 0x67 = 0b0000_0100_0101_0110_0111;
   pkg_flag_pin =  (GPIO_SEL & (0b1111 << 16)) >> 16;
   fifo_flag_pin = (GPIO_SEL & (0b1111 << 12)) >> 12;
   brclk_pin =     (GPIO_SEL & (0b1111 <<  8)) >>  8;
   test1_pin =     (GPIO_SEL & (0b1111 <<  4)) >>  4;
   test2_pin =     (GPIO_SEL & (0b1111 <<  0)) >>  0;

   so the output of pkg_flag_pin by default is
   pkt_int | preamble_in | sync_int
   see also r(0x0e)
 */
  write(0x4a, 0x60);
  write(0x4d, 0x0b);
  write(0x4e, 0x7c);
  write(0x4f, 0xc5);

  write(0x15, 0x21);
  write(0x07, 0x5d);
  write(0x18, 0x20);
  write(0x2a, 0x14);
  write(0x37, 0x99);

  /*
  Real data often contain long sequences of zeros and ones.
  In these cases, performance can be improved by whitening the data before
  transmitting, and de-whitening the data in the receiver.

   r(0x06) Packet Control
   0x3a = 0b00111010
   7        0        SYNC_WORD_LEN: 同步字长度设置
   6         0       LENGTH_SEL: 默认为数据包的第一个字节为包长度
                     (0: 1 byte,1: 2 bytes)
                     0:2bytes {r(0x11),r(0x12)}
                     1:4bytes {r(0x11), r(0x12), r(0x13), r(0x14)}
   5          1      CRC_EN
   4           1     SCRAMBLE_EN i.e. Whitening
   3            1    FIFO_SHARE_EN
   2             0   DIRECT_MODE
   1              1  PKT_LENGTH_EN
   0               0 HW_TERM_EN
  */
  write(0x06, 0x3a);
  // r(0x04) Preamble Length
  // should be same across all nodes
  write(0x04, 0x0a);
  /* r(0x05) Packet Setting
   0x30 = 0b00110000
   7        0          Reserved
   6         0         Preamble Format (0: 1010, 1: 0101)
   5          1        Sync Word Enable
   4           1       Preamble Enable
   [3:2]        00     Packet Encoding Scheme
                       (00: NRZ, 11: Interleave, else:Reserved)
   [1:0]          00   FEC (01: 1/3, 10: 2/3, else: None)
  */
  // interleave + 2/3 FEC
  write(0x05, 0x30);
  //  write(0x05, 0b00111110);
  // r(0x3b) Preamble Threshold
  write(0x3B, 0x04);
  /* r(0x3c) Demod Config
   0x03 = 0b00000011
            0         Find Spec New En
             0        RSSI Sel
              0       RSSI Clr: Write 1 to clear RSSI lock value
               -      trivial: don't modify
  */
  write(0x3e, 0x83);
  // required by r(0x3e)::FindSpecNewEn
  write(0x38, 0x56);
}

void RfSystem::setRefFreq(const double freq) {
  auto f = static_cast<size_t>(freq * pow(2.0, 24.0));

  auto reg73 = static_cast<uint8_t>(f & 0xFF);
  auto reg72 = static_cast<uint8_t>((f >> 8) & 0xFF);
  auto reg71 = static_cast<uint8_t>((f >> 16) & 0xFF);
  auto reg70 = static_cast<uint8_t>((f >> 24) & 0xFF);

  write(0x73, reg73);
  write(0x72, reg72);
  write(0x71, reg71);
  write(0x70, reg70);
}

void RfSystem::setPA(PowerAmpGain gain) {
  const uint8_t vReg25Tbl_h4[] = {0x3f, 0x38, 0x25, 0x1a, 0x0f, 0x0d, 0x0b,
                                  0x0a, 0x09, 0x08, 0x04, 0x03, 0x86, 0x82,
                                  0x01, 0x01, 0x02, 0x02, 0x00, 0x00, 0x24,
                                  0x20, 0x16, 0x14, 0x11, 0x0d, 0x0d};

  const uint8_t vReg26Tbl_h4[] = {0xb0, 0xb0, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf,
                                  0xbf, 0x9f, 0x9f, 0xbf, 0x8f, 0x80, 0xbf,
                                  0x9f, 0x84, 0x81, 0x80, 0xad, 0x88, 0x7f,
                                  0x7f, 0x7f, 0x7f, 0x7f, 0x72, 0x62};

  const uint8_t vReg25Tbl_h31[] = {0xbf, 0xff, 0x1d, 0x1c, 0x0f, 0x0e, 0x07,
                                   0x06, 0x05, 0x04, 0x03, 0x3f, 0x2f, 0x2f,
                                   0x1d, 0x17, 0x13, 0x11, 0x10, 0x10, 0x07,
                                   0x05, 0x05, 0x03, 0x03, 0x03, 0x03};

  const uint8_t vReg26Tbl_h31[] = {0x83, 0x81, 0xaf, 0x82, 0x8f, 0x85, 0x95,
                                   0xbf, 0x81, 0xab, 0x81, 0x68, 0x6b, 0x5c,
                                   0x75, 0x72, 0x7b, 0x79, 0x7a, 0x68, 0x6f,
                                   0x7c, 0x6a, 0x7f, 0x6e, 0x62, 0x58};
  auto v = version();
  auto idx = static_cast<size_t>(gain);
  if (v == 0x04) {
    write(0x25, vReg25Tbl_h4[idx]);
    write(0x26, vReg26Tbl_h4[idx]);
  } else {
    write(0x25, vReg25Tbl_h31[idx]);
    write(0x26, vReg26Tbl_h31[idx]);
  }
}

void RfSystem::setSyncLockRssi() { write(0x3e, read(0x3e) | 0x40); }

void RfSystem::setVcoFreq(double freq) {
  auto f = static_cast<size_t>(freq * pow(2.0, 20.0));

  auto reg77 = static_cast<uint8_t>(f & 0xFF);
  auto reg76 = static_cast<uint8_t>((f >> 8) & 0xFF);
  auto reg75 = static_cast<uint8_t>((f >> 16) & 0xFF);
  auto reg74 = static_cast<uint8_t>(((f >> 24) & 0xFF) | (read(0x74) & 0xc0));

  auto temp = read(0x00);
  write(0x00, (0x80 | temp));

  write(0x77, reg77);
  write(0x76, reg76);
  write(0x75, reg75);
  write(0x74, reg74);
}

// TODO: find documentation for this
void RfSystem::setFreq(uint8_t N) {
  if (N > 0x7F) {
    return;
  }
  write(0x00, (0x80 | N));
}

void RfSystem::setFreqStep(double step) {
  auto fre = static_cast<size_t>(step * pow(2.0, 20.0));
  auto reg3 = static_cast<uint8_t>(fre & 0xFF);
  auto reg2 = static_cast<uint8_t>((fre >> 8) & 0xFF);
  auto reg1 = static_cast<uint8_t>((fre >> 16) & 0x7F);
  write(0x03, reg3);
  write(0x02, reg2);
  write(0x01, reg1);
}

inline void RfSystem::setFreq(const double f0, const uint8_t N, const double step) {
  setVcoFreq(f0);
  setFreq(N);
  setFreqStep(step);
}

inline void RfSystem::clrTxFifoWrPtr() {
  // 0x80 = 0b1000_0000
  write(0x53, 0x80);
}

// https://github.com/LSatan/SmartRC-CC1101-Driver-Lib/blob/b8c6af4c7c2214cd77a4e9b2e2cb37b24b393605/ELECHOUSE_CC1101_SRC_DRV.cpp#L1117-L1124
int RfSystem::rssi() {
  this->rx();
  auto raw = static_cast<int>(read(0x43));
  if (raw >= 128) {
    raw = (raw - 256) / 2 - 74;
  } else {
    raw = (raw / 2) - 74;
  }
  return raw;
}

esp_err_t RfSystem::writeFifo(const char *src, uint8_t len) {
  auto fifo_trans_desc = spi_transaction_t{
      .flags = 0,
      .cmd = 0,
      .addr = static_cast<uint8_t>(0x55 & 0x7f),
      .length = static_cast<size_t>(8 * len),
      .rxlength = 0,
      .tx_buffer = src,
  };
  return spi_device_transmit(_spi_handle, &fifo_trans_desc);
}

// https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-spi.h
void RfSystem::writeFifoWithSize(const char *src, uint8_t len) {
  auto trans_desc = spi_transaction_t{
      .flags = SPI_TRANS_VARIABLE_CMD,
      // TODO: how does cmd work? Have to grab the logic analyzer
      // If this won't work then the malloc should be used
      .cmd = len,
      .addr = static_cast<uint8_t>(0x55 & 0x7f),
      .length = static_cast<size_t>(8 * len),
      .tx_buffer = src,
  };
  auto ext = spi_transaction_ext_t{
      .base = trans_desc,
      .command_bits = 8,
  };
  spi_device_transmit(_spi_handle, reinterpret_cast<spi_transaction_t *>(&ext));
}

void RfSystem::readFifo(uint8_t *dst, uint8_t len) {
  auto trans_desc = spi_transaction_t{
      // TODO: how does cmd work? Have to grab the logic analyzer
      .addr = static_cast<uint8_t>(0x52 & 0x80),
      .length = 0,
      .rxlength = static_cast<size_t>(8 * len),
      .rx_buffer = dst,
  };
  spi_device_transmit(_spi_handle, &trans_desc);
}

RfStatus RfSystem::pollStatus() {
  auto s = read(0x46);
  // idle is bit 7
  auto status = RfStatus{
      .idle = shift_equal(s, 7),
      .tx = shift_equal(s, 6),
      .rx = shift_equal(s, 5),
      .fs = shift_equal(s, 4),
      .scan = shift_equal(s, 3),
      .rc_cal = shift_equal(s, 2),
      .vco_cal = shift_equal(s, 1),
      .wor = shift_equal(s, 0),
  };
  return status;
}

inline void RfSystem::idle() { write(0x60, 0xff); }

inline void RfSystem::fs() { write(0x64, 0xff); }

// WHY IDLE?
// the CC1101 driver does this too
// https://github.com/LSatan/SmartRC-CC1101-Driver-Lib/blob/b8c6af4c7c2214cd77a4e9b2e2cb37b24b393605/ELECHOUSE_CC1101_SRC_DRV.cpp#L1079-L1084
inline void RfSystem::rx() {
  idle();
  write(0x66, 0xff);
}

// WHY IDLE?
// the CC1101 driver does this too
// https://github.com/LSatan/SmartRC-CC1101-Driver-Lib/blob/b8c6af4c7c2214cd77a4e9b2e2cb37b24b393605/ELECHOUSE_CC1101_SRC_DRV.cpp#L1091-L1097
inline void RfSystem::tx() {
  idle();
  write(0x65, 0xff);
}

inline void RfSystem::sleep() {
  idle();
  write(0x67, 0xff);
}

inline void RfSystem::standBy() {
  idle();
  write(0x68, 0xff);
}

void RfSystem::txCW() {
  write(0x24, (read(0x24) | 0x80));
  write(0x06, (read(0x06) & 0xFC));
  tx();
}

etl::optional<etl::monostate> RfSystem::send(const char *buffer, const uint8_t size,
                                             bool check_tx) {
  if (size > 0) {
    // must clear tx fifo write pointer before writing to fifo
    clrTxFifoWrPtr();
    writeFifoWithSize(buffer, size);
    tx();
    // check if tx mode entered
    auto counter = 0;
    if (check_tx) {
      while (!this->pollStatus().tx) {
        if (counter > 31) {
          return etl::nullopt;
        }
        counter += 1;
      }
    }
    // don't check pkg_flag in register
    // since that flag only works for GPIO pin output
  }
  auto u = etl::monostate{};
  return etl::make_optional(u);
}

etl::optional<size_t> RfSystem::recv(char *buf) {
  // 清接收FIFO读指针
  clrRxFifoRdPtr();
  // 读取接收FIFO的寄存器映射地址，此为接收到的数据长度
  size_t len = read(0x52 | 0x80);
  if (len == 0) {
    rx();
    return etl::nullopt;
  } else {
    readFifo(reinterpret_cast<uint8_t *>(buf), len);
    rx();
    return etl::make_optional(len);
  }
}

etl::optional<size_t> RfSystem::recv(char *buf,
                                     etl::delegate<void(size_t)> resize) {
  clrRxFifoRdPtr();
  size_t len = read(0x52 | 0x80);
  if (len == 0) {
    rx();
    return etl::nullopt;
  } else {
    resize(len);
    readFifo(reinterpret_cast<uint8_t *>(buf), len);
    rx();
    return etl::make_optional(len);
  }
}

etl::optional<size_t> RfSystem::recv(etl::ivector<char> &buf) {
  clrRxFifoRdPtr();
  size_t len = read(0x52 | 0x80);
  if (len == 0) {
    rx();
    return etl::nullopt;
  } else {
    buf.resize(len);
    readFifo(reinterpret_cast<uint8_t *>(buf.data()), len);
    rx();
    return etl::make_optional(len);
  }
}

void RfSystem::begin() {
  gpioConfigure();

  CS_HIGH();

  spiConfigure();
  DelayMs(100);
  reset();
  // a delay is needed after reset
  DelayMs(30);

  registerConfigure();

  setDR(RF::DataRate::K19_2);
  setSync(0x41, 0x53);

  // 设置参考频率
  write(0x70, 0x12);
  write(0x71, 0x14);
  write(0x72, 0x7A);// 6D 48    A1  7A 18.08
  write(0x73, 0xE1);// 32 00    84  E1

  // 设置锁频
  setSyncLockRssi();

  // 设置中心频点
  setFreq(476.0, 0, 0);

  setWorTimer(5000);
  setWorRxTimer(1000);

  // 设置发射功率
  setPA(PowerAmpGain::DBM20);
  fs();
  _is_initialized = true;
}

void RfSystem::printRegisters() {
  for (auto i = 0; i <= 0x7f; i++) {
    auto res = read(i);
    if (res.has_value()) {
      printf("r(0x%02x)=0x%02x \n", i, res.value());
    }
  }
}

inline uint8_t RfSystem::version() {
  auto tmp = read(0x47);
  return tmp & 0x07;
};

bool RfSystem::setPins(gpio_num_t rst_pin, gpio_num_t cs_pin,
                       gpio_num_t flag_pin, gpio_num_t sdn_pin) {
  if (_is_initialized) {
    return false;
  }
  RST_PIN = rst_pin;
  CS_PIN = cs_pin;
  PRIV_CS_PIN = cs_pin;
  this->PKT_FLAG_PIN = flag_pin;
  SDN_PIN = sdn_pin;
  return true;
}

void RF::printStatus(const RfStatus &status) {
  printf(
      "idle=%d, tx=%d, rx=%d, fs=%d, scan=%d, rc_cal=%d, vco_cal=%d, wor=%d\n",
      status.idle, status.tx, status.rx, status.fs, status.scan, status.rc_cal,
      status.vco_cal, status.wor);
};

void RF::printState(const RfState &state) {
  printf("sync_word_rev=%d, preamble_rev=%d, crc_error=%d, pkt_flag=%d, "
         "fifo_flag=%d, rx_pkt_state=0x%02x\n",
         state.sync_word_rev, state.preamble_rev, state.crc_error,
         state.pkt_flag, state.fifo_flag, state.rx_pkt_state);
}

/// Pkt_flag 分为 4 个功能:前导匹配、同步字匹配、接收或发送包完成。
/// 在 pkt_length_en=1(payload 第 1 个字节为包长度)的情况下，pkt_flag 可设为
/// 同步字匹配或包完成状态，默认为包完成。在 pkt_length_en=0 时，
/// pkt_flag 表示前导匹配或同步字匹配。在发送状态下表示包完成。
///
/// Fifo_flag 表示 FIFO full 或 empty，在发送模块时表示 fifo empty，
/// 在接收模式时表示 fifo full。
struct RfState RfSystem::pollState() {
  auto s = read(0x40);
  uint8_t rx_pkg_st = s & 0b111;
  auto state = RfState{
      .sync_word_rev = shift_equal(s, 7),
      .preamble_rev = shift_equal(s, 6),
      .crc_error = shift_equal(s, 5),
      .pkt_flag = shift_equal(s, 4),
      .fifo_flag = shift_equal(s, 3),
      .rx_pkt_state = rx_pkg_st,
  };
  return state;
}

uint8_t RfSystem::pollTxPktSt() {
  auto s = read(0x47);
  // 0b0011_1000
  return (s & 0b00111000) >> 3;
}

void RfSystem::scanR() { write(0x63, 0xff); }

void RfSystem::wor() { write(0x6a, 0xff); }

inline void RfSystem::setSync(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4) {
  write(0x11, s1);
  write(0x12, s2);
  write(0x13, s3);
  write(0x14, s4);
}

// hope the compiler can optimize this
void RfSystem::setDR(RF::DataRate data_rate) {
  switch (data_rate) {
    case RF::DataRate::K2_4:
      write(0x08, 0x81);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xb6);
      write(0x3C, 0x91);
      write(0x3E, 0x13);
      write(0x3F, 0x20);
      write(0x58, 0x00);
      write(0x59, 0x11);
      write(0x5A, 0x00);
      write(0x5B, 0x0B);
      write(0x5C, 0x01);
      write(0x5D, 0x60);
      write(0x5e, 0x00);
      write(0x5f, 0x9F);
      write(0x78, 0xC0);
      write(0x79, 0x51);
      write(0x7a, 0xC3);
      write(0x7b, 0x5D);
      write(0x7c, 0xF8);
      write(0x7d, 0x03);
      write(0x7e, 0x04);
      write(0x7f, 0x67);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K5:
      write(0x08, 0x71);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0x91);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x00);
      write(0x59, 0x3F);
      write(0x5A, 0x00);
      write(0x5B, 0x14);
      write(0x5C, 0x01);
      write(0x5D, 0x32);
      write(0x5E, 0x00);
      write(0x5F, 0x4F);
      write(0x78, 0xC0);
      write(0x79, 0x3D);
      write(0x7A, 0x71);
      write(0x7B, 0x5D);
      write(0x7C, 0x68);
      write(0x7D, 0x01);
      write(0x7E, 0x00);
      write(0x7F, 0xE1);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K9_6:
      write(0x08, 0x01);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0x91);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x00);
      write(0x59, 0x11);
      write(0x5A, 0x00);
      write(0x5B, 0x0B);
      write(0x5C, 0x01);
      write(0x5D, 0x60);
      write(0x5E, 0x00);
      write(0x5F, 0x4F);
      write(0x78, 0xC0);
      write(0x79, 0x99);
      write(0x7A, 0x4D);
      write(0x7B, 0x5A);
      write(0x7C, 0xF8);
      write(0x7D, 0x06);
      write(0x7E, 0x02);
      write(0x7F, 0x2F);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K10:
      write(0x08, 0x01);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xC6);
      write(0x3C, 0x91);
      write(0x3E, 0x03);
      write(0x3F, 0x08);
      write(0x58, 0x00);
      write(0x59, 0x07);
      write(0x5A, 0x08);
      write(0x5B, 0x09);
      write(0x5C, 0x03);
      write(0x5D, 0x71);
      write(0x5e, 0x00);
      write(0x5f, 0xDF);
      write(0x78, 0x40);
      write(0x79, 0x66);
      write(0x7a, 0x66);
      write(0x7b, 0x5A);
      write(0x7c, 0x78);
      write(0x7d, 0x01);
      write(0x7e, 0x00);
      write(0x7f, 0x70);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K19_2:
      write(0x08, 0x51);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0xC1);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x00);
      write(0x59, 0x4F);
      write(0x5A, 0x08);
      write(0x5B, 0x05);
      write(0x5C, 0x0C);
      write(0x5D, 0x71);
      write(0x5E, 0x02);
      write(0x5F, 0x29);
      write(0x78, 0xC0);
      write(0x79, 0x99);
      write(0x7A, 0x4D);
      write(0x7B, 0x5A);
      write(0x7C, 0xF8);
      write(0x7D, 0x0C);
      write(0x7E, 0x02);
      write(0x7F, 0x29);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K100:
      write(0x08, 0x23);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0x81);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x02);
      write(0x59, 0x95);
      write(0x5A, 0x08);
      write(0x5B, 0x01);
      write(0x5C, 0x59);
      write(0x5D, 0x53);
      write(0x5E, 0x00);
      write(0x5F, 0xC9);
      write(0x78, 0x40);
      write(0x79, 0x66);
      write(0x7A, 0x66);
      write(0x7B, 0x5B);
      write(0x7C, 0x48);
      write(0x7D, 0x0A);
      write(0x7E, 0x00);
      write(0x7F, 0x67);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K200:
      write(0x08, 0x36);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0xC1);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x02);
      write(0x59, 0x95);
      write(0x5A, 0x08);
      write(0x5B, 0x01);
      write(0x5C, 0x59);
      write(0x5D, 0x53);
      write(0x5E, 0x00);
      write(0x5F, 0xC9);
      write(0x78, 0x40);
      write(0x79, 0x66);
      write(0x7A, 0x66);
      write(0x7B, 0x5B);
      write(0x7C, 0x48);
      write(0x7D, 0x0A);
      write(0x7E, 0x00);
      write(0x7F, 0x67);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K250:
      write(0x08, 0x47);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0xC1);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x02);
      write(0x59, 0x95);
      write(0x5A, 0x08);
      write(0x5B, 0x01);
      write(0x5C, 0x59);
      write(0x5D, 0x53);
      write(0x5E, 0x00);
      write(0x5F, 0xC9);
      write(0x78, 0x40);
      write(0x79, 0x66);
      write(0x7A, 0x66);
      write(0x7B, 0x5B);
      write(0x7C, 0x48);
      write(0x7D, 0x0A);
      write(0x7E, 0x00);
      write(0x7F, 0x67);
      write(0x74, 0x9D);
    default:
      // unreachable
      break;
  }
}

void RfSystem::setWorTimer(uint16_t t) {
  auto h = static_cast<uint8_t>(t >> 8);
  auto l = static_cast<uint8_t>(t & 0xff);
  write(0x1c, h);
  write(0x1d, l);
}

void RfSystem::setWorRxTimer(uint16_t t) {
  auto h = static_cast<uint8_t>(t >> 8);
  auto l = static_cast<uint8_t>(t & 0xff);
  write(0x1e, h);
  write(0x1f, l);
}

inline void RfSystem::clrRxFifoRdPtr() { write(0x51, 0x80); }

inline void RfSystem::clrRxFifoWrPtr() { write(0x50, 0x80); }

void RfSystem::clrRxFifo() {
  write(0x51, 0x80);
  write(0x50, 0x80);
}

void RfSystem::setSync(uint8_t s1, uint8_t s2) {
  write(0x11, s1);
  write(0x12, s2);
}

void RfSystem::setWorEn(bool en) {
  auto temp = read(0x1b);
  if (temp.has_value()) {
    if (en) {
      *temp |= 0b00010000;
    } else {
      *temp &= 0b11101111;
    }
    write(0x1b, temp);
  }
};
