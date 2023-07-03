//
// Created by Kurosu Chan on 2023/5/29.
//

#ifndef SIMPLE_MESSAGE_WRAPPER_H
#define SIMPLE_MESSAGE_WRAPPER_H

#include <etl/optional.h>
#include <etl/variant.h>
#include <etl/vector.h>

/// TODO: unit test
namespace MessageWrapper {
  // total size = 3 + 3 + 1 + 1 + 2 + 1 = 11
  // use __attribute__((packed)) to avoid padding
  struct WrapperHeader {
    uint8_t src[3];
    uint8_t dst[3];
    uint8_t pkt_id;
    /// inference by encoder
    uint8_t pkt_cur_count;
    /// inference by encoder
    uint16_t total_payload_size;
    /// inference by encoder
    uint8_t cur_payload_size;
  } __attribute__((packed));
  // pad is needed since the transmission would corrupt the last few bytes (why????)
  constexpr size_t ENDING_PAD_SIZE = 3;
  constexpr size_t HEADER_SIZE = sizeof(WrapperHeader);
  constexpr size_t MAX_ENCODER_OUTPUT_SIZE = 254;
  constexpr size_t MAX_PAYLOAD_SIZE = MAX_ENCODER_OUTPUT_SIZE - HEADER_SIZE - ENDING_PAD_SIZE;
  constexpr size_t MAX_DECODER_OUTPUT_SIZE = 512;
  enum class WrapperDecodeResult {
    Finished,
    Unfinished,
    BadHeader,
    UnexpectedSrc,
    UnexpectedPktId,
    UnexpectedPktCount,
    UnexpectedTotalPayloadSize,
    TotalPayloadSizeTooLarge,
  };

  const char *decodeResultToString(WrapperDecodeResult result);

  /// the unique packet id is the packet id + packet current count
  uint16_t getUniquePktId(const WrapperHeader &header);

  class Encoder {
    etl::vector<char, MAX_ENCODER_OUTPUT_SIZE> output;
    WrapperHeader header{};
    char *message = nullptr;
    size_t total_message_size = 0;
    size_t cur_left = 0;
  public:
    void setPayload(const char *payload, size_t size);
    void setPayload(const uint8_t *payload, size_t size);

    /*
     * @brief iterator like. return `etl::nullopt` when finished, otherwise return the next encoded packet.
     */
    [[nodiscard]]
    etl::optional<etl::vector<char, MAX_ENCODER_OUTPUT_SIZE>> next();

    /*
     * @param src 3 bytes
     * @param dst 3 bytes
     * @param pkt_id 1 byte
     */
    Encoder(const uint8_t *src, const uint8_t *dst, uint8_t pkt_id);

    void reset(const uint8_t *src, const uint8_t *dst, uint8_t pkt_id);
  };

  class Decoder {
    /// if decoding then header from first packet is determined
    bool _decoding = false;
    etl::vector<char, MAX_DECODER_OUTPUT_SIZE> output;
    WrapperHeader header{};
  public:
    /// pure function
    static etl::optional<WrapperHeader> decodeHeader(const char *message, size_t size);

    static void printHeader(const WrapperHeader &header);

    /*
     * @brief when `WrapperDecodeResult::Finished` is returned then use `getOutput()` to retrieve the output
     */
    WrapperDecodeResult decode(const char *message, size_t size);

    [[nodiscard]]
    const etl::vector<char, MAX_DECODER_OUTPUT_SIZE> &getOutput() const;

    void reset();
  };
}


#endif //SIMPLE_MESSAGE_WRAPPER_H
