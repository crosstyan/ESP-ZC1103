//
// Created by Kurosu Chan on 2023/5/29.
//
#include "message_wrapper.h"

MessageWrapper::Encoder::Encoder(const uint8_t *src, const uint8_t *dst, uint8_t pkt_id) {
  reset(src, dst, pkt_id);
}

void MessageWrapper::Encoder::reset(const uint8_t *src, const uint8_t *dst, uint8_t pkt_id) {
  output.clear();
  header = {
      .src                = {0},
      .dst                = {0},
      .pkt_id             = 0,
      .pkt_cur_count      = 0,
      .total_payload_size = 0,
      .cur_payload_size   = 0,
  };
  memcpy(header.src, src, 3);
  memcpy(header.dst, dst, 3);
  header.pkt_id = pkt_id;
}

inline void static push_back_many(etl::ivector<uint8_t> &vec, const char *data, size_t size) {
  for (size_t i = 0; i < size; i++) {
    vec.push_back(data[i]);
  }
}

inline void static push_back_many(etl::ivector<uint8_t> &vec, const uint8_t *data, size_t size) {
  auto d = reinterpret_cast<const uint8_t *>(data);
  for (size_t i = 0; i < size; i++) {
    vec.push_back(d[i]);
  }
}

inline void static add_padding(etl::ivector<uint8_t> &vec, size_t size) {
  for (size_t i = 0; i < size; i++) {
    vec.push_back(0);
  }
}

void MessageWrapper::Encoder::setPayload(const uint8_t *payload, size_t size) {
  this->message            = const_cast<uint8_t *>(payload);
  this->total_message_size = size;
  this->cur_left           = size;
}

void MessageWrapper::Encoder::setPayload(const char *payload, size_t size) {
  setPayload(reinterpret_cast<const uint8_t *>(payload), size);
}

// TODO: optimize this
// eats 256 bytes of Flash
etl::optional<etl::vector<uint8_t, MessageWrapper::MAX_ENCODER_OUTPUT_SIZE>> MessageWrapper::Encoder::next() {
  if (this->message == nullptr) {
    return etl::nullopt;
  }
  output.clear();
  push_back_many(output, header.src, 3);
  push_back_many(output, header.dst, 3);
  output.push_back(header.pkt_id);
  output.push_back(header.pkt_cur_count);
  header.total_payload_size       = total_message_size;
  auto network_total_payload_size = __htons(header.total_payload_size);
  for (auto i = 0; i < 2; i++) {
    output.push_back(*(reinterpret_cast<uint8_t *>(&network_total_payload_size) + i));
  }
  auto current_payload_size = std::min(cur_left, MAX_PAYLOAD_SIZE);
  header.cur_payload_size   = current_payload_size;
  output.push_back(header.cur_payload_size);
  push_back_many(output, message, header.cur_payload_size);
  add_padding(output, ENDING_PAD_SIZE);
  cur_left -= current_payload_size;
  if (cur_left < 0) [[unlikely]] {
    this->message            = nullptr;
    this->total_message_size = 0;
    this->cur_left           = 0;
    return etl::nullopt;
    // last packet
  } else if (cur_left == 0) {
    this->message            = nullptr;
    this->total_message_size = 0;
    this->cur_left           = 0;
    return etl::make_optional(output);
  } else {
    message += current_payload_size;
    header.pkt_cur_count++;
    return etl::make_optional(output);
  }
}

// TODO: optimize this
// eats 132 bytes of Flash
etl::optional<MessageWrapper::WrapperHeader> MessageWrapper::Decoder::decodeHeader(const uint8_t *message, size_t size) {
  if (size < HEADER_SIZE) {
    return etl::nullopt;
  }
  WrapperHeader header{};
  memcpy(header.src, message, 3);
  memcpy(header.dst, message + 3, 3);
  header.pkt_id        = message[6];
  header.pkt_cur_count = message[7];
  // decode 8 & 9
  auto host_total_payload_size = __ntohs(*reinterpret_cast<const uint16_t *>(message + 8));
  header.total_payload_size    = host_total_payload_size;
  header.cur_payload_size      = message[10];
  return etl::make_optional(header);
}

MessageWrapper::WrapperDecodeResult MessageWrapper::Decoder::decode(const uint8_t *message, size_t size) {
  auto h = decodeHeader(message, size);
  if (!h.has_value()) {
    return WrapperDecodeResult::BadHeader;
  }
  // first packet
  if (!_decoding) {
    this->header = h.value();
    _decoding    = true;
    if (header.total_payload_size > MAX_DECODER_OUTPUT_SIZE) {
      return WrapperDecodeResult::TotalPayloadSizeTooLarge;
    }
    output.clear();
    push_back_many(output, message + HEADER_SIZE, header.cur_payload_size);
    if (header.total_payload_size == header.cur_payload_size) {
      _decoding = false;
      return WrapperDecodeResult::Finished;
    } else {
      return WrapperDecodeResult::Unfinished;
    }
    // following packets
  } else {
    if (header.pkt_id != h.value().pkt_id) {
      return WrapperDecodeResult::UnexpectedPktId;
    }
    if (header.total_payload_size != h.value().total_payload_size) {
      return WrapperDecodeResult::UnexpectedTotalPayloadSize;
    }
    if ((header.pkt_cur_count + 1) != h.value().pkt_cur_count) {
      return WrapperDecodeResult::UnexpectedPktCount;
    }
    if (memcmp(header.src, h.value().src, 3) != 0) {
      return WrapperDecodeResult::UnexpectedSrc;
    }
    push_back_many(output, message + HEADER_SIZE, header.cur_payload_size);
    if (output.size() >= header.total_payload_size) {
      _decoding = false;
      return WrapperDecodeResult::Finished;
    } else {
      return WrapperDecodeResult::Unfinished;
    }
  }
}

void MessageWrapper::Decoder::reset() {
  header = {};
  output.clear();
  _decoding = false;
}

const etl::vector<uint8_t, MessageWrapper::MAX_DECODER_OUTPUT_SIZE> &MessageWrapper::Decoder::getOutput() const {
  return output;
}

void MessageWrapper::Decoder::printHeader(const MessageWrapper::WrapperHeader &header) {
  printf("src=\"%02x:%02x:%02x\"; ", header.src[0], header.src[1], header.src[2]);
  printf("dst=\"%02x:%02x:%02x\"; ", header.dst[0], header.dst[1], header.dst[2]);
  printf("pkt_id=%d; ", header.pkt_id);
  printf("pkt_cur_count=%d; ", header.pkt_cur_count);
  printf("total_payload_size=%d; ", header.total_payload_size);
  printf("cur_payload_size=%d; ", header.cur_payload_size);
  printf("\n");
}

const char *MessageWrapper::decodeResultToString(MessageWrapper::WrapperDecodeResult result) {
  switch (result) {
    case WrapperDecodeResult::BadHeader:
      return "BadHeader";
    case WrapperDecodeResult::Finished:
      return "Finished";
    case WrapperDecodeResult::Unfinished:
      return "Unfinished";
    case WrapperDecodeResult::UnexpectedPktId:
      return "UnexpectedPktId";
    case WrapperDecodeResult::UnexpectedTotalPayloadSize:
      return "UnexpectedTotalPayloadSize";
    case WrapperDecodeResult::UnexpectedPktCount:
      return "UnexpectedPktCount";
    case WrapperDecodeResult::UnexpectedSrc:
      return "UnexpectedSrc";
    case WrapperDecodeResult::TotalPayloadSizeTooLarge:
      return "TotalPayloadSizeTooLarge";
    default:
      return "Unknown";
  }
};

uint16_t MessageWrapper::getUniquePktId(const WrapperHeader &header) {
  return (header.pkt_id << 8) | header.pkt_cur_count;
}
