//
// Created by Kurosu Chan on 2023/7/3.
//

#ifndef SIMPLE_BORING_H
#define SIMPLE_BORING_H

#include <stdint.h>
#include <etl/vector.h>
#include <etl/optional.h>

namespace boring {
const uint8_t BORING_MAGIC            = 0x50;
const uint8_t MAX_BORING_COMMENT_SIZE = 32;
struct Boring {
  uint8_t led;
  etl::vector<uint8_t, MAX_BORING_COMMENT_SIZE> comments;
  static size_t sizeNeeded(const Boring &boring) {
    auto num_comments = static_cast<uint8_t>(boring.comments.size());
    return sizeof(boring.led) + sizeof(num_comments) + num_comments * sizeof(uint8_t);
  };
};

size_t toBytes(Boring &boring, uint8_t *buffer) {
  size_t offset     = 0;
  auto num_comments = static_cast<uint8_t>(boring.comments.size());
  buffer[offset]    = BORING_MAGIC;
  offset += sizeof(BORING_MAGIC);
  buffer[offset] = boring.led;
  offset += sizeof(boring.led);
  buffer[offset] = num_comments;
  offset += sizeof(num_comments);
  for (auto b : boring.comments) {
    buffer[offset] = b;
    offset += sizeof(b);
  }
  return offset;
}
etl::optional<Boring> fromBytes(const uint8_t *buffer) {
  if (buffer[0] != BORING_MAGIC) {
    return etl::nullopt;
  }
  Boring boring;
  size_t offset = sizeof(BORING_MAGIC);
  boring.led    = buffer[offset];
  offset += sizeof(boring.led);
  auto num_comments = buffer[offset];
  boring.comments.reserve(num_comments);
  offset += sizeof(num_comments);
  for (size_t i = 0; i < num_comments; i++) {
    boring.comments.push_back(buffer[offset]);
    offset += sizeof(uint8_t);
  }
  return etl::make_optional(boring);
}

}

#endif // SIMPLE_BORING_H
