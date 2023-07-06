//
// Created by Kurosu Chan on 2023/7/5.
//

#ifndef TRACKRF_UTILS_H
#define TRACKRF_UTILS_H

//
// Created by Kurosu Chan on 2023/5/22.
//
#include "etl/string.h"

namespace utils {
/// won't add trailing `LF` or `CRLF` and caller should decide whether to add one.
void printWithSize(const uint8_t *bytes, size_t size, bool hex = false) {
  for (size_t i = 0; i < size; i++) {
    if (hex) {
      // https://stackoverflow.com/questions/61518810/print-the-value-of-a-pointer-in-hex-format-without-printf
      uint8_t hi     = (bytes[i] >> 4) & 0xf;
      uint8_t lo     = bytes[i] & 0xf;
      uint8_t tmp[2] = {hi, lo};

      tmp[0] += hi < 10 ? '0' : 'a' - 10;
      tmp[1] += lo < 10 ? '0' : 'a' - 10;
      putchar(tmp[0]);
      putchar(tmp[1]);
    } else {
      putchar(bytes[i]);
    }
  }
};

size_t sprintHexWithSize(char *out, const uint8_t *bytes, size_t size) {
  size_t i = 0;
  while (i < (size * 2)) {
    // https://stackoverflow.com/questions/61518810/print-the-value-of-a-pointer-in-hex-format-without-printf
    uint8_t hi     = (bytes[i] >> 4) & 0xf;
    uint8_t lo     = bytes[i] & 0xf;
    uint8_t tmp[2] = {hi, lo};

    tmp[0] += hi < 10 ? '0' : 'a' - 10;
    tmp[1] += lo < 10 ? '0' : 'a' - 10;
    out[i]     = tmp[0];
    out[i + 1] = tmp[1];
    i += 2;
  }
  out[i] = '\0';
  return i;
};

void printWithSize(const etl::ivector<uint8_t> &vec, bool hex = false) {
  utils::printWithSize(vec.data(), vec.size(), hex);
};

void printWithSize(const etl::ivector<char> &vec, bool hex = false) {
  utils::printWithSize(reinterpret_cast<const uint8_t *>(vec.data()), vec.size(), hex);
};
}

#endif // TRACKRF_UTILS_H
