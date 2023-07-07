//
// Created by Kurosu Chan on 2023/7/7.
//

#ifndef TRACKRF_PKT_ID_H
#define TRACKRF_PKT_ID_H

#include <stdint.h>

namespace PacketId {
static uint8_t id = 0;
/**
 * @brief Get the next packet id (increase the counter by 1)
 */
uint8_t next() {
  return id++;
};
/**
 * @brief Get the current packet id (won't touch the counter)
 */
uint8_t current() {
  return id;
};
}

#endif // TRACKRF_PKT_ID_H
