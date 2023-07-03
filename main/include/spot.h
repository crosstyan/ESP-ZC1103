//
// Created by Kurosu Chan on 2023/6/28.
//

#ifndef SIMPLE_SPOT_H
#define SIMPLE_SPOT_H
#include "cnl_def.h"
#include "error.h"
#include <etl/unordered_map.h>
#include <etl/vector.h>
#include <etl/delegate.h>
#include <etl/expected.h>

const auto SPOT_CONFIG_MAGIC = 0x80;
const auto SPOT_MAGIC        = 0x76;

const auto MAX_SPEED_MAP_SIZE  = 16;
const auto MAX_ENABLED_ID_SIZE = 16;
const auto MAX_TRACK_SIZE      = 3;

// could choose fixed_8_8 or fixed_16_16
using speed_type = fixed_8_8;

inline static int64_t millis() {
  return (esp_timer_get_time() / 1000);
}

/**
 * @brief a function retrieve value by the number nearing the key. always move a unit up.
 *
 * @tparam T the type of value of map
 * @param keys a std::vector of int which should be sorted
 * @param m  a map whose key is int
 * @param val a value you want
 * @return T
 */
template <typename U, typename T>
T retrieveByVal(etl::ivector<U> const &keys, etl::iunordered_map<U, T> const &m, int val) {
  size_t idx = 0;
  // since keys is sorted we can get result easily
  for (auto key : keys) {
    if (val < key) {
      break;
    }
    idx += 1;
  }
  if (idx >= keys.size()) {
    idx = keys.size() - 1;
  }
  return m.at(keys[idx]);
}

enum class SpotState {
  STOP,
  START,
};

struct SpotConfig {
  fixed_16_16 circleLength;
  fixed_16_16 lineLength;
  uint16_t total;
  /// when current < 0 the device would read its current from flash
  int16_t current;
  /// in ms
  uint16_t updateInterval;
  static size_t sizeNeeded() {
    // magic, circleLength, lineLength, total, current, updateInterval
    size_t sz = 0;
    sz += sizeof SPOT_CONFIG_MAGIC;
    sz += sizeof circleLength;
    sz += sizeof lineLength;
    sz += sizeof total;
    sz += sizeof current;
    sz += sizeof updateInterval;
    return sz;
  }

  /**
   * @brief populate a SpotConfig from bytes
   * @param config an empty SpotConfig to be populated
   * @param bytes the bytes to be read
   * @param size the size of bytes
   * @see https://github.com/crosstyan/ch32v003-ZC1103/blob/cnl/docs/protocol/spot_config.ksy
   */
  static etl::expected<SpotConfig, ParseResult> fromBytes(const uint8_t *bytes) {
    auto config = SpotConfig();
    auto magic  = bytes[0];
    auto offset = 0;
    if (magic != SPOT_CONFIG_MAGIC) {
      auto ue = etl::unexpected<ParseResult>(ParseResult::MAGIC_ERROR);
      return etl::expected<SpotConfig, ParseResult>(ue);
    }
    offset += sizeof SPOT_CONFIG_MAGIC;
    auto circleLength      = __ntohl(*reinterpret_cast<const uint32_t *>(bytes + offset));
    auto fixedCircleLength = cnl::wrap<fixed_16_16>(circleLength);
    offset += sizeof circleLength;
    if (fixedCircleLength > 500) {
      auto ue = etl::unexpected<ParseResult>(ParseResult::VALUE_ERROR);
      return etl::expected<SpotConfig, ParseResult>(ue);
    }
    auto lineLength      = __ntohl(*reinterpret_cast<const uint32_t *>(bytes + offset));
    auto fixedLineLength = cnl::wrap<fixed_16_16>(lineLength);
    offset += sizeof lineLength;
    if (fixedLineLength > fixedCircleLength) {
      auto ue = etl::unexpected<ParseResult>(ParseResult::VALUE_ERROR);
      return etl::expected<SpotConfig, ParseResult>(ue);
    }
    auto total = __ntohs(*reinterpret_cast<const uint16_t *>(bytes + offset));
    offset += sizeof total;
    auto current = static_cast<int16_t>(__ntohs(*reinterpret_cast<const uint16_t *>(bytes + offset)));
    if (current > total) {
      auto ue = etl::unexpected<ParseResult>(ParseResult::VALUE_ERROR);
      return etl::expected<SpotConfig, ParseResult>(ue);
    }
    offset += sizeof current;
    auto updateInterval = __ntohs(*reinterpret_cast<const uint16_t *>(bytes + offset));
    offset += sizeof updateInterval;
    if (updateInterval < 50 || updateInterval > 1000) {
      auto ue = etl::unexpected<ParseResult>(ParseResult::VALUE_ERROR);
      return etl::expected<SpotConfig, ParseResult>(ue);
    }
    config.circleLength   = fixedCircleLength;
    config.lineLength     = fixedLineLength;
    config.total          = total;
    config.current        = current;
    config.updateInterval = updateInterval;
    return etl::expected<SpotConfig, ParseResult>(config);
  };
};

class Track {
public:
  /// only need 3 bits
  uint8_t color;
  uint8_t id;

private:
  etl::vector<uint16_t, MAX_SPEED_MAP_SIZE> keys;
  etl::unordered_map<uint16_t, speed_type, MAX_SPEED_MAP_SIZE> speeds;
  uint16_t maxKey;

public:
  explicit Track(uint8_t identifier = 0) : id(identifier) {
    color  = 0;
    maxKey = 0;
  }

  /**
   * @brief add a speed to the track
   * @param key the distance
   * @param speed the speed at the distance
   */
  void addSpeed(uint16_t key, speed_type speed) {
    keys.push_back(key);
    etl::sort(keys.begin(), keys.end());
    speeds.insert(etl::make_pair(key, speed));
    if (key > maxKey) {
      maxKey = key;
    }
  }

  [[nodiscard]] const auto &getSpeeds() const {
    return speeds;
  }

  [[nodiscard]] const auto &getKeys() const {
    return keys;
  }

  uint16_t getMaxKey() const {
    return maxKey;
  }

  speed_type getSpeed(uint16_t key) {
    return retrieveByVal<uint16_t, speed_type>(keys, speeds, key);
  }

  /**
   * @brief get the size needed to serialize the track
   * @return the size needed
   */
  [[nodiscard]] size_t sizeNeeded() const {
    // id, color, speed count, keys, speeds
    return 1 + 1 + 1 + keys.size() * 2 + keys.size() * sizeof(speed_type);
  }

  static etl::expected<Track, ParseResult> fromBytes(uint8_t *bytes) {
    size_t offset = 0;
    auto track    = Track();
    auto id       = bytes[offset];
    track.id      = id;
    offset += sizeof id;
    auto color  = bytes[offset];
    track.color = color;
    offset += sizeof color;
    auto speed_count = bytes[offset];
    if (speed_count > MAX_SPEED_MAP_SIZE) {
      auto ue = etl::unexpected<ParseResult>(ParseResult::VALUE_ERROR);
      return etl::expected<Track, ParseResult>(ue);
    }
    offset += sizeof speed_count;
    for (auto j = 0; j < speed_count; ++j) {
      auto distance = __ntohs(*reinterpret_cast<uint16_t *>(bytes + offset));
      if (distance > 6000) {
        auto ue = etl::unexpected<ParseResult>(ParseResult::VALUE_ERROR);
        return etl::expected<Track, ParseResult>(ue);
      }
      offset += sizeof distance;
      if constexpr (sizeof(speed_type) == 2) {
        auto speed       = __ntohs(*reinterpret_cast<uint16_t *>(bytes + offset));
        auto fixed_speed = cnl::wrap<speed_type>(speed);
        if (fixed_speed > 10) {
          auto ue = etl::unexpected<ParseResult>(ParseResult::VALUE_ERROR);
          return etl::expected<Track, ParseResult>(ue);
        }
        track.addSpeed(distance, fixed_speed);
      } else if constexpr (sizeof(speed_type) == 4) {
        auto speed       = __ntohl(*reinterpret_cast<uint32_t *>(bytes + offset));
        auto fixed_speed = cnl::wrap<speed_type>(speed);
        if (fixed_speed > 10) {
          auto ue = etl::unexpected<ParseResult>(ParseResult::VALUE_ERROR);
          return etl::expected<Track, ParseResult>(ue);
        }
        track.addSpeed(distance, fixed_speed);
      } else {
        static_assert(sizeof(speed_type) == 2 || sizeof(speed_type) == 4);
      }
      offset += sizeof(speed_type);
    }
    return etl::expected<Track, ParseResult>(track);
  }
};

namespace serd {
size_t toBytes(const Track &track, uint8_t *bytes) {
  auto offset = 0;
  bytes[0]    = track.id;
  offset += sizeof track.id;
  bytes[1] = track.color;
  offset += sizeof track.color;
  const auto &m = track.getSpeeds();
  auto count    = static_cast<uint8_t>(m.size());
  bytes[2]      = count;
  offset += sizeof count;
  // https://stackoverflow.com/questions/22880431/iterate-through-unordered-map-c
  for (const auto &[key, speed] : m) {
    auto k = __htons(key);
    memcpy(bytes + offset, &k, sizeof k);
    offset += sizeof k;
    if constexpr (sizeof(speed) == 2) {
      const auto sz = sizeof(speed);
      auto s        = __htons(cnl::unwrap(speed));
      memcpy(bytes + offset, &s, sz);
      offset += sz;
    } else if constexpr (sizeof(speed) == 4) {
      const auto sz = sizeof(speed);
      auto s        = __htonl(cnl::unwrap(speed));
      memcpy(bytes + offset, &s, sz);
      offset += sz;
    } else {
      static_assert(sizeof(speed) == 2 || sizeof(speed) == 4);
    }
  }
  return offset;
}
size_t toBytes(const SpotConfig &config, uint8_t *bytes) {
  auto offset = 0;
  bytes[0]    = SPOT_CONFIG_MAGIC;
  offset += 1;
  auto unwrapped_circleLength = cnl::unwrap(config.circleLength);
  auto circleLength           = __htonl(unwrapped_circleLength);
  memcpy(bytes + offset, &circleLength, sizeof circleLength);
  offset += sizeof circleLength;
  auto unwrapped_lineLength = cnl::unwrap(config.lineLength);
  auto lineLength           = __htonl(unwrapped_lineLength);
  memcpy(bytes + offset, &lineLength, sizeof lineLength);
  offset += sizeof lineLength;
  auto total = __htons(config.total);
  memcpy(bytes + offset, &total, sizeof total);
  offset += sizeof total;
  auto current = __htons(static_cast<uint16_t>(config.current));
  memcpy(bytes + offset, &current, sizeof current);
  offset += sizeof current;
  auto updateInterval = __htons(config.updateInterval);
  memcpy(bytes + offset, &updateInterval, sizeof updateInterval);
  offset += sizeof updateInterval;
  return offset;
}
}

struct CalcState {
  uint64_t startTime;
  uint64_t lastIntegralTime;
  uint32_t lastIntegralRelativeTime;
  fixed_16_16 lastIntegralDistance;
  uint16_t maxDistance;
};

/**
 * @brief calculate the next state
 * @param lastState last state
 * @param now current timestamp in millis
 * @param track Track
 * @param spot SpotConfig
 * @return optional<CalcState> if the next state is valid, return the next state, otherwise return nullopt
 */
etl::optional<CalcState> nextState(const CalcState &lastState, uint64_t now, Track &track, const SpotConfig &spot) {
  /// deltaT in ms (millis)
  auto deltaT = static_cast<fixed_16_16>(static_cast<uint32_t>(now - lastState.lastIntegralTime));
  if (deltaT < 0) {
    return etl::nullopt;
  }

  auto speed  = track.getSpeed(static_cast<uint16_t>(lastState.lastIntegralDistance));
  auto deltaL = speed * deltaT;
  auto d      = lastState.lastIntegralDistance + deltaL;

  if (d > (lastState.maxDistance + spot.lineLength)) {
    return etl::nullopt;
  }

  CalcState newState{
      .startTime                = lastState.startTime,
      .lastIntegralTime         = now,
      .lastIntegralRelativeTime = lastState.lastIntegralRelativeTime + static_cast<uint32_t>(deltaT),
      .lastIntegralDistance     = d,
      .maxDistance              = lastState.maxDistance};

  return newState;
}

etl::vector<uint16_t, MAX_ENABLED_ID_SIZE> calcEnabledId(const CalcState &state, const SpotConfig &spot) {
  auto headDist                       = state.lastIntegralDistance;
  auto head                           = headDist % spot.circleLength;
  etl::optional<decltype(head)> extra = etl::nullopt;
  decltype(spot.circleLength) tail;

  if (head < spot.lineLength) {
    auto extraVal = spot.lineLength - head;
    extra         = extraVal;
    tail          = spot.circleLength - extraVal;
  } else {
    tail = head - spot.lineLength;
  }

  auto spotDistance = spot.circleLength / spot.total;
  auto headSpotId   = static_cast<uint16_t>(cnl::floor(head / spotDistance));
  auto tailSpotId   = static_cast<uint16_t>(cnl::floor(tail / spotDistance));

  etl::vector<uint16_t, MAX_ENABLED_ID_SIZE> enabledIds;

  if (headDist > state.maxDistance) {
    // range(tailSpotId, spot.total)
    for (auto i = tailSpotId; i < spot.total; i++) {
      enabledIds.push_back(i);
    }
  } else if (extra.has_value()) {
    // range(0, headSpotId)
    for (auto i = 0; i < headSpotId; i++) {
      enabledIds.push_back(i);
    }
    if (headDist >= spot.circleLength) {
      // range(tailSpotId, spot.total)
      for (auto i = tailSpotId; i < spot.total; i++) {
        enabledIds.push_back(i);
      }
    }
  } else {
    // range(tailSpotId, headSpotId)
    for (auto i = tailSpotId; i < headSpotId; i++) {
      enabledIds.push_back(i);
    }
  }

  return enabledIds;
}

class Spot {
public:
  SpotState state;
  etl::delegate<void(uint8_t)> setColorCallback = [](uint8_t) {};

private:
  etl::vector<etl::pair<Track, CalcState>, MAX_TRACK_SIZE> tracks;
  SpotConfig config;

public:
  explicit Spot(SpotConfig config) : config(config) {
    state = SpotState::STOP;
  };

  [[nodiscard]] const auto &getConfig() const {
    return config;
  }

  [[nodiscard]] const auto &getTracks() const {
    return tracks;
  }

  ParseResult fromBytes(uint8_t *bytes) {
    auto offset = 0;
    if (bytes[offset] != SPOT_MAGIC) {
      return ParseResult::MAGIC_ERROR;
    }
    offset += 1;
    auto track_count = bytes[offset];
    if (track_count > MAX_TRACK_SIZE) {
      return ParseResult::VALUE_ERROR;
    }
    offset += 1;
    for (auto i = 0; i < track_count; ++i) {
      auto track = Track::fromBytes(bytes + offset);
      if (!track.has_value()) {
        return track.error();
      } else {
        offset += track.value().sizeNeeded();
        addTrack(std::move(track.value()));
      }
    }
    return ParseResult::OK;
  }

  /**
   * @brief return size needed to serialize
   */
  size_t sizeNeeded() {
    size_t size = 0;
    size += 1; // magic
    size += 1; // track count
    for (auto &track : tracks) {
      auto &[t, calc] = track;
      auto tSize      = t.sizeNeeded();
      size += tSize;
    }
    return size;
  }

  size_t toBytes(uint8_t *bytes) {
    size_t offset = 0;
    bytes[offset] = SPOT_MAGIC;
    offset += 1;
    bytes[offset] = static_cast<uint8_t>(tracks.size());
    offset += 1;
    for (auto &track : tracks) {
      auto &[t, calc] = track;
      auto tSize      = serd::toBytes(t, bytes + offset);
      offset += tSize;
    }
    return offset;
  }

  /// use std::move to avoid copy
  void setSpotConfig(SpotConfig cfg) {
    this->config = cfg;
  }

  /// use std::move to avoid copy
  void addTrack(Track track) {
    auto calcState = CalcState{
        .startTime                = 0,
        .lastIntegralTime         = 0,
        .lastIntegralRelativeTime = 0,
        .lastIntegralDistance     = 0,
        .maxDistance              = track.getMaxKey(),
    };
    tracks.push_back(etl::make_pair(track, calcState));
  }

  void start() {
    state = SpotState::START;
    for (auto &track : tracks) {
      auto &[t, calc]               = track;
      calc.startTime                = millis();
      calc.lastIntegralTime         = calc.startTime;
      calc.lastIntegralRelativeTime = 0;
      calc.lastIntegralDistance     = 0;
      calc.maxDistance              = t.getMaxKey();
    }
  }

  decltype(tracks) &getTracks() {
    return tracks;
  }

  void clearTracks() {
    tracks.clear();
  }

  void update() {
    if (state != SpotState::START) {
      return;
    }
    bool isChanged = false;
    etl::vector<bool, MAX_TRACK_SIZE> isAllStop;

    for (auto &track : tracks) {
      auto &[t, calc] = track;
      auto now        = millis();
      auto newState   = nextState(calc, now, t, config);

      if (newState.has_value()) {
        auto newCalc    = newState.value();
        auto enabledIds = calcEnabledId(newCalc, config);
        if (etl::find(enabledIds.begin(), enabledIds.end(), config.current) != enabledIds.end()) {
          isChanged = true;
          setColorCallback(t.color);
        }

        calc = newCalc;
        isAllStop.push_back(false);
      } else {
        isAllStop.push_back(true);
      }
      // identity
      if (etl::all_of(isAllStop.begin(), isAllStop.end(), [](bool v) { return v; })) {
        state = SpotState::STOP;
      }
    }
  }
};

#endif // SIMPLE_SPOT_H
