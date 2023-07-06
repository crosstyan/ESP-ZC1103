//
// Created by Kurosu Chan on 2023/7/6.
//

#ifndef TRACKRF_MESSAGE_H
#define TRACKRF_MESSAGE_H

#include "wrapper.pb.h"
#include "spot.pb.h"
#include "spot_config.pb.h"
#include "spot.h"
#include "pb_decode.h"
#include <etl/variant.h>
// #include <variant>

namespace RfMessage {
const static auto ADDR_SIZE = 3;
using AddrArray             = etl::array<uint8_t, ADDR_SIZE>;
using Message               = etl::variant<RfMessage::Tracks, RfMessage::SpotConfig, RfMessage::Ping, RfMessage::SetCurrent>;
// certain target or broadcast
using Destination = etl::variant<AddrArray, uint32_t>;

struct BleMsgDecodeRes {
  Message msg;
  Destination dest;
};

etl::optional<BleMsgDecodeRes> decodeBleMsg(const uint8_t *buffer, size_t size) {
  BleMessage msg                      = BleMessage_init_zero;
  auto addr                           = AddrArray{0x00, 0x00, 0x00};
  msg.destination.device.funcs.decode = [](pb_istream_t *stream, const pb_field_t *field, void **arg) {
    auto &dest = *static_cast<AddrArray *>(*arg);
    // TODO: is bytes_left the size of the array?
    if (!pb_read(stream, dest.data(), stream->bytes_left)) {
      return false;
    }
    return true;
  };
  msg.destination.device.arg           = &addr;
  auto dummy_tracks                    = Tracks{};
  msg.payload.spot.tracks.funcs.decode = [](pb_istream_t *stream, const pb_field_t *field, void **arg) {
    auto &tracks          = *static_cast<etl::vector<RfMessage::Track, MAX_TRACK_SIZE> *>(*arg);
    auto track            = Track{};
    ::Track t             = Track_init_zero;
    t.speeds.funcs.decode = [](pb_istream_t *stream, const pb_field_t *field, void **arg) {
      auto &track         = *static_cast<Track *>(*arg);
      Track_SpeedsEntry t = Track_SpeedsEntry_init_zero;
      bool st             = pb_decode(stream, Track_SpeedsEntry_fields, &t);
      if (st) {
        auto fixed = fixed_8_8{t.value};
        track.addSpeed(t.key, fixed);
        return true;
      } else {
        return false;
      }
    };
    t.speeds.arg = &track;
    if (!pb_decode(stream, Track_fields, &t)) {
      return false;
    }
    track.color = t.color;
    track.id    = t.id;
    tracks.push_back(track);
    return true;
  };
  msg.payload.spot.tracks.arg = &dummy_tracks;
  pb_istream_t istream        = pb_istream_from_buffer(buffer, size);
  auto res                    = pb_decode(&istream, BleMessage_fields, &msg);
  if (!res) {
    return etl::nullopt;
  }
  auto ret = BleMsgDecodeRes{};
  switch (msg.which_destination) {
    case BleMessage_broadcast_tag:
      ret.dest = msg.destination.broadcast;
      break;
    case BleMessage_device_tag:
      ret.dest = std::move(addr);
      break;
    default:
      return etl::nullopt;
  }
  switch (msg.which_payload) {
    case BleMessage_spot_tag:
      ret.msg = std::move(dummy_tracks);
      break;
    case BleMessage_config_tag: {
      auto cfg           = SpotConfig{};
      cfg.total          = msg.payload.config.total;
      cfg.circleLength   = fixed_16_16{msg.payload.config.circle_length};
      cfg.lineLength     = fixed_16_16{msg.payload.config.line_length};
      cfg.current        = msg.payload.config.current;
      cfg.updateInterval = msg.payload.config.update_interval;
      ret.msg            = std::move(cfg);
      break;
    }
    case BleMessage_ping_tag: {
      ret.msg = Ping{};
      break;
    }
    case BleMessage_set_current_tag: {
      auto set_current       = SetCurrent{};
      set_current.current_id = msg.payload.set_current;
      ret.msg                = std::move(set_current);
      break;
    }
    default:
      return etl::nullopt;
  }
  return etl::make_optional(ret);
}

}

#endif // TRACKRF_MESSAGE_H
