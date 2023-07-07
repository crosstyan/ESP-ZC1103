/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8-dev */

#ifndef PB_WRAPPER_PB_H_INCLUDED
#define PB_WRAPPER_PB_H_INCLUDED
#include <pb.h>
#include "spot_config.pb.h"
#include "spot.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _Command {
    Command_Start = 0,
    Command_Stop = 1,
    Command_Ping = 6
} Command;

/* Struct definitions */
/* https://github.com/crosstyan/track-esp32/blob/34563a59cc8ae7e6bf4c9a7cdc3d2d1ac27e8bf0/main/StripCallbacks.cpp#L60
 to wrap around */
typedef struct _BleMessage {
    pb_size_t which_destination;
    union {
        pb_callback_t device;
        /* magic value for broadcast */
        uint32_t broadcast;
    } destination;
    pb_size_t which_payload;
    union {
        SpotConfig config;
        Spot spot;
        /* with payload */
        uint32_t set_current;
        /* just a magic */
        Command command;
    } payload;
} BleMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _Command_MIN Command_Start
#define _Command_MAX Command_Ping
#define _Command_ARRAYSIZE ((Command)(Command_Ping+1))

#define BleMessage_payload_command_ENUMTYPE Command


/* Initializer values for message structs */
#define BleMessage_init_default                  {0, {{{NULL}, NULL}}, 0, {SpotConfig_init_default}}
#define BleMessage_init_zero                     {0, {{{NULL}, NULL}}, 0, {SpotConfig_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define BleMessage_device_tag                    1
#define BleMessage_broadcast_tag                 2
#define BleMessage_config_tag                    3
#define BleMessage_spot_tag                      4
#define BleMessage_set_current_tag               5
#define BleMessage_command_tag                   6

/* Struct field encoding specification for nanopb */
#define BleMessage_FIELDLIST(X, a) \
X(a, CALLBACK, ONEOF,    BYTES,    (destination,device,destination.device),   1) \
X(a, STATIC,   ONEOF,    UINT32,   (destination,broadcast,destination.broadcast),   2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,config,payload.config),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,spot,payload.spot),   4) \
X(a, STATIC,   ONEOF,    UINT32,   (payload,set_current,payload.set_current),   5) \
X(a, STATIC,   ONEOF,    UENUM,    (payload,command,payload.command),   6)
#define BleMessage_CALLBACK pb_default_field_callback
#define BleMessage_DEFAULT NULL
#define BleMessage_payload_config_MSGTYPE SpotConfig
#define BleMessage_payload_spot_MSGTYPE Spot

extern const pb_msgdesc_t BleMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define BleMessage_fields &BleMessage_msg

/* Maximum encoded size of messages (where known) */
#if defined(Spot_size)
union BleMessage_payload_size_union {char f4[(6 + Spot_size)]; char f0[30];};
#endif
/* BleMessage_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
