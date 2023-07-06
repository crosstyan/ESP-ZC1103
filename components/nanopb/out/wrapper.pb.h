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

/* Struct definitions */
/* https://github.com/crosstyan/track-esp32/blob/34563a59cc8ae7e6bf4c9a7cdc3d2d1ac27e8bf0/main/StripCallbacks.cpp#L60
 to wrap around */
typedef struct _Message {
    pb_size_t which_destination;
    union {
        pb_callback_t device;
        /* magic value for broadcast */
        int32_t broadcast;
    } destination;
    pb_size_t which_payload;
    union {
        SpotConfig config;
        Spot spot;
    } payload;
} Message;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define Message_init_default                     {0, {{{NULL}, NULL}}, 0, {SpotConfig_init_default}}
#define Message_init_zero                        {0, {{{NULL}, NULL}}, 0, {SpotConfig_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define Message_device_tag                       1
#define Message_broadcast_tag                    2
#define Message_config_tag                       3
#define Message_spot_tag                         4

/* Struct field encoding specification for nanopb */
#define Message_FIELDLIST(X, a) \
X(a, CALLBACK, ONEOF,    BYTES,    (destination,device,destination.device),   1) \
X(a, STATIC,   ONEOF,    INT32,    (destination,broadcast,destination.broadcast),   2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,config,payload.config),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,spot,payload.spot),   4)
#define Message_CALLBACK pb_default_field_callback
#define Message_DEFAULT NULL
#define Message_payload_config_MSGTYPE SpotConfig
#define Message_payload_spot_MSGTYPE Spot

extern const pb_msgdesc_t Message_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Message_fields &Message_msg

/* Maximum encoded size of messages (where known) */
#if defined(Spot_size)
union Message_payload_size_union {char f4[(6 + Spot_size)]; char f0[30];};
#endif
/* Message_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
