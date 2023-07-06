/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8-dev */

#ifndef PB_SPOT_CONFIG_PB_H_INCLUDED
#define PB_SPOT_CONFIG_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _SpotConfig {
    float circle_length;
    float line_length;
    uint32_t total;
    uint32_t current;
    uint32_t update_interval;
} SpotConfig;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define SpotConfig_init_default                  {0, 0, 0, 0, 0}
#define SpotConfig_init_zero                     {0, 0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define SpotConfig_circle_length_tag             1
#define SpotConfig_line_length_tag               2
#define SpotConfig_total_tag                     3
#define SpotConfig_current_tag                   4
#define SpotConfig_update_interval_tag           5

/* Struct field encoding specification for nanopb */
#define SpotConfig_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    circle_length,     1) \
X(a, STATIC,   SINGULAR, FLOAT,    line_length,       2) \
X(a, STATIC,   SINGULAR, UINT32,   total,             3) \
X(a, STATIC,   SINGULAR, UINT32,   current,           4) \
X(a, STATIC,   SINGULAR, UINT32,   update_interval,   5)
#define SpotConfig_CALLBACK NULL
#define SpotConfig_DEFAULT NULL

extern const pb_msgdesc_t SpotConfig_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define SpotConfig_fields &SpotConfig_msg

/* Maximum encoded size of messages (where known) */
#define SpotConfig_size                          28

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
