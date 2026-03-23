//
// Simple seqpacket IPC between vgp/vgpw (client) and vgp-privd (root daemon)
//
#ifndef VGP_PRIVD_PROTO_H
#define VGP_PRIVD_PROTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VGP_PRIVD_MAGIC 0x56475044u /* 'VGPD' */
#define VGP_PRIVD_VER   1

/* Commands */
enum {
  VGP_CMD_GET40   = 1,      /* -> vgp_pin_state_t[41] indexed by physical pin */
  VGP_CMD_GET_PIN = 2,      /* ch/ln -> one vgp_pin_state_t */
  VGP_CMD_SET_ALT = 3,      /* ch/ln/alt */
  VGP_CMD_SET_DIR = 4,      /* ch/ln/dir */
  VGP_CMD_SET_VAL = 5,      /* ch/ln/val (and enforce DDR=OUT) */
  /* Edge-event fanout */
  VGP_CMD_SUBSCRIBE   = 6,  /* pin/edges -> start receiving VGP_CMD_EVENT */
  VGP_CMD_UNSUBSCRIBE = 7,  /* pin -> stop receiving events for pin */
  VGP_CMD_EVENT       = 8,  /* async vgpd -> client: vgp_evt_t */
};

/* Pin flags */
enum {
  VGP_PINF_VALID = 1 << 0, /* IO pin on 40-pin header */
  VGP_PINF_POWER = 1 << 1, /* 3.3V/5V/GND etc (non-IO) */
};

typedef struct __attribute__((packed)) {
  int8_t  alt;   /* 0..3, or -1 if N/A */
  int8_t  dir;   /* 0=in, 1=out, or -1 */
  int8_t  val;   /* 0/1, or -1 */
  uint8_t flags; /* VGP_PINF_* */
} vgp_pin_state_t;

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t ver;
  uint16_t cmd;
  uint32_t size;   /* total bytes including header */
  uint32_t seq;
} vgp_req_hdr_t;

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t ver;
  uint16_t cmd;
  int32_t  status; /* 0 ok, or -errno */
  uint32_t size;   /* total bytes including header */
  uint32_t seq;
} vgp_resp_hdr_t;

/* Request bodies */
typedef struct __attribute__((packed)) { uint8_t ch, ln; } vgp_req_pin_t;
typedef struct __attribute__((packed)) { uint8_t ch, ln; int8_t alt; } vgp_req_set_alt_t;
typedef struct __attribute__((packed)) { uint8_t ch, ln; int8_t dir; } vgp_req_set_dir_t;
typedef struct __attribute__((packed)) { uint8_t ch, ln; int8_t val; } vgp_req_set_val_t;

/* Subscribe/unsubscribe body (physical pin) */
typedef struct __attribute__((packed)) {
  uint8_t pin;     /* 1..40 */
  uint8_t edges;   /* 1=rising, 2=falling, 3=both (same as vgplib GPIO_*_EDGE) */
  uint16_t _pad;
} vgp_req_sub_t;

typedef struct __attribute__((packed)) {
  uint8_t pin;
  uint8_t _pad[3];
} vgp_req_unsub_t;

/* Response bodies */
typedef struct __attribute__((packed)) { vgp_pin_state_t st; } vgp_resp_pin_t;
typedef struct __attribute__((packed)) { vgp_pin_state_t pins[41]; } vgp_resp_40_t;

/* Async event body */
typedef struct __attribute__((packed)) {
  uint8_t pin;   /* physical pin */
  uint8_t edge;  /* 1=rising, 2=falling */
  int8_t  val;   /* 1 for rising, 0 for falling */
  uint8_t _pad;
} vgp_evt_t;

#ifdef __cplusplus
}
#endif

#endif
