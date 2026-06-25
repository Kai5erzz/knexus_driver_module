#ifndef KNX_TYPES_H
#define KNX_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ---------- Status codes ---------- */
typedef enum {
    KNX_OK          =  0,
    KNX_ERROR       = -1,
    KNX_TIMEOUT     = -2,
    KNX_INVALID_ARG = -3,
    KNX_BUSY        = -4,
    KNX_NOT_READY   = -5,
} knx_status_t;

/* ---------- Boolean convenience ---------- */
#ifndef TRUE
  #define TRUE   1
#endif
#ifndef FALSE
  #define FALSE  0
#endif

/* ---------- Bit manipulation ---------- */
#define KNX_BIT(n)         (1U << (n))
#define KNX_BIT_SET(v, b)  ((v) |=  KNX_BIT(b))
#define KNX_BIT_CLR(v, b)  ((v) &= ~KNX_BIT(b))
#define KNX_BIT_TST(v, b)  ((v) &   KNX_BIT(b))

#endif /* KNX_TYPES_H */
