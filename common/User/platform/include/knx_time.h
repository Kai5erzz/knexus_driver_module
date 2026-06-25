#ifndef KNX_TIME_H
#define KNX_TIME_H

#include <stdint.h>

/* Millisecond tick — wraps at 0xFFFFFFFF (~49 days) */
uint32_t knx_millis(void);

/* Microsecond tick — platform-dependent, may return 0 if not available */
uint32_t knx_micros(void);

/* Blocking delay in milliseconds */
void     knx_delay_ms(uint32_t ms);

#endif /* KNX_TIME_H */
