#ifndef KNX_CUSTOM_I2C_GUARD_H
#define KNX_CUSTOM_I2C_GUARD_H

#include <stdbool.h>

/* OLED and the line sensor share the same physical I2C pins.  The guard only
 * protects one complete bus transaction; callers never wait while holding a
 * real-time task and simply retry on their next period. */
bool knx_custom_i2c_try_lock(void);
void knx_custom_i2c_unlock(void);

#endif
