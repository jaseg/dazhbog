#ifndef __MAC_H__
#define __MAC_H__

#include <unistd.h>

/* Device MAC address.
 *
 * 32 bits might seem a little short for a device MAC, but at 20 bus nodes the probablility of a collision is about 1 in
 * 10 million. Check for yourself using the python code below.
 *
 *     #!/usr/bin/env python3
 *     from operator import mul
 *     from functools import reduce
 *     m = 32
 *     n = 20
 *     print(reduce(mul, [2**m-i for i in range(n)]) / ((2**m)**n))
 *     # -> 0.9999999557621786
 */

extern uint32_t device_mac;

#endif /* __MAC_H__ */
