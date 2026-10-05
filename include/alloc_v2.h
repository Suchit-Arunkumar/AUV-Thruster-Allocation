/*
 * alloc_v2.h - improved allocators. NOT FLOWN.
 *
 * Verified in simulation (python/reference.py, bench/analysis.py) and on a
 * NUCLEO-F446RE bench only. The allocator that flew is alloc.h /
 * alloc_wrench_to_thrust(), which stays the reference implementation.
 *
 * Both functions replace only the wrench -> thrust step; PWM mapping and
 * slew are alloc_thrust_to_pwm() / alloc_slew_step() unchanged.
 *
 * alloc_v2_wrench_to_thrust
 *   T = B+ U with the full B+ (no block truncation). If any |T_i| > 1, all
 *   eight thrusts are divided by max |T_i|: the achieved wrench is U / max,
 *   so its direction is exactly the requested one and only magnitude is lost.
 *
 * alloc_v2p_wrench_to_thrust
 *   Same, with priority: the depth/attitude part (heave, roll, pitch) is
 *   allocated first and scaled uniformly only if it alone saturates. The
 *   surge/sway/yaw part then gets the largest common scale s in [0, 1] that
 *   keeps every thruster within [-1, 1].
 */
#ifndef ALLOC_V2_H
#define ALLOC_V2_H

#include "alloc.h"

#define ALLOC_V2_SCALED     0x08u /* v2: whole wrench scaled.
                                     v2p: heave/roll/pitch part scaled and
                                     surge/sway/yaw dropped. */
#define ALLOC_V2_SECONDARY  0x10u /* v2p: surge/sway/yaw part scaled by s < 1 */

uint8_t alloc_v2_wrench_to_thrust(const float wrench[ALLOC_N_DOF],
                                  float thrust[ALLOC_N_THR]);

uint8_t alloc_v2p_wrench_to_thrust(const float wrench[ALLOC_N_DOF],
                                   float thrust[ALLOC_N_THR]);

#endif /* ALLOC_V2_H */
