"""B_pinv exactly as it appears in the vehicle firmware.

Copied from Pico_Main.ino (SECTION 7, `B_pinv[N_THR][N_DOF]`). The same
literals appear in TIBURON-AUV-FreeRTOS and Nucleo_AUV_Bare_Metal
(Core/Src/control_loop.c).
"""

FIRMWARE_B_PINV = [
    [-0.0349, -0.0776,  0.2493,  2.0482, -1.6573, -0.2618],
    [-0.0448, -0.0089,  0.3060, -2.0430, -2.0306,  0.0586],
    [ 0.0439,  0.0148,  0.1989,  1.6903,  1.9984, -0.0310],
    [ 0.0358,  0.0717,  0.2459, -1.6955,  1.6895,  0.2341],
    [ 0.0765, -0.5493,  0.0036, -0.0622, -0.1885, -1.0178],
    [ 0.0617,  0.8347,  0.0029, -0.0503, -0.1523,  1.9589],
    [ 0.7072,  0.7072,  0.0000,  0.0000,  0.0000,  0.0000],
    [ 0.6925,  0.6768, -0.0007,  0.0120,  0.0363,  2.9767],
]
