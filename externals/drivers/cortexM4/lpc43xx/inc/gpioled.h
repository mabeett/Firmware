/* Operar leds para depuracón */

#ifndef GPIOLED_H
#define GPIOLED_H

#if (ciaa_nxp == BOARD)
#ifndef __CHIP_H_

#ifdef TRUE
#undef TRUE
#endif
#ifdef FALSE
#undef FALSE
#endif

#include "chip.h"
#endif /* __CHIP_H_*/

#define SET_O1(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 5, 1, n)  // D0 (relé)
#define SET_O2(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 2, 6, n)  // D1 (relé)
#define SET_O3(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 2, 5, n)  // D2 (relé)
#define SET_O4(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 2, 4, n)  // D3 (relé)
#define SET_O5(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 5, 12, n) // D4
#define SET_O6(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 5, 13, n) // D5
#define SET_O7(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 5, 14, n) // D6
#define SET_O8(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 1, 8, n)  // D7
#define SET_O9(n) NOHAY_SALIDA_O9(n)

#else 
#if (edu_ciaa_nxp == BOARD)
#ifndef __CHIP_H_

#ifdef TRUE
#undef TRUE
#endif
#ifdef FALSE
#undef FALSE
#endif

#include "chip.h"
#endif /* __CHIP_H_*/

#define SET_O1(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 5, 0, n)  // LED_R
#define SET_O2(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 5, 1, n)  // LED_G
#define SET_O3(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 5, 2, n)  // LED_B
#define SET_O4(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, n) // LED_L1
#define SET_O5(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 1, 11, n) // LED_L2
#define SET_O6(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 1, 12, n) // LED_L3
#define SET_O7(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 3, 0, n)  // GPIO_0
#define SET_O8(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 3, 3, n)  // GPIO_1
#define SET_O9(n) Chip_GPIO_SetPinState(LPC_GPIO_PORT, 3, 4, n)  // GPIO_2

#else
/* otras placas o arquitecturas no importan */

#define SET_O1(n)
#define SET_O2(n)
#define SET_O3(n)
#define SET_O4(n)
#define SET_O5(n)
#define SET_O6(n)
#define SET_O7(n)
#define SET_O8(n)

#endif /* (edu_ciaa_nxp == BOARD) */ 
#endif /*  (ciaa_nxp == BOARD) */
#endif
