/*
 * Source Modified by Zubeen Tolani < ZeekHuge - zeekhuge@gmail.com >
 * Based on the examples distributed by TI
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <stdint.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include "resource_table_empty.h"

// Get these addresses with figure 25-5 http://www.ti.com/lit/ug/spruh73p/spruh73p.pdf
// You can test them using devmem2
#define GPIO1 0x4804C000		// Base address of GPIO1
#define GPIO_CLEARDATAOUT 0x190         // This added with GPIO1 gives the register
#define GPIO_DATAIN 0x138
#define GPIO_DATAOUT 0x13C
#define GPIO_SETDATAOUT 0x194

#define USR0 (1<<21)			// Bit number of USR0 in register. It is like a photoshop mask, it is a 1 with 21 zeros after it, and that is the bit that will be changed in R30
#define USR1 (1<<22)			// This number can be found with "sudo perl /opt/scripts/device/bone/show-pins.pl -v"
#define USR2 (1<<23)
#define USR3 (1<<24)
#define P812 12				// P8.12 is the input button, 12 is the number of that bit in R31

// Volatile keyword tells the compiler that the variable can change at any time without action being taken by any code nearby
unsigned int volatile * const GPIO1_CLEAR = (unsigned int *) (GPIO1 + GPIO_CLEARDATAOUT);	// A pointer to the register that is used to clear a particular bit
unsigned int volatile * const GPIO1_SET = (unsigned int *) (GPIO1 + GPIO_SETDATAOUT);		// A pointer to the register that is used to set a particular bit
unsigned int volatile * const GPIO1_READ = (unsigned int *) (GPIO1 + GPIO_DATAIN);		// A pointer to the register for io in

volatile register unsigned int __R30;
volatile register unsigned int __R31;

#define PRU0_GPIO (1<<2)		// Bit for PRU0_GPIO, a 1 with two zeros after it

void SET_USR1() {
	#ifndef LED_DISABLE
        *GPIO1_SET = USR0;
       	#endif
        #ifndef GPIO_DISABLE
        __R30 |= PRU0_GPIO;
        #endif
}

void CLEAR_USR1() {
        #ifndef LED_DISABLE
        *GPIO1_CLEAR = USR0;
        #endif
        #ifndef GPIO_DISABLE
        __R30 ^= PRU0_GPIO;
        #endif
}

void main(void) {

	/* Clear SYSCFG[STANDBY_INIT] to enable OCP master port */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	while(1==1) {
		if( (*GPIO1_READ >> P812) & 1 ) {
			SET_USR1();
		} else {
			CLEAR_USR1();
		}
	}
}
