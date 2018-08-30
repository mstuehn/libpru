/*-
 * Copyright (c) 2014-2015 Rui Paulo <rpaulo@felyko.com>
 * Copyright (c) 2015 Manuel Stuehn
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#define AM33XX_REV      0x4E82A900

#define AM33XX_IRAM_SIZE    0x00002000
#define AM33XX_RAM_REG      0x00000000
#define AM33XX_RAM_SIZE     0x00020000
#define AM33XX_PRUnRAM(n)   (AM33XX_RAM_REG + (n) * AM33XX_RAM_SIZE)
#define AM33XX_INTC_REG     0x00020000
#define AM33XX_PRU0CTL_REG  0x00022000
#define AM33XX_PRU1CTL_REG  0x00024000
#define AM33XX_PRUnCTL(n)   (AM33XX_PRU0CTL_REG + (n) * 0x2000)
#define AM33XX_PRUnSTATUS(n)(AM33XX_PRU0CTL_REG + 0x4 + (n) * 0x2000)
#define AM33XX_PRU0DBG_REG  0x00022400
#define AM33XX_PRU1DBG_REG  0x00024400
#define AM33XX_PRUnDBG(n)   (AM33XX_PRU0DBG_REG + (n) * 0x2000)
#define AM33XX_PRU0IRAM_REG 0x00034000
#define AM33XX_PRU1IRAM_REG 0x00038000
#define AM33XX_PRUnIRAM(n)  (AM33XX_PRU0IRAM_REG + (n) * 0x4000)
#define AM33XX_MMAP_SIZE    0x00040000
#define AM33XX_NUM_PRUSS    2
#define AM33XX_NUM_HOST_INTS     7

/* Control register */
#define CTL_REG_RESET           (1U << 0)   /* Clear to reset */
#define CTL_REG_ENABLE      (1U << 1)
#define CTL_REG_SLEEP       (1U << 2)
#define CTL_REG_COUNTER     (1U << 3)   /* Cycle counter */
#define CTL_REG_SINGLE_STEP (1U << 8)
#define CTL_REG_RUNSTATE    (1U << 15)

#define CTL_REG_STS     0x04
#define CTL_WAKEUP_EN   0x08
#define CTL_CYCLE       0x0C
#define CTL_STALL       0x10
#define CTL_CTBIR0      0x20
#define CTL_CTBIR1      0x24
#define CTL_CTPPR0      0x28
#define CTL_CTPPR1      0x2C


/* INTC-Register */
#define INTC_REG_REVID  (AM33XX_INTC_REG + 0x000 )
#define INTC_REG_CR     (AM33XX_INTC_REG + 0x004 )
#define INTC_REG_HCR    (AM33XX_INTC_REG + 0x00C )
#define INTC_REG_GER    (AM33XX_INTC_REG + 0x010 )
#define INTC_REG_GNLR   (AM33XX_INTC_REG + 0x01C )
#define INTC_REG_SISR   (AM33XX_INTC_REG + 0x020 )
#define INTC_REG_SICR   (AM33XX_INTC_REG + 0x024 )
#define INTC_REG_EISR   (AM33XX_INTC_REG + 0x028 )
#define INTC_REG_EICR   (AM33XX_INTC_REG + 0x02C )
#define INTC_REG_HIEISR (AM33XX_INTC_REG + 0x034 )
#define INTC_REG_HIDISR (AM33XX_INTC_REG + 0x038 )
#define INTC_REG_GPIR   (AM33XX_INTC_REG + 0x080 )
#define INTC_REG_SRSR0  (AM33XX_INTC_REG + 0x200 )
#define INTC_REG_SRSR1  (AM33XX_INTC_REG + 0x204 )
#define INTC_REG_SECR0  (AM33XX_INTC_REG + 0x280 )
#define INTC_REG_SECR1  (AM33XX_INTC_REG + 0x284 )
#define INTC_REG_ESR0   (AM33XX_INTC_REG + 0x300 )
#define INTC_REG_ESR1   (AM33XX_INTC_REG + 0x304 )
#define INTC_REG_ECR0   (AM33XX_INTC_REG + 0x380 )
#define INTC_REG_ECR1   (AM33XX_INTC_REG + 0x384 )
#define INTC_REG_CMR0   (AM33XX_INTC_REG + 0x400 )
#define INTC_REG_CMR1   (AM33XX_INTC_REG + 0x404 )
#define INTC_REG_CMR2   (AM33XX_INTC_REG + 0x408 )
#define INTC_REG_CMR3   (AM33XX_INTC_REG + 0x40C )
#define INTC_REG_CMR4   (AM33XX_INTC_REG + 0x410 )
#define INTC_REG_CMR5   (AM33XX_INTC_REG + 0x414 )
#define INTC_REG_CMR6   (AM33XX_INTC_REG + 0x418 )
#define INTC_REG_CMR7   (AM33XX_INTC_REG + 0x41C )
#define INTC_REG_CMR8   (AM33XX_INTC_REG + 0x420 )
#define INTC_REG_CMR9   (AM33XX_INTC_REG + 0x424 )
#define INTC_REG_CMR10  (AM33XX_INTC_REG + 0x428 )
#define INTC_REG_CMR11  (AM33XX_INTC_REG + 0x42C )
#define INTC_REG_CMR12  (AM33XX_INTC_REG + 0x430 )
#define INTC_REG_CMR13  (AM33XX_INTC_REG + 0x434 )
#define INTC_REG_CMR14  (AM33XX_INTC_REG + 0x438 )
#define INTC_REG_CMR15  (AM33XX_INTC_REG + 0x43C )
#define INTC_REG_HMR0   (AM33XX_INTC_REG + 0x800 )
#define INTC_REG_HMR1   (AM33XX_INTC_REG + 0x804 )
#define INTC_REG_HMR2   (AM33XX_INTC_REG + 0x808 )
#define INTC_REG_SIPR0  (AM33XX_INTC_REG + 0xD00 )
#define INTC_REG_SIPR1  (AM33XX_INTC_REG + 0xD04 )
#define INTC_REG_SITR0  (AM33XX_INTC_REG + 0xD80 )
#define INTC_REG_SITR1  (AM33XX_INTC_REG + 0xD84 )
#define INTC_REG_HIER   (AM33XX_INTC_REG + 0x1500)

#define INTC_REG_xMRx(  base_reg, value ) ((base_reg) + ((value)/4)*4)
#define INTC_REG_CMRx( event ) INTC_REG_xMRx( INTC_REG_CMR0, event )
#define INTC_REG_HMRx( event ) INTC_REG_xMRx( INTC_REG_HMR0, event )

#define INTC_REG_xMRx_POS( value ) (((value)%4)*8)
#define INTC_REG_CMRx_POS( event ) INTC_REG_xMRx_POS(event)
#define INTC_REG_HMRx_POS( event ) INTC_REG_xMRx_POS(event)

#define BITMASK_REG( base, value ) ((base) + ((value)/32)*4)
#define INTC_REG_SECRx( value ) BITMASK_REG(INTC_REG_SECR0, value)
#define INTC_REG_SIPRx( value ) BITMASK_REG(INTC_REG_SIPR0, value)
#define INTC_REG_SITRx( value ) BITMASK_REG(INTC_REG_SITR0, value)


