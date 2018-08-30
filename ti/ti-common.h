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

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "libpru.h"
#include "pru-private.h"
#include "ti-pru.h"

#if !defined(__unused)
#define __unused
#endif

static inline uint8_t
ti_reg_read_1(uint8_t *mem, unsigned int reg)
{
    DPRINTF("reg 0x%x\n", reg);
    return *(volatile uint8_t *)(void *)(mem + reg);
}

static inline uint32_t
ti_reg_read_4(uint8_t *mem, unsigned int reg)
{
    DPRINTF("reg 0x%x\n", reg);
    return *(volatile uint32_t *)(void *)(mem + reg);
}

static inline void
ti_reg_write_4(uint8_t *mem, unsigned int reg, uint32_t value)
{
    DPRINTF("reg 0x%x, val 0x%x\n", reg, value);
    *(volatile uint32_t *)(void *)(mem + reg) = value;
}

static inline void
ti_reg_set( uint8_t* mem, uint32_t offset, uint32_t mask )
{
    ti_reg_write_4(mem, offset,
                   ti_reg_read_4(mem, offset) | mask);
}

static inline void
ti_reg_clr( uint8_t* mem, uint32_t offset, uint32_t mask )
{
    ti_reg_write_4(mem, offset,
                   ti_reg_read_4(mem, offset) & ~mask);
}

static inline void
ti_reg_tgl( uint8_t* mem, uint32_t offset, uint32_t mask )
{
    ti_reg_write_4(mem, offset,
                   ti_reg_read_4(mem, offset) ^ mask);
}

static inline void
ti_reg_str(uint8_t reg, char *buf, size_t len)
{
    if (reg < 0xe0) {
        snprintf(buf, len, "r%d.%c%d", reg & 0x1f,
            reg & 0x80 ? 'w' : 'b', (reg >> 5) & 0x3);
    } else
        snprintf(buf, len, "r%d", reg - 0xe0);
}

static inline void
ti_std_ins(const char *ins, const char *op1, const char *op2, const char *op3,
    char *buf, size_t len)
{
    bool comma = false;

    snprintf(buf, len, "%s", ins);
    if (op1 && op1[0] != 0) {
        snprintf(buf, len, "%s %s", buf, op1);
        comma = true;
    }
    if (op2 && op2[0] != 0) {
        snprintf(buf, len, "%s%s %s", buf, comma == true ? "," : "",
            op2);
        comma = true;
    }
    if (op3 && op3[0] != 0)
        snprintf(buf, len, "%s%s %s", buf, comma == true ? "," : "",
            op3);
}

