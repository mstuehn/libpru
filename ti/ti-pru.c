/*-
 * Copyright (c) 2014-2015 Rui Paulo <rpaulo@felyko.com>
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
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <stdbool.h>

#include <sys/errno.h>
#include <sys/types.h>
#include <sys/event.h>
#include <sys/mman.h>
#include <sys/cdefs.h>

#if !defined(__unused)
#define __unused
#endif

#include "libpru.h"
#include "pru-private.h"
#include "ti-pru.h"
#include "ti-common.h"

static struct drivers
{
    const char* name;
    int32_t (*initialize)( pru_t pru );
} initializers []  = {
        {
                .name = "am335x",
                .initialize = am335x_initialize,
        },
};


static int
ti_disassemble(pru_t pru __unused, uint32_t opcode, char *buf, size_t len)
{
    uint8_t ins, op1, op2, op3;
    uint16_t imm;
    char c_op1[16], c_op2[16], c_op3[16];
    const char *c_ins;

    DPRINTF("disassembling 0x%x\n", opcode);
    c_ins = NULL;
    bzero(c_op1, sizeof(c_op1));
    bzero(c_op2, sizeof(c_op2));
    bzero(c_op3, sizeof(c_op3));
    ins = (opcode & 0xff000000) >> 24;
    op1 = (opcode & 0x00ff0000) >> 16;
    op2 = (opcode & 0x0000ff00) >> 8;
    op3 = opcode & 0xff;
    /*
     * Parse the name of the instruction.
     */
    switch (ins) {
    case TI_OP_ADD:
    case TI_OP_ADDI:
        c_ins = "add";
        break;
    case TI_OP_ADC:
    case TI_OP_ADCI:
        c_ins = "adc";
        break;
    case TI_OP_SUB:
    case TI_OP_SUBI:
        c_ins = "sub";
        break;
    case TI_OP_SUC:
    case TI_OP_SUCI:
        c_ins = "suc";
        break;
    case TI_OP_LSL:
    case TI_OP_LSLI:
        c_ins = "lsl";
        break;
    case TI_OP_LSR:
    case TI_OP_LSRI:
        c_ins = "lsr";
        break;
    case TI_OP_RSB:
    case TI_OP_RSBI:
        c_ins = "rsb";
        break;
    case TI_OP_RSC:
    case TI_OP_RSCI:
        c_ins = "rsc";
        break;
    case TI_OP_AND:
        /* An instruction following 'slp'. We ignore it. */
        if (opcode == 0x10000000) {
            buf[0] = 0;
            return EINVAL;
        }
        if (op1 == op2)
            c_ins = "mov";
        else
            c_ins = "and";
        break;
    case TI_OP_OR:
    case TI_OP_ORI:
        c_ins = "or";
        break;
    case TI_OP_XOR:
    case TI_OP_XORI:
        c_ins = "xor";
        break;
    case TI_OP_NOT:
        c_ins = "not";
        break;
    case TI_OP_MIN:
        c_ins = "min";
        break;
    case TI_OP_MAX:
        c_ins = "max";
        break;
    case TI_OP_CLR:
    case TI_OP_CLRI:
        c_ins = "clr";
        break;
    case TI_OP_SET:
        c_ins = "set";
        break;
    case TI_OP_JMP:
        if (op1 == 0x9e)
            c_ins = "ret";
        else
            c_ins = "jmp";
        break;
    case TI_OP_JAL:
        if (op3 == 0x9e)
            c_ins = "call";
        else
            c_ins = "jal";
        break;
    case TI_OP_LDI:
        c_ins = "ldi";
        break;
    case TI_OP_LMBD:
        c_ins = "lmbd";
        break;
    case TI_OP_HALT:
        c_ins = "halt";
        break;
    case TI_OP_MVI:
        if (op1 == 0x20)
            c_ins = "mvib";
        else if (op1 == 0x21)
            c_ins = "mviw";
        else if (op1 == 0x22)
            c_ins = "mvid";
        else
            c_ins = "mvi";
        break;
    case TI_OP_XIN:
        c_ins = "xin";
        break;
    case TI_OP_XOUT:
        c_ins = "xout";
        break;
    case TI_OP_SLP:
        c_ins = "slp";
        break;
    case TI_OP_QBLT:
        c_ins = "qblt";
        break;
    case TI_OP_QBEQ:
        c_ins = "qbeq";
        break;
    case TI_OP_QBLE:
        c_ins = "qble";
        break;
    case TI_OP_QBGT:
        c_ins = "qbgt";
        break;
    case TI_OP_QBNE:
        c_ins = "qbne";
        break;
    case TI_OP_QBGE:
        c_ins = "qbge";
        break;
    case TI_OP_QBA:
        c_ins = "qba";
        break;
    case TI_OP_SBCO:
        c_ins = "sbco";
        break;
    case TI_OP_LBCO:
        c_ins =  "lbco";
        break;
    case TI_OP_QBBC:
        c_ins = "qbbc";
        break;
    case TI_OP_QBBS:
        c_ins = "qbbs";
        break;
    case TI_OP_SBBO:
        c_ins = "sbbo";
        break;
    case TI_OP_LBBO:
        c_ins = "lbbo";
        break;
    default:
        c_ins = "UD#";
        break;
    }
    /*
     * Process operators.
     */
    switch (ins) {
    /* Standard 3 register instructions. */
    case TI_OP_ADD:
    case TI_OP_ADC:
    case TI_OP_SUB:
    case TI_OP_SUC:
    case TI_OP_LSL:
    case TI_OP_LSR:
    case TI_OP_RSB:
    case TI_OP_RSC:
    case TI_OP_AND:
    case TI_OP_OR:
    case TI_OP_XOR:
    case TI_OP_MIN:
    case TI_OP_MAX:
    case TI_OP_SET:
    case TI_OP_LMBD:
        ti_reg_str(op1, c_op1, sizeof(c_op1));
        ti_reg_str(op2, c_op2, sizeof(c_op2));
        ti_reg_str(op3, c_op3, sizeof(c_op3));
        if (ins == TI_OP_AND && op1 == op2)
            bzero(c_op2, sizeof(c_op2));
        break;
    /* Instructions with two registers. */
    case TI_OP_NOT:
        /* N.B. op1 is never used. */
        ti_reg_str(op2, c_op1, sizeof(c_op1));
        ti_reg_str(op3, c_op2, sizeof(c_op2));
        break;
    case TI_OP_CLR:
        ti_reg_str(op1, c_op1, sizeof(c_op1));
        ti_reg_str(op2, c_op2, sizeof(c_op2));
        break;
    /* Instructions with immediate values */
    case TI_OP_ADDI:
    case TI_OP_ADCI:
    case TI_OP_CLRI:
    case TI_OP_SUBI:
    case TI_OP_SUCI:
    case TI_OP_LSLI:
    case TI_OP_LSRI:
    case TI_OP_RSCI:
    case TI_OP_RSBI:
    case TI_OP_ORI:
    case TI_OP_XORI:
        snprintf(c_op1, sizeof(c_op1), "0x%x", op1);
        ti_reg_str(op2, c_op2, sizeof(c_op2));
        ti_reg_str(op3, c_op3, sizeof(c_op3));
        break;
    /* Special instructions. */
    case TI_OP_LDI:
        imm = (opcode & 0x00ffff00) >> 8;
        snprintf(c_op1, sizeof(c_op1), "0x%x", imm);
        ti_reg_str(op3, c_op3, sizeof(c_op3));
        break;
    case TI_OP_MVI:
        ti_reg_str(op3, c_op3, sizeof(c_op3));
        ti_reg_str(op2, c_op2, sizeof(c_op2));
        memmove(c_op2 + 1, c_op2, sizeof(c_op2));
        c_op2[0] = '*';
        break;
    case TI_OP_JMP:
        if (op1 != 0x9e)
            snprintf(c_op1, sizeof(c_op1), "r%d.w0", op1 - 0x80);
        break;
    case TI_OP_JAL:
        if (op3 == 0x9e) {
            snprintf(c_op1, sizeof(c_op1), "r%d.w0", op1 - 0x80);
        } else {
            snprintf(c_op1, sizeof(c_op1), "r%d.w0", op1 - 0xc0);
            snprintf(c_op3, sizeof(c_op3), "r%d.w0", op3 - 0x80);
        }
    case TI_OP_SLP:
        if (op1 == 0x80)
            snprintf(c_op1, sizeof(c_op1), "1");
        else
            snprintf(c_op1, sizeof(c_op1), "0");
    }
    /* N.B. the order of the arguments is reversed. */
    ti_std_ins(c_ins, c_op3, c_op2, c_op1, buf, len);

    return 0;
}

static int
ti_initialise_device(pru_t pru, const char* device)
{
    uint32_t i;

    for( i = 0; i < sizeof(initializers)/sizeof(initializers[0]); i++ )
    {
        if( strcmp( device, initializers[i].name ) == 0 )
            return initializers[i].initialize( pru );
    }

    fprintf(stderr, "Device not compatible, supported are: ");
    for( i = 0; i < sizeof(initializers)/sizeof(initializers[0]); i++ )
    {
        fprintf(stderr, "%s ", initializers[i].name );
    }
    fprintf(stderr, "\n");

    return -1;
}

int
ti_initialise(pru_t pru, const char* device)
{
    size_t i;
    int fd = 0;
    char dev[64];
    int result = 0;

    for (i = 0; i < 4; i++) {
        snprintf(dev, sizeof(dev), "/dev/pruss%zu", i);
        fd = open(dev, O_RDWR);
        if (fd == -1 && errno == EACCES)
            break;
        if (fd > 0)
            break;
    }
    if (fd < 0)
        return -EINVAL;

    pru->disassemble = ti_disassemble;
    pru->fd = fd;

    result = ti_initialise_device(pru, device);

    if( result != 0 )
    {
        close(fd);
        return -EINVAL;
    }

    return 0;
}
