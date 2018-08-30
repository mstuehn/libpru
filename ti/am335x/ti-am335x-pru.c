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

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <stdbool.h>

#include <pthread.h>
#include <pthread_np.h>
#include <sys/rtprio.h>
#include <sys/errno.h>
#include <sys/types.h>
#include <sys/event.h>
#include <sys/mman.h>
#include <sys/cdefs.h>
#include <sys/sysctl.h>
#include <sched.h>
#include <assert.h>

#include "libpru.h"
#include "pru-private.h"
#include "ti-pru.h"
#include "ti-common.h"

#include "ti-am335x-pru.h"

static const char glob_irq_enable[] =  "dev.ti_pruss.0.global_interrupt_enable";
static const char sysctlenable[] = "dev.ti_pruss.0.irq.%i.enable";
static const char sysctlevent[] = "dev.ti_pruss.0.irq.%i.event";
static const char sysctlchannel[] = "dev.ti_pruss.0.irq.%i.channel";
static const char path_format[] = "/dev/pruss0.irq%i";

struct irq_data
{
    int fd;
    int kq;
    pthread_t thread;
    handler_t intr_handler;
};

struct am335x_pru_priv
{
    struct irq_data irqs[AM33XX_NUM_HOST_INTS];
};

static int
am335x_disable(pru_t pru, unsigned int pru_number)
{
    const unsigned int reg = AM33XX_PRUnCTL(pru_number);
    assert( pru_number < AM33XX_NUM_PRUSS );

    ti_reg_clr(pru->mem, reg, CTL_REG_ENABLE);

    return 0;
}

static int
am335x_enable(pru_t pru, unsigned int pru_number, int single_step)
{
    uint32_t val;
    const unsigned int reg = AM33XX_PRUnCTL(pru_number);
    assert( pru_number < AM33XX_NUM_PRUSS );

    val = ti_reg_read_4(pru->mem, reg);
    if (single_step)
        val |= CTL_REG_SINGLE_STEP;
    else
        val &= ~CTL_REG_SINGLE_STEP;
    val |= CTL_REG_ENABLE;
    ti_reg_write_4(pru->mem, reg, val);

    return 0;
}

static int
am335x_reset(pru_t pru, unsigned int pru_number)
{
    const unsigned int reg = AM33XX_PRUnCTL(pru_number);
    assert( pru_number < AM33XX_NUM_PRUSS );

    ti_reg_clr(pru->mem, reg, CTL_REG_RESET);
    ti_reg_set(pru->mem, reg, CTL_REG_COUNTER);
    /* Set the PC. */
    ti_reg_clr(pru->mem, reg, (0xffff0000));

    return 0;
}

static int
am335x_upload(pru_t pru, unsigned int pru_number, const char *buffer, size_t size)
{
    assert( pru_number < AM33XX_NUM_PRUSS );

    uint8_t *iram;
    if (size > AM33XX_IRAM_SIZE ) {
        errno = EFBIG;
        return -1;
    }
    iram = pru->mem + AM33XX_PRUnIRAM(pru_number);

    DPRINTF("IRAM at %p\n", (void*)iram);
    memset(iram, 0, AM33XX_IRAM_SIZE);

    DPRINTF("copying buf %p size %zu\n", (const void*)buffer, size);
    memcpy(iram, buffer, size);

    return 0;
}

static int
am335x_wait(pru_t pru, unsigned int pru_number)
{
    const unsigned int reg = AM33XX_PRUnCTL(pru_number);
    struct timespec ts;
    int i;
    assert( pru_number < AM33XX_NUM_PRUSS );

    /* 0.01 seconds */
    ts.tv_nsec = 10000000;
    ts.tv_sec = 0;

    /*
     * Wait for the PRU to start running.
     */
    i = 0;
    while (i < 10 && !(ti_reg_read_4(pru->mem, reg) & CTL_REG_RUNSTATE)) {
        nanosleep(&ts, NULL);
        i++;
    }
    if (i == 10)
        return -1;
    while (ti_reg_read_4(pru->mem, reg) & CTL_REG_RUNSTATE)
        nanosleep(&ts, NULL);

    return 0;
}

static int
am335x_irq_ctrl( uint8_t irq, int8_t channel, int8_t sysevent, bool enable)
{
    int result;
    char sysctlname[128];
    char sysctl_value[5];

    snprintf(sysctlname, sizeof(sysctlname), sysctlchannel, irq);
    if( enable )
        snprintf(sysctl_value, sizeof(sysctl_value), "%i", channel);
    else
        snprintf(sysctl_value, sizeof(sysctl_value), "%s", "NONE");

    printf("sysctl: %s %s\n", sysctlname, sysctl_value);
    result = sysctlbyname(sysctlname, NULL, 0, &sysctl_value, strlen(sysctl_value));
    if( result ){
        fprintf(stderr, "Failed to sysctl: %s %s -> %i\n", sysctlname, sysctl_value, result);
        return result;
    }

    snprintf(sysctlname, sizeof(sysctlname), sysctlevent, irq);
    if( enable )
        snprintf(sysctl_value, sizeof(sysctl_value), "%i", sysevent);
    else
        snprintf(sysctl_value, sizeof(sysctl_value), "%s", "NONE");

    printf("sysctl: %s %s\n", sysctlname, sysctl_value);
    result = sysctlbyname(sysctlname, NULL, 0, &sysctl_value, strlen(sysctl_value));
    if( result ){
        fprintf(stderr, "Failed to sysctl: %s %s -> %i\n", sysctlname, sysctl_value, result);
        return result;
    }

    snprintf(sysctlname, sizeof(sysctlname), sysctlenable, irq);
    printf("sysctl: %s %s\n", sysctlname, sysctl_value);
    result = sysctlbyname(sysctlname, NULL, 0, &enable, sizeof(enable));
    if( result ){
        fprintf(stderr, "Failed to sysctl: %s %s -> %i\n", sysctlname, sysctl_value, result);
        return result;
    }

    printf("sysctl: %s %i\n", sysctlname, enable);
    if( enable ) return sysctlbyname(glob_irq_enable, NULL, 0, &enable, sizeof(enable));
    else return result;
}

static void*
am335x_handle_intr( void* irq )
{
    struct irq_data* priv = irq;

    struct rtprio rtp = { .type = RTP_PRIO_REALTIME, .prio = 20 } ;
    int ret = rtprio_thread(RTP_SET, 0, &rtp );
    printf("setting rtprio %i\n", ret );

    struct kevent change;
    struct kevent event;

    struct timespec timeout;
    timeout.tv_sec = 1;
    timeout.tv_nsec = 0;

    for(;;)
    {
        if(priv->fd == -1 ) return NULL;

        EV_SET(&change, priv->fd, EVFILT_READ, EV_ADD | EV_CLEAR, 0, 0, NULL);

        int num_events = kevent(priv->kq, &change, 1, &event, 1, &timeout);

        if( num_events == 0 ) continue; // timeout
        if( num_events < 0 ) return NULL;

        if( event.flags & EVFILT_READ )
        {
            uint64_t timestamp;
            while( read( priv->fd, &timestamp, sizeof(timestamp) ) > 0 )
            {
                priv->intr_handler(timestamp);
            }
        }
    }

    return NULL;
}

static int
am335x_register_irq(pru_t pru, uint8_t irq, int8_t channel, int8_t sysevent, handler_t handler)
{
    struct am335x_pru_priv* priv = pru->priv;
    if( irq > 9 || channel > 9 || sysevent > 63 )
        return -1;
    ASSIGN_FUNC(priv->irqs[irq].intr_handler, handler);

    int result = am335x_irq_ctrl( irq, channel, sysevent, true);
    if( result < 0 ) return result;

    char path[32];
    snprintf(path, sizeof(path), path_format , irq);
    priv->irqs[irq].fd = open( path, O_RDONLY | O_NONBLOCK );

    printf("Opening device %s returned %i\n", path, priv->irqs[irq].fd);

    if (pthread_create(&priv->irqs[irq].thread, NULL, am335x_handle_intr, &priv->irqs[irq]) != 0) {
        pru_free(pru);
        errno = EINVAL; /* XXX */
        return -1;
    }
    pthread_set_name_np( priv->irqs[irq].thread, "pru_isr" );

    priv->irqs[irq].kq = kqueue();
    if( priv->irqs[irq].kq < 0 )
    {
        errno = ENXIO;
        return -1;
    }

    return priv->irqs[irq].fd;
}

static int
am335x_deregister_irq(pru_t pru, uint8_t irq)
{
    struct am335x_pru_priv* priv = pru->priv;
    RELASE_FUNC(priv->irqs[irq].intr_handler);
    priv->irqs[irq].fd = -1;

    pthread_cancel( priv->irqs[irq].thread );
    pthread_join( priv->irqs[irq].thread, NULL );

    close( priv->irqs[irq].kq );

    return am335x_irq_ctrl( irq, 0, 0, false);
}


static int
am335x_deinit(pru_t pru)
{
    for(unsigned int i = 0 ; i < AM33XX_NUM_PRUSS; i++ )
    {
        am335x_disable(pru, i);
        am335x_reset(pru, i);
    }

    if (pru->mem != MAP_FAILED)
        munmap(pru->mem, pru->mem_size);

    if( pru->priv )
    {
        struct am335x_pru_priv* priv = pru->priv;
        for( int i = 0; i < AM33XX_NUM_HOST_INTS; i++)
        {
            if( priv->irqs[i].fd != -1 )
                close(priv->irqs[i].fd);
        }

        free(priv);
        priv = NULL;
    }
    close(pru->fd);

    return 0;
}

static uint32_t
am335x_read_imem(pru_t pru, unsigned int pru_number, uint32_t mem)
{
    const unsigned int reg = AM33XX_PRUnIRAM(pru_number);
    assert( pru_number < AM33XX_NUM_PRUSS );

    /* XXX missing bounds check. */
    return ti_reg_read_4(pru->mem, reg + mem);
}

static int
am335x_write_imem(pru_t pru, unsigned int pru_number, uint32_t mem, uint32_t ins)
{
    const unsigned int reg = AM33XX_PRUnIRAM(pru_number);
    assert( pru_number < AM33XX_NUM_PRUSS );

    /* XXX missing bounds check. */
    ti_reg_write_4(pru->mem, reg + mem, ins);

    return 0;
}

static uint8_t
am335x_read_mem(pru_t pru, unsigned int pru_number, uint32_t mem)
{
    const unsigned int reg = AM33XX_PRUnRAM(pru_number);
    assert( pru_number < AM33XX_NUM_PRUSS );

    /* XXX missing bounds check. */
    return (ti_reg_read_1(pru->mem, reg + mem));
}


static uint32_t
am335x_read_reg(pru_t pru, unsigned int pru_number, uint32_t reg)
{
    assert( pru_number < AM33XX_NUM_PRUSS );
    const uint32_t base = AM33XX_PRUnDBG(pru_number);
    return ti_reg_read_4(pru->mem, base + reg * 4);
}

static int
am335x_write_reg(pru_t pru, unsigned int pru_number, uint32_t reg, uint32_t val)
{
    assert( pru_number < AM33XX_NUM_PRUSS );
    const uint32_t base = AM33XX_PRUnDBG(pru_number);
    ti_reg_write_4(pru->mem, base + reg * 4, val);

    return 0;
}

static uint16_t
am335x_get_pc(pru_t pru, unsigned int pru_number)
{
    assert( pru_number < AM33XX_NUM_PRUSS );
    const uint32_t reg = AM33XX_PRUnCTL(pru_number) + CTL_REG_STS;
    return (ti_reg_read_4(pru->mem, reg ) & 0xffff) * 4;
}

static int
am335x_set_pc(pru_t pru, unsigned int pru_number, uint16_t pc)
{
    uint32_t val;

    assert( pru_number < AM33XX_NUM_PRUSS );
    const uint32_t reg = AM33XX_PRUnCTL(pru_number) + CTL_REG_STS;

    val = ti_reg_read_4(pru->mem, reg);
    val &= 0x0000ffff;
    val |= (uint32_t)pc << 16;
    ti_reg_write_4(pru->mem, reg, val);

    return 0;
}

static int
am335x_insert_breakpoint(pru_t pru, unsigned int pru_number, uint32_t pc,
    uint32_t *orig_ins)
{
    if (orig_ins)
        *orig_ins = am335x_read_imem(pru, pru_number, pc);
    DPRINTF("inserting breakpoint: pc 0x%x, ins 0x%x\n", pc,
        orig_ins ? *orig_ins : 0);
    am335x_write_imem(pru, pru_number, pc, TI_OP_HALT << 24);

    return 0;
}


int32_t
am335x_initialize( pru_t pru )
{
    int saved_errno;

    DPRINTF("Mapping device %i\n", 0);
    pru->mem = mmap(0, AM33XX_MMAP_SIZE, PROT_READ|PROT_WRITE,
        MAP_SHARED, pru->fd, 0);
    saved_errno = errno;

    if (pru->mem == MAP_FAILED)
    {
        DPRINTF("mmap failed %d\n", saved_errno);
        errno = saved_errno;
        return -1;
    }

    pru->mem_size = AM33XX_MMAP_SIZE;
    if( ti_reg_read_4(pru->mem, AM33XX_INTC_REG) != AM33XX_REV )
    {
        munmap(pru->mem, pru->mem_size);
        errno = ENXIO;
        return -1;
    }

    pru->disable = am335x_disable;
    pru->enable = am335x_enable;
    pru->reset = am335x_reset;
    pru->upload_buffer = am335x_upload;
    pru->wait = am335x_wait;
    pru->deinit = am335x_deinit;
    pru->read_imem = am335x_read_imem;
    pru->write_imem = am335x_write_imem;
    pru->read_mem = am335x_read_mem;
    pru->read_reg = am335x_read_reg;
    pru->write_reg = am335x_write_reg;
    pru->get_pc = am335x_get_pc;
    pru->set_pc = am335x_set_pc;
    pru->insert_breakpoint = am335x_insert_breakpoint;
    pru->register_irq = am335x_register_irq;
    pru->deregister_irq = am335x_deregister_irq;

    struct am335x_pru_priv *priv = malloc(sizeof(struct am335x_pru_priv));
    if( priv == NULL ) {
        errno = ENOMEM;
        return -1;
    }

    pru->priv = priv;

    DPRINTF("found AM335x PRU @ %p\n", (void*)pru->mem);

    for(unsigned int i = 0 ; i < AM33XX_NUM_PRUSS; i++ )
    {
        am335x_disable(pru, i);
        am335x_reset(pru, i);
    }
    return 0;
}
