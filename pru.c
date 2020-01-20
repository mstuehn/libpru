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
#include <stdlib.h>
#include <strings.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <Block.h>
#include <sched.h>
#include <pthread.h>
#include <pthread_np.h>
#include <string.h>

#include <sys/types.h>
#include <sys/rtprio.h>
#include <sys/event.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sched.h>

#include <libpru.h>
#include <pru-private.h>
#include <ti-pru.h>


int libpru_debug = 0;

static struct drivers
{
    const char* name;
    int (*initialize)( pru_t pru, const char* );
} initializers []  = {
        {
                .name = "ti",
                .initialize = ti_initialise,
        },
};

int pru_register_irq(pru_t pru, uint8_t irq, int8_t channel, int8_t event )
{
    return pru->register_irq(pru, irq, channel, event );
}

int pru_loop_irq(pru_t pru, uint8_t irq, handler_t on_irq)
{
    return pru->loop_irq(pru, irq, on_irq);
}

int pru_deregister_irq(pru_t pru, uint8_t irq )
{
    return pru->deregister_irq(pru, irq);
}

static const char*
pru_is_compatible( const char* device, const char* driver )
{
    while( *device == *driver )
	{
		++device; ++driver;
	}
    if( *device == ',' ) return device+1;
    else return NULL;
}

static int
pru_initialize(pru_t pru, const char* device)
{
    uint32_t i;

    for( i = 0; i < sizeof(initializers)/sizeof(initializers[0]); i++ )
    {
        const char* subtype = pru_is_compatible( device, initializers[i].name );
        if( subtype != NULL )
        {
            return initializers[i].initialize( pru, subtype );
        }
    }

    fprintf(stderr, "Manufacturer not compatible, supported are: ");
    for( i = 0; i < sizeof(initializers)/sizeof(initializers[0]); i++ )
    {
        fprintf(stderr, "%s ", initializers[i].name );
    }
    fprintf(stderr, "\n");

    return -1;
}

pru_t
pru_alloc(const char* device)
{
	pru_t pru;
	int saved_errno;
	char *pruenv;

	pruenv = getenv("PRU_DEBUG");
	if (pruenv)
		libpru_debug = atoi(pruenv);
	DPRINTF("type %s\n", device);
	pru = malloc(sizeof(*pru));
	if (pru == NULL)
		return NULL;
	bzero(pru, sizeof(*pru));

	if( pru_initialize( pru, device ) < 0 )
	{
		saved_errno = errno;
		free(pru);
		errno = saved_errno;
		return NULL;
	}

	DPRINTF("pru %p allocated and initialised\n", (void*)pru);

	return pru;
}

void
pru_free(pru_t pru)
{
	DPRINTF("pru %p\n", (void*)pru);

	pru->deinit(pru);
	free(pru);
}

int
pru_reset(pru_t pru, unsigned int pru_number)
{
	return pru->reset(pru, pru_number);
}

int
pru_disable(pru_t pru, unsigned int pru_number)
{
	return pru->disable(pru, pru_number);
}

int
pru_enable(pru_t pru, unsigned int pru_number, int single_step)
{
	return pru->enable(pru, pru_number, single_step);
}

int
pru_upload(pru_t pru, unsigned int pru_number, const char *file)
{
	int error, saved_errno;
	int fd;
	struct stat sb;
	char *buffer;

	DPRINTF("pru %d file %s\n", pru_number, file);
	fd = open(file, O_RDONLY);
	if (fd < 0)
		return errno;
	if (fstat(fd, &sb) != 0) {
		saved_errno = errno;
		close(fd);
		return saved_errno;
	}
	buffer = mmap(0, (size_t)sb.st_size, PROT_READ, MAP_SHARED, fd, 0);
	if (buffer == MAP_FAILED) {
		saved_errno = errno;
		close(fd);
		return saved_errno;
	}
	error = pru->upload_buffer(pru, pru_number, buffer,
	    (size_t)sb.st_size);
	munmap(buffer, (size_t)sb.st_size);
	close(fd);

	return error;
}

int
pru_upload_buffer(pru_t pru, unsigned int pru_number, const char *buffer,
    size_t len)
{
    DPRINTF("pru %d buffer %p len %zu\n", pru_number, (const void*)buffer, len);
    return pru->upload_buffer(pru, pru_number, buffer, len);
}

int
pru_wait(pru_t pru, unsigned int pru_number)
{
	return pru->wait(pru, pru_number);
}

uint8_t
pru_read_mem(pru_t pru, unsigned int pru_number, uint32_t mem)
{
	return pru->read_mem(pru, pru_number, mem);
}

uint32_t
pru_read_imem(pru_t pru, unsigned int pru_number, uint32_t mem)
{
	return pru->read_imem(pru, pru_number, mem);
}

int
pru_write_imem(pru_t pru, unsigned int pru_number, uint32_t mem, uint32_t ins)
{
	return pru->write_imem(pru, pru_number, mem, ins);
}

uint32_t
pru_read_reg(pru_t pru, unsigned int pru_number, enum pru_reg reg)
{
	if (reg == REG_PC)
		return pru->get_pc(pru, pru_number);
	else
		return pru->read_reg(pru, pru_number, reg);
}

int
pru_write_reg(pru_t pru, unsigned int pru_number, enum pru_reg reg,
    uint32_t val)
{
	if (reg == REG_PC)
		return pru->set_pc(pru, pru_number, (uint16_t)val);
	else
		return pru->write_reg(pru, pru_number, reg, val);
}

int
pru_disassemble(pru_t pru, uint32_t opcode, char *buf, size_t len)
{
	return pru->disassemble(pru, opcode, buf, len);
}

int
pru_insert_breakpoint(pru_t pru, unsigned int pru_number, uint32_t pc,
    uint32_t *orig_ins)
{
	return pru->insert_breakpoint(pru, pru_number, pc, orig_ins);
}
