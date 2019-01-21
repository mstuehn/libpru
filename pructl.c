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
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <inttypes.h>
#include <libpru.h>

static bool run = true;

static void sighandler( int signal )
{
    if( signal == SIGINT ) run = false;
}

static bool callback_function( uint64_t ts )
{
    uint64_t last;
    uint64_t last_diff;
    uint64_t diff = ts - last;
    printf("TS: %" PRIu64 ".%09"PRIu64 "duration (%"PRIu64"ns/%"PRIu64".%0"PRIu64") \n",
           //diff.tv_sec, diff.tv_nsec);
           ts/1000000000, ts%1000000000,
           diff,
           (int64_t)(diff - last_diff)/1000,
           (diff - last_diff)%1000 );
    last = ts;
    last_diff = diff;
    return run;
}

static void __attribute__((noreturn))
usage(void)
{
    fprintf(stderr, "usage: %s -t type [-p pru-number] [-i irq] [-v event] [-edrwc] [program]\n",
        getprogname());
	exit(1);
}

int
main(int argc, char **argv)
{
	int ch;
	int reset, enable, disable, wait, callback, reg_irq, unreg_irq;
	const char *type = "ti";
	unsigned int pru_number;
	int8_t irq_number, event_number;
	pru_t pru;
	int error;

	reset = enable = disable = pru_number = wait = callback = reg_irq = unreg_irq = 0;
	irq_number = event_number = -1;
	type = NULL;
	error = 0;
	while ((ch = getopt(argc, argv, "t:p:edrwci:v:u")) != -1) {
		switch (ch) {
		case 't':
			type = optarg;
			break;
		case 'p':
			pru_number = (unsigned int)strtoul(optarg, NULL, 10);
			break;
		case 'e':
			enable = 1;
			break;
		case 'd':
			disable = 1;
			break;
		case 'r':
			reset = 1;
			break;
		case 'w':
			wait = 1;
			break;
		case 'c':
			callback = 1;
			break;
		case 'i':
			irq_number = (unsigned int)strtoul(optarg, NULL, 10);
			break;
		case 'v':
			event_number = (unsigned int)strtoul(optarg, NULL, 10);
			break;
		case 'u':
			unreg_irq = 1;
			break;
		case '?':
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;
	if (enable && disable) {
		fprintf(stderr, "%s: conflicting options: -e and -d\n",
		    getprogname());
		usage();
	}
	if (type == NULL) {
		fprintf(stderr, "%s: missing type (-t)\n", getprogname());
		usage();
	}

	pru = pru_alloc(type);
	if (pru == NULL) {
		fprintf(stderr, "%s: unable to allocate PRU structure: %s\n",
		    getprogname(), strerror(errno));
		return 3;
	}
	if (reset) {
		error = pru_reset(pru, pru_number);
		if (error) {
			fprintf(stderr, "%s: unable to reset PRU %d\n",
			    getprogname(), pru_number);
			return 4;
		}
	}
	if (argc > 0) {
		error = pru_upload(pru, pru_number, argv[0]);
		if (error) {
			fprintf(stderr, "%s: unable to upload %s: %s\n",
			    getprogname(), argv[0], strerror(errno));
			return 5;
		}
	}
	if (enable) {
		error = pru_enable(pru, pru_number, 0);
		if (error) {
			fprintf(stderr, "%s: unable to enable PRU %d\n",
			    getprogname(), pru_number);
			return 6;
		}
	}
	if (disable) {
		error = pru_disable(pru, pru_number);
		if (error) {
			fprintf(stderr, "%s: unable to disable PRU %d\n",
			    getprogname(), pru_number);
			return 7;
		}
	}
	if (wait) {
		error = pru_wait(pru, pru_number);
		if (error) {
			fprintf(stderr, "%s: unable to wait for PRU %d\n",
			    getprogname(), pru_number);
			return 8;
		}
	}
	if( irq_number > 0 && event_number > 0 )
	{
		pru_register_irq( pru, irq_number, irq_number, event_number );
	}
	if( callback && irq_number > 0 ) {
		signal( SIGINT, sighandler );
		pru_wait_irq( pru, irq_number, callback_function );
	}
	if( unreg_irq && irq_number > 0 )
	{
		pru_deregister_irq(pru, irq_number );
	}

	pru_free(pru);
}
