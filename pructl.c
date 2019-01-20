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
#include <libpru.h>

static void __attribute__((noreturn))
usage(void)
{
	fprintf(stderr, "usage: %s -t type [-p pru-number] [-edrw] [program]\n",
	    getprogname());
	exit(1);
}

int
main(int argc, char **argv)
{
	int ch;
	int reset, enable, disable, wait, callback;
	const char *type = "ti";
	unsigned int pru_number;
	pru_t pru;
    int error;

  reset = enable = disable = pru_number = wait = callback = 0;
	type = NULL;
	error = 0;
	while ((ch = getopt(argc, argv, "t:p:edrwc")) != -1) {
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
/* Example code for interrupts
	if( callback )
    {
        __block uint64_t last;
        __block uint64_t last_diff;
        struct timespec time;

        pru_register_irq( pru, 3, 3, 17,
                ^(uint64_t ts){
                //struct timespec diff;
                //clock_gettime( CLOCK_MONOTONIC, &time );
                //diff.tv_nsec = time.tv_nsec - ts%1000000000;
                //diff.tv_sec = time.tv_sec - ts/1000000000;
                uint64_t diff = ts - last;
                printf("TS: %lli.%09lli duration (%llins/%lli.%0lli) \n",
                        //diff.tv_sec, diff.tv_nsec);
                        ts/1000000000, ts%1000000000,
                        diff,
                        (int64_t)(diff - last_diff)/1000,
                        (diff - last_diff)%1000
                );
        last = ts;
        last_diff = diff;
                }
        );
        sleep(10);

        pru_deregister_irq(pru, 3 );

    }
    */
	pru_free(pru);
}
