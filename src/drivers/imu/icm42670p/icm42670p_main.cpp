/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ICM42670P.hpp"

namespace icm42670p
{

//Bus options for icm42670p for extendable interfaces
struct icm42670_bus_option {
    enum ICM42670P_BUS busid;
    uint8_t busnum;
    uint32_t address;
    ICM42670P	*dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_ICM_42670P)
    {ICM42670P_BUS_SPI_INTERNAL, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_ICM_42670P, NULL},
#endif
    {ICM42670P_BUS_SPI_EXTERNAL, PX4_SPI_BUS_EXT, PX4_SPIDEV_ICM_42670P, NULL}
};

#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

//driver interface functions
void    start(enum ICM42670P_BUS bus_id, enum Rotation rotation);
bool    start_bus(struct icm42670_bus_option &bus, enum Rotation rotation);
struct icm42670_bus_option &find_bus(enum ICM42670P_BUS busid);
void    stop(enum ICM42670P_BUS bus_id);
void    info(enum ICM42670P_BUS bus_id);
void    usage();



void start(enum ICM42670P_BUS bus_id, enum Rotation rotation)
{

    bool started = false;

    for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
        if (bus_options[i].dev != nullptr) {
            // this device is already started
            continue;
        }

        if (bus_id != ICM42670P_BUS_ALL && bus_options[i].busid != bus_id) {
            // not the one that is asked for
            continue;
        }

        started |= start_bus(bus_options[i], rotation);

        if (started) { break; }
    }

    exit(started ? 0 : 1);

}



bool start_bus(struct icm42670_bus_option &bus, enum Rotation rotation)
{
    if (bus.dev != nullptr) {
        warnx("SPI not available");
        return false;
    }

    bus.dev = new ICM42670P(bus.busnum, bus.address, rotation);

    if (bus.dev == nullptr) {
        return false;
    }

    if (OK != bus.dev->init()) {
        goto fail;
    }

    return true;

fail:

    if (bus.dev != nullptr) {
        delete bus.dev;
        bus.dev = nullptr;
    }

    return false;

}


/**
 * find a bus structure for a busid
 */
struct icm42670_bus_option &find_bus(enum ICM42670P_BUS busid)
{
    for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
        if ((busid == ICM42670P_BUS_ALL ||
             busid == bus_options[i].busid) && bus_options[i].dev != nullptr) {
            return bus_options[i];
        }
    }

    errx(1, "bus %u not started", (unsigned)busid);
}


void stop(enum ICM42670P_BUS bus_id)
{
    struct icm42670_bus_option &bus = find_bus(bus_id);

    if (bus.dev != nullptr) {
        delete bus.dev;
        bus.dev = nullptr;

    } else {
        /* warn, but not an error */
        warnx("driver already stopped.");
    }

    exit(0);

}

void info(enum ICM42670P_BUS bus_id)
{
    struct icm42670_bus_option &bus = find_bus(bus_id);

    if (bus.dev == nullptr) {
        errx(1, "driver not running");
    }

    printf("state @ %p\n", bus.dev);
    bus.dev->print_info();

    exit(0);
}


void usage()
{
    warnx("missing command: try 'start', 'info/status', 'stop' ");
    warnx("options:");
    warnx("    -S   external SPI bus");
    warnx("    -s   internal SPI bus");
    warnx("    -R   rotation");
}

} // namespace


/** driver 'main' command */
extern "C" { __EXPORT int icm42670p_main(int argc, char *argv[]); }


int icm42670p_main(int argc, char *argv[])
{
    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    enum ICM42670P_BUS bus_id = ICM42670P_BUS_ALL;
    enum Rotation rotation = ROTATION_NONE;

    while ((ch = px4_getopt(argc, argv, "SsR:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {

        case 's':
            bus_id = ICM42670P_BUS_SPI_INTERNAL;
            break;

        case 'S':
            bus_id = ICM42670P_BUS_SPI_EXTERNAL;
            break;

        case 'R':
            rotation = (enum Rotation)atoi(myoptarg);
            break;

        default:
            icm42670p::usage();
            return 0;

        }
    }

    if (myoptind >= argc) {
        icm42670p::usage();
        return -1;
    }

    const char *verb = argv[myoptind];

    /*
     * Start/load the driver.
     */
    if (!strcmp(verb, "start")) {
        icm42670p::start(bus_id, rotation);
    }

    if (!strcmp(verb, "stop")) {
        icm42670p::stop(bus_id);
    }

    /*
     * Print driver information.
     */
    if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
        icm42670p::info(bus_id);
    }

    icm42670p::usage();
	return -1;
}
