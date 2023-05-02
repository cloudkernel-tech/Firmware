/**
 * @file BMM150_SPI.cpp
 *
 * SPI interface for BMM150 magnetometer
 */

#include <px4_platform_common/px4_config.h>
#include <drivers/device/spi.h>
#include <px4_platform_common/defines.h>

#include "Bosch_BMM150_registers.hpp"

using namespace Bosch_BMM150;

/* SPI protocol address bits */
#define DIR_READ        (1<<7)
#define DIR_WRITE       (0<<7)

device::Device *BMM150_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode);

class BMM150_SPI : public device::SPI
{
public:
    BMM150_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
    virtual ~BMM150_SPI() = default;

    virtual int	init();

    uint8_t RegisterRead(Register reg);
    void RegisterWrite(Register reg, uint8_t value);

    virtual int	read(unsigned address, void *data, unsigned count);
    virtual int	write(unsigned address, void *data, unsigned count);

protected:
    virtual int	probe();

};

device::Device *
BMM150_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode)
{
    return new BMM150_SPI(bus, devid, bus_frequency, spi_mode);
}


BMM150_SPI::BMM150_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
    SPI(DRV_MAG_DEVTYPE_BMM150, MODULE_NAME, bus, device, spi_mode, bus_frequency)
{
}

int BMM150_SPI::init()
{
    int ret;

    ret = SPI::init();

    if (ret != PX4_OK) {
        DEVICE_DEBUG("SPI init failed");
        return PX4_ERROR;
    }

    return ret;
}

int BMM150_SPI::probe()
{
    //chip probe: 3 retries
    for (int i = 0; i < 3; i++) {
        const uint8_t POWER_CONTROL = RegisterRead(Register::POWER_CONTROL);
        const uint8_t CHIP_ID = RegisterRead(Register::CHIP_ID);

        PX4_DEBUG("POWER_CONTROL: 0x%02hhX, CHIP_ID: 0x%02hhX", POWER_CONTROL, CHIP_ID);

        if (CHIP_ID == chip_identification_number) {
            return PX4_OK;

        } else if (!(POWER_CONTROL & POWER_CONTROL_BIT::PowerControl)) {
            // in suspend Chip ID read (register 0x40) returns “0x00” (I²C) or high-Z (SPI).
            PX4_INFO("Chip ID read high-z state in SPI");
            return PX4_OK;
        }
    }

    return PX4_ERROR;

}

uint8_t BMM150_SPI::RegisterRead(Register reg)
{
    const uint8_t cmd = static_cast<uint8_t>(reg);

    uint8_t buffer{};
    int ret = read(cmd, &buffer, 1);

    if (ret != PX4_OK) {
        PX4_DEBUG("register read 0x%02hhX failed, ret = %d", cmd, ret);
        return -1;
    }

    return buffer;
}

void BMM150_SPI::RegisterWrite(Register reg, uint8_t value)
{
    const uint8_t cmd = static_cast<uint8_t>(reg);

    int ret = write(cmd, &value, 1);

    if (ret != PX4_OK) {
        PX4_DEBUG("register write 0x%02hhX failed, ret = %d", cmd, ret);
    }
}


int BMM150_SPI::read(unsigned address, void *data, unsigned count)
{
    uint8_t buf[32];

    if (sizeof(buf) < (count + 1)) {
        return -EIO;
    }

    buf[0] = address | DIR_READ;

    int ret = transfer(&buf[0], &buf[0], count + 1);
    memcpy(data, &buf[1], count);
    return ret;
}

int BMM150_SPI::write(unsigned address, void *data, unsigned count)
{
    uint8_t buf[32];

    if (sizeof(buf) < (count + 1)) {
        return -EIO;
    }

    buf[0] = address | DIR_WRITE;
    memcpy(&buf[1], data, count);

    return transfer(&buf[0], &buf[0], count + 1);
}
