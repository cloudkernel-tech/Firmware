/**
 * @file BMM150_I2C.cpp
 *
 * I2C interface for BMM150 magnetometer
 */


#include <px4_platform_common/px4_config.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/defines.h>

#include "Bosch_BMM150_registers.hpp"

using namespace Bosch_BMM150;

device::Device *BMM150_I2C_interface(int bus, int bus_frequency);

class BMM150_I2C : public device::I2C
{
public:
    BMM150_I2C(int bus, int bus_frequency);
    virtual ~BMM150_I2C() = default;

    uint8_t RegisterRead(Register reg);
    void RegisterWrite(Register reg, uint8_t value);

    virtual int	read(unsigned address, void *data, unsigned count);
    virtual int	write(unsigned address, void *data, unsigned count);

protected:
    virtual int	probe();
};


device::Device *
BMM150_I2C_interface(int bus, int bus_frequency)
{
    return new BMM150_I2C(bus, bus_frequency);
}

BMM150_I2C::BMM150_I2C(int bus, int bus_frequency) :
    I2C(DRV_MAG_DEVTYPE_BMM150, MODULE_NAME, bus, I2C_ADDRESS_DEFAULT, bus_frequency)
{
}


int BMM150_I2C::probe()
{
    // 3 retries
    for (int i = 0; i < 3; i++) {
        const uint8_t POWER_CONTROL = RegisterRead(Register::POWER_CONTROL);
        const uint8_t CHIP_ID = RegisterRead(Register::CHIP_ID);

        PX4_DEBUG("POWER_CONTROL: 0x%02hhX, CHIP_ID: 0x%02hhX", POWER_CONTROL, CHIP_ID);

        if (CHIP_ID == chip_identification_number) {
            return PX4_OK;

        } else if ((CHIP_ID == 0) && !(POWER_CONTROL & POWER_CONTROL_BIT::PowerControl)) {
            // in suspend Chip ID read (register 0x40) returns “0x00” (I²C) or high-Z (SPI).
            return PX4_OK;
        }
    }

    return PX4_ERROR;

}


uint8_t BMM150_I2C::RegisterRead(Register reg)
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

void BMM150_I2C::RegisterWrite(Register reg, uint8_t value)
{
    const uint8_t cmd = static_cast<uint8_t>(reg);

    int ret = write(cmd, &value, 1);

    if (ret != PX4_OK) {
        PX4_DEBUG("register write 0x%02hhX failed, ret = %d", cmd, ret);
    }
}

int BMM150_I2C::read(unsigned address, void *data, unsigned count)
{
    uint8_t cmd = address;
    return transfer(&cmd, 1, (uint8_t *)data, count);
}

int BMM150_I2C::write(unsigned address, void *data, unsigned count)
{
    uint8_t buf[32];

    if (sizeof(buf) < (count + 1)) {
        return -EIO;
    }

    buf[0] = address;
    memcpy(&buf[1], data, count);

    return transfer(&buf[0], count + 1, nullptr, 0);
}


