
#pragma once

#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <drivers/device/spi.h>

constexpr uint8_t BME688_I2C_ADDR = 0x77;

namespace bme688
{

class IBME688 {
public:
    virtual ~IBME688() = default;

    virtual int init() = 0;

    virtual void delay_us(uint32_t period) = 0;

    virtual int8_t write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len) = 0;

    virtual int8_t read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len) = 0;

    virtual uint32_t get_device_id() const = 0;

    virtual uint8_t get_device_address() const = 0;

    virtual void set_device_type(uint8_t devtype) = 0;
};

} /* namespace */

/* interface factories */
#if defined(CONFIG_SPI)
extern bme688::IBME688 *bme688_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
#endif // CONFIG_SPI
#if defined(CONFIG_I2C)
extern bme688::IBME688 *bme688_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency);
#endif // CONFIG_I2C
