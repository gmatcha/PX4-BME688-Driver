
#include "bme68x-sensor-api/bme68x.h"
#include "bme688.h"

#include "BME688.hpp"

#include <px4_platform_common/px4_config.h>
#include <drivers/device/spi.h>

#if defined(CONFIG_SPI)

/* SPI protocol address bits */
#define DIR_READ			(1<<7)  //for set
#define DIR_WRITE			~(1<<7) //for clear

struct __attribute__((__packed__)) spi_data_s {
    uint8_t addr;
    uint8_t data[1024];
};

class BME688_SPI: public device::SPI, public bme688::IBME688 {
public:
    BME688_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
    virtual ~BME688_SPI() override = default;

    void delay_us(uint32_t period) override { system_usleep(period); }

    int8_t write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len) override {
        int8_t trans_bytes;
        uint8_t data[1 + len];
        data[0] = (uint8_t) (reg_addr & DIR_WRITE);
        for (uint32_t i = 0; i < len; i++) {
            data[i + 1] = reg_data[i];
        }
        trans_bytes = transfer(data, nullptr, sizeof(data));
        return trans_bytes;
    }

    int8_t read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len) override {
        _data.addr = (uint8_t) (reg_addr | DIR_READ);
        if (transfer((uint8_t*)&_data, (uint8_t*)&_data, len+1) == OK) {
            memcpy(reg_data, _data.data, len);
            return 0;
        } else {
            return -1;
        }
    }

    int init() override { return SPI::init(); }

    uint32_t get_device_id() const override { return device::SPI::get_device_id(); }

    uint8_t get_device_address() const override { return device::SPI::get_device_address(); }

    void set_device_type(uint8_t devtype) {
        device::Device::set_device_type(devtype);
    }

private:
    spi_data_s _data{};
};

bme688::IBME688 *
bme688_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode)
{
    return new BME688_SPI(busnum, device, bus_frequency, spi_mode);
};

BME688_SPI::BME688_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
        SPI(DRV_BARO_DEVTYPE_BME688, MODULE_NAME, bus, device, spi_mode, bus_frequency)
{
};

#endif // CONFIG_SPI

