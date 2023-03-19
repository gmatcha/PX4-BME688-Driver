
#include "bme68x-sensor-api/bme68x.h"
#include "bme688.h"

#include "BME688.hpp"

#include <px4_platform_common/px4_config.h>
#include <drivers/device/i2c.h>

#if defined(CONFIG_I2C)

class BME688_I2C: public device::I2C, public bme688::IBME688 {
public:
    BME688_I2C(uint8_t bus, uint32_t device, int bus_frequency);

    virtual ~BME688_I2C() override = default;

    void delay_us(uint32_t period) override { system_usleep(period); }

    int8_t write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len) override {
        int8_t trans_bytes;
        uint8_t data[1 + len];
        data[0] = reg_addr;
        for (uint32_t i = 0; i < len; i++) {
            data[i + 1] = reg_data[i];
        }
        trans_bytes = transfer(data, sizeof(data), nullptr, 0);
        return trans_bytes;
    }

    int8_t read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len) override {
        if (transfer(&reg_addr, sizeof(reg_addr), (uint8_t *) reg_data, len) == OK) {
            return 0;
        } else {
            return -1;
        }
    }

    int init() override { return I2C::init(); }

	uint32_t get_device_id() const override { return device::I2C::get_device_id(); }

	uint8_t get_device_address() const override { return device::I2C::get_device_address(); }

	void set_device_type(uint8_t devtype) {
	    device::Device::set_device_type(devtype);
	}
};

bme688::IBME688 *bme688_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency)
{
	return new BME688_I2C(busnum, device, bus_frequency);
}

BME688_I2C::BME688_I2C(uint8_t bus, uint32_t device, int bus_frequency) :
	I2C(DRV_BARO_DEVTYPE_BME688, MODULE_NAME, bus, device, bus_frequency) {
}

#endif // CONFIG_I2C

