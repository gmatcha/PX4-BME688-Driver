#pragma once

#include "bme68x-sensor-api/bme68x.h"

#include "bme688.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gas.h>
#include <lib/perf/perf_counter.h>

typedef struct {
    uint8_t i2c_slv_addr;
    device::I2C* i2c_cinst;
} i2c_intf_t;

typedef struct {
    uint8_t spi_slv_addr;
    device::SPI* spi_cinst;
} spi_intf_t;

class BME688 : public I2CSPIDriver<BME688> {
public:
    enum op_mode_t {
        SLEEP = BME68X_SLEEP_MODE,
        FORCE = BME68X_FORCED_MODE,
        PARALLEL = BME68X_PARALLEL_MODE,
        SEQUENTIAL = BME68X_SEQUENTIAL_MODE,
        OTHER
    };
    BME688(const I2CSPIDriverConfig &config, bme688::IBME688 *interface);
    virtual ~BME688();

    static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config,
                                         int runtime_instance);
    static void print_usage();

    static void dev_delay_us(uint32_t period, void *intf_ptr);
    static int8_t dev_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static int8_t dev_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

    int             init(uint8_t slv_addr);
    void			print_status();
    void			RunImpl();

private:
    void			Start();

    int8_t sensor_setup(const op_mode_t mode);
    int8_t sensor_acquire_data(struct bme68x_data *data, uint8_t *n_data);
    void sensor_output_data(struct bme68x_data *data, uint8_t *n_data);

    void check_rslt(const char api_name[], int8_t rslt);
    int8_t selftest_check();

    int			measure(); //start measure
    int			collect(); //get results and publish

    uORB::PublicationMulti<sensor_gas_s> _sensor_gas_pub{ORB_ID(sensor_gas)};

    bme688::IBME688		*_interface;

    static constexpr uint32_t	_measure_interval{1 * 1000000};

    i2c_intf_t i2c_intf;
    spi_intf_t spi_intf;

    struct bme68x_dev dev;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    op_mode_t dev_op_mode;

    struct bme68x_data sensor_data;
    uint8_t ndata;
    int8_t measure_status = -1;

    bool			_collect_phase{false};

    perf_counter_t		_sample_perf;
    perf_counter_t		_measure_perf;
    perf_counter_t		_comms_errors;
};

