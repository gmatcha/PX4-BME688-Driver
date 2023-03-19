
#include "BME688.hpp"

#include "bme68x-sensor-api/bme68x.h"

BME688::BME688(const I2CSPIDriverConfig &config, bme688::IBME688 *interface) :
    I2CSPIDriver(config),
    _interface(interface),
    _sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
    _measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
    _comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors")) { }

BME688::~BME688()
{
    // free perf counters
    perf_free(_sample_perf);
    perf_free(_measure_perf);
    perf_free(_comms_errors);

    delete _interface;
}

void BME688::dev_delay_us(uint32_t period, void *intf_ptr) {
#if defined(CONFIG_I2C)
    i2c_intf_t* intf_ptr_p = static_cast<i2c_intf_t*>(intf_ptr);
    bme688::IBME688* iface = reinterpret_cast<bme688::IBME688*>(intf_ptr_p->i2c_cinst);
#elif defined(CONFIG_SPI)
    spi_intf_t* intf_ptr_p = static_cast<spi_intf_t*>(intf_ptr);
    bme688::IBME688* iface = reinterpret_cast<bme688::IBME688*>(intf_ptr_p->spi_cinst);
#endif
    iface->delay_us(period);
}

int8_t BME688::dev_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
#if defined(CONFIG_I2C)
    i2c_intf_t* intf_ptr_p = static_cast<i2c_intf_t*>(intf_ptr);
    bme688::IBME688* iface = reinterpret_cast<bme688::IBME688*>(intf_ptr_p->i2c_cinst);
#elif defined(CONFIG_SPI)
    spi_intf_t* intf_ptr_p = static_cast<spi_intf_t*>(intf_ptr);
    bme688::IBME688* iface = reinterpret_cast<bme688::IBME688*>(intf_ptr_p->spi_cinst);
#endif
    return iface->write(reg_addr, reg_data, len);
}

int8_t BME688::dev_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
#if defined(CONFIG_I2C)
    i2c_intf_t* intf_ptr_p = static_cast<i2c_intf_t*>(intf_ptr);
    bme688::IBME688* iface = reinterpret_cast<bme688::IBME688*>(intf_ptr_p->i2c_cinst);
#elif defined(CONFIG_SPI)
    spi_intf_t* intf_ptr_p = static_cast<spi_intf_t*>(intf_ptr);
    bme688::IBME688* iface = reinterpret_cast<bme688::IBME688*>(intf_ptr_p->spi_cinst);
#endif
    return iface->read(reg_addr, reg_data, len);
}

int BME688::init(uint8_t slv_addr) {

    _interface->set_device_type(DRV_BARO_DEVTYPE_BME688);

#if defined(CONFIG_I2C)
    i2c_intf.i2c_slv_addr = slv_addr;
    i2c_intf.i2c_cinst = reinterpret_cast<device::I2C*>(_interface);

    dev.intf_ptr = &i2c_intf;
#endif

#if defined(CONFIG_SPI)
    spi_intf.spi_slv_addr = slv_addr;
    spi_intf.spi_cinst = reinterpret_cast<device::SPI*>(_interface);

    dev.intf_ptr = &spi_intf;
#endif

   dev.read = dev_read;
   dev.write = dev_write;
   dev.delay_us = dev_delay_us;

   dev.amb_temp = 25;

   int8_t rslt = bme68x_init(&dev);
   if(rslt != BME68X_OK) return -1;

   int8_t status = selftest_check();
   if(status != BME68X_OK) return -1;

   status = sensor_setup(FORCE);
   if(status != BME68X_OK) return -1;

   Start();

   return OK;
}

int8_t BME688::sensor_setup(const op_mode_t mode) {
    int8_t rslt = BME68X_OK;
    dev_op_mode = SLEEP;
    switch(mode) {
        case FORCE:
            /* Check if rslt == BME68X_OK, report or handle if otherwise */
            conf.filter = BME68X_FILTER_OFF;
            conf.odr = BME68X_ODR_NONE;
            conf.os_hum = BME68X_OS_16X;
            conf.os_pres = BME68X_OS_1X;
            conf.os_temp = BME68X_OS_2X;
            rslt = bme68x_set_conf(&conf, &dev);
            check_rslt("bme68x_set_conf", rslt);
            if(rslt != BME68X_OK) return rslt;

            /* Check if rslt == BME68X_OK, report or handle if otherwise */
            heatr_conf.enable = BME68X_ENABLE;
            heatr_conf.heatr_temp = 300;
            heatr_conf.heatr_dur = 100;
            rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &dev);
            check_rslt("bme68x_set_heatr_conf", rslt);
            if(rslt != BME68X_OK) return rslt;
            break;
        case PARALLEL:
            break;
        case SEQUENTIAL:
            break;
        default:
            printf("unknown operation mode provided.\n");
            break;
    }
    if(rslt == BME68X_OK) dev_op_mode = mode;
    return rslt;
}

int8_t BME688::sensor_acquire_data(struct bme68x_data *data, uint8_t *n_data) {
    int8_t rslt = -1;
    uint32_t del_period;
    if(dev_op_mode == SLEEP) return -1;
    switch(dev_op_mode) {
        case SLEEP:
            // do nothing
            break;
        case FORCE:
            rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &dev);
            check_rslt("bme68x_set_op_mode", rslt);

            /* Calculate delay period in microseconds */
            del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &dev) + (heatr_conf.heatr_dur * 1000);
            dev.delay_us(del_period, dev.intf_ptr);

            /* Check if rslt == BME68X_OK, report or handle if otherwise */
            rslt = bme68x_get_data(BME68X_FORCED_MODE, data, n_data, &dev);
            check_rslt("bme68x_get_data", rslt);
            if(rslt != BME68X_OK) return rslt;
            break;
        default:
            break;
    }
    return rslt;
}

void BME688::sensor_output_data(struct bme68x_data *data, uint8_t *n_data) {
    if(n_data == nullptr) return;
    uint8_t n_fields = *n_data;
    for(uint8_t i=0; i<n_fields; i++) {
        printf("Sample, Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status\n");
#ifdef BME68X_USE_FPU
        printf("%u, %.2f, %.2f, %.2f, %.2f, 0x%x\n",
               i,
               static_cast<double>(data->temperature),
               static_cast<double>(data->pressure),
               static_cast<double>(data->humidity),
               static_cast<double>(data->gas_resistance),
               data->status);
#else
        printf("%u, %lu, %d, %lu, %lu, %lu, 0x%x\n",
                   sample_count,
                   (long unsigned int)time_ms,
                   (data->temperature / 100),
                   (long unsigned int)data->pressure,
                   (long unsigned int)(data->humidity / 1000),
                   (long unsigned int)data->gas_resistance,
                   data->status);
#endif
    }
}

void BME688::check_rslt(const char *api_name, int8_t rslt) {
    switch (rslt) {
        case BME68X_OK:
            //printf("API name [%s] - BME68X_OK! [%d]\r\n", api_name, rslt);
            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t BME688::selftest_check(void) {
    return bme68x_selftest_check((const struct bme68x_dev *) &dev);
}

void BME688::Start() {
    // reset the report ring and state machine
    _collect_phase = false;

    // schedule a cycle to start things
    ScheduleNow();
}

void BME688::RunImpl() {
    if (_collect_phase) {
        collect();
    } else {
        measure();
    }

    ScheduleDelayed(_measure_interval);
}

int BME688::measure() {
    perf_begin(_measure_perf);

    _collect_phase = true;

    // start measure/acquisition
    int8_t ret = sensor_acquire_data(&sensor_data, &ndata);
    measure_status = ret;

    if (ret != OK) {
        perf_count(_comms_errors);
        perf_cancel(_measure_perf);
        return -1;
    }

    perf_end(_measure_perf);

    return OK;
}

int
BME688::collect()
{
    perf_begin(_sample_perf);

    _collect_phase = false;

    // this should be fairly close to the end of the conversion, so the best approximation of the time
    const hrt_abstime timestamp_sample = hrt_absolute_time();

    if(measure_status >= 0) {
        sensor_output_data(&sensor_data, &ndata);
    } else {

    }

    if (measure_status < 0) {
        perf_count(_comms_errors);
        perf_cancel(_sample_perf);
        return -1;
    }

    // publish
    sensor_gas_s sensor_gas{};
    sensor_gas.timestamp = hrt_absolute_time();
    sensor_gas.timestamp_sample = timestamp_sample;
    sensor_gas.device_id = _interface->get_device_id();
    sensor_gas.sample_count = ndata;
    sensor_gas.temperature = sensor_data.temperature;
    sensor_gas.pressure = sensor_data.pressure;
    sensor_gas.humidity = sensor_data.humidity;
    sensor_gas.gas_resistance = sensor_data.gas_resistance;
    sensor_gas.error_count = perf_event_count(_comms_errors);
    sensor_gas.device_status = sensor_data.status;
    _sensor_gas_pub.publish(sensor_gas);

    perf_end(_sample_perf);

    return OK;
}

void
BME688::print_status()
{
    I2CSPIDriverBase::print_status();
    perf_print_counter(_sample_perf);
    perf_print_counter(_measure_perf);
    perf_print_counter(_comms_errors);
}
