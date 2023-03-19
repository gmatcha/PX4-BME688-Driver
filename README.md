# PX4 BME688 Gas Sensor Interface

## Modular Driver integrating Bosch bme68x-sensor-api

### Build sensor driver on PX4 Autopilot
  1. Declare unique sensor/driver ID under `drv_sensor.h` - `#define DRV_GAS_DEVTYPE_BME688 0xC2`
  2. Add submodule under PX4 directory `src/drivers/gas`
      * `git submodule add <URL> src/drivers/gas`
  3. Enable BME688 driver: `make <board_type> boardconfig` 
      * Example: make nxp_fmuk66-v3_default boardconfig
  4. Sync, compile, and build PX4 for flashing
  ```
  cd build && cmake ..
  cd ..
  make -j4 <board_type>
  make <board_type> upload
  ```

### Publish uORB sensor data on topic
  
  1. Define uORB sensor topic by creating a msg file under `msg/sensor_gas.msg`
  ```
  uint64 timestamp          # time since system start (microseconds)
  uint64 timestamp_sample

  uint32 device_id          # unique device ID for the sensor that does not change between power cycles

  uint32 sample_count       # bme688 sample counts
  float32 temperature       # temperature in degrees Celsius
  float32 pressure          # static pressure measurement in Pascals
  float32 humidity          # Humidity in percentage (%)
  float32 gas_resistance    # gas resistance (ohm)
  uint8 device_status       # byte hex 0xB0

  uint32 error_count

  uint8 ORB_QUEUE_LENGTH = 4
  ```

#### SPI interface (qGroundControl MavLink Console)
```
NuttShell (NSH) NuttX-11.0.0
nsh> bme688
Usage: bme688 <command> [arguments...]
 Commands:

   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-s]        Internal SPI bus(es)
     [-S]        External SPI bus
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus (default=1))
     [-c <val>]  chip-select pin (for internal SPI) or index (for external SPI)
     [-m <val>]  SPI mode
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 119

   stop

   status        print status info
```

```
nsh> bme688 start -S
INFO  [SPI_I2C] adding a running instance -set started to true
bme688 #0 on SPI bus 3 (external, equal to '-b 1')

nsh> bme688 status
INFO  [SPI_I2C] Running on SPI Bus 3
bme688: sample: 2 events, 3296us elapsed, 1648.00us avg, min 1150us max 2146us 704.278us rms
bme688: measure: 2 events, 287781us elapsed, 143890.50us avg, min 143870us max 143911us 36.791us rms
bme688: comms errors: 0 events
```

```
nsh> listener sensor_gas

TOPIC: sensor_gas
 sensor_gas
    timestamp: 94108924 (0.835246 seconds ago)
    timestamp_sample: 94107860 (1064 us before timestamp)
    device_id: 12714010 (Type: 0xC2, SPI:3 (0x00))
    sample_count: 1
    temperature: 23.8355
    pressure: 101309.7344
    humidity: 55.2943
    gas_resistance: 27222.4590
    error_count: 0
    device_status: 176
```

#### I2C interface (qGroundControl MavLink Console)
```
nsh> bme688 start -X -f 200000
NFO  [SPI_I2C] adding a running instance -set started to true
bme688 #0 on I2C bus 1 address 0x77
nsh> bme688 status
INFO  [SPI_I2C] Running on I2C Bus 1, Address 0x77
bme688: sample: 13 events, 14433us elapsed, 1110.23us avg, min 920us max 1290us 106.065us rms
bme688: measure: 14 events, 2069552us elapsed, 147825.14us avg, min 147513us max 148240us 179.129us rms
bme688: comms errors: 0 events
```

```
nsh> listener sensor_gas

TOPIC: sensor_gas
 sensor_gas
    timestamp: 185075367 (0.630325 seconds ago)
    timestamp_sample: 185074305 (1062 us before timestamp)
    device_id: 12744457 (Type: 0xC2, I2C:1 (0x77))
    sample_count: 1
    temperature: 23.7633
    pressure: 101306.1875
    humidity: 59.5401
    gas_resistance: 35814.2148
    error_count: 0
    device_status: 176
```


