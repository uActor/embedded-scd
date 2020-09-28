# embedded-scd
This repository contains the ESP-IDF port of the embedded driver sources for Sensirion's SCD product
line.

## Add this repository as a component in your ESP-IDF app project

In your ESP-IDF application project root:

```
 mkdir components
 git clone --recursive git@gitlab.lrz.de:cm/sensirion-embedded-scd.git components/embedded-scd
```

Include the components from your main/CMakeLists.txt:

```
 ...
 set(COMPONENT_REQUIRES embedded-scd)
 ...
```

## Repository content
* embedded-common (submodule repository for the common embedded driver HAL)
* scd30 (SCD30 driver related)

## Integrating into the project

The driver does not intialize ESP-IDF I2C driver, so it must be done by the application.

**Note**: The sensor does longer I2C clock stretching than the ESP32 supports by default. This will result in frequent errors. To alleviate this, set the clock stretch to maximum while initializing the I2C drivers: `i2c_set_timeout( I2C_NUM_0, 0xFFFFF );`

Example application:

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/i2c.h>

#include "scd30.h"

#define I2C_DATA_PIN    (33)
#define I2C_CLOCK_PIN   (32)

static void init_i2c() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_DATA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_CLOCK_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config( I2C_NUM_0, &conf );
    i2c_driver_install( I2C_NUM_0, conf.mode, 0, 0, 0);
    i2c_set_timeout( I2C_NUM_0, 0xFFFFF );
}

void app_main( void ) {
    init_i2c();


    float co2_ppm, temperature, relative_humidity;
    int16_t err;
    uint16_t interval_in_seconds = 2;

    /* Initialize I2C */
    sensirion_i2c_init();

    /* Busy loop for initialization, because the main loop does not work without
     * a sensor.
     */
    while ( scd30_probe() != STATUS_OK ) {
        printf( "SCD30 sensor probing failed\n" );
        sensirion_sleep_usec( 1000000u );
    }
    printf( "SCD30 sensor probing successful\n" );

    scd30_set_measurement_interval( interval_in_seconds );
    sensirion_sleep_usec( 20000u );
    scd30_start_periodic_measurement( 0 );
    sensirion_sleep_usec( interval_in_seconds * 1000000u );

    while (1) {
        /* Measure co2, temperature and relative humidity and store into
         * variables.
         */
        err = scd30_read_measurement( &co2_ppm, &temperature, &relative_humidity );
        if ( err != STATUS_OK ) {
            printf("error reading measurement\n");
        } else {
            printf("measured co2 concentration: %0.2f ppm, "
                   "measured temperature: %0.2f degreeCelsius, "
                   "measured humidity: %0.2f %%RH\n",
                   co2_ppm, temperature, relative_humidity);
        }

        sensirion_sleep_usec( interval_in_seconds * 1000000u);
    }

    scd30_stop_periodic_measurement();
}
```