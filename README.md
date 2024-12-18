# Description

This is an STM32 HAL hardware driver for the [Sensirion SEN66 environmental gas sensor](https://www.digikey.com/short/bd481vz2).

It is a simple, lightweight, blocking driver that does not utilize DMA or IRQs. It should work agnostic to any particular STM32 you may be using.

# Installation

1. Clone the driver down to your STM32 HAL project, perhaps as a git submodule.

```BASH
git clone git@gitlab.com:Yankee14/driver_sensirion_sen66_stm32_hal.git my_stm_project/Drivers/Sensirion_SEN66
```

2. Add the driver to your compiler include paths, under `Project --> Properties --> C/C++ Build --> Settings --> Tool Settings tab --> MCU GCC Compiler --> Include Paths`, perhaps as a workspace include path.

# Usage

1. Import the driver into your code. For example, to the top of `main.c` add:

```c
#include "Sensirion_SEN66.h"
```

2. Declare an instance of a SEN66

```c
SEN66 my_sen66;
```

3. Initialize it

```c
SEN66_init(&my_sen66); // construct the SEN66
```

4. Turn the sensors on and start sampling

```c
SEN66_start_continuous_measurement(&my_sen66);
```

5. Wait for a new sample to become ready. A new sample is ready approximately every 1 second.

```c
bool data_ready = false;
do {
    SEN66_get_data_ready(&my_sen66); // poll the sensor readiness
    bool data_ready = SEN66_is_data_ready(&my_sen66); // interpret the readiness result as a boolean
    HAL_Delay(50); // check again in 50 milliseconds
} while (!data_ready);

```

6. Fetch the sample and read the results

```c
SEN66_read_measured_values(&my_sen66); // poll the sensor sampled values

// interpret the sampled values
uint16_t pm_1p0 = SEN66_get_mass_concentration_PM1p0(&my_sen66) / 10; // ug/m^3, 10x scaling
uint16_t pm_2p5 = SEN66_get_mass_concentration_PM2p5(&my_sen66) / 10; // ug/m^3, 10x scaling
uint16_t pm_4p0 = SEN66_get_mass_concentration_PM4p0(&my_sen66) / 10; // ug/m^3, 10x scaling
uint16_t pm_10p0 = SEN66_get_mass_concentration_PM10p0(&my_sen66) / 10; // ug/m^3, 10x scaling
int16_t ambient_humidity_pct = SEN66_get_ambient_humidity_pct(&my_sen66) / 100; // RH%, 100x scaling
int16_t ambient_temp_c = SEN66_get_ambient_temperature_c(&my_sen66) / 200; // deg C, 200x scaling
int16_t VOC_index = SEN66_get_VOC_index(&my_sen66) / 10; // unitless, 10x scaling
int16_t NOx_index = SEN66_get_NOx_index(&my_sen66) / 10; // unitless, 10x scaling
uint16_t CO2_ppm = SEN66_get_CO2_ppm(&my_sen66) / 1; // PPM, 1x scaling
```

If you want extra precision or decimal points, you can omit the divison at the end of each statement. Then you can deal with the value as an integer with the decimal point shifted, or convert it to a floating point number, etc.

# [Buy Me a Beer!](https://buymeacoffee.com/yankee14)

* If you found this useful, please consider throwing some beer money my way :)
* If you'd like me to fix bugs or approve pull requests faster, please consider throwing some beer money my way :)

https://buymeacoffee.com/yankee14

![author donation link](bmc_qr.png)

# License

This driver is licensed under the Apache 2.0 license. You can find details in the file `LICENSE` at the root of this repository.
