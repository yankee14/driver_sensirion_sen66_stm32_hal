/**
 * Sensirion_SEN66.h
 *
 * https://gitlab.com/Yankee14/driver_sensirion_sen66_stm32_hal#
 *
 * Author Information
 * Name:   Edward Joseph Peguillan III
 * E-mail: yankee14@protonmail.com
 *
 * Created Nov 30, 2024
 */
#ifndef SENSIRION_SEN66_INC_SENSIRION_SEN66_H_
#define SENSIRION_SEN66_INC_SENSIRION_SEN66_H_

#include <stdint.h>
#include <stdbool.h>
#include <main.h>

typedef struct SEN66 {
	I2C_HandleTypeDef *p_hi2c;

#define PRODUCT_NAME_LENGTH 32
	uint8_t product_name[PRODUCT_NAME_LENGTH];

#define SERIAL_NUMBER_LENGTH 32
	uint8_t serial_number[SERIAL_NUMBER_LENGTH];

#define DATA_READY_LENGTH 2
	uint8_t data_ready[DATA_READY_LENGTH];

#define DEVICE_STATUS_LENGTH 4
	uint8_t device_status[DEVICE_STATUS_LENGTH];

#define MEASURED_VALUES_LENGTH 18
	uint8_t measured_values[MEASURED_VALUES_LENGTH];
} SEN66;

HAL_StatusTypeDef SEN66_init(SEN66 *p_sen66, I2C_HandleTypeDef *p_hi2c);

/****
 * BEGIN READ-ONLY FUNCTIONS
 ****/
HAL_StatusTypeDef SEN66_get_serial_number(SEN66 *p_sen66);
HAL_StatusTypeDef SEN66_get_product_name(SEN66 *p_sen66);

HAL_StatusTypeDef SEN66_get_data_ready(SEN66 *p_sen66);
bool SEN66_is_data_ready(SEN66 const *p_sen66);

HAL_StatusTypeDef SEN66_read_device_status(SEN66 *p_sen66);
bool SEN66_is_fan_speed_warning(SEN66 const *p_sen66);
bool SEN66_is_particulate_matter_sensor_error(SEN66 const *p_sen66);
bool SEN66_is_CO2_sensor_error(SEN66 const *p_sen66);
bool SEN66_is_gas_sensor_error(SEN66 const *p_sen66);
bool SEN66_is_relative_humidity_and_temperature_sensor_error(
		SEN66 const *p_sen66);
bool SEN66_is_fan_error(SEN66 const *p_sen66);

HAL_StatusTypeDef SEN66_read_measured_values(SEN66 *p_sen66);
uint16_t SEN66_get_mass_concentration_PM1p0(SEN66 const *p_sen66); // ug/m^3, 10x scaling
uint16_t SEN66_get_mass_concentration_PM2p5(SEN66 const *p_sen66); // ug/m^3, 10x scaling
uint16_t SEN66_get_mass_concentration_PM4p0(SEN66 const *p_sen66); // ug/m^3, 10x scaling
uint16_t SEN66_get_mass_concentration_PM10p0(SEN66 const *p_sen66); // ug/m^3, 10x scaling
int16_t SEN66_get_ambient_humidity_pct(SEN66 const *p_sen66); // RH%, 100x scaling
int16_t SEN66_get_ambient_temperature_c(SEN66 const *p_sen66); // deg C, 200x scaling
int16_t SEN66_get_VOC_index(SEN66 const *p_sen66); // unitless, 10x scaling
int16_t SEN66_get_NOx_index(SEN66 const *p_sen66); // unitless, 10x scaling
uint16_t SEN66_get_CO2_ppm(SEN66 const *p_sen66); // PPM, 1x scaling
/****
 * END READ-ONLY FUNCTIONS
 ****/

/****
 * BEGIN READ-WRITE FUNCTIONS
 ****/
HAL_StatusTypeDef SEN66_read_and_clear_device_status(SEN66 *p_sen66);
/****
 * END READ-WRITE FUNCTIONS
 ****/

/****
 * BEGIN WRITE-ONLY FUNCTIONS
 ****/
HAL_StatusTypeDef SEN66_start_continuous_measurement(SEN66 const *p_sen66);
HAL_StatusTypeDef SEN66_stop_continuous_measurement(SEN66 const *p_sen66);

HAL_StatusTypeDef SEN66_device_reset(SEN66 const *p_sen66);
HAL_StatusTypeDef SEN66_start_fan_cleaning(SEN66 const *p_sen66);
HAL_StatusTypeDef SEN66_activate_SHT_heater(SEN66 const *p_sen66);
/****
 * END WRITE-ONLY FUNCTIONS
 ****/
#endif /* SENSIRION_SEN66_INC_SENSIRION_SEN66_H_ */
