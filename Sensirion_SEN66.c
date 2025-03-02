/**
 * Sensirion_SEN66.c
 *
 * https://gitlab.com/Yankee14/driver_sensirion_sen66_stm32_hal#
 *
 * Author Information
 * Name:   Edward Joseph Peguillan III
 * E-mail: yankee14@protonmail.com
 *
 * Created Nov 30, 2024
 */
#include "Sensirion_SEN66.h"

uint16_t const addr_i2c = 0x6B << 1;

/****
 * BEGIN PRIVATE VARIABLES FOR READ-ONLY FUNCTIONS
 ****/
uint8_t const addr_product_name[] = { 0xD0, 0x14 };
#define PRODUCT_NAME_REG_LENGTH 48
#define GET_PRODUCT_NAME_EXECUTION_TIME_ms 20

uint8_t const addr_serial_number[] = { 0xD0, 0x33 };
#define SERIAL_NUMBER_REG_LENGTH 48
#define GET_SERIAL_NUMBER_EXECUTION_TIME_ms 20

uint8_t const addr_get_data_ready[] = { 0x02, 0x02 };
#define DATA_READY_REG_LENGTH 3
#define GET_DATA_READY_EXECUTION_TIME_ms 20

uint8_t const addr_read_device_status[] = { 0xD2, 0x06 };
#define DEVICE_STATUS_REG_LENGTH 6
#define READ_DEVICE_STATUS_EXECUTION_TIME_ms 20
#define READ_AND_CLEAR_DEVICE_STATUS_EXECUTION_TIME_ms 20
#define DEVICE_STATUS_FAN_SPEED_WARNING_byte 1
#define DEVICE_STATUS_FAN_SPEED_WARNING_bit 5
#define DEVICE_STATUS_PARTICULATE_MATTER_SENSOR_ERROR_byte 2
#define DEVICE_STATUS_PARTICULATE_MATTER_SENSOR_ERROR_bit 1
#define DEVICE_STATUS_CO2_SENSOR_ERROR_byte 2
#define DEVICE_STATUS_CO2_SENSOR_ERROR_bit 1
#define DEVICE_STATUS_GAS_SENSOR_ERROR_byte 3
#define DEVICE_STATUS_GAS_SENSOR_ERROR_bit 7
#define DEVICE_STATUS_RELATIVE_HUMIDITY_AND_TEMPERATURE_SENSOR_ERROR_byte 3
#define DEVICE_STATUS_RELATIVE_HUMIDITY_AND_TEMPERATURE_SENSOR_ERROR_bit 6
#define DEVICE_STATUS_FAN_ERROR_byte 3
#define DEVICE_STATUS_FAN_ERROR_bit 4

uint8_t const addr_read_measured_values[] = { 0x03, 0x00 };
#define MEASURED_VALUES_REG_LENGTH 27
#define READ_MEASURED_VALUES_EXECUTION_TIME_ms 20
#define MEASURED_VALUES_MASS_CONCENTRATION_PM1p0_MSB_index 0-0/3
#define MEASURED_VALUES_MASS_CONCENTRATION_PM1p0_LSB_index 1-1/3
#define MEASURED_VALUES_MASS_CONCENTRATION_PM2p5_MSB_index 3-3/3
#define MEASURED_VALUES_MASS_CONCENTRATION_PM2p5_LSB_index 4-4/3
#define MEASURED_VALUES_MASS_CONCENTRATION_PM4p0_MSB_index 6-6/3
#define MEASURED_VALUES_MASS_CONCENTRATION_PM4p0_LSB_index 7-7/3
#define MEASURED_VALUES_MASS_CONCENTRATION_PM10p0_MSB_index 9-9/3
#define MEASURED_VALUES_MASS_CONCENTRATION_PM10p0_LSB_index 10-10/3
#define MEASURED_VALUES_AMBIENT_HUMIDITY_pct_MSB_index 12-12/3
#define MEASURED_VALUES_AMBIENT_HUMIDITY_pct_LSB_index 13-13/3
#define MEASURED_VALUES_AMBIENT_TEMPERATURE_c_MSB_index 15-15/3
#define MEASURED_VALUES_AMBIENT_TEMPERATURE_c_LSB_index 16-16/3
#define MEASURED_VALUES_VOC_index_MSB_index 18-18/3
#define MEASURED_VALUES_VOC_index_LSB_index 19-19/3
#define MEASURED_VALUES_NOx_index_MSB_index 21-21/3
#define MEASURED_VALUES_NOx_index_LSB_index 22-22/3
#define MEASURED_VALUES_CO2_ppm_MSB_index 24-24/3
#define MEASURED_VALUES_CO2_ppm_LSB_index 25-25/3
/****
 * END PRIVATE VARIABLES FOR READ-ONLY FUNCTIONS
 ****/

/****
 * BEGIN PRIVATE VARIABLES FOR READ-WRITE FUNCTIONS
 ****/
uint8_t const addr_read_and_clear_device_status[] = { 0xD2, 0x10 };
/****
 * END PRIVATE VARIABLES FOR READ-WRITE FUNCTIONS
 ****/

/****
 * BEGIN PRIVATE VARIABLES FOR WRITE-ONLY FUNCTIONS
 ****/
uint8_t const addr_start_continuous_measurement[] = { 0x00, 0x21 };
#define START_CONTINUOUS_MEASUREMENT_EXECUTION_TIME_ms 50

uint8_t const addr_stop_measurement[] = { 0x01, 0x04 };
#define STOP_MEASUREMENT_EXECUTION_TIME_ms 1000

uint8_t const addr_device_reset[] = { 0xD3, 0x04 };
#define DEVICE_RESET_EXECUTION_TIME_ms 1200

uint8_t const addr_start_fan_cleaning[] = { 0x56, 0x07 };
#define START_FAN_CLEANING_EXECUTION_TIME_ms 10020

uint8_t const addr_activate_SHT_heater[] = { 0x67, 0x65 };
#define ACTIVATE_SHT_HEATER_EXECUTION_TIME_ms 21300
/****
 * END PRIVATE VARIABLES FOR WRITE-ONLY FUNCTIONS
 ****/

/****
 * BEGIN INTERNAL FUNCTION PROTOTYPES
 ****/
/**
 * @brief  Calculates additional HAL_Delay() time for inaccurate CPU clocks (such as HSI OSC).
 * @param  base_delay_ms The minimum delay time in ms
 * @param  compensation_log2 The additional delay time in ms desired, see retval.
 * @retval base_delay_ms + (base_delay_ms >> compensation_log2)
 */
static uint32_t SEN66_calculate_clock_tolerance_compensation_ms(
		uint32_t base_delay_ms, uint32_t compensation_log2);

#define SEN66_CRC_8_DALLAS_INIT 0xFF
#define SEN66_CRC_8_DALLAS_POLYNOMIAL 0x31
static uint8_t SEN66_crc_8_dallas(uint8_t const data[],
		size_t const data_length);
static bool SEN66_crc_ok(uint8_t const data[], size_t const data_length);
static void SEN66_fill_array_discard_crc(uint8_t dest[],
		size_t const dest_length, uint8_t const src[], size_t const src_length);
/****
 * END INTERNAL FUNCTION PROTOTYPES
 ****/

HAL_StatusTypeDef SEN66_init(SEN66_t *p_sen66, I2C_HandleTypeDef *p_hi2c) {
	p_sen66->p_hi2c = p_hi2c;

	for (int i = 0; i < PRODUCT_NAME_LENGTH; ++i)
		p_sen66->product_name[i] = 0x15; // default to ASCII NAK

	for (int i = 0; i < SERIAL_NUMBER_LENGTH; ++i)
		p_sen66->serial_number[i] = 0x15; // default to ASCII NAK

	for (int i = 0; i < DATA_READY_LENGTH; ++i)
		p_sen66->data_ready[i] = 0; // default to false

	for (int i = 0; i < DEVICE_STATUS_LENGTH; ++i)
		p_sen66->device_status[i] = -1; // default all to indicate errors

	for (int i = 0; i < MEASURED_VALUES_LENGTH; ++i)
		p_sen66->measured_values[i] = -1; // default all to nonsense values

	SEN66_get_serial_number(p_sen66);
	SEN66_get_product_name(p_sen66);
	SEN66_read_device_status(p_sen66);

	return HAL_OK;
}

/****
 * BEGIN READ-ONLY FUNCTIONS
 ****/
HAL_StatusTypeDef SEN66_get_serial_number(SEN66_t *p_sen66) {
	uint8_t rx_serial_number[SERIAL_NUMBER_REG_LENGTH] = { 0x00 };
	HAL_StatusTypeDef i2c_status = HAL_ERROR;

	i2c_status = HAL_I2C_Master_Transmit(p_sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_serial_number, sizeof(addr_serial_number),
			HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	GET_SERIAL_NUMBER_EXECUTION_TIME_ms, 3));

	i2c_status = HAL_I2C_Master_Receive(p_sen66->p_hi2c, addr_i2c,
			rx_serial_number, SERIAL_NUMBER_REG_LENGTH,
			HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	if (!SEN66_crc_ok(rx_serial_number, SERIAL_NUMBER_REG_LENGTH))
		return HAL_ERROR;
	SEN66_fill_array_discard_crc(p_sen66->serial_number,
	SERIAL_NUMBER_LENGTH, rx_serial_number, SERIAL_NUMBER_REG_LENGTH);
	return i2c_status;
}

HAL_StatusTypeDef SEN66_get_product_name(SEN66_t *p_sen66) {
	uint8_t rx_product_name[PRODUCT_NAME_REG_LENGTH] = { 0x00 };
	HAL_StatusTypeDef i2c_status = HAL_ERROR;

	i2c_status = HAL_I2C_Master_Transmit(p_sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_product_name, sizeof(addr_product_name),
			HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	GET_PRODUCT_NAME_EXECUTION_TIME_ms, 3));

	i2c_status = HAL_I2C_Master_Receive(p_sen66->p_hi2c, addr_i2c,
			rx_product_name,
			PRODUCT_NAME_REG_LENGTH, HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	if (!SEN66_crc_ok(rx_product_name, PRODUCT_NAME_REG_LENGTH))
		return HAL_ERROR;
	SEN66_fill_array_discard_crc(p_sen66->product_name,
	PRODUCT_NAME_LENGTH, rx_product_name,
	PRODUCT_NAME_REG_LENGTH);
	return i2c_status;
}

HAL_StatusTypeDef SEN66_get_data_ready(SEN66_t *p_sen66) {
	uint8_t rx_data_ready[DATA_READY_REG_LENGTH] = { 0x00 };
	HAL_StatusTypeDef i2c_status = HAL_ERROR;

	i2c_status = HAL_I2C_Master_Transmit(p_sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_get_data_ready, sizeof(addr_get_data_ready),
			HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	GET_DATA_READY_EXECUTION_TIME_ms, 3));

	i2c_status = HAL_I2C_Master_Receive(p_sen66->p_hi2c, addr_i2c,
			rx_data_ready,
			DATA_READY_REG_LENGTH, HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	if (!SEN66_crc_ok(rx_data_ready,
	DATA_READY_REG_LENGTH))
		return HAL_ERROR;
	SEN66_fill_array_discard_crc(p_sen66->data_ready,
	DATA_READY_LENGTH, rx_data_ready,
	DATA_READY_REG_LENGTH);
	return i2c_status;
}

bool SEN66_is_data_ready(SEN66_t const *p_sen66) {
	return p_sen66->data_ready[1];
}

HAL_StatusTypeDef SEN66_read_device_status(SEN66_t *p_sen66) {
	uint8_t rx_device_status[DEVICE_STATUS_REG_LENGTH] = { 0x00 };
	HAL_StatusTypeDef i2c_status = HAL_ERROR;

	i2c_status = HAL_I2C_Master_Transmit(p_sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_read_device_status, sizeof(addr_read_device_status),
			HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	READ_DEVICE_STATUS_EXECUTION_TIME_ms, 3));

	i2c_status = HAL_I2C_Master_Receive(p_sen66->p_hi2c, addr_i2c,
			rx_device_status,
			DEVICE_STATUS_REG_LENGTH,
			HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	if (!SEN66_crc_ok(rx_device_status,
	DEVICE_STATUS_REG_LENGTH))
		return HAL_ERROR;
	SEN66_fill_array_discard_crc(p_sen66->data_ready,
	DEVICE_STATUS_LENGTH, rx_device_status,
	DEVICE_STATUS_REG_LENGTH);
	return i2c_status;
}

bool SEN66_is_fan_speed_warning(SEN66_t const *p_sen66) {
	uint8_t const fan_speed_warning_byte =
			p_sen66->device_status[DEVICE_STATUS_FAN_SPEED_WARNING_byte];
	uint8_t const fan_speed_warning_masked = fan_speed_warning_byte
			& (1 << DEVICE_STATUS_FAN_SPEED_WARNING_bit);
	return 0 != fan_speed_warning_masked ?
	true :
											false;
}

bool SEN66_is_particulate_matter_sensor_error(SEN66_t const *p_sen66) {
	uint8_t const particulate_matter_sensor_error_byte =
			p_sen66->device_status[DEVICE_STATUS_PARTICULATE_MATTER_SENSOR_ERROR_byte];
	uint8_t const particulate_matter_sensor_error_masked =
			particulate_matter_sensor_error_byte
					& (1 << DEVICE_STATUS_PARTICULATE_MATTER_SENSOR_ERROR_bit);
	return 0 != particulate_matter_sensor_error_masked ?
	true :
															false;
}

bool SEN66_is_CO2_sensor_error(SEN66_t const *p_sen66) {
	uint8_t const CO2_sensor_error_byte =
			p_sen66->device_status[DEVICE_STATUS_CO2_SENSOR_ERROR_byte];
	uint8_t const CO2_sensor_error_masked = CO2_sensor_error_byte
			& (1 << DEVICE_STATUS_CO2_SENSOR_ERROR_bit);
	return 0 != CO2_sensor_error_masked ?
	true :
											false;
}

bool SEN66_is_gas_sensor_error(SEN66_t const *p_sen66) {
	uint8_t const gas_sensor_error_byte =
			p_sen66->device_status[DEVICE_STATUS_GAS_SENSOR_ERROR_byte];
	uint8_t const gas_sensor_error_masked = gas_sensor_error_byte
			& (1 << DEVICE_STATUS_GAS_SENSOR_ERROR_bit);
	return 0 != gas_sensor_error_masked ?
	true :
											false;
}

bool SEN66_is_relative_humidity_and_temperature_sensor_error(
		SEN66_t const *p_sen66) {
	uint8_t relative_humidity_and_temperature_sensor_error_byte =
			p_sen66->device_status[DEVICE_STATUS_RELATIVE_HUMIDITY_AND_TEMPERATURE_SENSOR_ERROR_byte];
	uint8_t relative_humidity_and_temperature_sensor_error_masked =
			relative_humidity_and_temperature_sensor_error_byte
					& (1
							<< DEVICE_STATUS_RELATIVE_HUMIDITY_AND_TEMPERATURE_SENSOR_ERROR_bit);
	return 0 != relative_humidity_and_temperature_sensor_error_masked ?
	true :
																		false;
}

bool SEN66_is_fan_error(SEN66_t const *p_sen66) {
	uint8_t fan_error_byte =
			p_sen66->device_status[DEVICE_STATUS_FAN_ERROR_byte];
	uint8_t fan_error_masked = fan_error_byte
			& (1 << DEVICE_STATUS_FAN_ERROR_bit);
	return 0 != fan_error_masked ? true : false;
}

HAL_StatusTypeDef SEN66_read_measured_values(SEN66_t *p_sen66) {
	uint8_t rx_measured_values[MEASURED_VALUES_REG_LENGTH] = { 0x00 };
	HAL_StatusTypeDef i2c_status = HAL_ERROR;

	i2c_status = HAL_I2C_Master_Transmit(p_sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_read_measured_values,
			sizeof(addr_read_measured_values),
			HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	READ_MEASURED_VALUES_EXECUTION_TIME_ms, 3));

	i2c_status = HAL_I2C_Master_Receive(p_sen66->p_hi2c, addr_i2c,
			rx_measured_values,
			MEASURED_VALUES_REG_LENGTH,
			HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	if (!SEN66_crc_ok(rx_measured_values,
	MEASURED_VALUES_REG_LENGTH))
		return HAL_ERROR;
	SEN66_fill_array_discard_crc(p_sen66->data_ready,
	MEASURED_VALUES_LENGTH, rx_measured_values,
	MEASURED_VALUES_REG_LENGTH);
	return i2c_status;
}

uint16_t SEN66_get_mass_concentration_PM1p0(SEN66_t const *p_sen66) {
	uint16_t mass_concentration_PM1p0 =
			(p_sen66->measured_values[MEASURED_VALUES_MASS_CONCENTRATION_PM1p0_MSB_index]
					<< 8)
					| p_sen66->measured_values[MEASURED_VALUES_MASS_CONCENTRATION_PM1p0_LSB_index];
	return mass_concentration_PM1p0;
}

uint16_t SEN66_get_mass_concentration_PM2p5(SEN66_t const *p_sen66) {
	uint16_t mass_concentration_PM2p5 =
			(p_sen66->measured_values[MEASURED_VALUES_MASS_CONCENTRATION_PM2p5_MSB_index]
					<< 8)
					| p_sen66->measured_values[MEASURED_VALUES_MASS_CONCENTRATION_PM2p5_LSB_index];
	return mass_concentration_PM2p5;
}

uint16_t SEN66_get_mass_concentration_PM4p0(SEN66_t const *p_sen66) {
	uint16_t mass_concentration_PM4p0 =
			(p_sen66->measured_values[MEASURED_VALUES_MASS_CONCENTRATION_PM4p0_MSB_index]
					<< 8)
					| p_sen66->measured_values[MEASURED_VALUES_MASS_CONCENTRATION_PM4p0_LSB_index];
	return mass_concentration_PM4p0;
}

uint16_t SEN66_get_mass_concentration_PM10p0(SEN66_t const *p_sen66) {
	uint16_t mass_concentration_PM10p0 =
			(p_sen66->measured_values[MEASURED_VALUES_MASS_CONCENTRATION_PM10p0_MSB_index]
					<< 8)
					| p_sen66->measured_values[MEASURED_VALUES_MASS_CONCENTRATION_PM10p0_LSB_index];
	return mass_concentration_PM10p0;
}

int16_t SEN66_get_ambient_humidity_pct(SEN66_t const *p_sen66) {
	uint16_t ambient_humidity_pct =
			(p_sen66->measured_values[MEASURED_VALUES_AMBIENT_HUMIDITY_pct_MSB_index]
					<< 8)
					| p_sen66->measured_values[MEASURED_VALUES_AMBIENT_HUMIDITY_pct_LSB_index];
	return (int16_t) ambient_humidity_pct;
}

int16_t SEN66_get_ambient_temperature_c(SEN66_t const *p_sen66) {
	uint16_t ambient_temperature_c =
			(p_sen66->measured_values[MEASURED_VALUES_AMBIENT_TEMPERATURE_c_MSB_index]
					<< 8)
					| p_sen66->measured_values[MEASURED_VALUES_AMBIENT_TEMPERATURE_c_LSB_index];
	return (int16_t) ambient_temperature_c;
}

int16_t SEN66_get_VOC_index(SEN66_t const *p_sen66) {
	uint16_t VOC_index =
			(p_sen66->measured_values[MEASURED_VALUES_VOC_index_MSB_index] << 8)
					| p_sen66->measured_values[MEASURED_VALUES_VOC_index_LSB_index];
	return (int16_t) VOC_index;
}

int16_t SEN66_get_NOx_index(SEN66_t const *p_sen66) {
	uint16_t NOx_index =
			(p_sen66->measured_values[MEASURED_VALUES_NOx_index_MSB_index] << 8)
					| p_sen66->measured_values[MEASURED_VALUES_NOx_index_LSB_index];
	return (int16_t) NOx_index;
}

uint16_t SEN66_get_CO2_ppm(SEN66_t const *p_sen66) {
	uint16_t CO2_ppm =
			(p_sen66->measured_values[MEASURED_VALUES_CO2_ppm_MSB_index] << 8)
					| p_sen66->measured_values[MEASURED_VALUES_CO2_ppm_LSB_index];
	return CO2_ppm;
}
/****
 * END READ-ONLY FUNCTIONS
 ****/

/****
 * BEGIN READ-WRITE FUNCTIONS
 ****/
HAL_StatusTypeDef SEN66_read_and_clear_device_status(SEN66_t *p_sen66) {
	uint8_t rx_device_status[DEVICE_STATUS_REG_LENGTH] = { 0x00 };
	HAL_StatusTypeDef i2c_status = HAL_ERROR;

	i2c_status = HAL_I2C_Master_Transmit(p_sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_read_and_clear_device_status,
			sizeof(addr_read_and_clear_device_status),
			HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	READ_AND_CLEAR_DEVICE_STATUS_EXECUTION_TIME_ms, 3));

	i2c_status = HAL_I2C_Master_Receive(p_sen66->p_hi2c, addr_i2c,
			rx_device_status,
			DEVICE_STATUS_REG_LENGTH,
			HAL_MAX_DELAY);
	if (HAL_OK != i2c_status)
		return i2c_status;
	if (!SEN66_crc_ok(rx_device_status,
	DEVICE_STATUS_REG_LENGTH))
		return HAL_ERROR;
	SEN66_fill_array_discard_crc(p_sen66->data_ready,
	DEVICE_STATUS_LENGTH, rx_device_status,
	DEVICE_STATUS_REG_LENGTH);
	return i2c_status;
}
/****
 * END READ-WRITE FUNCTIONS
 ****/

/****
 * BEGIN WRITE-ONLY FUNCTIONS
 ****/
HAL_StatusTypeDef SEN66_device_reset(SEN66_t const *p_sen66) {
	HAL_StatusTypeDef i2c_status = HAL_ERROR;
	i2c_status = HAL_I2C_Master_Transmit(p_sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_device_reset, sizeof(addr_device_reset),
			HAL_MAX_DELAY);
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	DEVICE_RESET_EXECUTION_TIME_ms, 3));

	return i2c_status;
}

HAL_StatusTypeDef SEN66_start_fan_cleaning(SEN66_t const *sen66) {
	HAL_StatusTypeDef i2c_status = HAL_ERROR;
	i2c_status = HAL_I2C_Master_Transmit(sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_start_fan_cleaning, sizeof(addr_start_fan_cleaning),
			HAL_MAX_DELAY);
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	START_FAN_CLEANING_EXECUTION_TIME_ms, 3));

	return i2c_status;
}

HAL_StatusTypeDef SEN66_start_continuous_measurement(SEN66_t const *p_sen66) {
	HAL_StatusTypeDef i2c_status = HAL_ERROR;
	i2c_status = HAL_I2C_Master_Transmit(p_sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_start_continuous_measurement,
			sizeof(addr_start_continuous_measurement),
			HAL_MAX_DELAY);
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	START_CONTINUOUS_MEASUREMENT_EXECUTION_TIME_ms, 3));

	return i2c_status;
}

HAL_StatusTypeDef SEN66_stop_measurement(SEN66_t const *p_sen66) {
	HAL_StatusTypeDef i2c_status = HAL_ERROR;
	i2c_status = HAL_I2C_Master_Transmit(p_sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_stop_measurement, sizeof(addr_stop_measurement),
			HAL_MAX_DELAY);
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	STOP_MEASUREMENT_EXECUTION_TIME_ms, 3));

	return i2c_status;
}

HAL_StatusTypeDef SEN66_activate_SHT_heater(SEN66_t const *p_sen66) {
	HAL_StatusTypeDef i2c_status = HAL_ERROR;
	i2c_status = HAL_I2C_Master_Transmit(p_sen66->p_hi2c, addr_i2c,
			(uint8_t*) addr_activate_SHT_heater,
			sizeof(addr_activate_SHT_heater),
			HAL_MAX_DELAY);
	HAL_Delay(SEN66_calculate_clock_tolerance_compensation_ms(
	ACTIVATE_SHT_HEATER_EXECUTION_TIME_ms, 3));

	return i2c_status;
}
/****
 * END WRITE-ONLY FUNCTIONS
 ****/

/****
 * BEGIN INTERNAL HELPER & UTILITY FUNCTIONS
 ****/
uint32_t SEN66_calculate_clock_tolerance_compensation_ms(uint32_t base_delay_ms,
		uint32_t compensation_log2) {
	uint32_t new_delay_ms = base_delay_ms
			+ (base_delay_ms >> compensation_log2);
	return new_delay_ms;
}

uint8_t SEN66_crc_8_dallas(uint8_t const data[], size_t const data_length) {
	if (2 != data_length)
		return -1;

	uint8_t crc =
	SEN66_CRC_8_DALLAS_INIT;
	for (size_t current_byte = 0; current_byte < data_length; ++current_byte) {
		crc ^= (data[current_byte]);
		for (uint8_t crc_bit = 8; crc_bit > 0; --crc_bit) {
			if (crc & 0x80)
				crc = (crc << 1) ^ SEN66_CRC_8_DALLAS_POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}
	return crc;
}

bool SEN66_crc_ok(uint8_t const data[], size_t const data_length) {
	if (0 != (data_length % 3))
		return false;

	for (size_t current_byte = 0; current_byte < data_length; current_byte +=
			3) {
		uint8_t const data_chunk[2] = { data[current_byte], data[current_byte
				+ 1] };
		uint8_t const crc_rx = data[current_byte + 2];
		uint8_t const crc_calc = SEN66_crc_8_dallas(data_chunk,
				sizeof(data_chunk));
		if (crc_rx != crc_calc)
			return false;
	}

	return true;
}

void SEN66_fill_array_discard_crc(uint8_t dest[], size_t const dest_length,
		uint8_t const src[], size_t const src_length) {
	for (size_t dest_iterator = 0, src_iterator = 0;
			(dest_iterator < dest_length) && (src_iterator < src_length);
			++dest_iterator, ++src_iterator) {
		dest[dest_iterator] = src[src_iterator];
		if (1 == (src_iterator % 3)) // if the next src index will be the crc
			++src_iterator; // skip it
	}
}
/****
 * END INTERNAL HELPER & UTILITY FUNCTIONS
 ****/
