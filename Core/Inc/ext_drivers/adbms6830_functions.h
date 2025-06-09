/*
 * adbms6830_functions.h
 *
 *  Created on: May 13, 2025
 *      Author: realb
 */

#ifndef INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_
#define INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_

void adBms6830_init_config(adbms6830_driver_t* dev,
						   uint8_t num_ics,
						   adbms6830_asic* ics,
						   SPI_HandleTypeDef* hspi_a,
						   SPI_HandleTypeDef* hspi_b,
						   GPIO_TypeDef* cs_port_a,
						   GPIO_TypeDef* cs_port_b,
						   uint16_t cs_pin_a,
						   uint16_t cs_pin_b);

void wakeup_ics(adbms6830_driver_t* dev);

void adbms_write_data(adbms6830_driver_t *dev, uint8_t cmd[ADBMS6830_CMD_SIZE], TYPE type, GRP group);

void adbms6830_create_config(adbms6830_driver_t* dev, GRP group);

void adbms6830_create_comm(adbms6830_driver_t* dev);

void adbms6830_create_clr_flag_data(adbms6830_driver_t* dev);

void adbms6830_spi_write(adbms6830_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs);

void adbms6830_cmd(adbms6830_driver_t* dev, uint8_t cmd[ADBMS6830_CMD_SIZE]);

void adbms6830_wrdata(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data);


#endif /* INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_ */
