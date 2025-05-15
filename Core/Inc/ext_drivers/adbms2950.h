/*
 * adbms2950.h
 *
 *  Created on: May 13, 2025
 *      Author: cole
 */

#ifndef INC_EXT_DRIVERS_ADBMS2950_H_
#define INC_EXT_DRIVERS_ADBMS2950_H_

#include "ext_drivers/adbms2950_defs.h"

void adbms2950_init(adbms2950_driver_t* dev,
					uint8_t num_asics,
					adbms2950_asic* ics,
					SPI_HandleTypeDef* hspi,
					GPIO_TypeDef* CSA_Port,
					GPIO_TypeDef* CSB_Port,
					uint16_t CSA_Pin,
					uint16_t CSB_Pin,
					TIM_HandleTypeDef* htim);
void adbms2950_parse_cfga(adbms2950_driver_t* dev, uint8_t* data);
void adbms2950_parse_cfgb(adbms2950_driver_t* dev, uint8_t* data);

void adbms2950_wakeup(adbms2950_driver_t *dev);

void adbms2950_wrcmd(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ]);
void adbms2950_wrdata(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data);
void adbms2950_rddata(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data, uint8_t size);

void adbms2950_set_cs(adbms2950_driver_t* dev, uint8_t state);
void adbms2950_usleep(adbms2950_driver_t* dev, uint32_t microseconds);
void adbms2950_spi_write(adbms2950_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs);
void adbms2950_spi_write_read(adbms2950_driver_t *dev,
							  uint8_t* tx_Data,
							  uint8_t tx_len,
							  uint8_t* rx_data,
							  uint8_t rx_len,
							  uint8_t use_cs);
uint16_t Pec15_Calc(uint8_t len, uint8_t *data);
uint16_t pec10_calc(uint8_t rx_cmd, int len, uint8_t *data);
uint16_t pec10_calc_modular(uint8_t * data, uint8_t PEC_Format);
uint16_t pec10_calc_int(uint16_t remainder, uint8_t bit);

#endif /* INC_EXT_DRIVERS_ADBMS2950_H_ */
