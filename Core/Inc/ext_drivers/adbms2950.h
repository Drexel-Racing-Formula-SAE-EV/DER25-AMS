/*
 * adbms2950.h
 *
 *  Created on: May 13, 2025
 *      Author: cole
 */

#ifndef INC_EXT_DRIVERS_ADBMS2950_H_
#define INC_EXT_DRIVERS_ADBMS2950_H_

#include "ext_drivers/adbms2950_defs.h"

/*!< ADBMS2950 IC main structure */
typedef struct
{
  cfa_ tx_cfga;
  cfa_ rx_cfga;
  cfb_ tx_cfgb;
  cfb_ rx_cfgb;
  flag_ flag;
  flag_ clflag;
  crg_ i;
  iacc_ iacc;
  vbat_ vbat;
  vbacc_ vbacc;
  i_vbat_ ivbat;
  i_vbacc_ i_vbacc;
  vr_  vr;
  rvr_ rvr;
  oc_ oc;
  auxa_ auxa;
  auxb_ auxb;
  auxc_ auxc;
  rdalli_ rdalli;
  rdalla_ rdalla;
  rdallc_ rdallc;
  rdallv_ rdallv;
  rdallr_ rdallr;
  rdallx_ rdallx;
  state_ state;
  com_ tx_comm;
  com_ rx_comm;
  sid_ sid;
  ic_register_ configa;
  ic_register_ configb;
  ic_register_ clrflag;
  ic_register_ reg;
  ic_register_ axa;
  ic_register_ axb;
  ic_register_ axc;
  ic_register_ flg;
  ic_register_ ste;
  ic_register_ rdlli;
  ic_register_ rdlla;
  ic_register_ rdllc;
  ic_register_ rdllv;
  ic_register_ rdllr;
  ic_register_ rdllx;
  ic_register_ com;
  ic_register_ rsid;
  cmdcnt_pec_ cccrc;
  uint32_t pladc_count;
  uint32_t OCTicks;
  uint32_t cal_count;
  tm48_ tm48;
  uint8_t Result;
  uint8_t ResultLoc;
  uint8_t OC_PWM_Result;
} adbms2950_asic;

/* adbms2950 main driver */
typedef struct
{
  uint8_t num_ics;
  adbms2950_asic *ics;
  loop_manager_t loop_manager;
  pladc_manager_t pladc_manager;
  adc_configuration_t config;
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port[2];
	uint16_t cs_pin[2];
	adbms2950_string string;
	TIM_HandleTypeDef *htim;
} adbms2950_driver_t;

void adbms2950_init(adbms2950_driver_t* dev,
					uint8_t num_asics,
					adbms2950_asic* ics,
					SPI_HandleTypeDef* hspi,
					GPIO_TypeDef* CSA_Port,
					GPIO_TypeDef* CSB_Port,
					uint16_t CSA_Pin,
					uint16_t CSB_Pin,
					TIM_HandleTypeDef* htim);

void adbms2950_reset_cfg_regs(adbms2950_driver_t* dev);
void adbms2950_wrcfga(adbms2950_driver_t* dev);
void adbms2950_wrcfgb(adbms2950_driver_t* dev);
void adbms2950_rdcfga(adbms2950_driver_t* dev);
void adbms2950_rdcfgb(adbms2950_driver_t* dev);
void adbms2950_parse_cfga(adbms2950_driver_t* dev, uint8_t* data);
void adbms2950_pack_cfga(adbms2950_driver_t* dev);
void adbms2950_parse_cfgb(adbms2950_driver_t* dev, uint8_t* data);
void adbms2950_pack_cfgb(adbms2950_driver_t* dev);

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
