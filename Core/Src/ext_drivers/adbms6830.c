/*
 * adbms6830.c
 *
 *  Created on: May 13, 2025
 *      Author: realb
 */

#include "ext_drivers/adbms6830_data.h"
#include "ext_drivers/adbms6830_functions.h"
#include <stdint.h>
#include <string.h>

/* configuration registers commands */
uint8_t WRCFGA[2]        = { 0x00, 0x01 };
uint8_t WRCFGB[2]        = { 0x00, 0x24 };
uint8_t RDCFGA[2]        = { 0x00, 0x02 };
uint8_t RDCFGB[2]        = { 0x00, 0x26 };

/* Read cell voltage result registers commands */
uint8_t RDCVA[2]         = { 0x00, 0x04 };
uint8_t RDCVB[2]         = { 0x00, 0x06 };
uint8_t RDCVC[2]         = { 0x00, 0x08 };
uint8_t RDCVD[2]         = { 0x00, 0x0A };
uint8_t RDCVE[2]         = { 0x00, 0x09 };
uint8_t RDCVF[2]         = { 0x00, 0x0B };
uint8_t RDCVALL[2]       = { 0x00, 0x0C };

/* Read average cell voltage result registers commands commands */
uint8_t RDACA[2]         = { 0x00, 0x44 };
uint8_t RDACB[2]         = { 0x00, 0x46 };
uint8_t RDACC[2]         = { 0x00, 0x48 };
uint8_t RDACD[2]         = { 0x00, 0x4A };
uint8_t RDACE[2]         = { 0x00, 0x49 };
uint8_t RDACF[2]         = { 0x00, 0x4B };
uint8_t RDACALL[2]       = { 0x00, 0x4C };

/* Read s voltage result registers commands */
uint8_t RDSVA[2]         = { 0x00, 0x03 };
uint8_t RDSVB[2]         = { 0x00, 0x05 };
uint8_t RDSVC[2]         = { 0x00, 0x07 };
uint8_t RDSVD[2]         = { 0x00, 0x0D };
uint8_t RDSVE[2]         = { 0x00, 0x0E };
uint8_t RDSVF[2]         = { 0x00, 0x0F };
uint8_t RDSALL[2]        = { 0x00, 0x10 };

/* Read c and s results */
uint8_t RDCSALL[2]       = { 0x00, 0x11 };
uint8_t RDACSALL[2]      = { 0x00, 0x51 };

/* Read all AUX and all Status Registers */
uint8_t RDASALL[2]       = { 0x00, 0x35 };

/* Read filtered cell voltage result registers*/
uint8_t RDFCA[2]         = { 0x00, 0x12 };
uint8_t RDFCB[2]         = { 0x00, 0x13 };
uint8_t RDFCC[2]         = { 0x00, 0x14 };
uint8_t RDFCD[2]         = { 0x00, 0x15 };
uint8_t RDFCE[2]         = { 0x00, 0x16 };
uint8_t RDFCF[2]         = { 0x00, 0x17 };
uint8_t RDFCALL[2]       = { 0x00, 0x18 };

/* Read aux results */
uint8_t RDAUXA[2]        = { 0x00, 0x19 };
uint8_t RDAUXB[2]        = { 0x00, 0x1A };
uint8_t RDAUXC[2]        = { 0x00, 0x1B };
uint8_t RDAUXD[2]        = { 0x00, 0x1F };

/* Read redundant aux results */
uint8_t RDRAXA[2]        = { 0x00, 0x1C };
uint8_t RDRAXB[2]        = { 0x00, 0x1D };
uint8_t RDRAXC[2]        = { 0x00, 0x1E };
uint8_t RDRAXD[2]        = { 0x00, 0x25 };

/* Read status registers */
uint8_t RDSTATA[2]       = { 0x00, 0x30 };
uint8_t RDSTATB[2]       = { 0x00, 0x31 };
uint8_t RDSTATC[2]       = { 0x00, 0x32 };
uint8_t RDSTATCERR[2]    = { 0x00, 0x72 };              /* ERR */
uint8_t RDSTATD[2]       = { 0x00, 0x33 };
uint8_t RDSTATE[2]       = { 0x00, 0x34 };

/* Pwm registers commands */
uint8_t WRPWM1[2]        = { 0x00, 0x20 };
uint8_t RDPWM1[2]        = { 0x00, 0x22 };

uint8_t WRPWM2[2]        = { 0x00, 0x21 };
uint8_t RDPWM2[2]        = { 0x00, 0x23 };

/* Clear commands */
uint8_t CLRCELL[2]       = { 0x07, 0x11 };
uint8_t CLRAUX [2]       = { 0x07, 0x12 };
uint8_t CLRSPIN[2]       = { 0x07, 0x16 };
uint8_t CLRFLAG[2]       = { 0x07, 0x17 };
uint8_t CLRFC[2]         = { 0x07, 0x14 };
uint8_t CLOVUV[2]        = { 0x07, 0x15 };

/* Poll adc command */
uint8_t PLADC[2]         = { 0x07, 0x18 };
uint8_t PLAUT[2]         = { 0x07, 0x19 };
uint8_t PLCADC[2]        = { 0x07, 0x1C };
uint8_t PLSADC[2]        = { 0x07, 0x1D };
uint8_t PLAUX1[2]        = { 0x07, 0x1E };
uint8_t PLAUX2[2]        = { 0x07, 0x1F };

/* Diagn command */
uint8_t DIAGN[2]         = {0x07 , 0x15};

/* GPIOs Comm commands */
uint8_t WRCOMM[2]        = { 0x07, 0x21 };
uint8_t RDCOMM[2]        = { 0x07, 0x22 };
uint8_t STCOMM[13]       = { 0x07, 0x23, 0xB9, 0xE4 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};

/* Mute and Unmute commands */
uint8_t MUTE[2]          = { 0x00, 0x28 };
uint8_t UNMUTE[2]        = { 0x00, 0x29 };

uint8_t RSTCC[2]         = { 0x00, 0x2E };
uint8_t SNAP[2]          = { 0x00, 0x2D };
uint8_t UNSNAP[2]        = { 0x00, 0x2F };
uint8_t SRST[2]          = { 0x00, 0x27 };

/* Read SID command */
uint8_t RDSID[2]         = { 0x00, 0x2C };


const uint16_t Crc15Table[256] =
{
  0x0000,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,
  0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
  0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
  0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
  0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
  0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
  0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
  0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
  0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
  0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
  0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
  0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
  0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
  0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
  0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
  0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
  0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
  0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
  0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
  0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
  0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
  0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
  0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
};

// adapted from 2950

// Tx/Rx Utility
void adbms6830_wr48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data);
void adbms6830_rd48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data);

// SPI communication
void adbms6830_set_cs(adbms6830_driver_t* dev, uint8_t state);
void adbms6830_spi_write(adbms6830_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs);
void adbms6830_spi_write_read(adbms6830_driver_t *dev, uint8_t* tx_Data, uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len, uint8_t use_cs);

uint16_t Pec15_Calc(uint8_t len, uint8_t *data)
{
  uint16_t remainder, addr;
  remainder = 16u; /*!< initialize the PEC */
  for (uint8_t i = 0; i<len; i++) /*!< loops for each byte in data array */
  {
    addr = (((remainder>>7u)^data[i])&0xff);/*!< calculate PEC table address */
    remainder = ((remainder<<8u)^Crc15Table[addr]);
  }
  return(remainder*2u);/*!< The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2 */
}

void adBms6830_init_config(adbms6830_driver_t* dev,
						   uint8_t num_ics,
						   adbms6830_asic* ics,
						   SPI_HandleTypeDef* hspi_a,
						   SPI_HandleTypeDef* hspi_b,
						   GPIO_TypeDef* cs_port_a,
						   GPIO_TypeDef* cs_port_b,
						   uint16_t cs_pin_a,
						   uint16_t cs_pin_b)
{
	uint16_t vov_value;
	uint16_t vuv_value;
	uint8_t rbits = 12;
	float over_voltage = dev->thresholds.OV_THRESHOLD;
	float under_voltage = dev->thresholds.UV_THRESHOLD;

	dev->num_ics = num_ics;
	dev->ics = ics;
	dev->hspi_a = hspi_a;
	dev->hspi_b = hspi_b;
	dev->cs_port_a = cs_port_a;
	dev->cs_port_b = cs_port_b;
	dev->cs_pin_a = cs_pin_a;
	dev->cs_pin_b = cs_pin_b;

	/* ADC Command Configuration */
	dev->adc_config.REDUNDANT_MEASUREMENT = RD_OFF;
	dev->adc_config.AUX_CH_TO_CONVERT = AUX_ALL;
	dev->adc_config.CONTINUOUS_MEASUREMENT = SINGLE;
	dev->adc_config.CELL_OPEN_WIRE_DETECTION = OW_OFF_ALL_CH;
	dev->adc_config.AUX_OPEN_WIRE_DETECTION = AUX_OW_OFF;
	dev->adc_config.OPEN_WIRE_CURRENT_SOURCE = PUP_DOWN;
	dev->adc_config.DISCHARGE_PERMITTED = DCP_OFF;
	dev->adc_config.RESET_FILTER = RSTF_OFF;
	dev->adc_config.INJECT_ERR_SPI_READ = WITHOUT_ERR;

	/* Set Over & Under Voltage conditions */
	dev->thresholds.OV_THRESHOLD = 4.2;
	dev->thresholds.UV_THRESHOLD = 3.0;
	dev->thresholds.OWC_Threshold = 2000;
	dev->thresholds.OWA_Threshold = 50000;

	/* Loop measurement setup */
	dev->loop_manager.LOOP_MEASUREMENT_COUNT = 1;
	dev->loop_manager.MEASUREMENT_LOOP_TIME = 10;
	dev->loop_manager.loop_count = 0;
	dev->loop_manager.pladc_count = 0;
	dev->loop_manager.MEASURE_CELL = ENABLED;
	dev->loop_manager.MEASURE_AVG_CELL = ENABLED;
	dev->loop_manager.MEASURE_F_CELL = ENABLED;
	dev->loop_manager.MEASURE_S_VOLTAGE = ENABLED;
	dev->loop_manager.MEASURE_AUX = DISABLED;
	dev->loop_manager.MEASURE_RAUX = DISABLED;
	dev->loop_manager.MEASURE_STAT = DISABLED;

	for(uint8_t i = 0; i < dev->num_ics; i++)
	{
		/* Setup cell_asic */
		/* Init config A */
		dev->ics[i].tx_cfga.refon = PWR_UP;
		dev->ics[i].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */

		/* Init config B */
		over_voltage = (over_voltage - 1.5);
		over_voltage = over_voltage / (16 * 0.000150);
		vov_value = (uint16_t )(over_voltage + 2 * (1 << (rbits - 1)));
		vov_value &= 0xFFF;
		dev->ics[i].tx_cfgb.vov = vov_value;

		under_voltage = (under_voltage - 1.5);
		under_voltage = under_voltage / (16 * 0.000150);
		vuv_value = (uint16_t )(under_voltage + 2 * (1 << (rbits - 1)));
		vuv_value &= 0xFFF;
		dev->ics[i].tx_cfgb.vuv = vuv_value;
	}

	wakeup_ics(dev);
	adbms6830_write_data(dev, WRCFGA, Config, A);
	adbms6830_write_data(dev, WRCFGB, Config, B);
}

void wakeup_ics(adbms6830_driver_t* dev)
{
	for(int i = 0; i < dev->num_ics; i++)
	{
		HAL_GPIO_WritePin(dev->cs_port_a, dev->cs_pin_a, 0);
		HAL_Delay(CS_WAIT_TIME);
		HAL_GPIO_WritePin(dev->cs_port_a, dev->cs_pin_a, 1);
		HAL_Delay(CS_WAIT_TIME);
	}
}

void adbms6830_write_data(adbms6830_driver_t *dev, uint8_t cmd[CMDSZ], TYPE type, GRP group)
{
	uint8_t cmd_len = 4 + (RX_DATA * dev->num_ics);
	uint16_t pec15, pec10;
	adbms6830_asic *ics = dev->ics;

	for(uint8_t i = 0; i < TX_DATA * dev->num_ics; i++) shared_buf[i] = 0;
	for(uint8_t i = 0; i < 100; i++) write_buf[i] = 0;

	// configure registers
	switch(type)
	{
	case Config:
		adbms6830_create_config(dev, group);
		for (uint8_t cic = 0; cic < dev->num_ics; cic++)
		{
			for (uint8_t data = 0; data < TX_DATA; data++)
			{
				if(group == A) shared_buf[(cic * TX_DATA) + data] = ics[cic].configa.tx_data[data];
				else shared_buf[(cic * TX_DATA) + data] = ics[cic].configb.tx_data[data];
		    }
		}
		break;

	case Comm:
	      adbms6830_create_comm(dev);
	      for (uint8_t cic = 0; cic < dev->num_ics; cic++)
	      {
	    	  for (uint8_t data = 0; data < TX_DATA; data++)
			  {
	    		  shared_buf[(cic * TX_DATA) + data] = ics[cic].com.tx_data[data];
	          }
	      }
	      break;

	case Clrflag:
		adbms6830_create_clr_flag_data(dev);
		for (uint8_t cic = 0; cic < dev->num_ics; cic++)
		{
			for (uint8_t data = 0; data < TX_DATA; data++)
			{
				shared_buf[(cic * TX_DATA) + data] = ics[cic].clrflag.tx_data[data];
 		    }
		}
		break;
	}

	wakeup_ics(dev);

	// begin start of spi_write from Analog Devices library
//	write_buf[0] // dunno what I was starting to do here

	adbms6830_spi_write(dev, &shared_buf, TX_DATA * dev->num_ics, 1);
}

void adbms6830_create_config(adbms6830_driver_t* dev, GRP group)
{
	adbms6830_asic *ics = dev->ics;
	if(group == A)
	{
		for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
		{
			ics[curr_ic].configa.tx_data[0] = (((ics[curr_ic].tx_cfga.refon & 0x01) << 7) | (ics[curr_ic].tx_cfga.cth & 0x07));
			ics[curr_ic].configa.tx_data[1] = (ics[curr_ic].tx_cfga.flag_d & 0xFF);
			ics[curr_ic].configa.tx_data[2] = (((ics[curr_ic].tx_cfga.soakon & 0x01) << 7) | ((ics[curr_ic].tx_cfga.owrng & 0x01) << 6) | ((ics[curr_ic].tx_cfga.owa & 0x07) << 3));
			ics[curr_ic].configa.tx_data[3] = ((ics[curr_ic].tx_cfga.gpo & 0x00FF));
			ics[curr_ic].configa.tx_data[4] = ((ics[curr_ic].tx_cfga.gpo & 0x0300)>>8);
			ics[curr_ic].configa.tx_data[5] = (((ics[curr_ic].tx_cfga.snap & 0x01) << 5) | ((ics[curr_ic].tx_cfga.mute_st & 0x01) << 4) | ((ics[curr_ic].tx_cfga.comm_bk & 0x01) << 3) | (ics[curr_ic].tx_cfga.fc & 0x07));
		}
		return ;
	}
	// group B
	for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	{
		ics[curr_ic].configb.tx_data[0] = ((ics[curr_ic].tx_cfgb.vuv ));
		ics[curr_ic].configb.tx_data[1] = (((ics[curr_ic].tx_cfgb.vov & 0x000F) << 4) | ((ics[curr_ic].tx_cfgb.vuv ) >> 8));
		ics[curr_ic].configb.tx_data[2] = ((ics[curr_ic].tx_cfgb.vov >>4)&0x0FF);
		ics[curr_ic].configb.tx_data[3] = (((ics[curr_ic].tx_cfgb.dtmen & 0x01) << 7) | ((ics[curr_ic].tx_cfgb.dtrng & 0x01) << 6) | ((ics[curr_ic].tx_cfgb.dcto & 0x3F) << 0));
		ics[curr_ic].configb.tx_data[4] = ((ics[curr_ic].tx_cfgb.dcc & 0xFF));
		ics[curr_ic].configb.tx_data[5] = ((ics[curr_ic].tx_cfgb.dcc >>8 ));
	}
}

void adbms6830_create_comm(adbms6830_driver_t* dev)
{
	adbms6830_asic *ics = dev->ics;
	for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	{
		ics[curr_ic].com.tx_data[0] = ((ics[curr_ic].comm.icomm[0] & 0x0F)  << 4  | (ics[curr_ic].comm.fcomm[0]   & 0x0F));
		ics[curr_ic].com.tx_data[1] = ((ics[curr_ic].comm.data[0] ));
		ics[curr_ic].com.tx_data[2] = ((ics[curr_ic].comm.icomm[1] & 0x0F)  << 4 ) | (ics[curr_ic].comm.fcomm[1]   & 0x0F);
		ics[curr_ic].com.tx_data[3] = ((ics[curr_ic].comm.data[1]));
		ics[curr_ic].com.tx_data[4] = ((ics[curr_ic].comm.icomm[2] & 0x0F)  << 4  | (ics[curr_ic].comm.fcomm[2]   & 0x0F));
		ics[curr_ic].com.tx_data[5] = ((ics[curr_ic].comm.data[2]));
	}
}

void adbms6830_create_clr_flag_data(adbms6830_driver_t* dev)
{
	adbms6830_asic *ics = dev->ics;
	for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	{
		ics[curr_ic].clrflag.tx_data[0] = (ics[curr_ic].clflag.cl_csflt & 0x00FF);
		ics[curr_ic].clrflag.tx_data[1] = ((ics[curr_ic].clflag.cl_csflt & 0xFF00) >> 8);
		ics[curr_ic].clrflag.tx_data[2] = 0x00;
		ics[curr_ic].clrflag.tx_data[3] = 0x00;
		ics[curr_ic].clrflag.tx_data[4] = ((ics[curr_ic].clflag.cl_vaov << 7) | (ics[curr_ic].clflag.cl_vauv << 6) | (ics[curr_ic].clflag.cl_vdov << 5) | (ics[curr_ic].clflag.cl_vduv << 4)
			  																  |(ics[curr_ic].clflag.cl_ced << 3)   | (ics[curr_ic].clflag.cl_cmed << 2) | (ics[curr_ic].clflag.cl_sed << 1) | (ics[curr_ic].clflag.cl_smed));
		ics[curr_ic].clrflag.tx_data[5] = ((ics[curr_ic].clflag.cl_vde << 7)  | (ics[curr_ic].clflag.cl_vdel << 6) | (ics[curr_ic].clflag.cl_spiflt << 4) |(ics[curr_ic].clflag.cl_sleep << 3)
																			  | (ics[curr_ic].clflag.cl_thsd << 2) | (ics[curr_ic].clflag.cl_tmode << 1) | (ics[curr_ic].clflag.cl_oscchk));
	}
}

void adbms6830_spi_write(adbms6830_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs)
{
	if(use_cs) adbms6830_set_cs(dev, 0);
	HAL_SPI_Transmit(dev->hspia, data, len, SPI_TIMEOUT);
	if(use_cs) adbms6830_set_cs(dev, 1);
}

void adbms6830_cmd(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ])
{
	uint16_t pec15;

	write_buf[0] = cmd[0];
	write_buf[1] = cmd[1];
	pec15 = pec15_calc(CMDSZ, cmd);
	write_buf[2] = (uint8_t)(pec15 >> 8);
	write_buf[3] = (uint8_t)(pec15);
	adbms6830_spi_write(dev, write_buf, CMDSZ + PEC15SZ, 1);
}


void adBms6830_write_read_config(adbms6830_driver_t* dev)
{
  adBmsWakeupIc(dev->num_ics);
  adbms_write_data(dev, WRCFGA, Config, A);
  adbms_write_data(dev, WRCFGB, Config, B);
  adbms_read_data(dev, RDCFGA, Config, A);
  adbms_read_data(dev, RDCFGB, Config, B);
//  printWriteConfig(dev, Config, ALL_GRP);
//  printReadConfig(dev, Config, ALL_GRP);
}


// Tx/Rx Utility
void adbms6830_wr48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data)
{
	uint16_t pec15;
	uint16_t pec10;
	uint16_t tx_sz = CMDSZ + PEC15SZ + ((TX_DATA + DPECSZ) * dev->num_ics);
	uint16_t cmd_index;
	uint8_t src_addr = 0;
	uint8_t temp[TX_DATA];

	write_buf[0] = cmd[0];
	write_buf[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	write_buf[2] = (uint8_t)(pec15 >> 8);
	write_buf[3] = (uint8_t)pec15;
	cmd_index = 4;

	for (uint8_t current_ic = dev->num_ics; current_ic > 0; current_ic--)
	{
	  src_addr = ((current_ic-1) * TX_DATA);
	  /*!< The first configuration written is received by the last IC in the daisy chain */
	  for (uint8_t current_byte = 0; current_byte < TX_DATA; current_byte++)
	  {
		write_buf[cmd_index] = tx_data[((current_ic-1)*6)+current_byte];
		cmd_index = cmd_index + 1;
	  }
	  /*!< Copy each ic correspond data + pec value for calculate data pec */
	  memcpy(temp, &tx_data[src_addr], TX_DATA); /*!< dst, src, size */
	  /*!< calculating the PEC for each Ics configuration register data */
	  // pec10 = (uint16_t)pec10_calc_modular(temp, PEC10_WRITE);
	  // data_pec = (uint16_t)pec10_calc(true,BYTES_IN_REG, &copyArray[0]);
	  write_buf[cmd_index] = (uint8_t)(pec10 >> 8);
	  cmd_index = cmd_index + 1;
	  write_buf[cmd_index] = (uint8_t)pec10;
	  cmd_index = cmd_index + 1;
	}

	adbms6830_spi_write(dev, write_buf, tx_sz, 1);
}


void adbms6830_rd48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data)
{
	uint16_t pec15, received_pec, calculated_pec;
	uint16_t rx_sz = RX_DATA * dev->num_ics;
	uint8_t wrcmd[CMDSZ + PEC15SZ] = {0};
	uint8_t src_addr = 0;
	uint8_t temp[RX_DATA];

	wrcmd[0] = cmd[0];
	wrcmd[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	wrcmd[2] = (uint8_t)(pec15 >> 8);
	wrcmd[3] = (uint8_t)pec15;

	adbms6830_spi_write_read(dev, wrcmd, CMDSZ + PEC15SZ, rx_data, rx_sz, 1);

    for (uint8_t current_ic = 0; current_ic < dev->num_ics; current_ic++)     /*!< executes for each ic in the daisy chain and packs the data */
    {
      for (uint8_t current_byte = 0; current_byte < (RX_DATA); current_byte++)
      {
        rx_data[(current_ic * RX_DATA) + current_byte] = rx_data[current_byte + (current_ic * RX_DATA)];
      }
      /*!< Get received pec value from ic*/
      received_pec = (uint16_t)(((rx_data[(current_ic * RX_DATA) + (RX_DATA - 2)] & 0x03) << 8) | rx_data[(current_ic * RX_DATA) + (RX_DATA - 1)]);
      /*!< Copy each ic correspond data + pec value for calculate data pec */
      memcpy(temp, &rx_data[src_addr], RX_DATA);
      src_addr = ((current_ic+1) * (RX_DATA));
      /*!< Calculate data pec */
      //calculated_pec = (uint16_t) pec10_calc(1, (RX_DATA - DPECSZ), temp);
     //dev->ics[current_ic].rx_pec_error = (received_pec != calculated_pec);
    }
}

// SPI communication
void adbms6830_set_cs(adbms6830_driver_t* dev, uint8_t state)
{
	HAL_GPIO_WritePin(dev->cs_port[dev->string], dev->cs_pin[dev->string], state);
}


void adbms6830_spi_write_read(adbms6830_driver_t *dev, uint8_t* tx_Data, uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len, uint8_t use_cs)
{
	HAL_StatusTypeDef ret = 0;
	if(use_cs) adbms6830_set_cs(dev, 0);
	ret |= HAL_SPI_Transmit(dev->hspi, tx_Data, tx_len, 100);
	ret |= HAL_SPI_Receive(dev->hspi, rx_data, rx_len, 100);
	if(use_cs) adbms6830_set_cs(dev, 1);
}


