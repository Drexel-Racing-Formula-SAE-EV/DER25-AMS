/*
 * adbms6830.c
 *
 *  Created on: May 13, 2025
 *      Author: realb
 */

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

#include "ext_drivers/adbms6830_data.h"
#include "ext_drivers/adbms6830_functions.h"

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

void adbms6830_write_data(adbms6830_driver_t *dev, uint8_t cmd[ADBMS6830_CMD_SIZE], TYPE type, GRP group)
{
	uint8_t cmd_len = 4 + (RX_DATA * dev->num_ics);
	uint8_t
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
	write_buf[0] = cmd

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
	if(use_cs) HAL_GPIO_WritePin(dev->cs_port_a, dev->cs_pin_a, 0);
	HAL_SPI_Transmit(dev->hspia, data, len, SPI_TIMEOUT);
	if(use_cs) HAL_GPIO_WritePin(dev->cs_port_a, dev->cs_pin_a, 1);
}

void adbms6830_write_cmd(adbms6830_driver_t* dev, uint8_t cmd[ADBMS6830_CMD_SIZE])
{
	uint16_t pec15;

	write_buf[0] = cmd[0];
	write_buf[1] = cmd[1];
	pec15 = pec15_calc(ADBMS6830_CMD_SIZE, cmd);
	write_buf[2] = (uint8_t)(pec15 >> 8);
	write_buf[3] = (uint8_t)(pec15);
	adbms6830_spi_write(dev, write_buf, ADBMS6830_CMD_SIZE + PEC15_SIZE, 1);
}


void adBms6830_write_read_config(adbms6830_driver_t* dev)
{
  adBmsWakeupIc(dev->num_ics);
  adbms_write_data(dev, WRCFGA, Config, A);
  adbms_write_data(dev, WRCFGB, Config, B);
  adbms_read_data(dev, RDCFGA, Config, A);
  adbms_read_data(dev, RDCFGB, Config, B);
  printWriteConfig(dev, Config, ALL_GRP);
  printReadConfig(dev, Config, ALL_GRP);
}


void adbms2950_wrdata(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data)
{
	uint16_t pec15;
	uint16_t pec10;
	uint16_t tx_sz = CMDSZ + PEC15SZ + ((TX_DATA + DPECSZ) * dev->num_ics);
	uint16_t cmd_index;
	uint8_t src_addr;

	wrbuf[0] = cmd[0];
	wrbuf[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	wrbuf[2] = (uint8_t)(pec15 >> 8);
	wrbuf[3] = (uint8_t)pec15;
	cmd_index = 4;

	for(uint8_t ic = dev->num_ics; ic < dev->num_ics; ic--)
	{
		src_addr = cmd_index;
		for (uint8_t current_byte = 0; current_byte < TX_DATA; current_byte++)
		{
			wrbuf[cmd_index] = tx_data[((ic - 1) * TX_DATA) + current_byte];
			cmd_index++;
		}
		pec10 = pec10_calc_modular(&wrbuf[src_addr], PEC10_WRITE);
		wrbuf[cmd_index++] = (uint8_t)(pec10 >> 8);
		wrbuf[cmd_index++] = (uint8_t)pec10;
	}

	adbms2950_spi_write(dev, wrbuf, tx_sz, 1);
}

void adbms2950_rddata(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data, uint8_t size)
{
	uint16_t pec15;
	uint16_t rx_sz = size * dev->num_ics;
	uint8_t wrcmd[CMDSZ + PEC15SZ] = {0};

	wrcmd[0] = cmd[0];
	wrcmd[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	wrcmd[2] = (uint8_t)(pec15 >> 8);
	wrcmd[3] = (uint8_t)pec15;

	adbms2950_spi_write_read(dev, wrcmd, CMDSZ + PEC15SZ, buf, rx_sz, 1);
}




