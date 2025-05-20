/*
 * adbms6830.c
 *
 *  Created on: May 13, 2025
 *      Author: realb
 */

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
	adbms_write_data(dev, WRCFGA, Config, A);
	adbms_write_data(dev, WRCFGB, Config, B);
}

void wakeup_ics(adbms6830_driver_t* dev)
{
	for(int i = 0; i < dev->num_ics; i++)
	{
		//TODO: should it be both SPIs?
		HAL_GPIO_WritePin(dev->cs_port_a, dev->cs_pin_a, 0);
		HAL_Delay(1);
		HAL_GPIO_WritePin(dev->cs_port_a, dev->cs_pin_a, 1);
		HAL_Delay(1);
	}
}

void adbms_write_data(adbms6830_driver_t *dev, uint8_t cmd_arg[2], TYPE type, GRP group)
{
	adbms6830_asic *ics = dev->ics;
	for(uint8_t i = 0; i < TX_DATA * dev->num_ics; i++)
	{
		shared_buf[i] = 0;
	}

	switch(type)
	{
	case Config:
		adbms6830_create_config(dev, group);
		for (uint8_t cic = 0; cic < dev->num_ics; cic++)
		{
			for (uint8_t data = 0; data < TX_DATA; data++)
			{
				if(group == A) shared_buf[(cic * TX_DATA) + data] = ics[cic].configa.tx_data[data];
				else  shared_buf[(cic * TX_DATA) + data] = ics[cic].configb.tx_data[data];
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
//	spi_write_data(tIC, cmd_arg, &write_buffer[0]);  //TODO: implement
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





