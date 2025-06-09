/*
 * adbms2950.c
 *
 *  Created on: May 13, 2025
 *      Author: cole
 */

#include "ext_drivers/adbms2950.h"
#include <string.h>

uint8_t buf[BUFSZ] = {0};
uint8_t wrbuf[BUFSZ] = {0};

/*!< configuration registers commands */
uint8_t WRCFGA[2]        = { 0x00, 0x01 };
uint8_t WRCFGB[2]        = { 0x00, 0x24 };
uint8_t RDCFGA[2]        = { 0x00, 0x02 };
uint8_t RDCFGB[2]        = { 0x00, 0x26 };

/* Read VBxADC and IxADC result registers commands */
uint8_t RDI[2]           = { 0x00, 0x04 };
uint8_t RDVB[2]          = { 0x00, 0x06 };
uint8_t RDIVB1[2]        = { 0x00, 0x08 };
uint8_t RDIACC[2]        = { 0x00, 0x44 };
uint8_t RDVBACC[2]       = { 0x00, 0x46 };
uint8_t RDIVB1ACC[2]      = { 0x00, 0x48 };

/* Read OCxADC result registers commands */
uint8_t RDOC[2]         = { 0x00, 0x0B };

/* Read VxADC result registers commands */
uint8_t RDV1A[2]         = { 0x00, 0x0A };
uint8_t RDV1B[2]         = { 0x00, 0x09 };
uint8_t RDV1C[2]         = { 0x00, 0x03 };
uint8_t RDV1D[2]         = { 0x00, 0x1B };
uint8_t RDV2A[2]         = { 0x00, 0x07 };//RDRVA
uint8_t RDV2B[2]         = { 0x00, 0x0D };
uint8_t RDV2C[2]         = { 0x00, 0x05 };
uint8_t RDV2D[2]         = { 0x00, 0x1F };
uint8_t RDV2E[2]         = { 0x00, 0x25 };

/* Read Status register */
uint8_t RDSTAT[2]       = { 0x00, 0x34 };

/* Read Flag register */
uint8_t RDFLAG[2]       = { 0x00, 0x32 };
uint8_t RDFLAGERR[2]    = { 0x00, 0x72 };   /* ERR */

/* Read AUX ADC result registers */
uint8_t RDXA[2]       = { 0x00, 0x30 };
uint8_t RDXB[2]       = { 0x00, 0x31 };
uint8_t RDXC[2]       = { 0x00, 0x33 };

/* Read all commands */
//------Read All IxADC and VBxADC results+Status+Flag-------
uint8_t RDALLI[2]        = { 0x00, 0x0C };

//------Read All IxACC and VBxACC results+Status+Flag-------
uint8_t RDALLA[2]        = { 0x00, 0x4C };

//------Read All configuration registers+Status+Flag-------
uint8_t RDALLC[2]        = { 0x00, 0x10 };

//------Read All Voltages-------
uint8_t RDALLV[2]        = { 0x00, 0x35 };

//------Read All Redundant Voltages-------
uint8_t RDALLR[2]        = { 0x00, 0x11 };

//------Read All Aux Voltages-------
uint8_t RDALLX[2]        = { 0x00, 0x51 };

/* Pwm registers commands */
uint8_t WRPWMA[2]         = { 0x00, 0x20 };
uint8_t RDPWMA[2]         = { 0x00, 0x22 };
uint8_t WRPWMB[2]         = { 0x00, 0x21 };
uint8_t RDPWMB[2]         = { 0x00, 0x23 };

/* Clear commands */
//uint8_t CLRAB[2]         = { 0x07, 0x11 };
uint8_t CLRI[2]         = { 0x07, 0x11 };
uint8_t CLRA[2]         = { 0x07, 0x14 };
uint8_t CLRO[2]         = { 0x07, 0x13 };
uint8_t CLRC[2]          = { 0x07, 0x16 };//Ask about CLRC//Sayani
uint8_t CLRVX [2]       = { 0x07, 0x12 };
//uint8_t CLRSTAT [2]      = { 0x07, 0x13 };
uint8_t CLRFLAG[2]       = { 0x07, 0x17 };

/*!< Poll adc command */
uint8_t PLADC[2]         = { 0x07, 0x18 };
uint8_t PLI1[2]       = { 0x07, 0x1C };
uint8_t PLI2[2]       = { 0x07, 0x1D };
uint8_t PLV[2]        = { 0x07, 0x1E };
uint8_t PLX[2]         = { 0x07, 0x1F };

/*!< GPIOs Comm commands */
uint8_t WRCOMM[2]        = { 0x07, 0x21 };
uint8_t RDCOMM[2]        = { 0x07, 0x22 };
/*!< command + dummy data for 72 clock cycles */
uint8_t STCOMM[13]       = { 0x07, 0x23, 0xB9, 0xE4 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};

/*!< Control Commands */
uint8_t RDSID[2]         = { 0x00, 0x2C };
uint8_t RSTCC[2]         = { 0x00, 0x2E };
uint8_t SNAP[2]          = { 0x00, 0x2D };
uint8_t UNSNAP[2]        = { 0x00, 0x2F };
uint8_t SRST[2]          = { 0x00, 0x27 };
uint8_t sADI1[2]         = { 0x02, 0x60 };
uint8_t sADI2[2]         = { 0x01, 0x68 };
uint8_t sADV[2]          = { 0x04, 0x30 };
uint8_t sADX[2]          = { 0x05, 0x30 };

//Command + pec
uint8_t RSTATD[4]        = {0x00, 0x33, 0x4D, 0x4A};  //Command +Pec
//uint8_t RSTATC[4]        = {0x00, 0x32, 0xc6, 0x78}; // Tiger CC is in Status C
uint8_t RFLAG[4]        = {0x00, 0x32, 0xc6, 0x78}; // Tiger CC is in Flag//Check PEC code//Sayani//Put breakpoint at Pec15_Calc for RDFLAG to check
uint8_t sRDI[4]          = {0x00, 0x04, 0x07, 0xC2};
uint8_t sCLRAB[4]        = {0x07,0x11,0xC9,0xC0};
//uint8_t sRSTATA [4]      = { 0x00, 0x30, 0x5B, 0x2E };
uint8_t sRDVA [4]        = { 0x00, 0x0A ,  0xC3 , 0x04};
uint8_t sRDVB [4]        = { 0x00, 0x09 , 0xD5 , 0x60};
uint8_t sRDVC [4]        = { 0x00, 0x03 , 0xA0, 0x38};
uint8_t sRDVD [4]        = { 0x00, 0x05 , 0x8C, 0xF0};
uint8_t sRDIAV[4]        = { 0x00, 0x44 , 0xE0, 0x48};
uint8_t sRDVBAT[4]       = { 0x00, 0x06 , 0x9A, 0x94};
/* Testmode and debugging commands */
uint8_t TM_48[2]       = { 0x00, 0x0E };        // LION: RDSVE

// Tx/Rx Utility
void adbms2950_cmd(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ]);
void adbms2950_wr48(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data);
void adbms2950_rd48(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data);

// SPI communication
void adbms2950_set_cs(adbms2950_driver_t* dev, uint8_t state);
void adbms2950_spi_write(adbms2950_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs);
void adbms2950_spi_write_read(adbms2950_driver_t *dev, uint8_t* tx_Data, uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len, uint8_t use_cs);

// Data parsing
void adbms2950_parse_cfga(adbms2950_driver_t* dev, uint8_t* data);
void adbms2950_parse_cfgb(adbms2950_driver_t* dev, uint8_t* data);
void adbms2950_parse_rdvb(adbms2950_driver_t* dev, uint8_t* vbat_data);
void adbms2950_parse_rdi(adbms2950_driver_t* dev, uint8_t* i_data);
void adbms2950_parse_rdv1d(adbms2950_driver_t* dev, uint8_t* v_data);

// Data packing
void adbms2950_pack_cfga(adbms2950_driver_t* dev);
void adbms2950_pack_cfgb(adbms2950_driver_t* dev);

// Data validation
uint16_t Pec15_Calc(uint8_t len, uint8_t *data);
uint16_t pec10_calc(uint8_t rx_cmd, int len, uint8_t *data);
uint16_t pec10_calc_modular(uint8_t * data, uint8_t PEC_Format);
uint16_t pec10_calc_int(uint16_t remainder, uint8_t bit);

void adbms2950_init(adbms2950_driver_t *dev,
					uint8_t num_asics,
					adbms2950_asic *ics,
					SPI_HandleTypeDef *hspi,
					GPIO_TypeDef *CSA_Port,
					GPIO_TypeDef *CSB_Port,
					uint16_t CSA_Pin,
					uint16_t CSB_Pin,
					TIM_HandleTypeDef *htim)
{
	dev->num_ics = num_asics;
	dev->ics = ics;
	dev->hspi = hspi;
	dev->cs_port[0] = CSA_Port;
	dev->cs_port[1] = CSB_Port;
	dev->cs_pin[0] = CSA_Pin;
	dev->cs_pin[1] = CSB_Pin;
	dev->htim = htim;

	// Set CS pins high
	dev->string = STRING_B;
	adbms2950_set_cs(dev, 1);
	dev->string = STRING_A;
	adbms2950_set_cs(dev, 1);

	adbms2950_wakeup(dev);
	adbms2950_srst(dev);
	// 8ms delay. DS and vendor code recommend
	adbms2950_us_delay(dev, 8000);

	adbms2950_reset_cfg_regs(dev);
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		// GPO1 as PUSH-PULL, set to LOW
		dev->ics[cic].tx_cfga.gpo1od = PUSH_PULL;
		dev->ics[cic].tx_cfga.gpo1c = PULLED_DOWN;

		// GPO2 as PUSH-PULL, set to LOW
		dev->ics[cic].tx_cfga.gpo2od = PUSH_PULL;
		dev->ics[cic].tx_cfga.gpo2c = PULLED_DOWN;
	}

	adbms2950_wakeup(dev);
	adbms2950_wrcfga(dev);
	adbms2950_wrcfgb(dev);
	adbms2950_rdcfga(dev);
	adbms2950_rdcfgb(dev);

	// Using Redundant, Continuous measurement
	// See Table 42 page 34
	adi1_ adi1;
	adi1.rd = RD_ON; // Redundant measurement on, starts VB2ADC and I2ADC as well
	adi1.opt = OPT12_C; // Continuous measurement
	adbms2950_wakeup(dev);
	adbms2950_adi1(dev, &adi1);

	// Don't need this call since RD_ON starts VB2ADC and I2ADC above
	/*adi2_ adi2;
	adi2.opt = OPT12_C;
	adbms2950_wakeup(dev);
	adbms2950_adi2(dev, &adi2);
	*/

	// Add delay for device to start
	adbms2950_us_delay(dev, 8000);
}

void adbms2950_cmd(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ])
{
	uint16_t pec15;
	wrbuf[0] = cmd[0];
	wrbuf[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	wrbuf[2] = (uint8_t)(pec15 >> 8);
	wrbuf[3] = (uint8_t)pec15;

	adbms2950_spi_write(dev, wrbuf, CMDSZ + PEC15SZ, 1);
}

void adbms2950_wr48(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data)
{
	uint16_t pec15;
	uint16_t pec10;
	uint16_t tx_sz = CMDSZ + PEC15SZ + ((TX_DATA + DPECSZ) * dev->num_ics);
	uint16_t cmd_index;
	uint8_t src_addr = 0;
	uint8_t temp[TX_DATA];

	wrbuf[0] = cmd[0];
	wrbuf[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	wrbuf[2] = (uint8_t)(pec15 >> 8);
	wrbuf[3] = (uint8_t)pec15;
	cmd_index = 4;

    for (uint8_t current_ic = dev->num_ics; current_ic > 0; current_ic--)
    {
      src_addr = ((current_ic-1) * TX_DATA);
      /*!< The first configuration written is received by the last IC in the daisy chain */
      for (uint8_t current_byte = 0; current_byte < TX_DATA; current_byte++)
      {
        wrbuf[cmd_index] = tx_data[((current_ic-1)*6)+current_byte];
        cmd_index = cmd_index + 1;
      }
      /*!< Copy each ic correspond data + pec value for calculate data pec */
      memcpy(temp, &tx_data[src_addr], TX_DATA); /*!< dst, src, size */
      /*!< calculating the PEC for each Ics configuration register data */
      pec10 = (uint16_t)pec10_calc_modular(temp, PEC10_WRITE);
      // data_pec = (uint16_t)pec10_calc(true,BYTES_IN_REG, &copyArray[0]);
      wrbuf[cmd_index] = (uint8_t)(pec10 >> 8);
      cmd_index = cmd_index + 1;
      wrbuf[cmd_index] = (uint8_t)pec10;
      cmd_index = cmd_index + 1;
    }

	adbms2950_spi_write(dev, wrbuf, tx_sz, 1);
}

void adbms2950_rd48(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data)
{
	uint16_t pec15;
	uint16_t rx_sz = RX_DATA * dev->num_ics;
	uint8_t wrcmd[CMDSZ + PEC15SZ] = {0};
	uint8_t src_addr = 0;
	uint16_t received_pec, calculated_pec;
	uint8_t temp[RX_DATA]; // should technically be RX_DATA but this is the rd48 and is only used for this size transmission

	wrcmd[0] = cmd[0];
	wrcmd[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	wrcmd[2] = (uint8_t)(pec15 >> 8);
	wrcmd[3] = (uint8_t)pec15;

	adbms2950_spi_write_read(dev, wrcmd, CMDSZ + PEC15SZ, rx_data, rx_sz, 1);

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
      calculated_pec = (uint16_t)pec10_calc(1, (RX_DATA - DPECSZ), temp);

      dev->ics[current_ic].rx_pec_error = (received_pec != calculated_pec);
    }
}

void adbms2950_reset_cfg_regs(adbms2950_driver_t* dev)
{
	// Set device registers to default
	for(uint8_t i = 0; i < dev->num_ics; i++)
	{
		//CFGA
		dev->ics[i].tx_cfga.gpo1c = PULLED_UP_TRISTATED;
		dev->ics[i].tx_cfga.gpo2c = PULLED_UP_TRISTATED;
		dev->ics[i].tx_cfga.gpo3c = PULLED_UP_TRISTATED;
		dev->ics[i].tx_cfga.gpo4c = PULLED_UP_TRISTATED;
		dev->ics[i].tx_cfga.gpo5c = PULLED_UP_TRISTATED;
		dev->ics[i].tx_cfga.gpo6c = PULLED_UP_TRISTATED;

		dev->ics[i].tx_cfga.gpo1od = OPEN_DRAIN;
		dev->ics[i].tx_cfga.gpo2od = OPEN_DRAIN;
		dev->ics[i].tx_cfga.gpo3od = OPEN_DRAIN;
		dev->ics[i].tx_cfga.gpo4od = OPEN_DRAIN;
		dev->ics[i].tx_cfga.gpo5od = OPEN_DRAIN;
		dev->ics[i].tx_cfga.gpo6od = OPEN_DRAIN;

		dev->ics[i].tx_cfga.vs1  = VSM_SGND;
		dev->ics[i].tx_cfga.vs2  = VSM_SGND;
		dev->ics[i].tx_cfga.vs3  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs4  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs5  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs6  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs7  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs8  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs9  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs10 = VSMV_SGND;

		dev->ics[i].tx_cfga.injosc = INJOSC0_NORMAL;
		dev->ics[i].tx_cfga.injmon = INJMON0_NORMAL;
		dev->ics[i].tx_cfga.injts  = NO_THSD;
		dev->ics[i].tx_cfga.injecc = NO_ECC;
		dev->ics[i].tx_cfga.injtm  = NO_TMODE;

		dev->ics[i].tx_cfga.soak    = SOAK_DISABLE;
		dev->ics[i].tx_cfga.ocen    = OC_DISABLE;
		dev->ics[i].tx_cfga.gpio1fe = FAULT_STATUS_DISABLE;
		dev->ics[i].tx_cfga.spi3w   = FOUR_WIRE;

		dev->ics[i].tx_cfga.acci    = ACCI_8;
		dev->ics[i].tx_cfga.commbk  = COMMBK_OFF;
		dev->ics[i].tx_cfga.vb1mux  = SINGLE_ENDED_SGND;
		dev->ics[i].tx_cfga.vb2mux  = SINGLE_ENDED_SGND;

		//CFGB
		dev->ics[i].tx_cfgb.gpio1c = PULL_DOWN_OFF;
		dev->ics[i].tx_cfgb.gpio2c = PULL_DOWN_OFF;
		dev->ics[i].tx_cfgb.gpio3c = PULL_DOWN_OFF;
		dev->ics[i].tx_cfgb.gpio4c = PULL_DOWN_OFF;

		dev->ics[i].tx_cfgb.oc1th = 0x0;
		dev->ics[i].tx_cfgb.oc2th = 0x0;
		dev->ics[i].tx_cfgb.oc3th = 0x0;

		dev->ics[i].tx_cfgb.oc1ten = NORMAL_INPUT;
		dev->ics[i].tx_cfgb.oc2ten = NORMAL_INPUT;
		dev->ics[i].tx_cfgb.oc3ten = NORMAL_INPUT;

		dev->ics[i].tx_cfgb.ocdgt  = OCDGT0_1oo1;
		dev->ics[i].tx_cfgb.ocdp   = OCDP0_NORMAL;
		dev->ics[i].tx_cfgb.reften = NORMAL_INPUT;
		dev->ics[i].tx_cfgb.octsel = OCTSEL0_OCxADC_P140_REFADC_M20;

		dev->ics[i].tx_cfgb.ocod   = PUSH_PULL;
		dev->ics[i].tx_cfgb.oc1gc  = GAIN_1;
		dev->ics[i].tx_cfgb.oc2gc  = GAIN_1;
		dev->ics[i].tx_cfgb.oc3gc  = GAIN_1;
		dev->ics[i].tx_cfgb.ocmode = OCMODE0_DISABLED;
		dev->ics[i].tx_cfgb.ocax   = OCABX_ACTIVE_HIGH;
		dev->ics[i].tx_cfgb.ocbx   = OCABX_ACTIVE_HIGH;

		dev->ics[i].tx_cfgb.diagsel   = DIAGSEL0_IAB_VBAT;
		dev->ics[i].tx_cfgb.gpio2eoc  = EOC_DISABLED;
	}
}

void adbms2950_srst(adbms2950_driver_t* dev)
{
	adbms2950_cmd(dev, SRST);
}

void adbms2950_wrcfga(adbms2950_driver_t* dev)
{
	uint8_t address;

	adbms2950_pack_cfga(dev);
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		address = cic * TX_DATA;
		for(uint8_t byte = 0; byte < TX_DATA; byte++)
		{
			buf[address + byte] = dev->ics[cic].configa.tx_data[byte];
		}
	}
	adbms2950_wr48(dev, WRCFGA, buf);
}

void adbms2950_wrcfgb(adbms2950_driver_t* dev)
{
	uint8_t address;

	adbms2950_pack_cfgb(dev);
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		address = cic * TX_DATA;
		for(uint8_t byte = 0; byte < TX_DATA; byte++)
		{
			buf[address + byte] = dev->ics[cic].configb.tx_data[byte];
		}
	}
	adbms2950_wr48(dev, WRCFGB, buf);
}

void adbms2950_rdcfga(adbms2950_driver_t* dev)
{
	adbms2950_rd48(dev, RDCFGA, buf);
	adbms2950_parse_cfga(dev, buf);
}

void adbms2950_rdcfgb(adbms2950_driver_t* dev)
{
	adbms2950_rd48(dev, RDCFGB, buf);
	adbms2950_parse_cfgb(dev, buf);
}

void adbms2950_parse_cfga(adbms2950_driver_t* dev, uint8_t *data)
{
  uint8_t address = 0;
  adbms2950_asic* ic = dev->ics;
  for(uint8_t cic = 0; cic < dev->num_ics; cic++)
  {
	  address = cic * RX_DATA;
    memcpy(ic[cic].configa.rx_data, &data[address], RX_DATA);

    ic[cic].rx_cfga.vs1             = (ic[cic].configa.rx_data[0] & 0x03);
    ic[cic].rx_cfga.vs2             = (ic[cic].configa.rx_data[0] & 0x0C) >> 2;
    ic[cic].rx_cfga.vs3             = (ic[cic].configa.rx_data[0] & 0x10) >> 4;
    ic[cic].rx_cfga.vs4             = (ic[cic].configa.rx_data[0] & 0x20) >> 5;
    ic[cic].rx_cfga.vs5             = (ic[cic].configa.rx_data[0] & 0x40) >> 6;
    ic[cic].rx_cfga.ocen            = (ic[cic].configa.rx_data[0] & 0x80) >> 7;

    ic[cic].rx_cfga.injosc          = (ic[cic].configa.rx_data[1] & 0x03);
    ic[cic].rx_cfga.injmon          = (ic[cic].configa.rx_data[1] & 0x0C) >> 2;
    ic[cic].rx_cfga.injts           = (ic[cic].configa.rx_data[1] & 0x10) >> 4;
    ic[cic].rx_cfga.injecc          = (ic[cic].configa.rx_data[1] & 0x40) >> 6;
    ic[cic].rx_cfga.injtm           = (ic[cic].configa.rx_data[1] & 0x80) >> 7;

    ic[cic].rx_cfga.vs6             = (ic[cic].configa.rx_data[2] & 0x01);
    ic[cic].rx_cfga.vs7             = (ic[cic].configa.rx_data[2] & 0x02) >> 1;
    ic[cic].rx_cfga.vs8             = (ic[cic].configa.rx_data[2] & 0x04) >> 2;
    ic[cic].rx_cfga.vs9             = (ic[cic].configa.rx_data[2] & 0x08) >> 3;
    ic[cic].rx_cfga.vs10            = (ic[cic].configa.rx_data[2] & 0x10) >> 4;
    ic[cic].rx_cfga.soak            = (ic[cic].configa.rx_data[2] & 0xE0) >> 5;

    ic[cic].rx_cfga.gpo1c           = (ic[cic].configa.rx_data[3] & 0x01);
    ic[cic].rx_cfga.gpo2c           = (ic[cic].configa.rx_data[3] & 0x02) >> 1;
    ic[cic].rx_cfga.gpo3c           = (ic[cic].configa.rx_data[3] & 0x04) >> 2;
    ic[cic].rx_cfga.gpo4c           = (ic[cic].configa.rx_data[3] & 0x08) >> 3;
    ic[cic].rx_cfga.gpo5c           = (ic[cic].configa.rx_data[3] & 0x10) >> 4;
    ic[cic].rx_cfga.gpo6c           = (ic[cic].configa.rx_data[3] & 0x60) >> 5;

    ic[cic].rx_cfga.gpo1od          = (ic[cic].configa.rx_data[4] & 0x01);
    ic[cic].rx_cfga.gpo2od          = (ic[cic].configa.rx_data[4] & 0x02) >> 1;
    ic[cic].rx_cfga.gpo3od          = (ic[cic].configa.rx_data[4] & 0x04) >> 2;
    ic[cic].rx_cfga.gpo4od          = (ic[cic].configa.rx_data[4] & 0x08) >> 3;
    ic[cic].rx_cfga.gpo5od          = (ic[cic].configa.rx_data[4] & 0x10) >> 4;
    ic[cic].rx_cfga.gpo6od          = (ic[cic].configa.rx_data[4] & 0x20) >> 5;
    ic[cic].rx_cfga.gpio1fe         = (ic[cic].configa.rx_data[4] & 0x40) >> 6;
    ic[cic].rx_cfga.spi3w           = (ic[cic].configa.rx_data[4] & 0x80) >> 7;

    ic[cic].rx_cfga.acci            = (ic[cic].configa.rx_data[5] & 0x07);
    ic[cic].rx_cfga.commbk          = (ic[cic].configa.rx_data[5] & 0x08) >> 3;
    ic[cic].rx_cfga.refup           = (ic[cic].configa.rx_data[5] & 0x10) >> 4;
    ic[cic].rx_cfga.snapst          = (ic[cic].configa.rx_data[5] & 0x20) >> 5;
    ic[cic].rx_cfga.vb1mux          = (ic[cic].configa.rx_data[5] & 0x40) >> 6;
    ic[cic].rx_cfga.vb2mux          = (ic[cic].configa.rx_data[5] & 0x80) >> 7;
  }
}

void adbms2950_pack_cfga(adbms2950_driver_t* dev)
{
	adbms2950_asic *ic = dev->ics;
  for(uint8_t cic = 0; cic < dev->num_ics; cic++)
  {
    ic[cic].configa.tx_data[0] = (((ic[cic].tx_cfga.ocen & 0x01) << 7) | ((ic[cic].tx_cfga.vs5 & 0x01) << 6) | ((ic[cic].tx_cfga.vs4 & 0x01) << 5)
                                      | ((ic[cic].tx_cfga.vs3 & 0x01) << 4) | ((ic[cic].tx_cfga.vs2 & 0x03) << 2) | (ic[cic].tx_cfga.vs1 & 0x03));
    ic[cic].configa.tx_data[1] = (((ic[cic].tx_cfga.injtm & 0x01) << 7) | ((ic[cic].tx_cfga.injecc & 0x01) << 6) | ((ic[cic].tx_cfga.injts & 0x01) << 4)
                                      | ((ic[cic].tx_cfga.injmon & 0x03) << 2) | (ic[cic].tx_cfga.injosc & 0x03));
    ic[cic].configa.tx_data[2] = (((ic[cic].tx_cfga.soak & 0x07) << 5) | ((ic[cic].tx_cfga.vs10 & 0x01) << 4) | ((ic[cic].tx_cfga.vs9 & 0x01) << 3)
                                      | ((ic[cic].tx_cfga.vs8 & 0x01) << 2) | ((ic[cic].tx_cfga.vs7 & 0x01) << 1) | (ic[cic].tx_cfga.vs6 & 0x01));
    ic[cic].configa.tx_data[3] = (((ic[cic].tx_cfga.gpo6c & 0x03) << 5) | ((ic[cic].tx_cfga.gpo5c & 0x01) << 4)  | ((ic[cic].tx_cfga.gpo4c & 0x01) << 3)
                                      | ((ic[cic].tx_cfga.gpo3c & 0x01) << 2) | ((ic[cic].tx_cfga.gpo2c & 0x01) << 1)| (ic[cic].tx_cfga.gpo1c & 0x01));   // GPO1 is at position 0
    ic[cic].configa.tx_data[4] = (((ic[cic].tx_cfga.spi3w & 0x01)  << 7)| ((ic[cic].tx_cfga.gpio1fe & 0x01) << 6) | ((ic[cic].tx_cfga.gpo6od & 0x01) << 5)
                                      | ((ic[cic].tx_cfga.gpo5od & 0x01) << 4) |((ic[cic].tx_cfga.gpo4od & 0x01) << 3) | ((ic[cic].tx_cfga.gpo3od & 0x01) << 2)
                                        | ((ic[cic].tx_cfga.gpo2od & 0x01) << 1) | (ic[cic].tx_cfga.gpo1od & 0x01));
    ic[cic].configa.tx_data[5] = (((ic[cic].tx_cfga.vb2mux & 0x01)  << 7)| ((ic[cic].tx_cfga.vb1mux & 0x01) << 6) | ((ic[cic].tx_cfga.snapst & 0x01) << 5)
                                      | ((ic[cic].tx_cfga.refup & 0x01) << 4) |((ic[cic].tx_cfga.commbk & 0x01) << 3) | (ic[cic].tx_cfga.acci & 0x07));
  }
}

void adbms2950_parse_cfgb(adbms2950_driver_t* dev, uint8_t *data)
{
	uint8_t address = 0;
	adbms2950_asic *ic = dev->ics;
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		memcpy(ic[cic].configb.rx_data, &data[address], RX_DATA); /* dst , src , size */
		address = ((cic+1) * (RX_DATA));

		ic[cic].rx_cfgb.oc1th           = (ic[cic].configb.rx_data[0] & 0x7F);
		ic[cic].rx_cfgb.oc1ten          = (ic[cic].configb.rx_data[0] & 0x80) >> 7;

		ic[cic].rx_cfgb.oc2th           = (ic[cic].configb.rx_data[1] & 0x7F);
		ic[cic].rx_cfgb.oc2ten          = (ic[cic].configb.rx_data[1] & 0x80) >> 7;

		ic[cic].rx_cfgb.oc3th           = (ic[cic].configb.rx_data[2] & 0x7F);
		ic[cic].rx_cfgb.oc3ten          = (ic[cic].configb.rx_data[2] & 0x80) >> 7;

		ic[cic].rx_cfgb.ocdgt           = (ic[cic].configb.rx_data[3] & 0x03);
		ic[cic].rx_cfgb.ocdp            = (ic[cic].configb.rx_data[3] & 0x08) >> 3;
		ic[cic].rx_cfgb.reften          = (ic[cic].configb.rx_data[3] & 0x20) >> 5;
		ic[cic].rx_cfgb.octsel          = (ic[cic].configb.rx_data[3] & 0xC0) >> 6;

		ic[cic].rx_cfgb.ocod            = (ic[cic].configb.rx_data[4] & 0x01);
		ic[cic].rx_cfgb.oc1gc           = (ic[cic].configb.rx_data[4] & 0x02) >> 1;
		ic[cic].rx_cfgb.oc2gc           = (ic[cic].configb.rx_data[4] & 0x04) >> 2;
		ic[cic].rx_cfgb.oc3gc           = (ic[cic].configb.rx_data[4] & 0x08) >> 3;
		ic[cic].rx_cfgb.ocmode          = (ic[cic].configb.rx_data[4] & 0x30) >> 4;
		ic[cic].rx_cfgb.ocax            = (ic[cic].configb.rx_data[4] & 0x40) >> 6;
		ic[cic].rx_cfgb.ocbx            = (ic[cic].configb.rx_data[4] & 0x80) >> 7;

		ic[cic].rx_cfgb.diagsel         = (ic[cic].configb.rx_data[5] & 0x07);
		ic[cic].rx_cfgb.gpio2eoc        = (ic[cic].configb.rx_data[5] & 0x08) >> 3;
		ic[cic].rx_cfgb.gpio1c          = (ic[cic].configb.rx_data[5] & 0x10) >> 4;
		ic[cic].rx_cfgb.gpio2c          = (ic[cic].configb.rx_data[5] & 0x20) >> 5;
		ic[cic].rx_cfgb.gpio3c          = (ic[cic].configb.rx_data[5] & 0x40) >> 6;
		ic[cic].rx_cfgb.gpio4c          = (ic[cic].configb.rx_data[5] & 0x80) >> 7;
	}
}

void adbms2950_pack_cfgb(adbms2950_driver_t* dev)
{
  adbms2950_asic *ic = dev->ics;
  for(uint8_t cic = 0; cic < dev->num_ics; cic++)
  {
    ic[cic].configb.tx_data[0] = (((ic[cic].tx_cfgb.oc1ten & 0x01) << 7) | (ic[cic].tx_cfgb.oc1th & 0x7F));
    ic[cic].configb.tx_data[1] = (((ic[cic].tx_cfgb.oc2ten & 0x01) << 7) | (ic[cic].tx_cfgb.oc2th & 0x7F));
    ic[cic].configb.tx_data[2] = (((ic[cic].tx_cfgb.oc3ten & 0x01) << 7) | (ic[cic].tx_cfgb.oc3th & 0x7F));
    ic[cic].configb.tx_data[3] = 0x00;
    ic[cic].configb.tx_data[4] = 0x00;
    ic[cic].configb.tx_data[5] = 0x00;
    ic[cic].configb.tx_data[3] = (((ic[cic].tx_cfgb.octsel & 0x03) << 6) | ((ic[cic].tx_cfgb.reften & 0x01) << 5)  | ((ic[cic].tx_cfgb.ocdp & 0x01) << 3)
                                      | (ic[cic].tx_cfgb.ocdgt & 0x03));   // GPO1 is at position 0
    ic[cic].configb.tx_data[4] = (((ic[cic].tx_cfgb.ocbx & 0x01)  << 7)| ((ic[cic].tx_cfgb.ocax & 0x01) << 6) | ((ic[cic].tx_cfgb.ocmode & 0x03) << 4)
                                      |((ic[cic].tx_cfgb.oc3gc & 0x01) << 3) | ((ic[cic].tx_cfgb.oc2gc & 0x01) << 2) | ((ic[cic].tx_cfgb.oc1gc & 0x01) << 1)
                                        | (ic[cic].tx_cfgb.ocod & 0x01));
    ic[cic].configb.tx_data[5] = (((ic[cic].tx_cfgb.gpio4c & 0x01)  << 7)| ((ic[cic].tx_cfgb.gpio3c & 0x01) << 6) | ((ic[cic].tx_cfgb.gpio2c & 0x01) << 5)
                                      | ((ic[cic].tx_cfgb.gpio1c & 0x01) << 4) |((ic[cic].tx_cfgb.gpio2eoc & 0x01) << 3) | (ic[cic].tx_cfgb.diagsel & 0x07));
  }
}

void adbms2950_adi1(adbms2950_driver_t* dev, adi1_* arg)
{
	uint8_t cmd[CMDSZ];
	uint8_t rd = arg->rd & 0x01;
	uint8_t opt = arg->opt & 0x0F;

	cmd[0] = sADI1[0] | rd;
	cmd[1] = sADI1[1] | ((opt & 0x08) << 4) | ((opt & 0x04) << 2) | (opt & 0x03);

	adbms2950_cmd(dev, cmd);
}

void adbms2950_adi2(adbms2950_driver_t* dev, adi2_* arg)
{
	uint8_t cmd[CMDSZ];
	uint8_t opt = arg->opt & 0x0F;

	cmd[0] = sADI2[0];
	cmd[1] = sADI2[1] | ((opt & 0x08) << 4) | ((opt & 0x04) << 2) | (opt & 0x03);

	adbms2950_cmd(dev, cmd);
}

void adbms2950_adv(adbms2950_driver_t* dev, adv_* arg)
{
	uint8_t cmd[CMDSZ];
	uint8_t OW = arg->ow & 0x03;
	uint8_t VCH = arg->ch & 0x0F;

	cmd[0] = sADV[0];
	cmd[1] = sADV[1] | (OW << 6) | VCH;

	adbms2950_cmd(dev, cmd);
}

void adbms2950_plv(adbms2950_driver_t* dev)
{
	adbms2950_cmd(dev, PLV);
}

void adbms2950_rdvb(adbms2950_driver_t* dev)
{
	adbms2950_rd48(dev, RDVB, buf);
	adbms2950_parse_rdvb(dev, buf);
}

void adbms2950_parse_rdvb(adbms2950_driver_t* dev, uint8_t* vbat_data)
{
	  uint8_t address = 0;
	  for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	  {
		  address = cic * RX_DATA;
	    memcpy(&dev->ics[cic].reg.rx_data[0], &vbat_data[address], RX_DATA);
	    dev->ics[cic].vbat.vbat1 = dev->ics[cic].reg.rx_data[2] + (dev->ics[cic].reg.rx_data[3] << 8);
	    dev->ics[cic].vbat.vbat2 = dev->ics[cic].reg.rx_data[4] + (dev->ics[cic].reg.rx_data[5] << 8);
	  }
}

void adbms2950_rdi(adbms2950_driver_t* dev)
{
	adbms2950_rd48(dev, RDI, buf);
	adbms2950_parse_rdi(dev, buf);
}

void adbms2950_parse_rdi(adbms2950_driver_t* dev, uint8_t* i_data)
{
	  uint8_t address = 0;
	  for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	  {
		  address = cic * RX_DATA;
	    memcpy(&dev->ics[cic].reg.rx_data[0], &i_data[address], RX_DATA);
	    dev->ics[cic].i.i1 = (uint32_t)0 + dev->ics[cic].reg.rx_data[0] + (dev->ics[cic].reg.rx_data[1] << 8) + (dev->ics[cic].reg.rx_data[2] << 16);
	    dev->ics[cic].i.i2 = (uint32_t)0 + dev->ics[cic].reg.rx_data[3] + (dev->ics[cic].reg.rx_data[4] << 8) + (dev->ics[cic].reg.rx_data[5] << 16);
	    // Sign extend signed 24 bit value to int32_t
	    if(dev->ics[cic].i.i1 & 0x800000) dev->ics[cic].i.i1 |= 0xFF000000;
	    if(dev->ics[cic].i.i2 & 0x800000) dev->ics[cic].i.i2 |= 0xFF000000;
	  }
}

void adbms2950_rdv1d(adbms2950_driver_t* dev)
{
	adbms2950_rd48(dev, RDV1D, buf);
	adbms2950_parse_rdv1d(dev, buf);
}

void adbms2950_parse_rdv1d(adbms2950_driver_t* dev, uint8_t* v_data)
{
	uint8_t address;
	uint8_t temp[RX_DATA];
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		address = cic * RX_DATA;
		memcpy(temp, &v_data[address], RX_DATA);
		dev->ics[cic].vr.v_codes[9] =  (temp[0] + (temp[1] << 8)); // V7A
		dev->ics[cic].vr.v_codes[10] =  (temp[2] + (temp[3] << 8)); // V8A
		dev->ics[cic].vr.v_codes[11] =  (temp[4] + (temp[5] << 8)); // V9B
	}
}

void adbms2950_gpo_set(adbms2950_driver_t* dev, GPO gpo, CFGA_GPO state)
{
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		switch(gpo)
		{
			case GPO1:
				dev->ics[cic].tx_cfga.gpo1c = state;
				break;
			case GPO2:
				dev->ics[cic].tx_cfga.gpo2c = state;
				break;
			case GPO3:
				dev->ics[cic].tx_cfga.gpo3c = state;
				break;
			case GPO4:
				dev->ics[cic].tx_cfga.gpo4c = state;
				break;
			case GPO5:
				dev->ics[cic].tx_cfga.gpo5c = state;
				break;
			case GPO6:
				dev->ics[cic].tx_cfga.gpo6c = state;
				break;
			default:
				return;
		}
	}
}

void adbms2950_wakeup(adbms2950_driver_t *dev)
{
	for(uint8_t i = 0; i < dev->num_ics; i++)
	{
		adbms2950_set_cs(dev, 0);
		adbms2950_us_delay(dev, WAKEUP_US_DELAY);
		adbms2950_set_cs(dev, 1);
		adbms2950_us_delay(dev, WAKEUP_BW_DELAY);
	}
}

void adbms2950_set_cs(adbms2950_driver_t* dev, uint8_t state)
{
	HAL_GPIO_WritePin(dev->cs_port[dev->string], dev->cs_pin[dev->string], state);
}

void adbms2950_us_delay(adbms2950_driver_t* dev, uint16_t microseconds)
{
	__HAL_TIM_SET_COUNTER(dev->htim, 0);
	while (__HAL_TIM_GET_COUNTER(dev->htim) < microseconds);
	return;
}

void adbms2950_spi_write(adbms2950_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs)
{
	if(use_cs) adbms2950_set_cs(dev, 0);
	HAL_SPI_Transmit(dev->hspi, data, len, SPI_TIMEOUT);
	if(use_cs) adbms2950_set_cs(dev, 1);
}

void adbms2950_spi_write_read(adbms2950_driver_t *dev,
							  uint8_t* tx_Data,
							  uint8_t tx_len,
							  uint8_t* rx_data,
							  uint8_t rx_len,
							  uint8_t use_cs)
{
	HAL_StatusTypeDef ret = 0;
	if(use_cs) adbms2950_set_cs(dev, 0);
	ret |= HAL_SPI_Transmit(dev->hspi, tx_Data, tx_len, 100);
	ret |= HAL_SPI_Receive(dev->hspi, rx_data, rx_len, 100);
	if(use_cs) adbms2950_set_cs(dev, 1);
}

/*!<
* @brief Precomputed CRC15 Table
*/
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

/* Pre-computed CRC10 Table */
/* UNUSED by my code. Don't know don't care
static const uint16_t crc10Table[256] =
{
	0x000, 0x08f, 0x11e, 0x191, 0x23c, 0x2b3, 0x322, 0x3ad, 0x0f7, 0x078, 0x1e9, 0x166, 0x2cb, 0x244, 0x3d5, 0x35a,
	0x1ee, 0x161, 0x0f0, 0x07f, 0x3d2, 0x35d, 0x2cc, 0x243, 0x119, 0x196, 0x007, 0x088, 0x325, 0x3aa, 0x23b, 0x2b4,
	0x3dc, 0x353, 0x2c2, 0x24d, 0x1e0, 0x16f, 0x0fe, 0x071, 0x32b, 0x3a4, 0x235, 0x2ba, 0x117, 0x198, 0x009, 0x086,
	0x232, 0x2bd, 0x32c, 0x3a3, 0x00e, 0x081, 0x110, 0x19f, 0x2c5, 0x24a, 0x3db, 0x354, 0x0f9, 0x076, 0x1e7, 0x168,
	0x337, 0x3b8, 0x229, 0x2a6, 0x10b, 0x184, 0x015, 0x09a, 0x3c0, 0x34f, 0x2de, 0x251, 0x1fc, 0x173, 0x0e2, 0x06d,
	0x2d9, 0x256, 0x3c7, 0x348, 0x0e5, 0x06a, 0x1fb, 0x174, 0x22e, 0x2a1, 0x330, 0x3bf, 0x012, 0x09d, 0x10c, 0x183,
	0x0eb, 0x064, 0x1f5, 0x17a, 0x2d7, 0x258, 0x3c9, 0x346, 0x01c, 0x093, 0x102, 0x18d, 0x220, 0x2af, 0x33e, 0x3b1,
	0x105, 0x18a, 0x01b, 0x094, 0x339, 0x3b6, 0x227, 0x2a8, 0x1f2, 0x17d, 0x0ec, 0x063, 0x3ce, 0x341, 0x2d0, 0x25f,
	0x2e1, 0x26e, 0x3ff, 0x370, 0x0dd, 0x052, 0x1c3, 0x14c, 0x216, 0x299, 0x308, 0x387, 0x02a, 0x0a5, 0x134, 0x1bb,
	0x30f, 0x380, 0x211, 0x29e, 0x133, 0x1bc, 0x02d, 0x0a2, 0x3f8, 0x377, 0x2e6, 0x269, 0x1c4, 0x14b, 0x0da, 0x055,
	0x13d, 0x1b2, 0x023, 0x0ac, 0x301, 0x38e, 0x21f, 0x290, 0x1ca, 0x145, 0x0d4, 0x05b, 0x3f6, 0x379, 0x2e8, 0x267,
	0x0d3, 0x05c, 0x1cd, 0x142, 0x2ef, 0x260, 0x3f1, 0x37e, 0x024, 0x0ab, 0x13a, 0x1b5, 0x218, 0x297, 0x306, 0x389,
	0x1d6, 0x159, 0x0c8, 0x047, 0x3ea, 0x365, 0x2f4, 0x27b, 0x121, 0x1ae, 0x03f, 0x0b0, 0x31d, 0x392, 0x203, 0x28c,
	0x038, 0x0b7, 0x126, 0x1a9, 0x204, 0x28b, 0x31a, 0x395, 0x0cf, 0x040, 0x1d1, 0x15e, 0x2f3, 0x27c, 0x3ed, 0x362,
	0x20a, 0x285, 0x314, 0x39b, 0x036, 0x0b9, 0x128, 0x1a7, 0x2fd, 0x272, 0x3e3, 0x36c, 0x0c1, 0x04e, 0x1df, 0x150,
	0x3e4, 0x36b, 0x2fa, 0x275, 0x1d8, 0x157, 0x0c6, 0x049, 0x313, 0x39c, 0x20d, 0x282, 0x12f, 0x1a0, 0x031, 0x0be
};
*/

uint16_t Pec15_Calc(uint8_t len, uint8_t *data)
{
  uint16_t remainder,addr;
  remainder = 16u; /*!< initialize the PEC */
  for (uint8_t i = 0; i<len; i++) /*!< loops for each byte in data array */
  {
    addr = (((remainder>>7u)^data[i])&0xff);/*!< calculate PEC table address */
    remainder = ((remainder<<8u)^Crc15Table[addr]);
  }
  return(remainder*2u);/*!< The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2 */
}

uint16_t pec10_calc(uint8_t rx_cmd, int len, uint8_t *data)
{
  uint16_t remainder = 16u; /*!< PEC_SEED;   0000010000 */
  uint16_t polynom = 0x8F; /*!< x10 + x7 + x3 + x2 + x + 1 <- the CRC15 polynomial         100 1000 1111   48F */

  /*!< Perform modulo-2 division, a byte at a time. */
  for (uint8_t pbyte = 0; pbyte < len; ++pbyte)
  {
    /*!< Bring the next byte into the remainder. */
    remainder ^= (uint16_t)((uint16_t)data[pbyte] << 2u);
    /*!< Perform modulo-2 division, a bit at a time.*/
    for (uint8_t bit_ = 8u; bit_ > 0u; --bit_)
    {
      /*!< Try to divide the current data bit. */
      if ((remainder & 0x200) > 0u)/*!<equivalent to remainder & 2^14 simply check for MSB */
      {
        remainder = (uint16_t)((remainder << 1u));
        remainder = (uint16_t)(remainder ^ polynom);
      }
      else
      {
        remainder = (uint16_t)(remainder << 1u);
      }
    }
  }
  if (rx_cmd)
  {
    remainder ^= (uint16_t)(((uint16_t)data[len] & (uint8_t)0xFC) << 2u);
  }
  /*!< Perform modulo-2 division, a bit at a time */
  for (uint8_t bit_ = 6u; bit_ > 0u; --bit_)
  {
    /*!< Try to divide the current data bit */
    if ((remainder & 0x200) > 0u)/*!<equivalent to remainder & 2^14 simply check for MSB*/
    {
      remainder = (uint16_t)((remainder << 1u));
      remainder = (uint16_t)(remainder ^ polynom);
    }
    else
    {
      remainder = (uint16_t)((remainder << 1u));
    }
  }
  return ((uint16_t)(remainder & 0x3FFu));
}

uint16_t pec10_calc_modular(uint8_t * data, uint8_t PEC_Format)
{
    uint16_t remainder = 16u; // PEC_SEED;
    uint16_t len;
    switch (PEC_Format)
    {
		case PEC10_WRITE:
			data[6] = 0; // for write commands the command counter is all zero
			// step through
		case PEC10_READ:
			len = 6;
			break;
		case PEC10_READ256:
			len = 256;
			break;
		case PEC10_READ512:
			len = 512;
			break;
		case PEC10_WRITE2:
			data[2] = 0;
			// step through
		case PEC10_READ2:
			len = 2;
			break;
		default:
			return 0xFFFF;
			break;
    }
    //Perform modulo-2 division, a byte at a time.
    for (uint8_t pbyte = 0; pbyte < len; ++pbyte)
    {
        // Bring the next byte into the remainder.
        remainder ^= (uint16_t)(data[pbyte]) << 2u;
        remainder = pec10_calc_int(remainder, 8u);
    }
    // the last byte is different as it holds the 6-bit command counter
    // Note: for write commands, those bits are zero!
    remainder ^= (uint16_t)((data[len] & 0xFC) << 2u);
    remainder = pec10_calc_int(remainder, 6u);
    return ((uint16_t)(remainder & 0x3FF));
}

uint16_t pec10_calc_int(uint16_t remainder, uint8_t bit)
{
  uint8_t PEC10_POLY = 0x8F;
    //Perform modulo-2 division, a bit at a time.
    //Perform modulo-2 division, a bit at a time.
    for (; bit > 0; --bit)
    {
        //Try to divide the current data bit.
        if ((remainder & 0x200u) > 0u)//equivalent to remainder & 2^14 simply check for MSB
        {
            remainder = (uint16_t)((remainder << 1u));
            remainder = (uint16_t)(remainder ^ PEC10_POLY);
        }
        else
        {
            remainder = (uint16_t)(remainder << 1u);
        }
    }
    return ((uint16_t)(remainder & 0x3FFu));
}
