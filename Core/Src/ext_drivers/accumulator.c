/*
 * accumulator.c
 *
 *  Created on: Feb 1, 2024
 *      Author: cole
 */

#include "ext_drivers/accumulator.h"
#include <math.h>

void accumulator_init(accumulator_t *dev,
				      SPI_HandleTypeDef *hspi,
					  GPIO_TypeDef *cs_port_a,
					  GPIO_TypeDef *cs_port_b,
					  uint16_t cs_pin_a,
					  uint16_t cs_pin_b,
					  TIM_HandleTypeDef* htim)
{
	dev->total_volt = 0;
	dev->max_temp = 0.0f;
	dev->max_volt = 0.0f;
	dev->min_volt = 0.0f;

	HAL_TIM_Base_Start(htim);

	// Init pack monitor, just on port A
	adbms2950_init(&dev->apm, NAPMS, dev->apm_ics, hspi, cs_port_b, cs_port_b, cs_pin_b, cs_pin_b, htim);
}

int accumulator_read_volt(accumulator_t *dev)
{
	int ret = 0;

	/* APM */
	// Set GPO to enabled to read VBAT voltage
	adbms2950_gpo_set(&dev->apm, HVEN1, GPO_SET);
	adbms2950_gpo_set(&dev->apm, HVEN2, GPO_SET);
	adbms2950_wakeup(&dev->apm);
	adbms2950_wrcfga(&dev->apm);
	adbms2950_rdcfga(&dev->apm);

	// Read VBxADC results (ADCs are in continuous mode)
	adbms2950_wakeup(&dev->apm);
	adbms2950_rdvb(&dev->apm);
	dev->apm.vbat_adc[0] = (int16_t)(dev->apm_ics[0].vbat.vbat1) * VBAT1_SCALE;
	dev->apm.vbat_adc[1] = (int16_t)(dev->apm_ics[0].vbat.vbat2) * VBAT2_SCALE;
	dev->apm.vbat[0] = dev->apm.vbat_adc[0] * VBAT_DIV_SCALE;
	dev->apm.vbat[1] = dev->apm.vbat_adc[1] * VBAT_DIV_SCALE;

	// Read VxADC results (ADCs are in continuous mode)
	adbms2950_wakeup(&dev->apm);
	adbms2950_rdi(&dev->apm);
	dev->apm.vi_adc[0] = (int32_t)(dev->apm_ics[0].i.i1) * VI1_SCALE;
	dev->apm.vi_adc[1] = (int32_t)(dev->apm_ics[0].i.i2) * VI2_SCALE;
	dev->apm.current[0] = dev->apm.vi_adc[0] * CURRENT_R_SCALE;
	dev->apm.current[1] = dev->apm.vi_adc[1] * CURRENT_R_SCALE; // TODO: Check if this is right sign

    return ret;
}

int accumulator_read_temp(accumulator_t *dev)
{
	int error = 0;

	adv_ adv;
	adv.ow = OW_OFF;
	adv.ch = SM_V7_V9;
	// Start aux ADC
	adbms2950_wakeup(&dev->apm);
	adbms2950_adv(&dev->apm, &adv);

	// Poll aux ADC
	adbms2950_wakeup(&dev->apm);
	adbms2950_plv(&dev->apm);

	// Read aux ADC
	adbms2950_wakeup(&dev->apm);
	adbms2950_rdv1d(&dev->apm);

	dev->apm.vtemp_adc[0] = (int16_t)(dev->apm_ics[0].vr.v_codes[9]) * VxA_SCALE; // V7A
	dev->apm.vtemp_adc[1] = (int16_t)(dev->apm_ics[0].vr.v_codes[11]) * VxB_SCALE; // V9B
	// TODO: calibrate NTCs on APM and set 'dev->apm.temps[]' values. this just copies the voltage for now
	dev->apm.temps[0] = dev->apm.vtemp_adc[0];
	dev->apm.temps[1] = dev->apm.vtemp_adc[1];

	return error;
}

int accumulator_convert_volt(accumulator_t *dev)
{
	return 0;
}

int accumulator_convert_temp(accumulator_t *dev, int channel)
{
	return 0;
}

int accumulator_stat_temp(accumulator_t *dev)
{

	return 0;
}

int accumulator_set_temp_ch(accumulator_t *dev, uint8_t channel)
{
	int error = 0;
	error |= accumulator_set_mux_ch(dev, channel, MUX_ADDR7_00);
	error |= accumulator_set_mux_ch(dev, channel, MUX_ADDR7_01);
    return error;
}

int accumulator_set_mux_ch(accumulator_t *dev, uint8_t channel, uint8_t addr7)
{
	int error = 0;
    return error;
}

float NXFT15XV103FEAB050_convert(float ratio)
{
	// TODO: Verify
	double a = 104.517;
	double b = 0.221876;
	return a * pow(b, ratio);
}
