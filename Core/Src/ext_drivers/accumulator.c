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
	adbms2950_gpo_set(&dev->apm, HVEN1, GPO_SET);
	adbms2950_gpo_set(&dev->apm, HVEN2, GPO_SET);
	adbms2950_wakeup(&dev->apm);
	adbms2950_wrcfga(&dev->apm);
	adbms2950_rdcfga(&dev->apm);

	adbms2950_us_delay(&dev->apm, 500);
	adbms2950_wakeup(&dev->apm);
	adbms2950_rdvb(&dev->apm);
	adbms2950_rdi(&dev->apm);

	dev->apm.vbat_adc[0] = dev->apm_ics[0].vbat.vbat1 * VBAT_SCALE;
	dev->apm.vbat_adc[1] = dev->apm_ics[0].vbat.vbat2 * VBAT_SCALE;

	dev->apm.vbat[0] = dev->apm.vbat_adc[0] * VBAT_DIV_SCALE;
	dev->apm.vbat[1] = dev->apm.vbat_adc[1] * VBAT_DIV_SCALE;

    return ret;
}

int accumulator_read_temp(accumulator_t *dev)
{
	int error = 0;
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
