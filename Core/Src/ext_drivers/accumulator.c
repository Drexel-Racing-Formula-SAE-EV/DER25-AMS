/*
 * accumulator.c
 *
 *  Created on: Feb 1, 2024
 *      Author: cole
 */

#include "ext_drivers/accumulator.h"
#include <math.h>

void accumulator_init(accumulator_t *dev,
				      SPI_HandleTypeDef *hspi_a,
					  SPI_HandleTypeDef *hspi_b,
					  GPIO_TypeDef *cs_port_a,
					  GPIO_TypeDef *cs_port_b,
					  uint16_t cs_pin_a,
					  uint16_t cs_pin_b)
{
	dev->total_volt = 0;
}

int accumulator_read_volt(accumulator_t *dev)
{
	int ret = 0;
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
