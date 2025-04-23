/*
 * fan_task.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Cassius Garcia
 */
#include "tasks/fan_task.h"
//#include "main.h"

void fan_task_fn(void *argument);

TaskHandle_t fan_task_start(app_data_t *data)
{
	TaskHandle_t handle;
	xTaskCreate(fan_task_fn, "fan task", 128, (void *)data, 7, &handle);
	return handle;
}

void fan_task_fn(void *argument)
{
	app_data_t *data = (app_data_t *) argument;
	uint32_t entry;

	for(int i = 0; i < NFANS; i++) set_fan_percent(&data->board.fans[i], 100.0);
	data->fan_state = true;
	osDelay(2000);
	for(int i = 0; i < NFANS; i++) set_fan_percent(&data->board.fans[i], 0.0);
	data->fan_state = false;

	for(;;)
	{
		entry = osKernelGetTickCount();

		if(data->max_temp > TEMP_THRESH_H)
		{
			for(int i = 0; i < NFANS; i++) set_fan_percent(&data->board.fans[i], 100.0);
			data->fan_state = true;
		}
		else if(data->max_temp < TEMP_THRESH_L)
		{
			for(int i = 0; i < NFANS; i++) set_fan_percent(&data->board.fans[i], 0.0);
			data->fan_state = false;
		}

		osDelayUntil(entry + (1000 / FAN_FREQ));
	}
}
