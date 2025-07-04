/*
 * air_task.c
 *
 *  Created on: Apr 3, 2024
 *      Author: Cole Bardin
 */
#include "tasks/air_task.h"

void air_task_fn(void *argument);

TaskHandle_t air_task_start(app_data_t *data)
{
	TaskHandle_t handle;
	xTaskCreate(air_task_fn, "air task", 128, (void *)data, AIR_PRIO, &handle);
	return handle;
}

void air_task_fn(void *argument)
{
	app_data_t *data = (app_data_t *) argument;
	uint32_t entry;

	for(;;)
	{
		// TODO: doesn't need to be its own task. can be in error task
		entry = osKernelGetTickCount();

		data->air_state = HAL_GPIO_ReadPin(AIR_CTRL_GPIO_Port, AIR_CTRL_Pin);
		// TESTING, blink BMS OK
		HAL_GPIO_TogglePin(BMS_OK_GPIO_Port, BMS_OK_Pin);

		osDelayUntil(entry + (1000 / AIR_FREQ));
	}
}



