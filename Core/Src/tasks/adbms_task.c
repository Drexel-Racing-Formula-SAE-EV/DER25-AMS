/*
 * adbms_task.h
 *
 *  Created on: June 3, 2025
 *      Author: Cole Bardin
 */

#include <tasks/adbms_task.h>

void adbms_task_fn(void *argument);

TaskHandle_t adbms_task_start(app_data_t *data)
{
	TaskHandle_t handle;
	xTaskCreate(adbms_task_fn, "adbms task", 1024, (void *)data, ADBMS_PRIO, &handle);
	return handle;
}

void adbms_task_fn(void *argument)
{
	app_data_t *data = (app_data_t *) argument;
	accumulator_t *acc = &data->acc;
	uint32_t entry;

	for(;;)
	{
        entry = osKernelGetTickCount();

        accumulator_read_volt(acc);
        data->total_voltage = acc->apm.vbat[0];

        accumulator_read_temp(acc);

        osDelayUntil(entry + (1000 / ADBMS_FREQ));
	}
}



