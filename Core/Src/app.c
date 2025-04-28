/*
 * app.c
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 */

#include "app.h"

#include "cmsis_os.h"
#include "tasks/fan_task.h"
#include "tasks/cli_task.h"
#include "tasks/canbus_task.h"
#include "tasks/air_task.h"
#include "tasks/imd_task.h"
#include "tasks/current_task.h"
#include "tasks/ltc_task.h"

app_data_t app = {0};

void app_create()
{
	app.hard_fault = false;
	app.soft_fault = false;
	app.fan_fault = false;
	app.cli_fault = false;
	app.canbus_fault = false;

	app.air_state = false;
	app.imd_ok = true;
	app.imd_status = IMD_NORMAL;

	app.fan_state = false;

	app.state = STATE_START;

	app.max_temp = 0.0;
	app.avg_temp = 0.0;
	app.max_voltage = 0.0;
	app.min_voltage = 0.0;
	app.current = 0.0;

	board_init(&app.board);
	/*
	accumulator_init(&app.acc,
					 &app.board.stm32f767z.hspi1,
					 &app.board.stm32f767z.hspi3,
					 CS_A_GPIO_Port,
					 CS_B_GPIO_Port,
					 CS_A_Pin,
					 CS_B_Pin);
*/
	//HAL_UART_Receive_IT(app.board.cli.huart, &app.board.cli.c, 1);

	//assert(app.cli_task = cli_task_start(&app));
	assert(app.fan_task = fan_task_start(&app));
	//assert(app.canbus_task = canbus_task_start(&app));
	assert(app.air_task = air_task_start(&app));
	//assert(app.imd_task = imd_task_start(&app));
	//assert(app.current_task = current_task_start(&app));
	//assert(app.ltc_task = ltc_task_start(&app));

	set_bms(1);
}

void set_bms(bool state)
{
	HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, state);
}
