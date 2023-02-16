/*
 * motors.h
 *
 *  Created on: Feb. 15, 2023
 *      Author: nicol
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

void motor_run(int s)
{
	TIM2 -> CCR1 = s; // set the speed of motor1
	TIM2 -> CCR2 = s; // set the speed of motor2

	//__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, s);
	//__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, s);


	//motor1 forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

	//motor2 forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

/*
	HAL_GPIO_WritePin(motor_driver_IN1_GPIO_Port, motor_driver_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor_driver_IN2_GPIO_Port, motor_driver_IN2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(motor_driver_IN3_GPIO_Port, motor_driver_IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor_driver_IN4_GPIO_Port, motor_driver_IN4_Pin, GPIO_PIN_RESET);
*/

}
void motor_run_back(int s){
	TIM2 -> CCR1 = s; // set the speed of motor1
	TIM2 -> CCR2 = s; // set the speed of motor2

	//motor1 forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

	//motor2 forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void motor_stop(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

	//motor2 forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

}
void motor_turn(int s){
	TIM2 -> CCR1 = s; // set the speed of motor1
	TIM2 -> CCR2 = s; // set the speed of motor2

	//motor1 forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

	//motor2 forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

#endif /* INC_MOTORS_H_ */
