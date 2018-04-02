/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Examen1.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "MK64F12.h"
#include "DataTypeDefinitions.h"
#include "GPIO.h"

#define YELLOW &StateMachine[0]
#define RED &StateMachine[1]
#define PURPLE &StateMachine[2]
#define BLUE &StateMachine[3]
#define GREEN &StateMachine[4]
#define WHITE &StateMachine[5]

typedef const struct State{
	void(*ColorLED)();
	const struct State* next[3];
}ChangeLED;

void printYellow();

void printRed();

void printPurple();

void printBlue();

void printGreen();

void printWhite();

const ChangeLED StateMachine[6] =
{
	{printYellow, {RED, GREEN, WHITE}},
	{printRed, {PURPLE, YELLOW, WHITE}},
	{printPurple, {BLUE, RED, WHITE}},
	{printBlue, {GREEN, PURPLE, WHITE}},
	{printGreen, {YELLOW, BLUE, WHITE}},
	{printWhite, {YELLOW, GREEN, WHITE}},
};
int main()
{
	ChangeLED* currentState = YELLOW;

		GPIO_clockGating(GPIO_A);
		GPIO_clockGating(GPIO_B);
		GPIO_clockGating(GPIO_C);
		GPIO_clockGating(GPIO_E);
		/** Codigo de la tarea 3*/
		/**Variables to capture the input values*/
		uint32 inputValueSW2 = 0;
		uint32 inputValueSW3 = 0;

		/**Pin control configuration of GPIOB pin22 and pin21 as GPIO*/
		PORTB->PCR[21] = 0x00000100;
		PORTB->PCR[22] = 0x00000100;

		/**Pin control configuration of GPIOC pin6 as GPIO with is pull-up resistor enabled*/
		PORTC->PCR[6] = 0x00000103;
		/**Pin control configuration of GPIOA pin4 as GPIO with is pull-up resistor enabled*/
		PORTA->PCR[4] = 0x00000103;

		/**Pin control configuration of GPIOE pin26 as GPIO*/
		PORTE->PCR[26] = 0x00000100;
		/**Assigns a safe value to the output pin21 of the GPIOB*/
		GPIOB->PDOR = 0x00200000;
		/**Assigns a safe value to the output pin22 of the GPIOB*/
		GPIOB->PDOR |= 0x00400000;
		/**Assigns a safe value to the output pin26 of the GPIOE*/
		GPIOE->PDOR |= 0x04000000;

		GPIOC->PDDR &=~(0x40);
		GPIOA->PDDR &=~(0x10);

		/**Configures GPIOB pin21 as output*/
		GPIOB->PDDR = 0x00200000;
		/**Configures GPIOB pin22 as output*/
		GPIOB->PDDR |= 0x00400000;
		/**Configures GPIOE pin26 as output*/
		GPIOE->PDDR |= 0x04000000;

		for(;;)
		{
			/**Reads all the GPIOC*/
			inputValueSW2 = GPIOC->PDIR;
			/**Masks the GPIOC in the bit of interest*/
			inputValueSW2 = inputValueSW2 & 0x40;

			/**Reads all the GPIOA*/
			inputValueSW3 = GPIOA->PDIR;
			/**Masks the GPIOA in the bit of interest*/
			inputValueSW3 = inputValueSW3 & 0x10;

			if(inputValueSW2 == 0 && inputValueSW3 != 0)
			{
				currentState = currentState->next[0];
			}
			else if(inputValueSW2 != 0 && inputValueSW3 == 0)
			{
				currentState = currentState->next[1];
			}
			else if(inputValueSW2 == 0 && inputValueSW3 == 0)
			{
				currentState = currentState->next[2];
			}
			else
			{

			}

			currentState->ColorLED();

			while((GPIOC->PDIR & 0x40) != 0 && (GPIOA->PDIR & 0x10) != 0);
		}
	return 0;
}

void printYellow()
{
	/** Codigo de la tarea 3*/
	GPIOB->PDOR = (0x00200000);
	GPIOE->PDOR = (0x0B000000);
}

void printRed()
{
	/** Codigo de la tarea 3*/
	GPIOB->PDOR = (0x00200000);
	GPIOE->PDOR = (0x04000000);
}

void printPurple()
{
	/** Codigo de la tarea 3*/
	GPIOB->PDOR = (0x00900000);
	GPIOE->PDOR = (0x04000000);
}

void printBlue()
{
	/** Codigo de la tarea 3*/
	GPIOB->PDOR = (0x00400000);
	GPIOE->PDOR = (0x04000000);
}

void printGreen()
{
	/** Codigo de la tarea 3*/
	GPIOB->PDOR = (0x00600000);
	GPIOE->PDOR = (0x0B000000);
}

void printWhite()
{
	/** Codigo de la tarea 3*/
	GPIOB->PDOR = 0;
	GPIOE->PDOR = (0x0B000000);
}
