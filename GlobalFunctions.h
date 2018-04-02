/*
 * GlobalFunctions.h
 *
 *  Created on: 16/08/2017
 *      Author: jlpe
 */

#ifndef GLOBALFUNCTIONS_H_
#define GLOBALFUNCTIONS_H_

#include "DataTypeDefinitions.h"


/*!
 	 \brief	 This function makes a delay in time.

 	 \param[in]  delay Port to clear interrupts.
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void delay(uint16 delay);

void NOP();


#endif /* GLOBALFUNCTIONS_H_ */
