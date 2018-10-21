/*
 * mcp4162.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludovic
 */

#ifndef HWT_MCP4162_H_
#define HWT_MCP4162_H_

/*** MCP4162 functions ***/

void MCP4162_Init(void);
void MCP4162_Increment(void);
void MCP4162_Decrement(void);
void MCP4162_SetStep(unsigned char step_number);

#endif /* HWT_MCP4162_H_ */
