/*
 * mcp4162.c
 *
 *  Created on: 11 aug 2018
 *      Author: Ludovic
 */

#include "mcp4162.h"

#include "gpio_reg.h"
#include "mcp4162_reg.h"
#include "spi.h"
#include "tim.h"

/*** MCP4162 local structures ***/

typedef struct {
	unsigned short mcp4162_status;
	unsigned short mcp4162_tcon;
	unsigned short mcp4162_vw0;
} MCP4162_Ctx;

/*** MCP4162 local global variables ***/

MCP4162_Ctx mcp4162_ctx;

/*** MCP4162 local functions ***/

/* READ MCP4162 MAIN REGISTERS.
 * @param:	None.
 * @return:	None.
 */
void MCP4162_UpdateContext(void) {

	/* Read status register */
	unsigned short spi_command_read = 0;
	spi_command_read |= (MCP4162_REG_STATUS << 12) | (0b11 << 10);
	GPIOA -> ODR &= ~(0b1 << 4); // CS='LOW'.
	SPI_WriteShort(spi_command_read);
	SPI_ReadShort(spi_command_read, &mcp4162_ctx.mcp4162_status);
	GPIOA -> ODR |= (0b1 << 4); // CS='HIGH'.
	TIM22_WaitMilliseconds(10);

	/* Read TCON register */
	spi_command_read = 0;
	spi_command_read |= (MCP4162_REG_TCON << 12) | (0b11 << 10);
	GPIOA -> ODR &= ~(0b1 << 4); // CS='LOW'.
	SPI_WriteShort(spi_command_read);
	SPI_ReadShort(spi_command_read, &mcp4162_ctx.mcp4162_tcon);
	GPIOA -> ODR |= (0b1 << 4); // CS='HIGH'.
	TIM22_WaitMilliseconds(10);

	/* Read WL0 register */
	spi_command_read = 0;
	spi_command_read |= (MCP4162_REG_VW0 << 12) | (0b11 << 10);
	GPIOA -> ODR &= ~(0b1 << 4); // CS='LOW'.
	SPI_WriteShort(spi_command_read);
	SPI_ReadShort(spi_command_read, &mcp4162_ctx.mcp4162_vw0);
	GPIOA -> ODR |= (0b1 << 4); // CS='HIGH'.
	TIM22_WaitMilliseconds(10);
}

/*** MCP4162 functions ***/

/* INIT DIGITAL POTENTIOMETER.
 * @param:	None.
 * @return:	None.
 */
void MCP4162_Init(void) {

	/* Init context */
	mcp4162_ctx.mcp4162_status = 0;
	mcp4162_ctx.mcp4162_tcon = 0;
	mcp4162_ctx.mcp4162_vw0 = 0;

	/* Init SPI */
	SPI_Init();
}

/* INCREMENT POTENTIOMETER VALUE BY ONE STEP.
 * @param:	None.
 * @return:	None.
 */
void MCP4162_Increment(void) {
	/* Build SPI command */
	unsigned char spi_command_increment = 0;
	spi_command_increment |= (MCP4162_REG_VW0 << 4) | (0b01 << 2);
	GPIOA -> ODR &= ~(0b1 << 4); // CS='LOW'.
	SPI_WriteShort(spi_command_increment);
	TIM22_WaitMilliseconds(1);
	GPIOA -> ODR |= (0b1 << 4); // CS='HIGH'.
	TIM22_WaitMilliseconds(10);
}

/* DECREMENT POTENTIOMETER VALUE BY ONE STEP.
 * @param:	None.
 * @return:	None.
 */
void MCP4162_Decrement(void) {
	/* Build SPI command */
	unsigned char spi_command_increment = 0;
	spi_command_increment |= (MCP4162_REG_VW0 << 4) | (0b10 << 2);
	GPIOA -> ODR &= ~(0b1 << 4); // CS='LOW'.
	SPI_WriteShort(spi_command_increment);
	TIM22_WaitMilliseconds(1);
	GPIOA -> ODR |= (0b1 << 4); // CS='HIGH'.
	TIM22_WaitMilliseconds(10);
}

/* SET DIGITAL POTENTIOMETER STEP.
 * @param step_number:	Step number between 0 and 255.
 * @return:				None.
 */
void MCP4162_SetStep(unsigned char step_number) {
	/* Build SPI command */
	unsigned short spi_command_write = 0;
	spi_command_write |= (MCP4162_REG_VW0 << 12) | (0b00 << 10) | step_number;
	GPIOA -> ODR &= ~(0b1 << 4); // CS='LOW'.
	SPI_WriteShort(spi_command_write);
	TIM22_WaitMilliseconds(1);
	GPIOA -> ODR |= (0b1 << 4); // CS='HIGH'.
	TIM22_WaitMilliseconds(10);
}
