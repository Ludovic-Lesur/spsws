/*
 * lpuart.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef LPUART_H
#define LPUART_H

/*** LPUART functions ***/

void LPUART1_init(unsigned char lpuart_use_lse);
void LPUART1_update_brr(void);
void LPUART1_enable_tx(void);
void LPUART1_enable_rx(void);
void LPUART1_disable(void);
void LPUART1_power_on(void);
void LPUART1_power_off(void);
void LPUART1_send_byte(unsigned char tx_byte);

#endif /* LPUART_H */
