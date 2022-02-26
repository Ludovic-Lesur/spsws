/*
 * iwdg.h
 *
 *  Created on: 30 march 2018
 *      Author: Ludovic
 */

#ifndef IWDG_H
#define IWDG_H

/*** IWDG macros ***/

#define IWDG_REFRESH_PERIOD_SECONDS		10

/*** IWDG functions ***/

void IWDG_init(void);
void IWDG_reload(void);

#endif /* IWDG_H */
