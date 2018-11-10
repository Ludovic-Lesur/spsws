/*
 * at.c
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludovic
 */

#include "at.h"

#include "usart.h"

/*** AT local structures ***/

typedef struct {

} AT_Context;

/*** AT local global variables ***/

AT_Context at_ctx;

/*** AT functions ***/

/* INIT AT MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_Init(void) {

}

/* PRINT GPS TIMESTAMP ON USART.
 * @param gps_timestamp:	Pointer to GPS timestamp to print.
 * @return:					None.
 */
void AT_PrintGpsTimestamp(GPS_TimestampData* gps_timestamp) {
	// Header.
	USART_SendString("GPS Timestamp *** ");
	// Year.
	USART_SendValue((gps_timestamp -> date_year), USART_Decimal);
	USART_SendString("-");
	// Month.
	if ((gps_timestamp -> date_month) < 10) {
		USART_SendValue(0, USART_Decimal);
	}
	USART_SendValue((gps_timestamp -> date_month), USART_Decimal);
	USART_SendString("-");
	// Day.
	if ((gps_timestamp -> date_day) < 10) {
		USART_SendValue(0, USART_Decimal);
	}
	USART_SendValue((gps_timestamp -> date_day), USART_Decimal);
	USART_SendString(" ");
	// Hours.
	if ((gps_timestamp -> time_hours) < 10) {
		USART_SendValue(0, USART_Decimal);
	}
	USART_SendValue((gps_timestamp -> time_hours), USART_Decimal);
	USART_SendString(":");
	// Minutes.
	if ((gps_timestamp -> time_minutes) < 10) {
		USART_SendValue(0, USART_Decimal);
	}
	USART_SendValue((gps_timestamp -> time_minutes), USART_Decimal);
	USART_SendString(":");
	// Seconds.
	if ((gps_timestamp -> time_seconds) < 10) {
		USART_SendValue(0, USART_Decimal);
	}
	USART_SendValue((gps_timestamp -> time_seconds), USART_Decimal);
	USART_SendString("\n");
}

/* PRINT GPS POSITION ON USART.
 * @param gps_position:	Pointer to GPS position to print.
 * @return:				None.
 */
void AT_PrintGpsPosition(GPS_PositionData* gps_position) {
	// Header.
	USART_SendString("GPS Position *** ");
	// Latitude.
	USART_SendString("Lat=");
	USART_SendValue((gps_position -> lat_degrees), USART_Decimal);
	USART_SendString("°");
	USART_SendValue((gps_position -> lat_minutes), USART_Decimal);
	USART_SendString("'");
	USART_SendValue((gps_position -> lat_seconds), USART_Decimal);
	USART_SendString("''-");
	USART_SendString(((gps_position -> lat_north)==1)? "N" : "S");
	// Longitude.
	USART_SendString(" Long=");
	USART_SendValue((gps_position -> long_degrees), USART_Decimal);
	USART_SendString("°");
	USART_SendValue((gps_position -> long_minutes), USART_Decimal);
	USART_SendString("'");
	USART_SendValue((gps_position -> long_seconds), USART_Decimal);
	USART_SendString("''-");
	USART_SendString(((gps_position -> long_east)==1)? "E" : "W");
	// Altitude.
	USART_SendString(" Alt=");
	USART_SendValue((gps_position -> altitude), USART_Decimal);
	USART_SendString("m\n");
}
