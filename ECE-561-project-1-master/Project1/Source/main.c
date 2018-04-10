/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include "gpio_defs.h"
#include "UART.h"
#include "LEDs.h"
#include "Geometry.h"
#include "profile.h"

#define NUM_TRACKS 25

#define PROFILING 0

typedef struct {
	float track;
	float d_xt;
	char * name;
} XTI_t;

int main (void) {
	#if PROFILING
		Init_Profiling();
	#endif
	float lat = 0, lon = 0, track = 0, d_xt=0;
	char * name;
	unsigned n;
	XTI_t xti[NUM_TRACKS];

	Init_UART0(115200);
	Init_RGB_LEDs();
	Control_RGB_LEDs(0, 0, 0);

	printf("\n\rCross-Track Error Distance Calculator\n\r");

	track = 0;
	Control_RGB_LEDs(0, 0, 1);
	
	#if PROFILING
		Enable_Profiling();
	#endif
	for (n=0; n<NUM_TRACKS; n++) {
		xti[n].track = track;
 		Find_Waypoint_Nearest_To_Track(35.772336, 78.673593, xti[n].track, &(xti[n].d_xt), &(xti[n].name));
		track += 14.15;
	}
	#if PROFILING
		Disable_Profiling();
	#endif
	Control_RGB_LEDs(0, 1, 0);
	
	for (n=0; n<NUM_TRACKS; n++) {
		printf(" Track: %.1f\t Closest is %s at %.3f km XTE\n\r", xti[n].track, xti[n].name, xti[n].d_xt);
	}
	printf("Done with testing standard tracks\n\r");
	printf("Ready to accept custom location and track.\n\r");
	printf("Enable local echo on terminal to see your entry when typing.\n\r");

	#if PROFILING
		printf("\n\r\n\r\n\r");
		Sort_Profile_Regions();
		Serial_Print_Sorted_Profile();
		printf("Done, please reset board when ready.\n\r\n");
	#endif
	
	while (1) {
		do {
			printf("Enter latitude:\n\r");
		} while (!scanf("%f", &lat) || (lat > 90) || (lat < -90));
		printf("%f\n\r", lat);

		do {
			printf("Enter longitude:\n\r");
		} while (!scanf("%f", &lon) || (lon > 180) || (lon < -180));
		printf("%f\n\r", lon);
			
		do {
			printf("Enter track:\n\r");
		} while (!scanf("%f", &track) || (track < 0) || (track >= 360));
		printf("%f\n\r", track);
		
		Control_RGB_LEDs(0, 0, 1);
		Find_Waypoint_Nearest_To_Track(lat, lon, track, &d_xt, &name);
		Control_RGB_LEDs(0, 1, 0);
		printf(" track: %.1f\t Closest is %s at %.3f km XTE\n\r", track, name, d_xt);
	}
	
}
