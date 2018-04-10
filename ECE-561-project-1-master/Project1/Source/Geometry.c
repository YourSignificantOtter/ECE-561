#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "LEDs.h"
#include "Geometry.h"
#include "Optimization_Additions.h"

#define PI 3.14159265f
#define PI_OVER_180	0.017453295f

extern const GPT_T waypoints[];

float Calc_Distance(GPT_T * p1, const GPT_T * p2) { 
// calculates distance in kilometers between locations
  return acosf(p1->Sin_Lat*p2->Sin_Lat + 
		p1->Cos_Lat*p2->Cos_Lat*cosf(p2->Lon - p1->Lon)) * EARTH_R;
}

float Calc_Bearing(GPT_T * p1, const GPT_T *  p2){
// calculates bearing in degrees between locations (represented in degrees)	
	float angle = atan2f(
		sinf(p1->Lon - p2->Lon)*p2->Cos_Lat,
		p1->Cos_Lat*p2->Sin_Lat - 
		p1->Sin_Lat*p2->Cos_Lat*cosf(p1->Lon - p2->Lon)
		);
	angle /= PI_OVER_180;
	
	// Center angles at 0 to simplify math comparing angles
	/*
 	if (angle < 0.0)	 	if (angle < 0.0)
 		angle += 360;	 		angle += 360;
	*/
	
	return angle;
}

void Calc_Crosstrack_Error(GPT_T * here, float track, const GPT_T * there, int * behind, float * xt){
	float d_xt;
	float d_ht, b_ht;
	
	b_ht = Calc_Bearing(here, there);
	*behind = Is_Point_Behind(b_ht, &track); //Remember this edits track!
	if(!*behind)
	{
		d_ht = Calc_Distance(here, there);
		d_xt = EARTH_R * asinf(sinf(d_ht/EARTH_R)*sinf((b_ht - track) * PI_OVER_180));
		*xt = fabs(d_xt);
	}
}

void Find_Waypoint_Nearest_To_Track(float cur_pos_lat, float cur_pos_lon, float track, float * xt, char * * name) { 
	// cur_pos_lat and cur_pos_lon are in degrees
	// distance is in kilometers
	// track is in degrees
		
	int i=0, closest_i=0, behind;
	GPT_T ref;
	float closest_d_xt=1E10;
	float d_xt;
	
	ref.Lat = cur_pos_lat * PI_OVER_180;
	ref.Sin_Lat = sinf(ref.Lat);
	ref.Cos_Lat = cosf(ref.Lat);
	ref.Lon = cur_pos_lon * PI_OVER_180;
	strcpy(ref.Name, "I Am Here");
	
	Control_RGB_LEDs(0, 0, 1);


	while (strcmp(waypoints[i].Name, "END")) {
		Approx_Crosstrack_Error(&ref, track, &(waypoints[i]), &behind, &d_xt);
		if (!behind) {	
			if (d_xt < closest_d_xt) {
				// if we found a closer waypoint, remember it and display it
				closest_d_xt = d_xt;
				closest_i = i;
			}
		}
		i++;
	}
	// return information about waypoint nearest to track
	Calc_Crosstrack_Error(&ref, track, &(waypoints[closest_i]), &behind, &closest_d_xt);
	*xt = closest_d_xt;
	*name = (char *)(waypoints[closest_i].Name);
}

void Find_Nearest_Waypoint(float cur_pos_lat, float cur_pos_lon, float * distance, float * bearing, 
	char * * name) {
	// cur_pos_lat and cur_pos_lon are in degrees
	// distance is in kilometers
	// bearing is in degrees
		
	int i=0, closest_i;
	GPT_T ref;
	float d, b, closest_d=1E10;

	*distance = *bearing = NULL;
	*name = NULL;
		
	ref.Lat = cur_pos_lat * PI_OVER_180;
	ref.Sin_Lat = sinf(ref.Lat);
	ref.Cos_Lat = cosf(ref.Lat);
	ref.Lon = cur_pos_lon * PI_OVER_180;
	strcpy(ref.Name, "I Am Here");

	while (strcmp(waypoints[i].Name, "END")) {
		d = Calc_Distance(&ref, &(waypoints[i]) );
		b = Calc_Bearing(&ref, &(waypoints[i]) );

		// if we found a closer waypoint, remember it and display it
		if (d<closest_d) {
			closest_d = d;
			closest_i = i;
		}	
		i++;
	}
	d = closest_d; // Calc_Distance(&ref, &(waypoints[closest_i]) );
	b = Calc_Bearing(&ref, &(waypoints[closest_i]) );

	// return information to calling function about closest waypoint 
	*distance = d;
	*bearing = b;
	*name = (char *)(waypoints[closest_i].Name);
}
