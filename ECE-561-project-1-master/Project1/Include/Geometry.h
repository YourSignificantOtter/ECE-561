#ifndef GEOMETRY_H
#define GEOMETRY_H
// geometry 

#define EARTH_R (6371)

#define RALEIGH_LAT 35.81888889
#define RALEIGH_LON	78.64472222

#define SFO_LAT 37.618889
#define SFO_LON 122.375

#define TEST1_LAT 30.0
#define TEST1_LON 80.0

#define TEST2_LAT 50.0
#define TEST2_LON 130.0

typedef struct {
	float Lat;
	float Lon;
	char Name[24];
} PT_T;

typedef struct {
	float Lat;
	float Sin_Lat;
	float Cos_Lat;
	float Lon;
	char Name[24];
} GPT_T;

void Find_Nearest_Waypoint(float cur_pos_lat, float cur_pos_lon, float * distance, float * bearing, 
	char * * name);

void Find_Waypoint_Nearest_To_Track(float cur_pos_lat, float cur_pos_lon, float track, float * xt, char * * name);

#endif
