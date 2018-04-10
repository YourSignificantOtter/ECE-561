#include <math.h>
#include "Geometry.h"
#include "Optimization_Additions.h"

#define PI 3.14159265f
#define PI_OVER_180 0.017453295f
#define ONEQTR_PI PI / 4.0f;
#define THRQTR_PI 3.0f * PI / 4.0f;

extern const GPT_T waypoints[];

int Is_Point_Behind(float Bearing, float * track)
{	
	if (*track > 180)
		*track -= 360; 
	
	if (fabs(*track - Bearing) < 90)
		return 0;
	else
			return 1;
}

float Approx_Distance(GPT_T * p1, const GPT_T * p2)
{
	// calculates distance in kilometers between locations
  return acosf(p1->Sin_Lat*p2->Sin_Lat + 
		p1->Cos_Lat*p2->Cos_Lat*cosf_approx(p2->Lon - p1->Lon));// * EARTH_R;
}


float Approx_Bearing(GPT_T * p1, const GPT_T * p2)
{
	float d_lon = p1->Lon - p2->Lon;
		
	float angle = atan2f(sinf_approx(d_lon)*p2->Cos_Lat,
		p1->Cos_Lat*p2->Sin_Lat - 
		p1->Sin_Lat*p2->Cos_Lat*cosf_approx(d_lon)
		);
	angle /= PI_OVER_180;
	return angle;
}

void  Approx_Crosstrack_Error(GPT_T * here, float track, const GPT_T * there, int * behind, float * xt)
{
	float d_xt;
	float d_ht, b_ht;
	
	b_ht = Approx_Bearing(here, there);
	*behind = Is_Point_Behind(b_ht, &track); //Remember this edits track!
	if(!*behind)
	{
		d_ht = Approx_Distance(here, there);
		d_xt = asinf(sinf_approx(d_ht/EARTH_R)*sinf_approx((b_ht - track) * PI_OVER_180));
		*xt = fabs(d_xt);
	}
}

/*  
			*******************************
			*			Trig Approximations			*
			*******************************
*/

float sinf_approx(float n)
{ //Assumed input is in rads
	float x3 = (n*n*n)/6.0f; //x^3/3!
	float x5 = (n*n*n*n*n)/120.0f; //x^5/5!
	//float x7 = (n*n*n*n*n*n*n)/5040.0f; //x^7/7!  //Dont need this level of percision to get within 100m
	return n - x3 + x5;// - x7;
}

float cosf_approx(float n)
{ //Assume input is in rads
	float x2 = (n*n)/2.0f; //x^2/2!
	float x4 = (n*n*n*n)/24.0f; //x^4/4!
	//float x6 = (n*n*n*n*n*n)/720.0f; //x^6/6! //Dont need this  level ... 
	return 1 - x2 + x4;// - x6;
}	
 
float atan2f_approx(float y, float x)  //Introduces too much error
{
		//Grabbed from
		//https://gist.github.com/volkansalma/2972237
	
		//http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
    //Volkan SALMA

	float r, angle;
	float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}
	angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return( -angle );     // negate if in quad III or IV
	else
		return( angle );
}
