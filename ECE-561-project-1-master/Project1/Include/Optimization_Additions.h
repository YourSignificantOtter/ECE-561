#ifndef OPTIMIZATION_ADDITIONS_H_
#define OPTIMIZATION_ADDITIONS_H_

int Is_Point_Behind(float Bearing, float * track);

void  Approx_Crosstrack_Error(GPT_T * here, float track, const GPT_T * there, int * behind, float * xt);
float Approx_Bearing(GPT_T * p1, const GPT_T * p2);
float Approx_Distance(GPT_T * p1, const GPT_T * p2);

float atan2f_approx(float y, float x);
float sinf_approx(float n);
float cosf_approx(float n);
#endif