#ifndef _SUPPORT_H_
#define _SUPPORT_H_


#include <stdint.h>
#include <stdbool.h>

int32_t 	Integral(int32_t datum);
int32_t 	Signal_Delay(int16_t datum);
int16_t 	Get_Median(int16_t *Buffer, int8_t FILTER_LENGTH, int16_t OFFSET);
uint32_t 	Integrate_Mains_RMS(int16_t ADC_Line_V);
void			Await_ZCP(int16_t *ADC_Line_V);
int16_t 	Running_Avg(int16_t datum);

#endif




