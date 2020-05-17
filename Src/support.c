#include "support.h"
#include "stm32f4xx_hal.h"

#define		SIGNAL_DELAY_SIZE 				64															// Number of iterations to delay by. 64 is 1/4 of 256 which is 90degrees
#define		INTEGRAL_SIZE							256															// Number of samples to integrate over. 256 is 1 full cycle
#define		RMS_INTEGRAL_SIZE					500

extern 		TIM_HandleTypeDef 	htim9;

//--------------------------------------------------------------------------------
// Keep a running summation of the last FILTER_SIZE values passed into this function.
int32_t Integral(int32_t datum)
{	
	static int32_t 		Buffer[INTEGRAL_SIZE];
	static uint16_t  	Index = 0; 
	static int32_t  	Sum = 0;	
	
	Sum += datum;	
	
  Buffer[Index++] = datum;
    
  if(Index == INTEGRAL_SIZE)                                    
    Index = 0;
	
	Sum -= Buffer[Index];
  return(Sum);
}


// This function simply fills a buffer from one end and returns values SIGNAL_DELAY_SIZE insertions later
int32_t Signal_Delay(int16_t datum)
{		
	static int16_t Delay_Buffer[SIGNAL_DELAY_SIZE];
	static int16_t Signal_Delay_Index = 0;	
	
  Delay_Buffer[Signal_Delay_Index++] = datum;	
    
  if(Signal_Delay_Index == SIGNAL_DELAY_SIZE)                                    
    Signal_Delay_Index = 0;
	
  return((int32_t)Delay_Buffer[Signal_Delay_Index]);
}


// Returns the median value of a buffer of unsigned values. Note FILTER_LENGTH must be odd in this implimentation 
int16_t Get_Current_Median(int16_t *Buffer, int8_t FILTER_LENGTH, int16_t OFFSET)
{		
	// Copy the buffer otherwise the DMA could overwrite a value mid calculation
	int16_t Buffer_Copy[FILTER_LENGTH];	
	for(int8_t x = 0; x < FILTER_LENGTH; x++)	{
		Buffer_Copy[x] = Buffer[x];
	}
	
	// This code arranges the copied buffer values into ascending order
	int8_t i=0, j=0; int16_t temp=0;	
	for(i=0; i<FILTER_LENGTH; i++)	{
			for(j=0; j<FILTER_LENGTH-1; j++)	{
					if(Buffer_Copy[j] > Buffer_Copy[j+1])	{
							temp        			= Buffer_Copy[j];
							Buffer_Copy[j]    = Buffer_Copy[j+1];
							Buffer_Copy[j+1]  = temp;
					}
			}
	}	
	
	int16_t median = OFFSET - Buffer_Copy[FILTER_LENGTH/2];	
	return(median);
}


uint32_t Integrate_Mains_RMS(int16_t ADC_Line_V)
{	
	static uint32_t 	Buffer[RMS_INTEGRAL_SIZE];
	static uint16_t  	Index = 0; 
	static uint32_t  	Sum = 0;
	
	int32_t 	Sample 			= ADC_Line_V;
	uint32_t 	Sample_Sqrd = Sample * Sample;
	
	Sum += Sample_Sqrd;	
	
  Buffer[Index++] = Sample_Sqrd;
    
  if(Index == RMS_INTEGRAL_SIZE)                                    
    Index = 0;
	
	Sum -= Buffer[Index];
  return(Sum/RMS_INTEGRAL_SIZE);	
}


void Await_ZCP(int16_t *ADC_Line_V)
{	
	static uint16_t Sample = 2000;
	
	while(Sample >= 1938)	
		Sample = *ADC_Line_V;
	
	while(Sample <= 1958)
		Sample = *ADC_Line_V;
	
	HAL_Delay(2);
	
	while(Sample > 1958) 
		Sample = *ADC_Line_V;
}





int16_t Running_Avg(int16_t datum)
{	
#define CONSTRAIN(x,lower,upper)	((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

static int16_t lastval = 0, difference;

	difference = datum - lastval;
	difference = CONSTRAIN(difference, -10, 10);
	
	lastval = lastval + difference;

	return(lastval);


//	#define length_Avg 8
//	
//	static int16_t 		Buffer[length_Avg];
//	static uint16_t  	Index = 0; 
//	static int32_t  	Sum = 0;
//	
//  Buffer[Index++] = datum;
//    
//  if(Index == length_Avg)                                    
//    Index = 0;
//	
//	Sum = 0;
//	for(uint8_t i = 0; i < length_Avg; i++)
//		Sum += Buffer[Index];
//	
//  return(Sum/length_Avg);
}

























