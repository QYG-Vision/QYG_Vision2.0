#ifndef __SERIALPLOT_H_
#define __SERIALPLOT_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx.h"
/* USER CODE END Includes */

void sentdata2SerialPlot(float data);
void sentMultiData2SerialPlot(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10);
void VofaTenDataChannel(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10);	

#ifdef __cplusplus
}
#endif

#endif
