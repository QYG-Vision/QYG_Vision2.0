#include "SerialPlot.h"
#include "stdio.h"

void sentdata2SerialPlot(float data)
{
		char buffer[16];
    	int len = sprintf(buffer, "%.2f\n", data);
		HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, 10);
}

void sentMultiData2SerialPlot(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10)
{
		char buffer[64];
    	int len = sprintf(buffer, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", data1, data2, data3, data4, data5, data6, data7, data8, data9, data10);
		HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, 10);
}

void VofaTenDataChannel(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10)
{
    printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n", data1, data2, data3, data4, data5, data6, data7, data8, data9, data10);
}
