#ifndef __CRC16_H_
#define __CRC16_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"
#include "stdbool.h"
#include "arm_math.h"
#include "CRC_table.h"

#define  __CRC16_False  (uint32_t)0x00000000
#define  __CRC16_True   (uint32_t)0x00000001

#define  CRC16_MODBUS   0x01
#define  CRC16_CCITT    0x02
#define  CRC16_CCITT_F  0x03
#define  CRC16_X25      0x04
#define  CRC16_USB      0x05
#define  CRC16_MAXIM    0x06
#define  CRC16_IBM      0x07
#define  CRC16_DNP      0x08
#define  CRC16_KERMIT   0x09
#define  CRC16_SICK     0x0A

typedef struct 
{
    uint8_t CRC16_type;

    CRC16_t polynomial;     //CRC16校验多项式
    CRC16_t init_value;     //CRC16校验初始值
    CRC16_t xor_output;     //输出数据异或值

    bool    reverse_input;  //输入数据是否反转
    bool    reverse_output; //输出数据是否反转

    const CRC16_t *TAB;           //CRC16校验表
}CRC16_INFO_t;

static CRC16_t crc16_reflect(uint16_t data);
static uint8_t crc8_reflect(uint8_t data);

extern uint32_t CRC16_Init(CRC16_INFO_t *crc_info, uint8_t CRC_TYPE);
extern uint16_t Get_CRC16(uint8_t *pchMessage,uint32_t dwLength, CRC16_INFO_t *crc_info);
extern uint32_t Verify_CRC16(uint8_t *pchMessage, uint32_t dwLength, CRC16_INFO_t *crc_info);
extern uint32_t Append_CRC16(uint8_t *pchMessage, uint32_t dwLength, CRC16_INFO_t *crc_info);

#ifdef __cplusplus
}
#endif

#endif
