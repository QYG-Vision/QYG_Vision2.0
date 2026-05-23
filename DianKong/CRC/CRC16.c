#include "CRC16.h"

extern const CRC16_t crc16_MODBUS_TAB[256];
extern const CRC16_t crc16_CCITT_TAB[256];
extern const CRC16_t crc16_CCITT_F_TAB[256];
extern const CRC16_t crc16_X25_TAB[256];
extern const CRC16_t crc16_USB_TAB[256];
extern const CRC16_t crc16_MAXIM_TAB[256];
extern const CRC16_t crc16_IBM_TAB[256];
extern const CRC16_t crc16_DNP_TAB[256];
extern const CRC16_t crc16_KERMIT_TAB[256];
extern const CRC16_t crc16_SICK_TAB[256];

static CRC16_t crc16_reflect(uint16_t data)
{
    data =  (data << 8)           |  (data >> 8)          ;
    data = ((data << 4) & 0xF0F0) | ((data >> 4) & 0x0F0F);
    data = ((data << 2) & 0xCCCC) | ((data >> 2) & 0x3333);
    data = ((data << 1) & 0xAAAA) | ((data >> 1) & 0x5555);
    return data;
}

static uint8_t crc8_reflect(uint8_t data) 
{
    data = (data << 4) | (data >> 4);
    data = ((data << 2) & 0xCC) | ((data >> 2) & 0x33);
    data = ((data << 1) & 0xAA) | ((data >> 1) & 0x55);
    return data;
}

/*  
** Descriptions: CRC16 checksum INIT
** Input: CRC TYPE
** Output: INIT TRUE or FALSE
*/ 
uint32_t CRC16_Init(CRC16_INFO_t *crc_info, uint8_t CRC_TYPE)
{
    if(crc_info == NULL || CRC_TYPE == NULL)
    {
        return __CRC16_False;
    }
    else{
        crc_info ->CRC16_type = CRC_TYPE;

        switch (CRC_TYPE)
        {
            case CRC16_MODBUS   :
            {
                crc_info ->polynomial = 0x8005;
                crc_info ->init_value = 0xFFFF;
                crc_info ->xor_output = 0x0000;

                crc_info ->reverse_input  = true;
                crc_info ->reverse_output = true;

                crc_info ->TAB = crc16_MODBUS_TAB;
                break;
            }
            case CRC16_CCITT    :
            {
                crc_info ->polynomial = 0x1021;
                crc_info ->init_value = 0x0000;
                crc_info ->xor_output = 0x0000;

                crc_info ->reverse_input  = false;
                crc_info ->reverse_output = false;

                crc_info ->TAB = crc16_CCITT_TAB;
                break;
            }
            case CRC16_CCITT_F  :
            {
                crc_info ->polynomial = 0x1021;
                crc_info ->init_value = 0xFFFF;
                crc_info ->xor_output = 0x0000;

                crc_info ->reverse_input  = false;
                crc_info ->reverse_output = false;

               crc_info ->TAB = crc16_CCITT_F_TAB;
                break;
            }
            case CRC16_X25      :
            {
                crc_info ->polynomial = 0x1021;
                crc_info ->init_value = 0xFFFF;
                crc_info ->xor_output = 0xFFFF;

                crc_info ->reverse_input  = true;
                crc_info ->reverse_output = true;

                crc_info ->TAB = crc16_X25_TAB;
                break;
            }
            case CRC16_USB      :
            {
                crc_info ->polynomial = 0x8005;
                crc_info ->init_value = 0xFFFF;
                crc_info ->xor_output = 0xFFFF;

                crc_info ->reverse_input  = true;
                crc_info ->reverse_output = true;

                crc_info ->TAB = crc16_USB_TAB;
                break;
            }
            case CRC16_MAXIM    :
            {
                crc_info ->polynomial = 0x8005;
                crc_info ->init_value = 0x0000;
                crc_info ->xor_output = 0xFFFF;

                crc_info ->reverse_input  = true;
                crc_info ->reverse_output = true;

                crc_info ->TAB = crc16_MAXIM_TAB;
                break;
            }
            case CRC16_IBM      :
            {
                crc_info ->polynomial = 0x1021;
                crc_info ->init_value = 0xFFFF;
                crc_info ->xor_output = 0xFFFF;

                crc_info ->reverse_input  = true;
                crc_info ->reverse_output = true;

                crc_info ->TAB = crc16_IBM_TAB;
                break;
            }
            case CRC16_DNP      :
            {
                crc_info ->polynomial = 0x3D65;
                crc_info ->init_value = 0x0000;
                crc_info ->xor_output = 0xFFFF;

                crc_info ->reverse_input  = true;
                crc_info ->reverse_output = true;

                crc_info ->TAB = crc16_DNP_TAB;
                break;
            }
            case CRC16_KERMIT   :
            {
                crc_info ->polynomial = 0x1021;
                crc_info ->init_value = 0x0000;
                crc_info ->xor_output = 0x0000;

                crc_info ->reverse_input  = true;
                crc_info ->reverse_output = true;

                crc_info ->TAB = crc16_KERMIT_TAB;
                break;
            }
            case CRC16_SICK     :
            {
                crc_info ->polynomial = 0x8005;
                crc_info ->init_value = 0x0000;
                crc_info ->xor_output = 0x0000;

                crc_info ->reverse_input  = true;
                crc_info ->reverse_output = false;

                crc_info ->TAB = crc16_SICK_TAB;
                break;
            }
            default             :
            {
                return __CRC16_False;
            }
        }
    }

    return __CRC16_True;
}

/*  
** Descriptions: CRC16 checksum function  
** Input: Data to check,Stream length, initialized checksum  
** Output: CRC checksum  
*/  
uint16_t Get_CRC16(uint8_t *pchMessage, uint32_t dwLength, CRC16_INFO_t *crc_info)
{
    uint16_t wCRC;
    
    if (pchMessage == NULL)  
    {  
        return 0xFFFF;  
    }

    wCRC = crc_info->init_value;

    while (dwLength--) {
        wCRC = (wCRC >> 8) ^ crc_info->TAB[((uint16_t)(wCRC) ^ (uint16_t)(*pchMessage++)) & 0x00ff];
    }

    wCRC ^= crc_info->xor_output;

    return wCRC;
}


/*  
** Descriptions: CRC16 Verify function  
** Input: Data to Verify,Stream length = Data + checksum，CRC16_TYPE
** Output: True or False (CRC Verify Result)  
*/  
uint32_t Verify_CRC16(uint8_t *pchMessage, uint32_t dwLength, CRC16_INFO_t *crc_info)
{
    uint16_t wExpected         = 0;
    uint8_t  received_crc_low  = 0;
    uint8_t  received_crc_high = 0;

    if ((pchMessage == NULL) || (dwLength <= 2) || (crc_info == NULL)) {
        return __CRC16_False;
    }

    wExpected = Get_CRC16(pchMessage, dwLength - 2, crc_info);
     
    /* Endianness distinction ... */
    /*
        Big-endian output (Big-Endian, high byte first) ：
            * CRC16-CCITT_FALSE (0x1021)
        Little-endian output (Little-Endian, low byte first) ：
            * CRC16-MODBUS (0x8005)
            * CRC16-CCITT (0x1021)
            * CRC16-USB (0x8005)
            * CRC16-MAXIM (0x8005)
            * CRC16-X25 (0x1021)
            * CRC16-IBM (0x8005)
            * CRC16-DNP (0x3D65)
            * CRC16-KERMIT (0x1021)
            * CRC16-SICK (0x8005)
    */
    // uint8_t received_crc_low  = pchMessage[dwLength - 2];
    // uint8_t received_crc_high = pchMessage[dwLength - 1];

    // uint16_t received_crc = (received_crc_high << 8) | received_crc_low;
    switch(crc_info ->CRC16_type){
        case CRC16_MODBUS      :
        {
            received_crc_low  = pchMessage[dwLength - 2];
            received_crc_high = pchMessage[dwLength - 1];
            break;
        }
        case CRC16_CCITT       :
        {
            received_crc_low  = pchMessage[dwLength - 2];
            received_crc_high = pchMessage[dwLength - 1];
            break;
        }
        case CRC16_CCITT_F     :
        {
            received_crc_high = pchMessage[dwLength - 2];
            received_crc_low  = pchMessage[dwLength - 1];
            break;
        }
        case CRC16_X25         :
        {
            received_crc_low  = pchMessage[dwLength - 2];
            received_crc_high = pchMessage[dwLength - 1];
            break;
        }
        case CRC16_USB         :
        {
            received_crc_low  = pchMessage[dwLength - 2];
            received_crc_high = pchMessage[dwLength - 1];
            break;
        }
        case CRC16_MAXIM       :
        {
            received_crc_low  = pchMessage[dwLength - 2];
            received_crc_high = pchMessage[dwLength - 1];
            break;
        }
        case CRC16_IBM         :
        {
            received_crc_low  = pchMessage[dwLength - 2];
            received_crc_high = pchMessage[dwLength - 1];
            break;
        }
        case CRC16_DNP         :
        {
            received_crc_low  = pchMessage[dwLength - 2];
            received_crc_high = pchMessage[dwLength - 1];
            break;
        }
        case CRC16_KERMIT      :
        {
            received_crc_low  = pchMessage[dwLength - 2];
            received_crc_high = pchMessage[dwLength - 1];
            break;
        }
        case CRC16_SICK        :
        {
            received_crc_low  = pchMessage[dwLength - 2];
            received_crc_high = pchMessage[dwLength - 1];
            break;
        }
        default                :
        {
            return __CRC16_False;
        }
    }

    uint16_t received_crc = (received_crc_high << 8) | received_crc_low;

    return (wExpected == received_crc) ? __CRC16_True : __CRC16_False;
}

/*  
** Descriptions: append CRC16 to the end of data  
** Input: Data to CRC and append,Stream length = Data + checksum, CRC16_TYPE
** Output: True or False (CRC Verify Result)  
*/  
uint32_t Append_CRC16(uint8_t *pchMessage, uint32_t dwLength, CRC16_INFO_t *crc_info) 
{
    if ((pchMessage == NULL) || (crc_info == NULL) || (dwLength <= 2)) {
        return __CRC16_False;
    }

    uint16_t wCRC = Get_CRC16(pchMessage, dwLength - 2, crc_info);

    /* Endianness distinction ... */
    /*
        Big-endian output (Big-Endian, high byte first) ：
            * CRC16-CCITT_FALSE (0x1021)
        Little-endian output (Little-Endian, low byte first) ：
            * CRC16-MODBUS (0x8005)
            * CRC16-CCITT (0x1021)
            * CRC16-USB (0x8005)
            * CRC16-MAXIM (0x8005)
            * CRC16-X25 (0x1021)
            * CRC16-IBM (0x8005)
            * CRC16-DNP (0x3D65)
            * CRC16-KERMIT (0x1021)
            * CRC16-SICK (0x8005)
    */
    switch(crc_info ->CRC16_type){
        case CRC16_MODBUS      :
        {
            pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00FF);
            pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00FF);
            break;
        }
        case CRC16_CCITT       :
        {
            pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00FF);
            pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00FF);
            break;
        }
        case CRC16_CCITT_F     :
        {
            pchMessage[dwLength - 2] = (uint8_t)((wCRC >> 8) & 0x00FF);
            pchMessage[dwLength - 1] = (uint8_t)(wCRC & 0x00FF);
            break;
        }
        case CRC16_X25         :
        {
            pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00FF);
            pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00FF);
            break;
        }
        case CRC16_USB         :
        {
            pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00FF);
            pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00FF);
            break;
        }
        case CRC16_MAXIM       :
        {
            pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00FF);
            pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00FF);
            break;
        }
        case CRC16_IBM         :
        {
            pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00FF);
            pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00FF);
            break;
        }
        case CRC16_DNP         :
        {
            pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00FF);
            pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00FF);
            break;
        }
        case CRC16_KERMIT      :
        {
            pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00FF);
            pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00FF);
            break;
        }
        case CRC16_SICK        :
        {
            pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00FF);
            pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00FF);
            break;
        }
        default                :
        {
            return __CRC16_False;
        }
    }

    return __CRC16_True;
}