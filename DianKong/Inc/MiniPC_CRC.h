//
// Created by ASUS on 2026/8/5.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_MINIPC_CRC_H
#define JIUXIAN_SP_GIMBAL_XPP_MINIPC_CRC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

    uint16_t minipc_get_crc16_check_sum(uint8_t *p_data, uint32_t data_len);
    int      minipc_verify_crc16_check_sum(uint8_t *p_frame, uint32_t frame_len);
    void     minipc_append_crc16_check_sum(uint8_t *p_frame, uint32_t total_len);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_MINIPC_CRC_H