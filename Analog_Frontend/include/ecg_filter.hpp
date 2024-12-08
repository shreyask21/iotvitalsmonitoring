#include <inttypes.h>
#include "common_defines.hpp"
#include "arm_math.h"
#ifndef __ECG_FILTER_H
#define __ECG_FILTER_H
#define NUM_STAGES 1
class ECGFilter
{
private:
    arm_biquad_casd_df1_inst_f32 filter;
    float32_t biquadCoeffs[5 * NUM_STAGES] = {0.3897, 0, -0.3897, 1.0383, -0.2206};
    float32_t biquadState[4 * NUM_STAGES];

public:
    ECGFilter()
    {
        arm_biquad_cascade_df1_init_f32(&filter, NUM_STAGES, biquadCoeffs, biquadState);
    }
    int16_t process_sample(int16_t input_sample)
    {
        static int16_t op_s16;
        static uint32_t __buff_idx = 0;
        static float32_t __buff_filter_ip[CONFIG_ECG_SAMPRATE];
        static float32_t __buff_filter_op[CONFIG_ECG_SAMPRATE];
        __buff_filter_ip[0] = input_sample;
        __buff_idx = (__buff_idx + 1) % (sizeof(__buff_filter_ip) / sizeof(__buff_filter_ip[0]));
        arm_biquad_cascade_df1_f32(&filter, __buff_filter_ip, __buff_filter_op, (sizeof(__buff_filter_ip) / sizeof(__buff_filter_ip[0])));
        op_s16 = (int16_t)__buff_filter_op[0];
        return op_s16;
    }
};
#endif