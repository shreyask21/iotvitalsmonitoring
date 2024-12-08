#include <deque>
#include <algorithm>
#include <inttypes.h>
#ifndef __ANALOG_AUTORANGE_H
#define __ANALOG_AUTORANGE_H

template <typename dtype>
class AnalogAutoRanging
{
private:
    std::deque<dtype> samplebuf_dq;
    size_t dq_max_size = 5;
    dtype min_ele, max_ele;
    bool autoranged = false;
    int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

public:
    AnalogAutoRanging(size_t window_size)
    {
        dq_max_size = window_size;
    }

    dtype addSample(dtype sample, dtype scale_min, dtype scale_max)
    {
        static dtype out;
        samplebuf_dq.push_back(sample);
        min_ele = *min_element(samplebuf_dq.begin(), samplebuf_dq.end());
        max_ele = *max_element(samplebuf_dq.begin(), samplebuf_dq.end());
        out = this->map(sample, min_ele, max_ele,
                        scale_min, scale_max);
        if (samplebuf_dq.size() >= dq_max_size)
        {
            samplebuf_dq.pop_front();
            autoranged = true;
            return out;
        }
        autoranged = false;
        return out;
    }

    bool isValid()
    {
        return autoranged;
    }

    void reset()
    {
        samplebuf_dq.clear();
    }

    bool isReset()
    {
        return samplebuf_dq.empty();
    }

    dtype getMin()
    {
        return min_ele;
    }

    dtype getMax()
    {
        return max_ele;
    }
};
#endif