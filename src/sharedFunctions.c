#include "sharedFunctions.h"


void lowPassFilter(float *current_value, float *filtered_value, float LPF_ALPHA)
{
    *filtered_value = LPF_ALPHA * (*current_value) + (1.0f - LPF_ALPHA) * (*filtered_value);
}

float interp_1D(float index, const float LUT_INDICES[], const float LUT_OUT[], int N)
{
    if(index <= LUT_INDICES[0])
    {
        return LUT_OUT[0];
    }
    if(index >= LUT_INDICES[N - 1])
    {
        return LUT_OUT[N - 1];
    }

    for(int i = 0; i < N - 1; i++)
    {
        if((index >= LUT_INDICES[i]) && (index <= LUT_INDICES[i + 1]))
        {
            float out = (index - LUT_INDICES[i]) / (LUT_INDICES[i + 1] - LUT_INDICES[i]);
            return LUT_OUT[i] + out * (LUT_OUT[i + 1] - LUT_OUT[i]);
        }
    }
    return LUT_OUT[N - 1];
}