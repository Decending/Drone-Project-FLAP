#include "biquad.h"

/**
 * @brief
 */
void BiquadUpdateCoeffs(biquad_coeffs_t *coeffs,
                        float sampling_frequency,
                        float center_frequency,
                        float Q,
                        biquad_type_t type)
{
    // Based on:
    // http://shepazu.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
    //std::cout.precision(4);
    const float omega = 2.0f * M_PI * center_frequency / sampling_frequency;
    //std::cout <<std::setw(10) << 2.0f << "\n";
    //std::cout <<std::setw(10) << M_PI << "\n";
    //std::cout <<std::setw(10) << center_frequency << "\n";
    //std::cout <<std::setw(10) << sampling_frequency << "\n";
    //std::cout <<std::setw(10) << omega << "\n";
    const float os    = sinf(omega);
    const float oc    = cosf(omega);
    const float alpha = os / (2.0f * Q);
    //std::cout <<std::setw(10) << os << "\n";
    //std::cout <<std::setw(10) << oc << "\n";
    //std::cout <<std::setw(10) << alpha << "\n";
    float b0 = 0;
    float b1 = 0;
    float b2 = 0;
    float a0 = 0;
    float a1 = 0;
    float a2 = 0;

    b0 = (1.0f - oc) * 0.5f;
    b1 =  1.0f - oc;
    b2 = (1.0f - oc) * 0.5f;
    a0 =  1.0f + alpha;
    a1 = -2.0f * oc;
    a2 =  1.0f - alpha;

    //std::cout <<std::setw(10) << a0 << "\n";
    //std::cout <<std::setw(10) << a1 << "\n";
    //std::cout <<std::setw(10) << a2 << "\n";
    //std::cout <<std::setw(10) << b0 << "\n";
    //std::cout <<std::setw(10) << b1 << "\n";
    //std::cout <<std::setw(10) << b2 << "\n";
    coeffs->b0 = b0 / a0;
    coeffs->b1 = b1 / a0;
    coeffs->b2 = b2 / a0;
    coeffs->a1 = a1 / a0;
    coeffs->a2 = a2 / a0;
    //std::cout <<std::setw(10) << coeffs->b0 << "\n";
    //std::cout <<std::setw(10) << coeffs->b1 << "\n";
    //std::cout <<std::setw(10) << coeffs->b2 << "\n";
    //std::cout <<std::setw(10) << coeffs->a1 << "\n";
    //std::cout <<std::setw(10) << coeffs->a2 << "\n";
}
