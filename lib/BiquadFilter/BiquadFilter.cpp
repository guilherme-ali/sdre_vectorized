#include "BiquadFilter.h"

BiquadFilter::BiquadFilter() {
    v1 = 0.0f;
    v2 = 0.0f;
}

void BiquadFilter::begin(float fc, float fs) {
    // Limita fc para um pouco abaixo de Nyquist (fs/2) para evitar falha matemática
    if (fc >= fs / 2.0f) fc = (fs / 2.0f) * 0.95f; 
    
    float omega = tan(PI * fc / fs);
    float omega2 = omega * omega;
    float sqrt2_omega = sqrt(2.0f) * omega;
    
    float inv_D = 1.0f / (1.0f + sqrt2_omega + omega2);
    
    b0 = omega2 * inv_D;
    b1 = 2.0f * b0;
    b2 = b0;
    
    a1 = 2.0f * (omega2 - 1.0f) * inv_D;
    a2 = (1.0f - sqrt2_omega + omega2) * inv_D;
}

float BiquadFilter::update(float x) {
    float v0 = x - a1 * v1 - a2 * v2;
    float y = b0 * v0 + b1 * v1 + b2 * v2;
    v2 = v1;
    v1 = v0;
    return y;
}
