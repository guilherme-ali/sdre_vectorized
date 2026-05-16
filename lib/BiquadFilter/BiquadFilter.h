#ifndef BIQUAD_FILTER_H
#define BIQUAD_FILTER_H

#include <Arduino.h>
#include <math.h>

// ===== Filtro Butterworth de 2ª Ordem =====
class BiquadFilter {
private:
    float v1, v2;
    float a1, a2, b0, b1, b2;
public:
    BiquadFilter();
    void begin(float fc, float fs);
    float update(float x);
};

#endif
