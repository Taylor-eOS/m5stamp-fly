/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "alt_kalman.hpp"
#include <Arduino.h>

Alt_kalman::Alt_kalman() {};

void Alt_kalman::update(float z_sens, float accel, float h) {
    step = h;
    velocity_ = velocity + (accel - bias) * step;
    altitude_ = altitude + velocity * step;
    bias_     = bias * (1 + step * beta);
    p11_ = p11 - step * (p31 + p13) + step * step * p33 + step * step * q1;
    p12_ = step * (p11 - p32) - step * step * p31 + p12;
    p13_ = (1 + beta * step) * (p13 - step * p33);
    p21_ = step * (p11 - p23) - step * step * p13 + p21;
    p22_ = step * step * p11 + step * (p21 + p12) + p22;
    p23_ = (1 + beta * step) * (p23 + step * p13);
    p31_ = (1 + beta * step) * (p31 - step * p33);
    p32_ = (1 + beta * step) * (p32 + step * p31);
    p33_ = (1 + beta * step) * (1 + beta * step) * p33 + step * step * q2;
    float s = p22_ + R;
    k1      = p12_ / s;
    k2      = p22_ / s;
    k3      = p32_ / s;
    float e = z_sens - altitude_;
    velocity = velocity_ + k1 * e;
    altitude = altitude_ + k2 * e;
    bias     = bias_ + k3 * e;
    Velocity = velocity;
    Altitude = altitude;
    Bias     = bias;
    p11 = p11_ - k1 * p21_;
    p12 = p12_ - k1 * p22_;
    p13 = p13_ - k1 * p23_;
    p21 = p21_ * (1 - k2);
    p22 = p22_ * (1 - k2);
    p23 = p23_ * (1 - k2);
    p31 = -k3 * p21_ + p31_;
    p32 = -k3 * p22_ + p32_;
    p33 = -k3 * p23_ + p33_;
}

void Alt_kalman::set_vel(float v) {
    velocity = v;
}

void Alt_kalman::reset(void) {
    p11 = 100.0, p12 = 0.0, p13 = 0.0;
    p21 = 0.0, p22 = 100.0, p23 = 0.0;
    p31 = 0.0, p32 = 0.0, p33 = 100.0;
    Velocity = 0.0, Altitude = 0.0, Bias = 0.0;
}

void mat_times(Mat A, Mat B) {
}

