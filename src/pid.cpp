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

#include <Arduino.h>
#include "pid.hpp"

PID::PID() {
    proportional_gain = 1.0e-8f;
    integral_time = 1.0e8f;
    derivative_time = 0.0f;
    derivative_filter_coefficient = 0.01;
    m_integral = 0.0f;
    m_differential = 0.0f;
    previous_error = 0.0f;
    sample_time = 0.01f;
}

void PID::set_parameter(float kp, float ti, float td, float eta, float h) {
    proportional_gain = kp;
    integral_time = ti;
    derivative_time = td;
    derivative_filter_coefficient = eta;
    sample_time = h;
}

void PID::reset(void) {
    m_integral = 0.0f;
    m_differential = 0.0f;
    previous_error = 0.0f;
    prev_error_2 = 0.0f;
    prev_error_3 = 0.0f;
}

void PID::i_reset(void) {
    m_integral = 0.0f;
}

void PID::printGain(void) {
    Serial.printf("#Kp: %8.4f Ti: %8.4f Td: %8.4f Eta: %8.4f h: %8.4f\r\n", proportional_gain, integral_time, derivative_time, derivative_filter_coefficient, sample_time);
}

void PID::set_error(float err) {
    previous_error = err;
}

float PID::update(float current_error, float h) {
    float output;
    sample_time = h;
    m_integral = m_integral + sample_time * (current_error + previous_error) / 2 / integral_time;
    if (m_integral > 30000.0f) m_integral = 30000.0f;
    if (m_integral < -30000.0f) m_integral = -30000.0f;
    m_differential = (2 * derivative_filter_coefficient * derivative_time - sample_time) * m_differential / (2 * derivative_filter_coefficient * derivative_time + sample_time) + 2 * derivative_time * (current_error - previous_error) / (2 * derivative_filter_coefficient * derivative_time + sample_time);
    previous_error = current_error;
    return proportional_gain * (current_error + m_integral + m_differential);
}

Filter::Filter() {
    filter_state = 0.0f;
    time_constant = 0.0025f;
    sample_time = 0.0025f;
}

void Filter::reset(void) {
    filter_state = 0.0f;
}

void Filter::set_parameter(float T, float h) {
    time_constant = T;
    sample_time = h;
}

float Filter::update(float input_value, float h) {
    sample_time = h;
    filter_state = filter_state * time_constant / (time_constant + sample_time) + input_value * sample_time / (time_constant + sample_time);
    filter_output = filter_state;
    return filter_output;
}

