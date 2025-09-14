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

#ifndef PID_HPP
#define PID_HPP

class PID {
   private:
    float proportional_gain;
    float integral_time;
    float derivative_time;
    float derivative_filter_coefficient;
    float previous_error, prev_error_2, prev_error_3;
    float sample_time;

   public:
    float m_differential;
    float m_integral;
    PID();
    void set_parameter(float kp, float ti, float td, float eta, float h);
    void reset(void);
    void i_reset(void);
    void printGain(void);
    void set_error(float err);
    float update(float current_error, float h);
};

class Filter {
   private:
    float filter_state;
    float time_constant;
    float sample_time;

   public:
    float filter_output;
    Filter();
    void set_parameter(float T, float h);
    void reset(void);
    float update(float input_value, float h);
};

#endif

