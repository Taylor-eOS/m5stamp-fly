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

#include "flight_control.hpp"
#include "rc.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "led.hpp"
#include "telemetry.hpp"
#include "button.hpp"
#include "buzzer.h"
#include "utils.hpp"
#include "status_transmitter.h"
#include "flow.hpp"

const int pwmFrontLeft = 5;
const int pwmFrontRight = 42;
const int pwmRearLeft = 10;
const int pwmRearRight = 41;
const int freq = 150000;
const int resolution = 8;
const int FrontLeft_motor = 0;
const int FrontRight_motor = 1;
const int RearLeft_motor = 2;
const int RearRight_motor = 3;
float Control_period = 0.0025f;  
const float Roll_rate_kp = 0.65f;
const float Roll_rate_ti = 0.7f;
const float Roll_rate_td = 0.01;
const float Roll_rate_eta = 0.125f;
const float Pitch_rate_kp = 0.95f;
const float Pitch_rate_ti = 0.7f;
const float Pitch_rate_td = 0.025f;
const float Pitch_rate_eta = 0.125f;
const float Yaw_rate_kp = 3.0f;
const float Yaw_rate_ti = 0.8f;
const float Yaw_rate_td = 0.01f;
const float Yaw_rate_eta = 0.125f;
const float Rall_angle_kp = 5.0f;  
const float Rall_angle_ti = 4.0f;
const float Rall_angle_td = 0.04f;
const float Rall_angle_eta = 0.125f;
const float Pitch_angle_kp = 5.0f;  
const float Pitch_angle_ti = 4.0f;
const float Pitch_angle_td = 0.04f;
const float Pitch_angle_eta = 0.125f;
const float alt_kp = 0.38f;  
const float alt_ti = 10.0f;  
const float alt_td = 0.5f;   
const float alt_eta = 0.125f;
const float alt_period = 0.0333;
const float z_dot_kp = 0.08f;  
const float z_dot_ti = 0.95f;  
const float z_dot_td = 0.08f;  
const float z_dot_eta = 0.125f;
const float Duty_bias_up = 1.581f;  
const float Duty_bias_down = 1.578f;  
volatile float Elapsed_time = 0.0f;
volatile float Old_Elapsed_time = 0.0f;
volatile float Interval_time = 0.0f;
volatile uint32_t S_time = 0, E_time = 0, D_time = 0, Dt_time = 0;
uint8_t AngleControlCounter = 0;
uint16_t RateControlCounter = 0;
uint16_t OffsetCounter = 0;
uint16_t Auto_takeoff_counter = 0;
volatile float FrontRight_motor_duty = 0.0f;
volatile float FrontLeft_motor_duty = 0.0f;
volatile float RearRight_motor_duty = 0.0f;
volatile float RearLeft_motor_duty = 0.0f;
volatile float Roll_rate_reference = 0.0f, Pitch_rate_reference = 0.0f, Yaw_rate_reference = 0.0f;
volatile float Roll_angle_reference = 0.0f, Pitch_angle_reference = 0.0f, Yaw_angle_reference = 0.0f;
volatile float Thrust_command = 0.0f, Thrust_command2 = 0.0f;
volatile float Roll_rate_command = 0.0f, Pitch_rate_command = 0.0f, Yaw_rate_command = 0.0f;
volatile float Roll_angle_command = 0.0f, Pitch_angle_command = 0.0f, Yaw_angle_command = 0.0f;
volatile float Roll_angle_offset = 0.0f, Pitch_angle_offset = 0.0f, Yaw_angle_offset = 0.0f;
volatile float Elevator_center = 0.0f, Aileron_center = 0.0f, Rudder_center = 0.0f;
float Timevalue = 0.0f;
volatile uint8_t Mode = INIT_MODE;
volatile uint8_t OldMode = INIT_MODE;
uint8_t Control_mode = ANGLECONTROL;
float Motor_on_duty_threshold = 0.1f;
float Angle_control_on_duty_threshold = 0.5f;
int8_t BtnA_counter = 0;
uint8_t BtnA_on_flag = 0;
uint8_t BtnA_off_flag = 1;
volatile uint8_t Loop_flag = 0;
uint8_t Stick_return_flag = 0;
uint8_t Throttle_control_mode = 0;
uint8_t Landing_state = 0;
uint8_t OladRange0flag = 0;
volatile uint8_t Ahrs_reset_flag = 0;
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
PID alt_pid;
PID z_dot_pid;
Filter Thrust_filtered;
Filter Duty_fr;
Filter Duty_fl;
Filter Duty_rr;
Filter Duty_rl;
volatile float Thrust0 = 0.0;
uint8_t Alt_flag = 0;
float Z_dot_ref = 0.0f;
const float Alt_ref0 = 0.5f;
volatile float Alt_ref = Alt_ref0;
uint8_t ahrs_reset_flag = 0;
uint8_t last_ahrs_reset_flag = 0;
void init_pwm();
void control_init();
void variable_init(void);
void get_command(void);
void angle_control(void);
void rate_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void motor_stop(void);
uint8_t judge_mode_change(void);
uint8_t get_arming_button(void);
void reset_rate_control(void);
void reset_angle_control(void);
uint8_t auto_landing(void);
float get_trim_duty(float voltage);
float get_rate_ref(float x);
void run_program(void);

hw_timer_t* timer = NULL;
void IRAM_ATTR onTimer() {
    Loop_flag = 1;
}

void init_copter(void) {
    Mode = INIT_MODE;
    led_init();
    esp_led(0x110000, 1);
    onboard_led1(WHITE, 1);
    onboard_led2(WHITE, 1);
    led_show();
    led_show();
    led_show();
    USBSerial.begin(115200);
    delay(1500);
    initStatusTransmitter();
    USBSerial.printf("Start init\r\n");
    init_pwm();
    sensor_init();
    control_init();
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2500, true);
    timerAlarmEnable(timer);
    init_button();
    setup_pwm_buzzer();
    setup_flow();
    USBSerial.printf("Finish init\r\n");
    if (0) test_spin();
    start_tone();
    Mode = AVERAGE_MODE;
    //OffsetCounter = AVERAGENUM;
    S_time = micros();
    USBSerial.printf("FORCED TEST MODE: Mode=%d OffsetCounter=%d\n", Mode, OffsetCounter);
    sendStatusMessage("ESP-NOW status messaging ready");
}

void loop_400Hz(void) {
    sensor_read();
    if (0) USBSerial.printf("Altitude2: %f RawRange: %d Range: %d ToFflag: %d Volt: %f OffsetCnt: %d Mode: %d Thrust0: %f ThCmdRatio: %f\n", Altitude2, RawRange, Range, ToF_bottom_data_ready_flag, Voltage, OffsetCounter, Mode, Thrust0, (BATTERY_VOLTAGE>0?Thrust_command/BATTERY_VOLTAGE: 0.0f));
    static uint8_t led = 1;
    float sense_time;
    while (Loop_flag == 0);
    static uint32_t debug_counter = 0;
    debug_counter++;
    if (debug_counter % 400 == 0) {
        if (0) USBSerial.printf("Loop running: %lu, Altitude2: %f, Mode: %d\n", debug_counter, Altitude2, Mode);
        //sendStatusMessage("Altitude2: " + String(Altitude2) + ", mode: " + String(Mode));
        sendStatusMessage("Altitude2: " + String(Altitude2) + ", mode: " + String(Mode) + ", throttle control mode: " + String(Throttle_control_mode));
    }
    Loop_flag = 0;
    E_time = micros();
    Old_Elapsed_time = Elapsed_time;
    Elapsed_time = 1e-6 * (E_time - S_time);
    Interval_time = Elapsed_time - Old_Elapsed_time;
    Timevalue += 0.0025f;
    sense_time = sensor_read();
    uint32_t cs_time = micros();
    led_drive();
    if (OffsetCounter < AVERAGENUM) {
        motor_stop();
        sensor_calc_offset_avarage();
        OffsetCounter++;
        if (OffsetCounter % 100 == 0) {
            if (0) USBSerial.printf("Offset calibration: %d/%d, Altitude2: %f\n", OffsetCounter, AVERAGENUM, Altitude2);
        }
        return;
    }
    if (S_time == 0) S_time = micros();
    Control_period = Interval_time;
    if (0) run_program();
    angle_control();
    rate_control();
    loop_flow();
    telemetry();
    uint32_t ce_time = micros();
    Dt_time = ce_time - cs_time;
}

void run_program(void) {
    enum {RP_INIT=0, RP_TAKEOFF=1, RP_ASCEND=2, RP_HOLD=3, RP_DESCEND=4, RP_DONE=5};
    static uint8_t state = RP_INIT;
    static uint32_t state_start_ms = 0;
    static float start_alt = 0.0f;
    const float target_alt = 0.1f;
    const uint32_t ascend_ms = 1500;
    const uint32_t hold_ms = 5000;
    const uint32_t descend_ms = 1500;
    uint32_t now_ms = micros() / 1000;
    float dt = Control_period;
    if (dt <= 0.0f) dt = 0.0025f;
    switch (state) {
    case RP_INIT:
        state_start_ms = now_ms;
        start_alt = Altitude2;
        Alt_flag = 0;
        Throttle_control_mode = 1;
        Roll_angle_command = 0.0f;
        Pitch_angle_command = 0.0f;
        Yaw_rate_reference = 0.0f;
        Thrust0 = 0.0f;
        Auto_takeoff_counter = 0;
        Alt_ref = start_alt;
        state = RP_TAKEOFF;
        break;
    case RP_TAKEOFF: {
        if (Auto_takeoff_counter < 200) {
            Thrust0 = (float)Auto_takeoff_counter / 400.0;
            if (Thrust0 > get_trim_duty(3.8)) Thrust0 = get_trim_duty(3.8);
            Auto_takeoff_counter++;
        } else if (Auto_takeoff_counter < 400) {
            Thrust0 = (float)Auto_takeoff_counter / 400.0;
            if (Thrust0 > get_trim_duty(Voltage)) Thrust0 = get_trim_duty(Voltage);
            Auto_takeoff_counter++;
        } else {
            Thrust0 = get_trim_duty(Voltage);
            start_alt = Altitude2;
            Alt_ref = start_alt;
            Alt_flag = 1;
            state = RP_ASCEND;
            state_start_ms = now_ms;
        }
        Thrust_command = Thrust0 * BATTERY_VOLTAGE;
        break;
    }
    case RP_ASCEND: {
        uint32_t dt_ms = now_ms - state_start_ms;
        if (dt_ms >= ascend_ms) {
            Alt_ref = target_alt;
            state = RP_HOLD;
            state_start_ms = now_ms;
        } else {
            float p = (float)dt_ms / (float)ascend_ms;
            Alt_ref = start_alt + (target_alt - start_alt) * p;
        }
        break;
    }
    case RP_HOLD:
        Alt_ref = target_alt;
        if (now_ms - state_start_ms >= hold_ms) {
            state = RP_DESCEND;
            state_start_ms = now_ms;
            start_alt = Alt_ref;
        }
        break;
    case RP_DESCEND: {
        uint32_t dt_ms = now_ms - state_start_ms;
        if (dt_ms >= descend_ms) {
            Thrust_command = 0.0f;
            Thrust0 = 0.0f;
            motor_stop();
            state = RP_DONE;
            return;
        } else {
            float p = (float)dt_ms / (float)descend_ms;
            Alt_ref = start_alt * (1.0f - p);
        }
        break;
    }
    case RP_DONE:
        return;
    }
    Thrust_command = Thrust0 * BATTERY_VOLTAGE;
}

void rate_control(void) {
    float p_rate, q_rate, r_rate;
    float p_ref, q_ref, r_ref;
    float p_err, q_err, r_err, z_dot_err;
    if (Thrust0 < Motor_on_duty_threshold) {
        reset_rate_control();
    } else {
        p_rate = Roll_rate;
        q_rate = Pitch_rate;
        r_rate = Yaw_rate;
        p_ref = Roll_rate_reference;
        q_ref = Pitch_rate_reference;
        r_ref = Yaw_rate_reference;
        p_err = p_ref - p_rate;
        q_err = q_ref - q_rate;
        r_err = r_ref - r_rate;
        Roll_rate_command = p_pid.update(p_err, Interval_time);
        Pitch_rate_command = q_pid.update(q_err, Interval_time);
        Yaw_rate_command = r_pid.update(r_err, Interval_time);
        if (Alt_flag == 1) {
            z_dot_err = Z_dot_ref - Alt_velocity;
            Thrust_command = Thrust_filtered.update(
                (Thrust0 + z_dot_pid.update(z_dot_err, Interval_time)) * BATTERY_VOLTAGE, Interval_time);
            if (Thrust_command / BATTERY_VOLTAGE > Thrust0 * 1.4f) Thrust_command = BATTERY_VOLTAGE * Thrust0 * 1.4f;
            if (Thrust_command / BATTERY_VOLTAGE < Thrust0 * 0.6f) Thrust_command = BATTERY_VOLTAGE * Thrust0 * 0.6f;
        }
        FrontRight_motor_duty = Duty_fr.update(
            (Thrust_command + (-Roll_rate_command + Pitch_rate_command + Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE, Interval_time);
        FrontLeft_motor_duty = Duty_fl.update(
            (Thrust_command + (Roll_rate_command + Pitch_rate_command - Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE, Interval_time);
        RearRight_motor_duty = Duty_rr.update(
            (Thrust_command + (-Roll_rate_command - Pitch_rate_command - Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE, Interval_time);
        RearLeft_motor_duty = Duty_rl.update(
            (Thrust_command + (Roll_rate_command - Pitch_rate_command + Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE, Interval_time);
        const float minimum_duty = 0.0f;
        const float maximum_duty = 0.95f;
        if (FrontRight_motor_duty < minimum_duty) FrontRight_motor_duty = minimum_duty;
        if (FrontRight_motor_duty > maximum_duty) FrontRight_motor_duty = maximum_duty;
        if (FrontLeft_motor_duty < minimum_duty) FrontLeft_motor_duty = minimum_duty;
        if (FrontLeft_motor_duty > maximum_duty) FrontLeft_motor_duty = maximum_duty;
        if (RearRight_motor_duty < minimum_duty) RearRight_motor_duty = minimum_duty;
        if (RearRight_motor_duty > maximum_duty) RearRight_motor_duty = maximum_duty;
        if (RearLeft_motor_duty < minimum_duty) RearLeft_motor_duty = minimum_duty;
        if (RearLeft_motor_duty > maximum_duty) RearLeft_motor_duty = maximum_duty;
        if (OverG_flag == 0) {
            set_duty_fr(FrontRight_motor_duty);
            set_duty_fl(FrontLeft_motor_duty);
            set_duty_rr(RearRight_motor_duty);
            set_duty_rl(RearLeft_motor_duty);
        } else {
            FrontRight_motor_duty = 0.0;
            FrontLeft_motor_duty = 0.0;
            RearRight_motor_duty = 0.0;
            RearLeft_motor_duty = 0.0;
            motor_stop();
        }
    }
}

void angle_control(void) {
    float phi_err, theta_err, alt_err;
    if (Thrust_command / BATTERY_VOLTAGE < Motor_on_duty_threshold) {
        reset_angle_control();
    } else {
        alt_err = Alt_ref - Altitude2;
        if (Alt_flag >= 1) Z_dot_ref = alt_pid.update(alt_err, Interval_time);
        Roll_angle_reference = 0.5f * PI * Roll_angle_command;
        Pitch_angle_reference = 0.5f * PI * Pitch_angle_command;
        if (Roll_angle_reference > (30.0f * PI / 180.0f)) Roll_angle_reference = 30.0f * PI / 180.0f;
        if (Roll_angle_reference < -(30.0f * PI / 180.0f)) Roll_angle_reference = -30.0f * PI / 180.0f;
        if (Pitch_angle_reference > (30.0f * PI / 180.0f)) Pitch_angle_reference = 30.0f * PI / 180.0f;
        if (Pitch_angle_reference < -(30.0f * PI / 180.0f)) Pitch_angle_reference = -30.0f * PI / 180.0f;
        phi_err = Roll_angle_reference - (Roll_angle - Roll_angle_offset);
        theta_err = Pitch_angle_reference - (Pitch_angle - Pitch_angle_offset);
        Roll_rate_reference = phi_pid.update(phi_err, Interval_time);
        Pitch_rate_reference = theta_pid.update(theta_err, Interval_time);
    }
}

uint8_t judge_mode_change(void) {
    uint8_t state;
    static uint8_t chatter = 0;
    state = 0;
    if (chatter == 0) {
        if (get_arming_button() == 1) {
            chatter = 1;
        }
    } else {
        if (get_arming_button() == 0) {
            chatter++;
            if (chatter > 40) {
                chatter = 0;
                state = 1;
            }
        }
    }
    return state;
}

void control_init(void) {
    p_pid.set_parameter(Roll_rate_kp, Roll_rate_ti, Roll_rate_td, Roll_rate_eta, Control_period);  
    q_pid.set_parameter(Pitch_rate_kp, Pitch_rate_ti, Pitch_rate_td, Pitch_rate_eta, Control_period);  
    r_pid.set_parameter(Yaw_rate_kp, Yaw_rate_ti, Yaw_rate_td, Yaw_rate_eta, Control_period);  
    phi_pid.set_parameter(Rall_angle_kp, Rall_angle_ti, Rall_angle_td, Rall_angle_eta, Control_period);  
    theta_pid.set_parameter(Pitch_angle_kp, Pitch_angle_ti, Pitch_angle_td, Pitch_angle_eta, Control_period);  
    alt_pid.set_parameter(0.5f, 5.0f, 0.4f, alt_eta, alt_period);
    z_dot_pid.set_parameter(0.15f, 0.7f, 0.06f, alt_eta, alt_period);
    Duty_fl.set_parameter(0.003, Control_period);
    Duty_fr.set_parameter(0.003, Control_period);
    Duty_rl.set_parameter(0.003, Control_period);
    Duty_rr.set_parameter(0.003, Control_period);
    Thrust_filtered.set_parameter(0.01, Control_period);
}

float get_trim_duty(float voltage) {
    return -0.2448f * voltage + 1.5892f;
}

float get_rate_ref(float x) {
    float ref;
    float h;
    if (x < -1.0f) x = -1.0f;
    if (x > 1.0f) x = 1.0f;
    if (x >= 0.0f) {
        h = x * (x * x * x * x * x * RATE_EXPO + x * (1 - RATE_EXPO));
        ref = PI / 180.0f * ((RATE_MAX - RATE_RATE) * h + RATE_RATE * x);
    } else {
        x = -x;
        h = x * (x * x * x * x * x * RATE_EXPO + x * (1 - RATE_EXPO));
        ref = -PI / 180.0f * ((RATE_MAX - RATE_RATE) * h + RATE_RATE * x);
    }
    return ref;
}

void get_command(void) {
    static uint16_t stick_count = 0;
    static float auto_throttle = 0.0f;
    static float old_alt = 0.0;
    float th, thlo;
    float throttle_limit = 0.7;
    float thrust_max;
    Control_mode = Stick[CONTROLMODE];
    if ((uint8_t)Stick[ALTCONTROLMODE] == MANUAL_ALT)
        //Throttle_control_mode = 1;
        Throttle_control_mode = 0;
    else if ((uint8_t)Stick[ALTCONTROLMODE] == MANUAL_ALT)
        Throttle_control_mode = 0;
    else
        Throttle_control_mode = 0;
    thlo = Stick[THROTTLE];
    if (Throttle_control_mode == 0) {
        if (thlo < 0.0) thlo = 0.0;
        if (thlo > 1.0f) thlo = 1.0f;
        if ((-0.2 < thlo) && (thlo < 0.2)) thlo = 0.0f;  
        th = (get_trim_duty(Voltage) + (thlo - 0.4)) * BATTERY_VOLTAGE;
        if (th < 0) th = 0.0f;
        Thrust_command = Thrust_filtered.update(th, Interval_time);
    } else if (Throttle_control_mode == 1) {
        Alt_flag = 1;
        if (Auto_takeoff_counter < 500) {
            Thrust0 = (float)Auto_takeoff_counter / 1000.0;
            if (Thrust0 > get_trim_duty(3.8)) Thrust0 = get_trim_duty(3.8);
            Auto_takeoff_counter++;
        } else if (Auto_takeoff_counter < 1000) {
            Thrust0 = (float)Auto_takeoff_counter / 1000.0;
            if (Thrust0 > get_trim_duty(Voltage)) Thrust0 = get_trim_duty(Voltage);
            Auto_takeoff_counter++;
        } else
            Thrust0 = get_trim_duty(Voltage);
        if ((-0.2 < thlo) && (thlo < 0.2)) thlo = 0.0f;  
        Alt_ref = Alt_ref + thlo * 0.001;
        if (Alt_ref > ALT_REF_MAX) Alt_ref = ALT_REF_MAX;
        if (Alt_ref < ALT_REF_MIN) Alt_ref = ALT_REF_MIN;
        if ((Range0flag > OladRange0flag) || (Range0flag == RNAGE0FLAG_MAX)) {
            Thrust0 = Thrust0 - 0.02;
            OladRange0flag = Range0flag;
        }
        Thrust_command = Thrust0 * BATTERY_VOLTAGE;
    }
    if (Control_mode == ANGLECONTROL) {
        Roll_angle_command = 0.4 * Stick[AILERON];
        if (Roll_angle_command < -1.0f) Roll_angle_command = -1.0f;
        if (Roll_angle_command > 1.0f) Roll_angle_command = 1.0f;
        Pitch_angle_command = 0.4 * Stick[ELEVATOR];
        if (Pitch_angle_command < -1.0f) Pitch_angle_command = -1.0f;
        if (Pitch_angle_command > 1.0f) Pitch_angle_command = 1.0f;
    } else if (Control_mode == RATECONTROL) {
        Roll_rate_reference = get_rate_ref(Stick[AILERON]);
        Pitch_rate_reference = get_rate_ref(Stick[ELEVATOR]);
    }
    Yaw_angle_command = Stick[RUDDER];
    if (Yaw_angle_command < -1.0f) Yaw_angle_command = -1.0f;
    if (Yaw_angle_command > 1.0f) Yaw_angle_command = 1.0f;
    Yaw_rate_reference = 2.0f * PI * (Yaw_angle_command - Rudder_center);
}

uint8_t auto_landing(void) {
    uint8_t flag;
    static float auto_throttle;
    static uint16_t counter = 0;
    static float old_alt[10];
    float thrust_max;
    Alt_flag = 0;
    if (Landing_state == 0) {
        Landing_state = 1;
        counter = 0;
        for (uint8_t i = 0; i < 10; i++) old_alt[i] = Altitude2;
        Thrust0 = get_trim_duty(Voltage);
    }
    if (old_alt[9] >= Altitude2)  
    {
        Thrust0 = Thrust0 * 0.9999;
    }
    if (Altitude2 < 0.15)  
    {
        Thrust0 = Thrust0 * 0.999;
    }
    if (Altitude2 < 0.1) {
        flag = 1;
        Landing_state = 0;
    } else
        flag = 0;
    for (int i = 1; i < 10; i++) old_alt[i] = old_alt[i - 1];
    old_alt[0] = Altitude2;
    Roll_angle_command = 0.4 * Stick[AILERON];
    if (Roll_angle_command < -1.0f) Roll_angle_command = -1.0f;
    if (Roll_angle_command > 1.0f) Roll_angle_command = 1.0f;
    Pitch_angle_command = 0.4 * Stick[ELEVATOR];
    if (Pitch_angle_command < -1.0f) Pitch_angle_command = -1.0f;
    if (Pitch_angle_command > 1.0f) Pitch_angle_command = 1.0f;
    Yaw_angle_command = Stick[RUDDER];
    if (Yaw_angle_command < -1.0f) Yaw_angle_command = -1.0f;
    if (Yaw_angle_command > 1.0f) Yaw_angle_command = 1.0f;
    Yaw_rate_reference = 2.0f * PI * (Yaw_angle_command - Rudder_center);
    if (Control_mode == RATECONTROL) {
        Roll_rate_reference = get_rate_ref(Stick[AILERON]);
        Pitch_rate_reference = get_rate_ref(Stick[ELEVATOR]);
    }
    return flag;
}

void reset_rate_control(void) {
    motor_stop();
    FrontRight_motor_duty = 0.0;
    FrontLeft_motor_duty = 0.0;
    RearRight_motor_duty = 0.0;
    RearLeft_motor_duty = 0.0;
    Duty_fr.reset();
    Duty_fl.reset();
    Duty_rr.reset();
    Duty_rl.reset();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    alt_pid.reset();
    z_dot_pid.reset();
    Roll_rate_reference = 0.0f;
    Pitch_rate_reference = 0.0f;
    Yaw_rate_reference = 0.0f;
    Rudder_center = Yaw_angle_command;
    Roll_rate_reference = 0.0f;
    Pitch_rate_reference = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
    Roll_angle_offset = 0;
    Pitch_angle_offset = 0;
}

void reset_angle_control(void) {
    Roll_rate_reference = 0.0f;
    Pitch_rate_reference = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
    Aileron_center = Roll_angle_command;
    Elevator_center = Pitch_angle_command;
    Roll_angle_offset = 0;
    Pitch_angle_offset = 0;
}

void set_duty_fr(float duty) {
    ledcWrite(FrontRight_motor, (uint32_t)(255 * duty));
}
void set_duty_fl(float duty) {
    ledcWrite(FrontLeft_motor, (uint32_t)(255 * duty));
}
void set_duty_rr(float duty) {
    ledcWrite(RearRight_motor, (uint32_t)(255 * duty));
}
void set_duty_rl(float duty) {
    ledcWrite(RearLeft_motor, (uint32_t)(255 * duty));
}

void init_pwm(void) {
    ledcSetup(FrontLeft_motor, freq, resolution);
    ledcSetup(FrontRight_motor, freq, resolution);
    ledcSetup(RearLeft_motor, freq, resolution);
    ledcSetup(RearRight_motor, freq, resolution);
    ledcAttachPin(pwmFrontLeft, FrontLeft_motor);
    ledcAttachPin(pwmFrontRight, FrontRight_motor);
    ledcAttachPin(pwmRearLeft, RearLeft_motor);
    ledcAttachPin(pwmRearRight, RearRight_motor);
}

uint8_t get_arming_button(void) {
    static int8_t chatta = 0;
    static uint8_t state = 0;
    if ((int)Stick[BUTTON_ARM] == 1) {
        chatta++;
        if (chatta > 10) {
            chatta = 10;
            state = 1;
        }
    } else {
        chatta--;
        if (chatta < -10) {
            chatta = -10;
            state = 0;
        }
    }
    return state;
}

void motor_stop(void) {
    set_duty_fr(0.0);
    set_duty_fl(0.0);
    set_duty_rr(0.0);
    set_duty_rl(0.0);
}

