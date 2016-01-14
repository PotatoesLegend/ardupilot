// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsBunny.cpp - ArduCopter motors library
 *       Code by Tao Du. MIT CSAIL
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsBunny.h"

extern const AP_HAL::HAL& hal;

// setup_motors - configures the motors for bunny rotors.
void AP_MotorsBunny::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // Motor positions:
    // Motor 1: tilted motor, left, ccw.
    // Motor 2: back, cw.
    // Motor 3: tilted motor, right, ccw.
    // Motor 4: front, cw.
    /*
    add_motor_raw(AP_MOTORS_MOT_1,   0.67f,   0.69f,   0.39f,  1);
    add_motor_raw(AP_MOTORS_MOT_2,   0.16f,  -1.12f,  -0.25f,  2);
    add_motor_raw(AP_MOTORS_MOT_3,  -0.75f,  -0.78f,   0.33f,  3);
    add_motor_raw(AP_MOTORS_MOT_4,  -0.10f,   1.19f,  -0.25f,  4);
    */
    /*
    add_motor_raw(AP_MOTORS_MOT_1,    1.0f,     0.0f,      1.0f,  1);
    add_motor_raw(AP_MOTORS_MOT_2,    0.0f,    -0.40f,   -0.707f,  1);
    add_motor_raw(AP_MOTORS_MOT_3,   -1.0f,     0.0f,      1.0f,  1);
    add_motor_raw(AP_MOTORS_MOT_4,    0.0f,     0.75f,   -0.707f,  1);
    */
    add_motor_raw(AP_MOTORS_MOT_1,    0.0f,    0.0f,    0.0f,  1);
    add_motor_raw(AP_MOTORS_MOT_2,    0.0f,    0.0f,    0.0f,  1);
    add_motor_raw(AP_MOTORS_MOT_3,    0.0f,    0.0f,    0.0f,  1);
    add_motor_raw(AP_MOTORS_MOT_4,    0.0f,    0.0f,    0.0f,  1);
    // Set up the throttle factors.
    for(int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        _throttle_factor[i] = 0.0f;
    }
    const float scale_factor = 0.8f;
    _throttle_factor[AP_MOTORS_MOT_1] = 0.65f * scale_factor;
    _throttle_factor[AP_MOTORS_MOT_2] = 0.67f * scale_factor;
    _throttle_factor[AP_MOTORS_MOT_3] = 0.65f * scale_factor;
    _throttle_factor[AP_MOTORS_MOT_4] = 0.77f * scale_factor;
}

void AP_MotorsBunny::output_armed_not_stabilizing()
{
    uint8_t i;
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];                    // final outputs sent to the motors
    int16_t out_min_pwm = _throttle_radio_min + _min_throttle;      // minimum pwm value we can send to the motors
    int16_t out_max_pwm = _throttle_radio_max;                      // maximum pwm value we can send to the motors

    // initialize limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    int16_t thr_in_min = rel_pwm_to_thr_range(_spin_when_armed_ramped);
    if (_throttle_control_input <= thr_in_min) {
        _throttle_control_input = thr_in_min;
        limit.throttle_lower = true;
    }

    if (_throttle_control_input >= _hover_out) {
        _throttle_control_input = _hover_out;
        limit.throttle_upper = true;
    }

    throttle_radio_output = calc_throttle_radio_output();

    // set output throttle
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // Scale the motor output by throttle factor.
            motor_out[i] = (int16_t)(_throttle_radio_min +
                    ((float)(throttle_radio_output - _throttle_radio_min)) * _throttle_factor[i]);
        }
    }

    if(throttle_radio_output >= out_min_pwm) {
        // apply thrust curve and voltage scaling
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = apply_thrust_curve_and_volt_scaling(motor_out[i], out_min_pwm, out_max_pwm);
            }
        }
    }

    // send output to each motor
    hal.rcout->cork();
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            rc_write(i, motor_out[i]);
        }
    }
    hal.rcout->push();
}

void AP_MotorsBunny::output_armed_stabilizing()
{
    int8_t i;
    int16_t roll_pwm;                                               // roll pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t pitch_pwm;                                              // pitch pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t yaw_pwm;                                                // yaw pwm value, initially calculated by calc_yaw_pwm() but may be modified after, +/- 400
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t out_min_pwm = _throttle_radio_min + _min_throttle;      // minimum pwm value we can send to the motors
    int16_t out_max_pwm = _throttle_radio_max;                      // maximum pwm value we can send to the motors
    int16_t out_mid_pwm = (out_min_pwm+out_max_pwm)/2;              // mid pwm value we can send to the motors
    int16_t out_best_thr_pwm;                                       // the is the best throttle we can come up which provides good control without climbing
    float rpy_scale = 1.0;                                          // this is used to scale the roll, pitch and yaw to fit within the motor limits

    int16_t rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors

    int16_t rpy_low = 0;    // lowest motor value
    int16_t rpy_high = 0;   // highest motor value
    int16_t yaw_allowed;    // amount of yaw we can fit in
    int16_t thr_adj;        // the difference between the pilot's desired throttle and out_best_thr_pwm (the throttle that is actually provided)

    // initialize limits flags
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // Ensure throttle is within bounds of 0 to 1000
    int16_t thr_in_min = rel_pwm_to_thr_range(_min_throttle);
    if (_throttle_control_input <= thr_in_min) {
        _throttle_control_input = thr_in_min;
        limit.throttle_lower = true;
    }
    if (_throttle_control_input >= _max_throttle) {
        _throttle_control_input = _max_throttle;
        limit.throttle_upper = true;
    }

    roll_pwm = calc_roll_pwm();
    pitch_pwm = calc_pitch_pwm();
    yaw_pwm = calc_yaw_pwm();
    throttle_radio_output = calc_throttle_radio_output();

    // calculate roll and pitch for each motor
    // set rpy_low and rpy_high to the lowest and highest values of the motors
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rpy_out[i] = roll_pwm * _roll_factor[i] * get_compensation_gain() +
                            pitch_pwm * _pitch_factor[i] * get_compensation_gain();

            // record lowest roll pitch command
            if (rpy_out[i] < rpy_low) {
                rpy_low = rpy_out[i];
            }
            // record highest roll pich command
            if (rpy_out[i] > rpy_high) {
                rpy_high = rpy_out[i];
            }
        }
    }

    // calculate throttle that gives most possible room for yaw (range 1000 ~ 2000) which is the lower of:
    //      1. mid throttle - average of highest and lowest motor (this would give the maximum possible room margin above the highest motor and below the lowest)
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the mid point between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favour reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favour reducing throttle instead of better yaw control because the pilot has commanded it
    int16_t motor_mid = (rpy_low+rpy_high)/2;
    out_best_thr_pwm = MIN(out_mid_pwm - motor_mid, MAX(throttle_radio_output, throttle_radio_output*MAX(0,1.0f-_throttle_thr_mix)+get_hover_throttle_as_pwm()*_throttle_thr_mix));

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller
    yaw_allowed = MIN(out_max_pwm - out_best_thr_pwm, out_best_thr_pwm - out_min_pwm) - (rpy_high-rpy_low)/2;
    yaw_allowed = MAX(yaw_allowed, _yaw_headroom);

    if (yaw_pwm >= 0) {
        // if yawing right
        if (yaw_allowed > yaw_pwm * get_compensation_gain()) {
            yaw_allowed = yaw_pwm * get_compensation_gain(); // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
        }else{
            limit.yaw = true;
        }
    }else{
        // if yawing left
        yaw_allowed = -yaw_allowed;
        if (yaw_allowed < yaw_pwm * get_compensation_gain()) {
            yaw_allowed = yaw_pwm * get_compensation_gain(); // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
        }else{
            limit.yaw = true;
        }
    }

    // add yaw to intermediate numbers for each motor
    rpy_low = 0;
    rpy_high = 0;
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rpy_out[i] =    rpy_out[i] +
                            yaw_allowed * _yaw_factor[i];

            // record lowest roll+pitch+yaw command
            if( rpy_out[i] < rpy_low ) {
                rpy_low = rpy_out[i];
            }
            // record highest roll+pitch+yaw command
            if( rpy_out[i] > rpy_high) {
                rpy_high = rpy_out[i];
            }
        }
    }

    // check everything fits
    thr_adj = throttle_radio_output - out_best_thr_pwm;

    // calculate upper and lower limits of thr_adj
    int16_t thr_adj_max = MAX(out_max_pwm-(out_best_thr_pwm+rpy_high),0);

    // if we are increasing the throttle (situation #2 above)..
    if (thr_adj > 0) {
        // increase throttle as close as possible to requested throttle
        // without going over out_max_pwm
        if (thr_adj > thr_adj_max){
            thr_adj = thr_adj_max;
            // we haven't even been able to apply full throttle command
            limit.throttle_upper = true;
        }
    }else if(thr_adj < 0){
        // decrease throttle as close as possible to requested throttle
        // without going under out_min_pwm or over out_max_pwm
        // earlier code ensures we can't break both boundaries
        int16_t thr_adj_min = MIN(out_min_pwm-(out_best_thr_pwm+rpy_low),0);
        if (thr_adj > thr_adj_max) {
            thr_adj = thr_adj_max;
            limit.throttle_upper = true;
        }
        if (thr_adj < thr_adj_min) {
            thr_adj = thr_adj_min;
        }
    }

    // do we need to reduce roll, pitch, yaw command
    // earlier code does not allow both limit's to be passed simultaneously with abs(_yaw_factor)<1
    if ((rpy_low+out_best_thr_pwm)+thr_adj < out_min_pwm){
        // protect against divide by zero
        if (rpy_low != 0) {
            rpy_scale = (float)(out_min_pwm-thr_adj-out_best_thr_pwm)/rpy_low;
        }
        // we haven't even been able to apply full roll, pitch and minimal yaw without scaling
        limit.roll_pitch = true;
        limit.yaw = true;
    }else if((rpy_high+out_best_thr_pwm)+thr_adj > out_max_pwm){
        // protect against divide by zero
        if (rpy_high != 0) {
            rpy_scale = (float)(out_max_pwm-thr_adj-out_best_thr_pwm)/rpy_high;
        }
        // we haven't even been able to apply full roll, pitch and minimal yaw without scaling
        limit.roll_pitch = true;
        limit.yaw = true;
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_out[i] = (int16_t)((float)(out_best_thr_pwm + thr_adj - _throttle_radio_min) * _throttle_factor[i]
                            + _throttle_radio_min + rpy_scale*rpy_out[i]);
        }
    }

    // apply thrust curve and voltage scaling
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_out[i] = apply_thrust_curve_and_volt_scaling(motor_out[i], out_min_pwm, out_max_pwm);
        }
    }

    // clip motor output if required (shouldn't be)
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_out[i] = constrain_int16(motor_out[i], out_min_pwm, out_max_pwm);
        }
    }

    // send output to each motor
    hal.rcout->cork();
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            rc_write(i, motor_out[i]);
        }
    }
    hal.rcout->push();
}
