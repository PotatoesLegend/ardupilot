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
 *       AP_MotorsMatrix.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/vectorN.h>
#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <AP_Math/matrix3.h>
#include "AP_MotorsMatrix.h"
// Tao Du
// taodu@csail.mit.edu
// Jul 5, 2018
#include "../../ArduCopter/Copter.h"
// Beginning of LQR. This part is auto generated. DO NOT MANUALLY MODIFY IT.
// Tao Du
// taodu@csail.mit.edu
static const float u_eq[5] = { 3.038193f, 4.198680f, 3.038193f, 2.320973f, 4.198680f };
static const float x_eq[12] = { 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f };
static const int kMotorNum = 5;
static const float K[kMotorNum][12] = {
    { -0.308043f, -0.950349f, -0.671349f, -5.438282f, 1.756804f, -0.383051f, -0.453130f, -1.398954f, -0.930757f, -1.056540f, 0.338380f, -0.520666f },
    { 0.807749f, -0.588706f, -0.787441f, -3.383546f, -4.611611f, 0.731718f, 1.188515f, -0.867362f, -1.137326f, -0.665645f, -0.889256f, 1.076180f },
    { -0.999026f, 0.000708f, -0.671349f, 0.009701f, 5.714996f, -0.383051f, -1.470509f, 0.001349f, -0.930757f, 0.004670f, 1.109394f, -0.520666f },
    { 0.811370f, 0.589495f, -0.598711f, 3.393288f, -4.670460f, -1.065702f, 1.195839f, 0.868828f, -0.802144f, 0.669581f, -0.921599f, -1.501987f },
    { -0.310285f, 0.950135f, -0.787441f, 5.431476f, 1.792877f, 0.731718f, -0.457639f, 1.398375f, -1.137326f, 1.051428f, 0.358271f, 1.076180f },
};

static int16_t thrust_to_pwm(const float thrust_in_newton, const float voltage)
{
    const float u0 = 14.768811f, us = 0.572843f, p0 = 1150.000000f, ps = 0.001333f, t0 = 0.050654f, ts = 0.081572f;
    const float u = (voltage - u0) * us;
    const float t = (thrust_in_newton - t0) * ts;
    // Now t = [u^2, up, p^2, u, p, 1].dot(func_param)
    const float s_u2 = 0.066712f, s_up = 0.228158f, s_p2 = 0.486307f, s_u = -0.076330f, s_p = 0.395554f, s1 = 0.009862f;
    // Reorganize them into the form of ap^2 + bp + c = 0.
    const float a = s_p2, b = s_p + s_up * u, c = s_u2 * u * u + s_u * u + s1 - t;
    const float delta = b * b - 4 * a * c;
    const int16_t pwm_min = 1000;
    const int16_t pwm_max = 2000;
    if (delta < 0) return pwm_min;
    const float p = (-b + sqrtf(delta)) / (2.0 * a);
    int16_t pwm = static_cast<int16_t>(p / ps + p0);
    // clamp PWM.
    pwm = (pwm > pwm_min) ? pwm : pwm_min;
    pwm = (pwm < pwm_max) ? pwm : pwm_max;
    return pwm;
}
// End of LQR. This part is auto generated. DO NOT MANUALLY MODIFY IT.

static float remap(const float x, const float x0, const float x1, const float y0, const float y1)
{
    const float t = (x - x0) / (x1 - x0);
    return (1.0 - t) * y0 + t * y1;
}

static float target_x = 0.0;
static float target_y = 0.0;
static float target_z = 0.0;
static float target_yaw = 0.0;
static bool enter_target_x = false;
static bool enter_target_y = false;
static bool enter_target_z = false;
static bool enter_target_yaw = false;

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsMatrix::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // record requested frame class and type
    _last_frame_class = frame_class;
    _last_frame_type = frame_type;

    // setup the motors
    setup_motors(frame_class, frame_type);

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// set update rate to motors - a value in hertz
void AP_MotorsMatrix::set_update_rate( uint16_t speed_hz )
{
    uint8_t i;

    // record requested speed
    _speed_hz = speed_hz;

    // check each enabled motor
    uint32_t mask = 0;
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
		mask |= 1U << i;
        }
    }
    rc_set_freq( mask, _speed_hz );
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsMatrix::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // exit immediately if armed or no change
    if (armed() || (frame_class == _last_frame_class && _last_frame_type == frame_type)) {
        return;
    }
    _last_frame_class = frame_class;
    _last_frame_type = frame_type;

    // setup the motors
    setup_motors(frame_class, frame_type);

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsMatrix::enable()
{
    int8_t i;

    // enable output channels
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            rc_enable_ch(i);
        }
    }
}

void AP_MotorsMatrix::output_to_motors()
{
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    // Tao Du
    // taodu@csail.mit.edu
    // Jul 5, 2018
    if (_vicon_mode) {
        switch (_spool_mode) {
            case SHUT_DOWN: {
                // sends minimum values out to the motors
                // set motor output based on thrust requests
                for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                    if (motor_enabled[i]) {
                        if (_disarm_disable_pwm && _disarm_safety_timer == 0 && !armed()) {
                            motor_out[i] = 0;
                        } else {
                            motor_out[i] = get_pwm_output_min();
                        }
                    }
                }
                break;
            }
            case SPIN_WHEN_ARMED:
            case SPOOL_UP:
            case THROTTLE_UNLIMITED:
            case SPOOL_DOWN:
                // Get states:
                // x, y, z, roll, pitch, yaw, u, v, w, roll_speed, pitch_speed, yaw_speed.
                VectorN<float, 12> x;
                _copter.get_vicon_pos(x[0], x[1], x[2]);
                // TODO: do we want to use Vicon here?
                _copter.get_vicon_rpy(x[3], x[4], x[5]);
                x[3] = _copter.get_roll();
                x[4] = _copter.get_pitch();
                // Compute the rotational matrix.
                Matrix3f R;
                R.from_euler(x[3], x[4], x[5]);
                Vector3f xyz_dot;
                _copter.get_vicon_pos_speed(xyz_dot[0], xyz_dot[1], xyz_dot[2]);
                const Vector3f uvw = R.transposed() * xyz_dot;
                x[6] = uvw[0]; x[7] = uvw[1]; x[8] = uvw[2];
                // TODO: do we want to use Vicon here?
                //_copter.get_vicon_rpy_speed(x[9], x[10], x[11]);
                x[9] = _copter.get_roll_rate();
                x[10] = _copter.get_pitch_rate();
                x[11] = _copter.get_yaw_rate();

                // Input PWMs are in the range of [1000, 2000].
                const float roll_pwm = static_cast<float>(hal.rcin->read(0));
                const float pitch_pwm = static_cast<float>(hal.rcin->read(1));
                const float thr_pwm = static_cast<float>(hal.rcin->read(2));
                const float yaw_pwm = static_cast<float>(hal.rcin->read(3));
                // Map roll_pwm from [1000, 2000] to y0 = y + [-max_d, max_d]
                // Map pitch_pwm from [1000, 2000] to x0 = x + [max_d, -max_d]
                // Map thr_pwm from [1000, 2000] to z0 = z + [max_d, -max_d]
                // Map yaw_pwm from [1000, 2000] to yaw0 = yaw + [-max_yaw, max_yaw]
                const float max_d = 1.0f;
                const float max_yaw = 0.5f;
                const int16_t pwm_min = get_pwm_output_min();
                const int16_t pwm_max = get_pwm_output_max();
                const float pwm_min_float = static_cast<float>(pwm_min);
                const float pwm_max_float = static_cast<float>(pwm_max);
                float y1 = remap(roll_pwm, pwm_min_float, pwm_max_float, -max_d, max_d);
                float x1 = remap(pitch_pwm, pwm_min_float, pwm_max_float, max_d, -max_d);
                float z1 = remap(thr_pwm, pwm_min_float, pwm_max_float, max_d, -max_d);
                float yaw1 = remap(yaw_pwm, pwm_min_float, pwm_max_float, -max_yaw, max_yaw);
                // Zero out small values in 10% deadzone.
                if (y1 >= -0.1 * max_d && y1 <= 0.1 * max_d) {
                    if (!enter_target_y) {
                        target_y = x[1];
                    }
                    enter_target_y = true;
                    y1 = target_y - x[1];
                } else {
                    enter_target_y = false;
                }
                if (x1 >= -0.1 * max_d && x1 <= 0.1 * max_d) {
                    if (!enter_target_x) {
                        target_x = x[0];
                    }
                    enter_target_x = true;
                    x1 = target_x - x[0];
                } else {
                    enter_target_x = false;
                }
                if (z1 >= -0.1 * max_d && z1 <= 0.1 * max_d) {
                    if (!enter_target_z) {
                        target_z = x[2];
                    }
                    enter_target_z = true;
                    z1 = target_z - x[2];
                } else {
                    enter_target_z = false;
                }
                if (yaw1 >= -0.1 * max_yaw && yaw1 <= 0.1 * max_yaw) {
                    if (!enter_target_yaw) {
                        target_yaw = x[5];
                    }
                    enter_target_yaw = true;
                    yaw1 = target_yaw - x[5];
                } else {
                    enter_target_yaw = false;
                }

                // u = -K(x - x0) + u0.
                VectorN<float, 12> x0(x_eq);
                x0[2] = -0.65f;
                const VectorN<float, kMotorNum> u0(u_eq);
                VectorN<float, kMotorNum> u = u0;
                VectorN<float, 12> dx = x - x0;
                //dx[0] = -x1; dx[1] = -y1; dx[2] = -z1; dx[5] = -yaw1;
                for (i=0; i<kMotorNum; ++i) {
                    const VectorN<float, 12> Ki(K[i]);
                    u[i] -= Ki * dx;
                }

                const float volt = _copter.get_battery_voltage();
                for (i=0; i<kMotorNum; ++i) {
                    motor_out[i] = thrust_to_pwm(u[i], volt);
                    if (motor_out[i] < pwm_min) motor_out[i] = pwm_min;
                    if (motor_out[i] > pwm_max) motor_out[i] = pwm_max;
                }
                break;
        }
    } else {
        switch (_spool_mode) {
            case SHUT_DOWN: {
                // sends minimum values out to the motors
                // set motor output based on thrust requests
                for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                    if (motor_enabled[i]) {
                        if (_disarm_disable_pwm && _disarm_safety_timer == 0 && !armed()) {
                            motor_out[i] = 0;
                        } else {
                            motor_out[i] = get_pwm_output_min();
                        }
                    }
                }
                break;
            }
            case SPIN_WHEN_ARMED:
                // sends output to motors when armed but not flying
                for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                    if (motor_enabled[i]) {
                        motor_out[i] = calc_spin_up_to_pwm();
                    }
                }
                break;
            case SPOOL_UP:
            case THROTTLE_UNLIMITED:
            case SPOOL_DOWN:
                // set motor output based on thrust requests
                for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                    if (motor_enabled[i]) {
                        motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
                    }
                }
                break;
        }
    }

    // send output to each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }
}


// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsMatrix::get_motor_mask()
{
    uint16_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            mask |= 1U << i;
        }
    }
    return rc_map_mask(mask);
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrix::output_armed_stabilizing()
{
    // Tao Du
    // taodu@csail.mit.edu
    // Jul 5, 2018
    // Do not plan to use this function when Vicon is enabled.
    if (_vicon_mode) return;

    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   rpy_low = 0.0f;             // lowest motor value
    float   rpy_high = 0.0f;            // highest motor value
    float   yaw_allowed = 1.0f;         // amount of yaw we can fit in
    float   unused_range;               // amount of yaw we can fit in the current channel
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in * get_compensation_gain();
    yaw_thrust = _yaw_in * get_compensation_gain();
    throttle_thrust = get_throttle() * get_compensation_gain();

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    _throttle_avg_max = constrain_float(_throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    // calculate throttle that gives most possible room for yaw which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller

    throttle_thrust_best_rpy = MIN(0.5f, _throttle_avg_max);

    // calculate roll and pitch for each motor
    // calculate the amount of yaw input that each motor can accept
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
            if (!is_zero(_yaw_factor[i])){
                if (yaw_thrust * _yaw_factor[i] > 0.0f) {
                    unused_range = fabsf((1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[i]))/_yaw_factor[i]);
                    if (yaw_allowed > unused_range) {
                        yaw_allowed = unused_range;
                    }
                } else {
                    unused_range = fabsf((throttle_thrust_best_rpy + _thrust_rpyt_out[i])/_yaw_factor[i]);
                    if (yaw_allowed > unused_range) {
                        yaw_allowed = unused_range;
                    }
                }
            }
        }
    }

    // todo: make _yaw_headroom 0 to 1
    yaw_allowed = MAX(yaw_allowed, (float)_yaw_headroom/1000.0f);

    if (fabsf(yaw_thrust) > yaw_allowed) {
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw = true;
    }

    // add yaw to intermediate numbers for each motor
    rpy_low = 0.0f;
    rpy_high = 0.0f;
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];

            // record lowest roll+pitch+yaw command
            if (_thrust_rpyt_out[i] < rpy_low) {
                rpy_low = _thrust_rpyt_out[i];
            }
            // record highest roll+pitch+yaw command
            if (_thrust_rpyt_out[i] > rpy_high) {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }

    // check everything fits
    throttle_thrust_best_rpy = MIN(0.5f - (rpy_low+rpy_high)/2.0, _throttle_avg_max);
    if (is_zero(rpy_low)){
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(-throttle_thrust_best_rpy/rpy_low, 0.0f, 1.0f);
    }

    // calculate how close the motors can come to the desired throttle
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f){
        // Full range is being used by roll, pitch, and yaw.
        limit.roll_pitch = true;
        limit.yaw = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else {
        if (thr_adj < -(throttle_thrust_best_rpy+rpy_low)){
            // Throttle can't be reduced to desired value
            thr_adj = -(throttle_thrust_best_rpy+rpy_low);
        } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy+rpy_high)){
            // Throttle can't be increased to desired value
            thr_adj = 1.0f - (throttle_thrust_best_rpy+rpy_high);
            limit.throttle_upper = true;
        }
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = throttle_thrust_best_rpy + thr_adj + rpy_scale*_thrust_rpyt_out[i];
        }
    }

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i], 0.0f, 1.0f);
        }
    }
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsMatrix::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // loop through all the possible orders spinning any motors that match that description
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _test_order[i] == motor_seq) {
            // turn on this motor
            rc_write(i, pwm);
        }
    }
}

// add_motor
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

        // increment number of motors if this motor is being newly motor_enabled
        if( !motor_enabled[motor_num] ) {
            motor_enabled[motor_num] = true;
        }

        // set roll, pitch, thottle factors and opposite motor (for stability patch)
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // call parent class method
        add_motor_num(motor_num);
    }
}

// add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
void AP_MotorsMatrix::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
}

// add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
void AP_MotorsMatrix::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor_raw(
        motor_num,
        cosf(radians(roll_factor_in_degrees + 90)),
        cosf(radians(pitch_factor_in_degrees)),
        yaw_factor,
        testing_order);
}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsMatrix::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {
        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = false;
        _roll_factor[motor_num] = 0;
        _pitch_factor[motor_num] = 0;
        _yaw_factor[motor_num] = 0;
    }
}

void AP_MotorsMatrix::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    bool success = false;

    switch (frame_class) {

        case MOTOR_FRAME_QUAD:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    add_motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
                    add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_V:
                    add_motor(AP_MOTORS_MOT_1,   45,  0.7981f,  1);
                    add_motor(AP_MOTORS_MOT_2, -135,  1.0000f,  3);
                    add_motor(AP_MOTORS_MOT_3,  -45, -0.7981f,  4);
                    add_motor(AP_MOTORS_MOT_4,  135, -1.0000f,  2);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_H:
                    // H frame set-up - same as X but motors spin in opposite directiSons
                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_VTAIL:
                    /*
                        Tested with: Lynxmotion Hunter Vtail 400
                        - inverted rear outward blowing motors (at a 40 degree angle)
                        - should also work with non-inverted rear outward blowing motors
                        - no roll in rear motors
                        - no yaw in front motors
                        - should fly like some mix between a tricopter and X Quadcopter

                        Roll control comes only from the front motors, Yaw control only from the rear motors.
                        Roll & Pitch factor is measured by the angle away from the top of the forward axis to each arm.

                        Note: if we want the front motors to help with yaw,
                            motors 1's yaw factor should be changed to sin(radians(40)).  Where "40" is the vtail angle
                            motors 3's yaw factor should be changed to -sin(radians(40))
                    */
                    add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
                    add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
                    add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
                    add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_ATAIL:
                    /*
                        The A-Shaped VTail is the exact same as a V-Shaped VTail, with one difference:
                        - The Yaw factors are reversed, because the rear motors are facing different directions

                        With V-Shaped VTails, the props make a V-Shape when spinning, but with
                        A-Shaped VTails, the props make an A-Shape when spinning.
                        - Rear thrust on a V-Shaped V-Tail Quad is outward
                        - Rear thrust on an A-Shaped V-Tail Quad is inward

                        Still functions the same as the V-Shaped VTail mixing below:
                        - Yaw control is entirely in the rear motors
                        - Roll is is entirely in the front motors
                    */
                    add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
                    add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
                    add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
                    success = true;
                    break;
                default:
                    // quad frame class does not support this frame type
                    break;
            }
            break;  // quad

        case MOTOR_FRAME_HEXA:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    add_motor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_3,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor(AP_MOTORS_MOT_4,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    add_motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
                    add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
                    add_motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
                    add_motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    add_motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
                    add_motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
                    success = true;
                    break;
                default:
                    // hexa frame class does not support this frame type
                    break;
            }
            break;

        // Tao Du
        // taodu@csail.mit.edu
        // Jul 5, 2018
        case MOTOR_FRAME_PENTA: {
            // for now we ignore the frame type.
            // rcout 1: right midfielder (CW)
            // rcout 2: right back. (CCW)
            // rcout 3: center forward. (CW)
            // rcout 4: left back. (CW)
            // rcout 5: left midfielder. (CCW)
            // if you are a soccer fan you will immediate understand this:
            /*               CF (rotor 3)
                              |
                              |
                              |
                 LMF <-----Pixhawk-----> RMF (rotor 1)
                             / \
                            /   \
                           /     \
               (rotor 4) LB       RB (rotor 2)
            */
            add_motor(AP_MOTORS_MOT_1,          72, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1);
            add_motor(AP_MOTORS_MOT_2,          144,    AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
            add_motor(AP_MOTORS_MOT_3,          0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
            add_motor(AP_MOTORS_MOT_4,          -144,   AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4);
            add_motor(AP_MOTORS_MOT_5,          -72,    AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
            success = true;
            break;
        }

        case MOTOR_FRAME_OCTA:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    add_motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    add_motor(AP_MOTORS_MOT_1,   22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_2, -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor(AP_MOTORS_MOT_3,   67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor(AP_MOTORS_MOT_4,  157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_5,  -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor(AP_MOTORS_MOT_6, -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor(AP_MOTORS_MOT_7,  -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor(AP_MOTORS_MOT_8,  112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_V:
                    add_motor_raw(AP_MOTORS_MOT_1,  1.0f,  0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor_raw(AP_MOTORS_MOT_2, -1.0f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    add_motor_raw(AP_MOTORS_MOT_3,  1.0f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor_raw(AP_MOTORS_MOT_4, -0.5f,  -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor_raw(AP_MOTORS_MOT_5,  1.0f,   1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor_raw(AP_MOTORS_MOT_6, -1.0f,  0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor_raw(AP_MOTORS_MOT_7, -1.0f,   1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor_raw(AP_MOTORS_MOT_8,  0.5f,  -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_H:
                    add_motor_raw(AP_MOTORS_MOT_1, -1.0f,    1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor_raw(AP_MOTORS_MOT_2,  1.0f,   -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor_raw(AP_MOTORS_MOT_3, -1.0f,  0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor_raw(AP_MOTORS_MOT_4, -1.0f,   -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor_raw(AP_MOTORS_MOT_5,  1.0f,    1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor_raw(AP_MOTORS_MOT_6,  1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor_raw(AP_MOTORS_MOT_7,  1.0f,  0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor_raw(AP_MOTORS_MOT_8, -1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    success = true;
                    break;
                default:
                    // octa frame class does not support this frame type
                    break;
            } // octa frame type
            break;

        case MOTOR_FRAME_OCTAQUAD:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    add_motor(AP_MOTORS_MOT_1,    0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
                    add_motor(AP_MOTORS_MOT_2,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor(AP_MOTORS_MOT_3,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
                    add_motor(AP_MOTORS_MOT_4,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    add_motor(AP_MOTORS_MOT_5,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor(AP_MOTORS_MOT_6,    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
                    add_motor(AP_MOTORS_MOT_7,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_8,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
                    add_motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    add_motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
                    add_motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_V:
                    add_motor(AP_MOTORS_MOT_1,   45,  0.7981f, 1);
                    add_motor(AP_MOTORS_MOT_2,  -45, -0.7981f, 7);
                    add_motor(AP_MOTORS_MOT_3, -135,  1.0000f, 5);
                    add_motor(AP_MOTORS_MOT_4,  135, -1.0000f, 3);
                    add_motor(AP_MOTORS_MOT_5,  -45,  0.7981f, 8);
                    add_motor(AP_MOTORS_MOT_6,   45, -0.7981f, 2);
                    add_motor(AP_MOTORS_MOT_7,  135,  1.0000f, 4);
                    add_motor(AP_MOTORS_MOT_8, -135, -1.0000f, 6);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_H:
                    // H frame set-up - same as X but motors spin in opposite directions
                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7);
                    add_motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    add_motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8);
                    add_motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
                    add_motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    success = true;
                    break;
                default:
                    // octaquad frame class does not support this frame type
                    break;
            }
            break;

        case MOTOR_FRAME_Y6:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_Y6B:
                    // Y6 motor definition with all top motors spinning clockwise, all bottom motors counter clockwise
                    add_motor_raw(AP_MOTORS_MOT_1, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor_raw(AP_MOTORS_MOT_2, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor_raw(AP_MOTORS_MOT_3,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    add_motor_raw(AP_MOTORS_MOT_4,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor_raw(AP_MOTORS_MOT_5,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor_raw(AP_MOTORS_MOT_6,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_Y6F:
                    // Y6 motor layout for FireFlyY6
                    add_motor_raw(AP_MOTORS_MOT_1,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    add_motor_raw(AP_MOTORS_MOT_2, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
                    add_motor_raw(AP_MOTORS_MOT_3,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
                    add_motor_raw(AP_MOTORS_MOT_4,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
                    add_motor_raw(AP_MOTORS_MOT_5, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
                    add_motor_raw(AP_MOTORS_MOT_6,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
                    success = true;
                    break;
                default:
                    add_motor_raw(AP_MOTORS_MOT_1, -1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor_raw(AP_MOTORS_MOT_2,  1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor_raw(AP_MOTORS_MOT_3,  1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor_raw(AP_MOTORS_MOT_4,  0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
                    add_motor_raw(AP_MOTORS_MOT_5, -1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor_raw(AP_MOTORS_MOT_6,  0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    success = true;
                    break;
            }
            break;

        default:
            // matrix doesn't support the configured class
            break;
        } // switch frame_class

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    _flags.initialised_ok = success;
}

// normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
void AP_MotorsMatrix::normalise_rpy_factors()
{
    float roll_fac = 0.0f;
    float pitch_fac = 0.0f;
    float yaw_fac = 0.0f;

    // find maximum roll, pitch and yaw factors
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (roll_fac < fabsf(_roll_factor[i])) {
                roll_fac = fabsf(_roll_factor[i]);
            }
            if (pitch_fac < fabsf(_pitch_factor[i])) {
                pitch_fac = fabsf(_pitch_factor[i]);
            }
            if (yaw_fac < fabsf(_yaw_factor[i])) {
                yaw_fac = fabsf(_yaw_factor[i]);
            }
        }
    }

    // scale factors back to -0.5 to +0.5 for each axis
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (!is_zero(roll_fac)) {
                _roll_factor[i] = 0.5f*_roll_factor[i]/roll_fac;
            }
            if (!is_zero(pitch_fac)) {
                _pitch_factor[i] = 0.5f*_pitch_factor[i]/pitch_fac;
            }
            if (!is_zero(yaw_fac)) {
                _yaw_factor[i] = 0.5f*_yaw_factor[i]/yaw_fac;
            }
        }
    }
}


/*
  call vehicle supplied thrust compensation if set. This allows
  vehicle code to compensate for vehicle specific motor arrangements
  such as tiltrotors or tiltwings
*/
void AP_MotorsMatrix::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
    }
}
