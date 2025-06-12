#include "hexacopter_sysid.h"
#include <cstdio>
#include <cstdarg>
#include <cmath>
#include <cstring>

// Stub implementations for standalone build
#ifndef ARDUPILOT_BUILD
static uint32_t millis_stub() { return 0; }
static uint64_t micros64_stub() { return 0; }
#define AP_HAL_millis() millis_stub()
#define AP_HAL_micros64() micros64_stub()

// Console stub
namespace {
    struct {
        struct {
            void printf(const char* fmt, ...) {
                va_list args;
                va_start(args, fmt);
                vprintf(fmt, args);
                va_end(args);
            }
        } console_stub;
        void* console = &console_stub;
    } hal;
}
#else
extern const AP_HAL::HAL& hal;
#define AP_HAL_millis() AP_HAL::millis()
#define AP_HAL_micros64() AP_HAL::micros64()
#endif

// Motor angles for X configuration hexacopter (degrees)
const float HexacopterSysID::_motor_angles[6] = {
    60.0f, 120.0f, 180.0f, 240.0f, 300.0f, 0.0f
};

const float HexacopterSysID::_motor_arm_length = 0.25f;

// Parameter table - simplified for standalone build
const AP_Param::GroupInfo HexacopterSysID::var_info[] = {
#ifdef ARDUPILOT_BUILD
    AP_GROUPINFO("ENABLE", 0, HexacopterSysID, _enable, 0),
    AP_GROUPINFO("THR_MIN", 1, HexacopterSysID, _test_throttle_min, 0.2f),
    AP_GROUPINFO("THR_MAX", 2, HexacopterSysID, _test_throttle_max, 0.5f),
    AP_GROUPINFO("DURATION", 3, HexacopterSysID, _test_duration, 20.0f),
    AP_GROUPINFO("TEST_AXIS", 4, HexacopterSysID, _test_axis, 0),
    AP_GROUPEND
#else
    {}  // Empty for standalone
#endif
};

HexacopterSysID::HexacopterSysID() :
    _running(false),
    _start_time_ms(0),
    _test_sequence(TEST_NONE),
    _sample_count(0),
    _fitness_score(0.0f),
    _forgetting_factor(0.98f)
{
    AP_Param::setup_object_defaults(this, var_info);
    
    // Initialize matrices
    _inertia_matrix.identity();
    _P_matrix.identity();
    
    // Initialize moment arms
    for (uint8_t i = 0; i < 6; i++) {
        float angle_rad = radians(_motor_angles[i]);
        _moment_arms[i].x = _motor_arm_length * cosf(angle_rad);
        _moment_arms[i].y = _motor_arm_length * sinf(angle_rad);
        _moment_arms[i].z = 0.0f;
    }
    
    // Default values
    _enable = 0;
    _test_throttle_min = 0.2f;
    _test_throttle_max = 0.5f;
    _test_duration = 20.0f;
    _test_axis = 0;
}

void HexacopterSysID::init()
{
    _running = false;
    _sample_count = 0;
    
    // Default inertia values
    _inertia_matrix.a.x = 0.02f;
    _inertia_matrix.b.y = 0.02f;
    _inertia_matrix.c.z = 0.04f;
    
    // Default damping
    _damping_coeff = Vector3f(0.01f, 0.01f, 0.005f);
    
    // Default coefficients
    _thrust_coeff = 1.5e-5f;
    _torque_coeff = 3.0e-7f;
}

void HexacopterSysID::start_identification()
{
    if (!_enable) {
        return;
    }
    
    _running = true;
    _start_time_ms = AP_HAL_millis();
    _sample_count = 0;
    
#ifdef ARDUPILOT_BUILD
    hal.console->printf("SysID: Starting identification sequence %d\n", _test_sequence);
#else
    printf("SysID: Starting identification sequence %d\n", _test_sequence);
#endif
}

void HexacopterSysID::stop_identification()
{
    if (!_running) {
        return;
    }
    
    _running = false;
    
    if (_sample_count > 10) {
        perform_identification();
    }
    
#ifdef ARDUPILOT_BUILD
    hal.console->printf("SysID: Stopped. Collected %u samples\n", _sample_count);
#else
    printf("SysID: Stopped. Collected %u samples\n", _sample_count);
#endif
}

void HexacopterSysID::update(float dt)
{
    if (!_enable || !_running) {
        return;
    }
    
    uint32_t elapsed_ms = AP_HAL_millis() - _start_time_ms;
    if (elapsed_ms > _test_duration * 1000) {
        stop_identification();
        return;
    }
    
    float roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd;
    generate_test_signal(roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd);
    
    Vector3f gyro = Vector3f(0, 0, 0);
    Vector3f motor_cmd = Vector3f(roll_cmd, pitch_cmd, yaw_cmd);
    
    collect_sample(gyro, motor_cmd, throttle_cmd);
    
    if (_sample_count > 1) {
        Vector3f angular_accel = (_samples[_sample_count-1].angular_rate - 
                                 _samples[_sample_count-2].angular_rate) / dt;
        rls_update(motor_cmd, angular_accel);
    }
    
    if (_sample_count % 10 == 0) {
        log_identification_data();
    }
}

void HexacopterSysID::collect_sample(const Vector3f& gyro, const Vector3f& motor_cmd, float throttle)
{
    if (_sample_count >= MAX_SAMPLES) {
        return;
    }
    
    Sample& sample = _samples[_sample_count];
    sample.timestamp_ms = AP_HAL_millis();
    sample.angular_rate = gyro;
    sample.motor_commands = motor_cmd;
    sample.throttle = throttle;
    
    if (_sample_count > 0) {
        float dt = (sample.timestamp_ms - _samples[_sample_count-1].timestamp_ms) * 0.001f;
        if (dt > 0) {
            sample.angular_accel = (gyro - _samples[_sample_count-1].angular_rate) / dt;
        }
    }
    
    _sample_count++;
}

void HexacopterSysID::generate_test_signal(float& roll_out, float& pitch_out, float& yaw_out, float& throttle_out)
{
    float elapsed_s = (AP_HAL_millis() - _start_time_ms) * 0.001f;
    float magnitude = 0.1f;
    
    roll_out = 0;
    pitch_out = 0;
    yaw_out = 0;
    throttle_out = (_test_throttle_min + _test_throttle_max) / 2.0f;
    
    float test_signal = 0;
    switch (_test_sequence) {
        case TEST_STEP_RESPONSE:
            test_signal = generate_step(elapsed_s, magnitude);
            break;
        case TEST_FREQUENCY_SWEEP:
            test_signal = generate_sweep(elapsed_s, 0.1f, 10.0f, magnitude);
            break;
        case TEST_DOUBLET:
            test_signal = generate_doublet(elapsed_s, 1.0f, magnitude);
            break;
        case TEST_PRBS:
            test_signal = generate_prbs(elapsed_s, 0.5f, magnitude);
            break;
        default:
            break;
    }
    
    switch (_test_axis) {
        case 0:
            roll_out = test_signal;
            break;
        case 1:
            pitch_out = test_signal;
            break;
        case 2:
            yaw_out = test_signal;
            break;
    }
}

float HexacopterSysID::generate_step(float time, float magnitude)
{
    return (time > 2.0f) ? magnitude : 0;
}

float HexacopterSysID::generate_sweep(float time, float f_start, float f_end, float magnitude)
{
    float duration = _test_duration;
    float f = f_start + (f_end - f_start) * (time / duration);
    return magnitude * sinf(2.0f * M_PI * f * time);
}

float HexacopterSysID::generate_doublet(float time, float width, float magnitude)
{
    if (time < 2.0f) return 0;
    if (time < 2.0f + width) return magnitude;
    if (time < 2.0f + 2*width) return -magnitude;
    return 0;
}

float HexacopterSysID::generate_prbs(float time, float bit_time, float magnitude)
{
    uint32_t bit_index = (uint32_t)(time / bit_time);
    uint32_t lfsr = bit_index;
    
    for (uint32_t i = 0; i < bit_index; i++) {
        uint32_t bit = ((lfsr >> 0) ^ (lfsr >> 1)) & 1;
        lfsr = (lfsr >> 1) | (bit << 14);
    }
    
    return (lfsr & 1) ? magnitude : -magnitude;
}

void HexacopterSysID::rls_update(const Vector3f& input, const Vector3f& output)
{
    // Simplified RLS update
    Vector3f error = output - (_inertia_matrix * input);
    
    float lambda = _forgetting_factor;
    float input_mag = input.length();
    
    if (input_mag > 0.0001f) {
        // Simplified covariance update
        float scale = 1.0f / (lambda + input_mag);
        
        // Update inertia estimate (simplified)
        _inertia_matrix.a.x += scale * error.x * input.x;
        _inertia_matrix.b.y += scale * error.y * input.y;
        _inertia_matrix.c.z += scale * error.z * input.z;
        
        // Keep positive definite
        _inertia_matrix.a.x = fmaxf(_inertia_matrix.a.x, 0.001f);
        _inertia_matrix.b.y = fmaxf(_inertia_matrix.b.y, 0.001f);
        _inertia_matrix.c.z = fmaxf(_inertia_matrix.c.z, 0.001f);
    }
}

void HexacopterSysID::perform_identification()
{
#ifdef ARDUPILOT_BUILD
    hal.console->printf("SysID: Performing batch identification with %u samples\n", _sample_count);
#else
    printf("SysID: Performing batch identification with %u samples\n", _sample_count);
#endif
    
    float total_thrust = 0;
    uint32_t steady_samples = 0;
    
    for (uint16_t i = 10; i < _sample_count; i++) {
        if (fabsf(_samples[i].angular_accel.length()) < 0.1f) {
            total_thrust += _samples[i].throttle;
            steady_samples++;
        }
    }
    
    if (steady_samples > 0) {
        _thrust_coeff = total_thrust / (steady_samples * 6.0f);
    }
    
#ifdef ARDUPILOT_BUILD
    hal.console->printf("SysID: Identified thrust coefficient: %f\n", _thrust_coeff);
    hal.console->printf("SysID: Inertia matrix diagonal: [%f, %f, %f]\n", 
                       _inertia_matrix.a.x, _inertia_matrix.b.y, _inertia_matrix.c.z);
#else
    printf("SysID: Identified thrust coefficient: %f\n", _thrust_coeff);
    printf("SysID: Inertia matrix diagonal: [%f, %f, %f]\n", 
           _inertia_matrix.a.x, _inertia_matrix.b.y, _inertia_matrix.c.z);
#endif
}

void HexacopterSysID::log_identification_data()
{
#if HAL_LOGGING_ENABLED && defined(ARDUPILOT_BUILD)
    if (_sample_count == 0) return;
    
    const Sample& sample = _samples[_sample_count-1];
    
    AP::logger().WriteStreaming("SYID",
        "TimeUS,Seq,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,CmdR,CmdP,CmdY,Thr,Ixx,Iyy,Izz",
        "QBffffffffffff",
        AP_HAL_micros64(),
        (uint8_t)_test_sequence,
        sample.angular_rate.x,
        sample.angular_rate.y,
        sample.angular_rate.z,
        sample.angular_accel.x,
        sample.angular_accel.y,
        sample.angular_accel.z,
        sample.motor_commands.x,
        sample.motor_commands.y,
        sample.motor_commands.z,
        sample.throttle,
        _inertia_matrix.a.x,
        _inertia_matrix.b.y,
        _inertia_matrix.c.z
    );
#endif
}