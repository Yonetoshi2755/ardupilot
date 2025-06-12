#ifndef AP_MATH_COMPAT_H
#define AP_MATH_COMPAT_H

/*
 * Compatibility header for standalone compilation
 * Provides minimal definitions when not building with ArduPilot
 */

#include <cmath>
#include <cstring>
#include <cstdint>

#ifndef GRAVITY_MSS
#define GRAVITY_MSS 9.80665f
#endif

// Basic vector/matrix classes for standalone build
#ifndef AP_MATH_H

// Forward declarations
class Vector3f;
Vector3f operator*(float s, const Vector3f& v);
Vector3f operator-(const Vector3f& v);

// Vector3f class
class Vector3f {
public:
    float x, y, z;
    
    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    void zero() { x = y = z = 0; }
    float length() const { return sqrtf(x*x + y*y + z*z); }
    void normalize() {
        float len = length();
        if (len > 0) {
            x /= len; y /= len; z /= len;
        }
    }
    
    // Operators
    Vector3f operator+(const Vector3f& v) const {
        return Vector3f(x + v.x, y + v.y, z + v.z);
    }
    Vector3f operator-(const Vector3f& v) const {
        return Vector3f(x - v.x, y - v.y, z - v.z);
    }
    Vector3f operator*(float s) const {
        return Vector3f(x * s, y * s, z * s);
    }
    Vector3f operator/(float s) const {
        return Vector3f(x / s, y / s, z / s);
    }
    Vector3f& operator+=(const Vector3f& v) {
        x += v.x; y += v.y; z += v.z; return *this;
    }
    Vector3f& operator-=(const Vector3f& v) {
        x -= v.x; y -= v.y; z -= v.z; return *this;
    }
    Vector3f& operator*=(float s) {
        x *= s; y *= s; z *= s; return *this;
    }
    Vector3f& operator/=(float s) {
        x /= s; y /= s; z /= s; return *this;
    }
    float dot(const Vector3f& v) const {
        return x * v.x + y * v.y + z * v.z;
    }
    Vector3f operator%(const Vector3f& v) const {
        return Vector3f(y * v.z - z * v.y,
                       z * v.x - x * v.z,
                       x * v.y - y * v.x);
    }
    
    // Element-wise multiplication
    Vector3f operator*(const Vector3f& v) const {
        return Vector3f(x * v.x, y * v.y, z * v.z);
    }
    float& operator[](size_t i) {
        return (&x)[i];
    }
    const float& operator[](size_t i) const {
        return (&x)[i];
    }
};

// Global operators for Vector3f
inline Vector3f operator*(float s, const Vector3f& v) {
    return v * s;
}

inline Vector3f operator-(const Vector3f& v) {
    return Vector3f(-v.x, -v.y, -v.z);
}

// Matrix3f class
class Matrix3f {
public:
    Vector3f a, b, c;
    
    Matrix3f() {}
    Matrix3f(const Vector3f& a_, const Vector3f& b_, const Vector3f& c_) :
        a(a_), b(b_), c(c_) {}
    
    void identity() {
        a = Vector3f(1, 0, 0);
        b = Vector3f(0, 1, 0);
        c = Vector3f(0, 0, 1);
    }
    
    Vector3f operator*(const Vector3f& v) const {
        return Vector3f(a.x*v.x + a.y*v.y + a.z*v.z,
                       b.x*v.x + b.y*v.y + b.z*v.z,
                       c.x*v.x + c.y*v.y + c.z*v.z);
    }
    
    Matrix3f operator*(const Matrix3f& m) const {
        Matrix3f result;
        result.a.x = a.x*m.a.x + a.y*m.b.x + a.z*m.c.x;
        result.a.y = a.x*m.a.y + a.y*m.b.y + a.z*m.c.y;
        result.a.z = a.x*m.a.z + a.y*m.b.z + a.z*m.c.z;
        result.b.x = b.x*m.a.x + b.y*m.b.x + b.z*m.c.x;
        result.b.y = b.x*m.a.y + b.y*m.b.y + b.z*m.c.y;
        result.b.z = b.x*m.a.z + b.y*m.b.z + b.z*m.c.z;
        result.c.x = c.x*m.a.x + c.y*m.b.x + c.z*m.c.x;
        result.c.y = c.x*m.a.y + c.y*m.b.y + c.z*m.c.y;
        result.c.z = c.x*m.a.z + c.y*m.b.z + c.z*m.c.z;
        return result;
    }
    
    Matrix3f operator*(float s) const {
        return Matrix3f(a * s, b * s, c * s);
    }
    
    Matrix3f transposed() const {
        return Matrix3f(Vector3f(a.x, b.x, c.x),
                       Vector3f(a.y, b.y, c.y),
                       Vector3f(a.z, b.z, c.z));
    }
    
    float operator()(uint8_t i, uint8_t j) const {
        return ((const Vector3f*)&a)[i][j];
    }
    
    // Matrix inverse (3x3 only)
    Matrix3f inverse() const {
        float det = a.x*(b.y*c.z - b.z*c.y) - 
                   a.y*(b.x*c.z - b.z*c.x) + 
                   a.z*(b.x*c.y - b.y*c.x);
        
        if (fabsf(det) < 1e-6f) {
            // Return identity if not invertible
            Matrix3f result;
            result.identity();
            return result;
        }
        
        float invDet = 1.0f / det;
        Matrix3f result;
        
        result.a.x = (b.y*c.z - b.z*c.y) * invDet;
        result.a.y = (a.z*c.y - a.y*c.z) * invDet;
        result.a.z = (a.y*b.z - a.z*b.y) * invDet;
        
        result.b.x = (b.z*c.x - b.x*c.z) * invDet;
        result.b.y = (a.x*c.z - a.z*c.x) * invDet;
        result.b.z = (a.z*b.x - a.x*b.z) * invDet;
        
        result.c.x = (b.x*c.y - b.y*c.x) * invDet;
        result.c.y = (a.y*c.x - a.x*c.y) * invDet;
        result.c.z = (a.x*b.y - a.y*b.x) * invDet;
        
        return result;
    }
};

// Quaternion class
class Quaternion {
public:
    float q1, q2, q3, q4;
    
    Quaternion() : q1(1), q2(0), q3(0), q4(0) {}
    Quaternion(float w, float x, float y, float z) : q1(w), q2(x), q3(y), q4(z) {}
    
    void normalize() {
        float norm = sqrtf(q1*q1 + q2*q2 + q3*q3 + q4*q4);
        if (norm > 0) {
            q1 /= norm; q2 /= norm; q3 /= norm; q4 /= norm;
        }
    }
    
    Quaternion inverse() const {
        return Quaternion(q1, -q2, -q3, -q4);
    }
    
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            q1*q.q1 - q2*q.q2 - q3*q.q3 - q4*q.q4,
            q1*q.q2 + q2*q.q1 + q3*q.q4 - q4*q.q3,
            q1*q.q3 - q2*q.q4 + q3*q.q1 + q4*q.q2,
            q1*q.q4 + q2*q.q3 - q3*q.q2 + q4*q.q1
        );
    }
    
    Quaternion operator*(float s) const {
        return Quaternion(q1*s, q2*s, q3*s, q4*s);
    }
    
    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(q1+q.q1, q2+q.q2, q3+q.q3, q4+q.q4);
    }
    
    Quaternion operator-() const {
        return Quaternion(-q1, -q2, -q3, -q4);
    }
    
    void rotation_matrix(Matrix3f& m) const {
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;
        
        m.a.x = q1q1 + q2q2 - q3q3 - q4q4;
        m.a.y = 2*(q2q3 - q1q4);
        m.a.z = 2*(q2q4 + q1q3);
        m.b.x = 2*(q2q3 + q1q4);
        m.b.y = q1q1 - q2q2 + q3q3 - q4q4;
        m.b.z = 2*(q3q4 - q1q2);
        m.c.x = 2*(q2q4 - q1q3);
        m.c.y = 2*(q3q4 + q1q2);
        m.c.z = q1q1 - q2q2 - q3q3 + q4q4;
    }
    
    void from_rotation_matrix(const Matrix3f& m) {
        float trace = m.a.x + m.b.y + m.c.z;
        if (trace > 0) {
            float s = 0.5f / sqrtf(trace + 1.0f);
            q1 = 0.25f / s;
            q2 = (m.c.y - m.b.z) * s;
            q3 = (m.a.z - m.c.x) * s;
            q4 = (m.b.x - m.a.y) * s;
        } else {
            if (m.a.x > m.b.y && m.a.x > m.c.z) {
                float s = 2.0f * sqrtf(1.0f + m.a.x - m.b.y - m.c.z);
                q1 = (m.c.y - m.b.z) / s;
                q2 = 0.25f * s;
                q3 = (m.a.y + m.b.x) / s;
                q4 = (m.a.z + m.c.x) / s;
            } else if (m.b.y > m.c.z) {
                float s = 2.0f * sqrtf(1.0f + m.b.y - m.a.x - m.c.z);
                q1 = (m.a.z - m.c.x) / s;
                q2 = (m.a.y + m.b.x) / s;
                q3 = 0.25f * s;
                q4 = (m.b.z + m.c.y) / s;
            } else {
                float s = 2.0f * sqrtf(1.0f + m.c.z - m.a.x - m.b.y);
                q1 = (m.b.x - m.a.y) / s;
                q2 = (m.a.z + m.c.x) / s;
                q3 = (m.b.z + m.c.y) / s;
                q4 = 0.25f * s;
            }
        }
        normalize();
    }
    
    void to_euler(float& roll, float& pitch, float& yaw) const {
        roll = atan2f(2*(q1*q2 + q3*q4), 1 - 2*(q2*q2 + q3*q3));
        pitch = asinf(2*(q1*q3 - q4*q2));
        yaw = atan2f(2*(q1*q4 + q2*q3), 1 - 2*(q3*q3 + q4*q4));
    }
};

// Template vector class
template<typename T, uint8_t N>
class VectorN {
public:
    T v[N];
    
    VectorN() { memset(v, 0, sizeof(v)); }
    
    T& operator[](uint8_t i) { return v[i]; }
    const T& operator[](uint8_t i) const { return v[i]; }
};

// Template matrix class
template<typename T, uint8_t N>
class MatrixN {
public:
    T v[N][N];
    
    MatrixN() { memset(v, 0, sizeof(v)); }
    
    T* operator[](uint8_t i) { return v[i]; }
    const T* operator[](uint8_t i) const { return v[i]; }
};

// Matrix operations
typedef MatrixN<float, 24> Matrix24;
typedef MatrixN<float, 4> Matrix4;

// Utility functions
inline float radians(float deg) { return deg * M_PI / 180.0f; }
inline float degrees(float rad) { return rad * 180.0f / M_PI; }
inline float constrain_float(float val, float min, float max) {
    return (val < min) ? min : ((val > max) ? max : val);
}
inline float norm(float x, float y) { return sqrtf(x*x + y*y); }

// 3x3 matrix inversion
inline bool inv3x3(float m[3][3], float inv[3][3]) {
    float det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    
    if (fabsf(det) < 1e-6f) return false;
    
    float invdet = 1.0f / det;
    
    inv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
    inv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
    inv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
    inv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
    inv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;
    
    return true;
}

#endif // AP_MATH_H


// Logger compatibility
#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 0
#endif

// Param compatibility
#ifndef AP_PARAM_H
#define AP_GROUPINFO(name, idx, clazz, var, def)
#define AP_GROUPEND static const AP_Param::GroupInfo var_info_end = {}
class AP_Param {
public:
    struct GroupInfo {};
    static void setup_object_defaults(void*, const GroupInfo*) {}
};
class AP_Int8 { public: AP_Int8() {} operator int8_t() const { return 0; } };
class AP_Float { public: AP_Float() {} operator float() const { return 0; } };
#endif

// Motors compatibility
#ifndef AP_MOTORS_H
class AP_Motors {};
#endif

// Console output
#ifndef AP_HAL_H
struct ConsoleStub {
    void printf(const char*, ...) {}
};
namespace AP_HAL {
    struct HAL {
        ConsoleStub* console;
    };
    inline uint32_t millis() { return 0; }
    inline uint64_t micros64() { return 0; }
}
#endif

// Logger stub
#if !HAL_LOGGING_ENABLED
namespace AP {
    struct LoggerStub {
        void WriteStreaming(const char*, const char*, const char*, ...) {}
    };
    inline LoggerStub& logger() { static LoggerStub l; return l; }
}
#endif

#endif // AP_MATH_COMPAT_H