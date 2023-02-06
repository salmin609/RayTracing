#include <iostream>
#include "geom.h"

float* Pntr(mat3& m) { return &(m[0][0]); }
float* Pntr(mat4& m) { return &(m[0][0]); }

mat4 Identity() {return mat4(1.0); }

const vec3 Xaxis() { return vec3(1,0,0); }
const vec3 Yaxis() { return vec3(0,1,0); }
const vec3 Zaxis() { return vec3(0,0,1); }

float length(vec3 vec)
{
    return sqrt(lengthSquared(vec));
}


vec3 operator/ (const vec3& v, float f) { return (1.0f/f) * v; }
vec3 operator/ (const vec3& v, double f) { return float(1.0f/f) * v; };
vec4 operator/ (const vec4& v, float f) { return (1.0f/f) * v; }
vec4 operator/ (const vec4& v, double f) { return float(1.0f/f) * v; }

float lengthSquared(vec3 vec)
{
    return vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
}

vec3 UnitDirection(const vec3& v)
{
    return v / static_cast<float>(v.length());
}

//vec3 random_in_unit_sphere() {
//    while (true) {
//        auto p = vec3::random(-1, 1);
//        if (p.length_squared() >= 1) continue;
//        return p;
//    }
//}

vec3 transformVector(const quat& q, const vec3 v) { return q*v; } //?? Really?
quat FromTwoVectors(const vec3& a, const vec3& b) {
    vec3 axis = cross(a,b);
    float l = length(axis);
    if (std::abs(l) < 0.0001) return quat(1,0,0,0);
    axis /= l;
    float ang = acos(dot(a,b)/(length(a)*length(b)));
    float c = cos(ang/2.0);
    float s = sin(ang/2.0);
    return quat(c, s*axis);
}

mat4 translate(const vec3& v) { return glm::translate(glm::mat4(1.0f), v); }

// The standard glFrustum perspective projection matrix.
mat4 frustum(float const& left,    float const& right,
                 float const& bottom,  float const& top,
                 float const& nearVal, float const& farVal)
{
    mat4 R(0.0f);
    R[0][0]= (2.0*nearVal)         / (right-left);
    R[1][1]= (2.0*nearVal)         / (top-bottom);
    R[2][0]= (right+left)          / (right-left);
    R[2][1]= (top+bottom)          / (top-bottom);
    R[2][2]= -(farVal+nearVal)     / (farVal-nearVal);
    R[2][3]= -1.0;
    R[3][2]= -(2.0*farVal*nearVal) / (farVal-nearVal);

    return R;
}

void Print(const std::string& s, const float& m) { std::cout << s << ": " << m << '\n'; }
void Print(const std::string& s, const double& m) { std::cout << s << ": " << m << '\n'; }

void Print(const std::string& s, const vec2& m) { std::cout << s << ": " << glm::to_string(m) << '\n'; }
void Print(const std::string& s, const vec3& m) { std::cout << s << ": " << glm::to_string(m) << '\n'; }
void Print(const std::string& s, const vec4& m) { std::cout << s << ": " << glm::to_string(m) << '\n'; }
void Print(const std::string& s, const mat4& m) { std::cout << s << ": " << glm::to_string(m) << '\n'; }
