#pragma once
#include <string>
#include <iostream>

#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/ext.hpp>          // For printing GLM objects with to_string
#include <glm/gtx/quaternion.hpp>

using glm::vec2;
using glm::vec3;
using glm::ivec3;
using glm::vec4;
using glm::mat3;
using glm::mat4;
using glm::quat;

using glm::dot;
using glm::toMat4;
using glm::normalize;
using glm::cross;
using glm::inverse;
using glm::scale;
using glm::rotate;
using glm::conjugate;
using glm::length;
using glm::toMat4;
using glm::angleAxis;

typedef glm::vec3 Color;

float* Pntr(mat3& m);
float* Pntr(mat4& m);

vec3 operator/ (const vec3& v, float f);
vec3 operator/ (const vec3& v, double f);
vec4 operator/ (const vec4& v, float f);
vec4 operator/ (const vec4& v, double f);

mat4 Identity();

const vec3 Xaxis();
const vec3 Yaxis();
const vec3 Zaxis();
float length(vec3 vec);
float lengthSquared(vec3 vec);
vec3 UnitDirection(const vec3& v);
//vec3 random_in_unit_sphere();

vec3 transformVector(const quat& q, const vec3 v);
quat FromTwoVectors(const vec3& a, const vec3& b);

mat4 translate(const vec3& v);

mat4 frustum(float const& left,    float const& right,
             float const& bottom,  float const& top, 
             float const& nearVal, float const& farVal);

void Print(const std::string&, const float&);
void Print(const std::string&, const double&);

void Print(const std::string&, const vec2&);
void Print(const std::string&, const vec3&);
void Print(const std::string&, const vec4&);
void Print(const std::string&, const mat4&);
