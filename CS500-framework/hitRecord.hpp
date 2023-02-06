#pragma once
#include "RayInfo.hpp"
class Obj;
class material;

class hitRecord
{
public:
	vec3 p = vec3(0.f, 0.f, 0.f);
	vec3 n = vec3(0.f, 0.f, 0.f);
	float t = 1000.f;
	vec3 matKd = vec3(0.f, 0.f, 0.f);
	vec3 matKs = vec3(0.f, 0.f, 0.f);
	float distance() const { return t; }
	bool isLight = false;
	bool hit = false;
	Obj* obj;
	float u = -1.f, v = -1.f;

	bool operator!=(hitRecord rec)
	{
		if (t > rec.t || t < rec.t)
			return true;

		if (p != rec.p || n != rec.n || matKd != rec.matKd)
			return true;

		return false;
	}
	
};