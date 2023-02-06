#pragma once
#include "geom.h"
#include "hitRecord.hpp"
#include "RayInfo.hpp"

struct Slab
{
	vec3 n;
	float d0;
	float d1;
};

class Interval
{
public:
	Interval();
	~Interval();

	bool Intersect(RayInfo* ray, hitRecord* hitInfo, Slab slab);
	float t0, t1;
	vec3 n0, n1;
};

inline Interval::Interval()
{
	t0 = 0;
	t1 = 1000.f;
}

inline Interval::~Interval()
{
}

inline bool Interval::Intersect(RayInfo* ray, hitRecord* hitInfo, Slab slab)
{
	float d0 = slab.d0;
	float d1 = slab.d1;
	vec3 normal = slab.n;
	float temp = dot(normal, ray->direction);
	if (temp < 0.f - glm::epsilon<float>() ||
		temp > 0.f + glm::epsilon<float>())
	{
		t0 = -(d0 + dot(normal, ray->origin)) / (dot(normal, ray->direction));
		t1 = -(d1 + dot(normal, ray->origin)) / (dot(normal, ray->direction));

		if (t0 > t1)
			std::swap(t0, t1);

		//if (t0 > 0.f)
		//	hitInfo->t = t0;
		//else if (t1 > 0.f)
		//	hitInfo->t = t1;
		//else
		//	return false;


		//hitInfo->p = ray->At(hitInfo->t);

		if (slab.d0 > slab.d1)
		{
			n0 = -normal;
			n1 = normal;
		}
		else
		{
			n0 = normal;
			n1 = -normal;
		}

		return true;
	}
	float s0 = dot(normal, ray->origin) + d0;
	float s1 = dot(normal, ray->origin) + d1;

	if (s0 < 0 && s1 < 0 || s0 > 0 && s1 > 0)
	{
		//ray is btw planes.
		t0 = 1.f;
		t1 = 0.f;
		return false;
	}
	t0 = 0.f;
	t1 = 1000.f;
	return false;
}
