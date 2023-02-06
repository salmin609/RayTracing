#pragma once


class RayInfo
{
public:
	vec3 origin;
	vec3 direction;
	float time;

	RayInfo(vec3 pos, vec3 dir, float t) : origin(pos), direction(dir), time(t)
	{
		
	}
	~RayInfo()
	{
		
	}

	vec3 At(float t)
	{
		return origin + direction * t;
	}

private:
};