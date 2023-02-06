/*
 * Sangmin Kim
 * 2022/03/28
 * Raytracing Assignment
 */

#include <fstream>
#include <vector>

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

#include "Intervals.hpp"
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);

MeshData* SphMesh()
{
    MeshData* meshdata = new MeshData();
    unsigned int n = 20;
    float d = 2.0f*PI/float(n*2);
    for (unsigned int i=0;  i<=n*2;  i++) {
        float s = i*2.0f*PI/float(n*2);
        for (unsigned int j=0;  j<=n;  j++) {
            float t = j*PI/float(n);
            float x = cos(s)*sin(t);
            float y = sin(s)*sin(t);
            float z = cos(t);
            meshdata->vertices.push_back(VertexData(vec3(x,y,z),
                                                    vec3(x,y,z),
                                                    vec2(s/(2*PI), t/PI),
                                                    vec3(sin(s), cos(s), 0.0)));
            if (i>0 && j>0) {
                meshdata->triangles.push_back(ivec3((i-1)*(n+1) + (j-1), 
                                                      (i-1)*(n+1) + (j  ), 
                                                      (i  )*(n+1) + (j  )));
                meshdata->triangles.push_back(ivec3((i-1)*(n+1) + (j-1),
                                                      (i  )*(n+1) + (j  ),
                                                      (i  )*(n+1) + (j-1))); } } }
    return meshdata;
}

MeshData* BoxMesh()
{
    mat4 face[6] = {
        Identity(),
        rotate(180.0f*Radians, vec3(1.0f, 0.0f, 0.0f)),
        rotate( 90.0f*Radians, vec3(1.0f, 0.0f, 0.0f)),
        rotate(-90.0f*Radians, vec3(1.0f, 0.0f, 0.0f)),
        rotate( 90.0f*Radians, vec3(0.0f, 1.0f, 0.0f)),
        rotate(-90.0f*Radians, vec3(0.0f, 1.0f, 0.0f))};
       
    mat4 half = translate(vec3(0.5f, 0.5f, 0.5f))*scale(vec3(0.5f, 0.5f, 0.5f));
    MeshData* meshdata = new MeshData();
    for (unsigned int f=0;  f<6;  f++) {
        mat4 m4 = half*face[f];
        mat3 m3 = mat3(m4); // Extracts 3x3 from a 4x4
        for (unsigned int i=0;  i<2;  i++) {
            for (unsigned int j=0;  j<2;  j++) {
              vec4 p = m4*vec4(float(2*i)-1.0f, float(2*j)-1.0f, 1.0f, 1.0f);
              vec3 tnrm = m3*vec3(0.0f, 0.0f, 1.0f);
              vec3 ttan = m3*vec3(1.0, 0.0, 0.0);
              meshdata->vertices.push_back(VertexData(vec3(p[0], p[1], p[2]),
                                                      vec3(tnrm[0], tnrm[1], tnrm[2]),
                                                      vec2(float(i), float(j)),
                                                      vec3(ttan[0], ttan[1], ttan[2])));
              meshdata->triangles.push_back(ivec3(4*f+0, 4*f+1, 4*f+3));
              meshdata->triangles.push_back(ivec3(4*f+0, 4*f+3, 4*f+2)); } } }
    return meshdata;
}

MeshData* CylMesh()
{
    MeshData* meshdata = new MeshData();
    unsigned int n = 20;
    float d = 2.0f*PI/float(n*2);
    for (unsigned int i=0;  i<=n;  i++) {
        float s = i*2.0f*PI/float(n);
        float x = cos(s);
        float y = sin(s);
        
        meshdata->vertices.push_back(VertexData(vec3(x, y, 0.0f),
                                                vec3(x, y, 0.0f),
                                                vec2(s/(2*PI), 0.0f),
                                                vec3(-sin(s), cos(s), 0.0f)));

        meshdata->vertices.push_back(VertexData(vec3(x, y, 1.0f),
                                                vec3(x, y, 0.0f),
                                                vec2(s/(2*PI), 0.0f),
                                                vec3(-sin(s), cos(s), 0.0f)));

        if (i>0) {
            meshdata->triangles.push_back(ivec3((i-1)*2+1, (i-1)*2, (i  )*2));
            meshdata->triangles.push_back(ivec3((i-1)*2+1, (i  )*2, (i  )*2+1)); } }
    return meshdata;
}

void GetSphereUV(vec3 p, float& u, float& v)
{
	auto theta = acos(-p.y);
	auto phi = atan2(-p.z, p.x) + PI;

	u = phi / (2 * PI);
	v = theta / PI;
}
bool SphereCollisionCheck(vec3 center, float radius, RayInfo r, hitRecord& hitInfo, float tMin, float tMax, bool isTextured = false)
{
	float a = dot(r.direction, r.direction);
	float b = 2 * dot(r.origin - center, r.direction);
	float c = dot(r.origin - center, r.origin - center) - (radius * radius);

	float disc = (b * b) - (4 * a * c);

	if (disc < 0)
		return false;

	float t0 = (-b - sqrt(disc)) / (2 * a);
	float t1 = (-b + sqrt(disc)) / (2 * a);

	if (t0 < 0.0001f && t1 < 0.0001f)
		return false;

	if (t0 > t1)
		std::swap(t0, t1);

	

	if(t0 > 0.0001f && hitInfo.t > t0)
	{
		hitInfo.t = t0;
		hitInfo.p = r.At(hitInfo.t);
		hitInfo.n = hitInfo.p - center;
		hitInfo.n = normalize(hitInfo.n);

		if(isTextured)
			GetSphereUV(hitInfo.n, hitInfo.u, hitInfo.v);

		return true;
	}
	if(t1 > 0.0001f && hitInfo.t > t1)
	{
		hitInfo.t = t1;
		hitInfo.p = r.At(hitInfo.t);
		hitInfo.n = hitInfo.p - center;
		hitInfo.n = normalize(hitInfo.n);

		if (isTextured)
			GetSphereUV(hitInfo.n, hitInfo.u, hitInfo.v);

		return true;
	}
	return false;
}

vec3 Mcenter(float time, vec3 center0, vec3 center1, float time0, float time1)
{
	return center0 + ((time - time0) / (time1 - time0)) * (center1 - center0);
}

bool MSphereCollisionCheck(vec3 center0, vec3 center1, float radius, float time0, float time1, RayInfo r, hitRecord& hitInfo)
{
	float a = dot(r.direction, r.direction);

	vec3 center = Mcenter(r.time, center0, center1, time0, time1);

	float b = 2 * dot(r.origin - center, r.direction);
	float c = dot(r.origin - center, r.origin - center) - (radius * radius);

	float disc = (b * b) - (4 * a * c);

	if (disc < 0)
		return false;

	float t0 = (-b - sqrt(disc)) / (2 * a);
	float t1 = (-b + sqrt(disc)) / (2 * a);

	if (t0 < 0.0001f && t1 < 0.0001f)
		return false;

	if (t0 > t1)
		std::swap(t0, t1);

	if (t0 > 0.0001f && hitInfo.t > t0)
	{
		hitInfo.t = t0;
		hitInfo.p = r.At(hitInfo.t);
		hitInfo.n = hitInfo.p - center;
		hitInfo.n = normalize(hitInfo.n);

		return true;
	}
	if (t1 > 0.0001f && hitInfo.t > t1)
	{
		hitInfo.t = t1;
		hitInfo.p = r.At(hitInfo.t);
		hitInfo.n = hitInfo.p - center;
		hitInfo.n = normalize(hitInfo.n);

		return true;
	}
	return false;
}

bool BoxCollisionCheck(vec3 min, vec3 diag, RayInfo r, hitRecord& hitInfo)
{
	Slab s1;
	s1.n = vec3(1.0f, 0.0f, 0.0f);
	s1.d0 = -min.x;
	s1.d1 = -min.x - diag.x;

	Slab s2;
	s2.n = vec3(0.0f, 1.0f, 0.0f);
	s2.d0 = -min.y;
	s2.d1 = -min.y - diag.y;

	Slab s3;
	s3.n = vec3(0.0f, 0.0f, 1.0f);
	s3.d0 = -min.z;
	s3.d1 = -min.z - diag.z;

	Interval i1, i2, i3;

	if (i1.Intersect(&r, &hitInfo, s1) &&
		i2.Intersect(&r, &hitInfo, s2) &&
		i3.Intersect(&r, &hitInfo, s3))
	{
		float minT = std::max(std::max(i1.t0, i2.t0), std::max(i2.t0, i3.t0));
		float maxT = std::min(std::min(i1.t1, i2.t1), std::min(i2.t1, i3.t1));

		if (minT > maxT)
			return false;

		if (minT > maxT)
			std::swap(minT, maxT);

		if (minT > 0.0001f && hitInfo.t > minT)
		{
			hitInfo.t = minT;
			hitInfo.p = r.At(hitInfo.t);

			if (hitInfo.t - i1.t0 < glm::epsilon<float>())
			{
				hitInfo.n = s1.n;
			}
			else if (hitInfo.t - i2.t0 < glm::epsilon<float>())
			{
				hitInfo.n = s2.n;
			}
			else if (hitInfo.t - i3.t0 < glm::epsilon<float>())
			{
				hitInfo.n = s3.n;
			}
			hitInfo.n = normalize(hitInfo.n);
		}
		else if (maxT > 0.0001f && hitInfo.t > maxT)
		{
			hitInfo.t = maxT;
			hitInfo.p = r.At(hitInfo.t);

			if (hitInfo.t - i1.t1 < glm::epsilon<float>())
			{
				hitInfo.n = -s1.n;
			}
			else if (hitInfo.t - i2.t1 < glm::epsilon<float>())
			{
				hitInfo.n = -s2.n;
			}
			else if (hitInfo.t - i3.t1 < glm::epsilon<float>())
			{
				hitInfo.n = -s3.n;
			}
			hitInfo.n = normalize(hitInfo.n);
		}
		else
			return false;

		return true;
	}
	return false;
}

bool CylinderCollisionCheck(vec3 center, vec3 axis, RayInfo ray, hitRecord& hitInfo)
{
	float radius = 0.05f;
	vec3 A = normalize(axis);

	vec3 randomVec;
	randomVec.x = myrandom(RNGen);
	randomVec.y = myrandom(RNGen);
	randomVec.z = myrandom(RNGen);
	vec3 B = normalize(cross(randomVec, A));
	vec3 C = cross(A, B);

	mat3 Rinversed = glm::mat3(B, C, A);
	mat3 R = glm::transpose(Rinversed);

	Slab slab;
	slab.n = vec3(0.0f, 0.0f, 1.0f);
	slab.d0 = 0.f;
	slab.d1 = -length(axis);

	RayInfo ogRay = ray;

	ray.direction = R * ray.direction;
	ray.origin = ray.origin - center;
	ray.origin = R * ray.origin;

	Interval interval;
	if (interval.Intersect(&ray, &hitInfo, slab))
	{

		float a, b, c;

		a = ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y;
		b = 2 * (ray.direction.x * ray.origin.x + ray.direction.y * ray.origin.y);
		c = (ray.origin.x * ray.origin.x) + (ray.origin.y * ray.origin.y) - (radius * radius);

		float delta = b * b - 4 * (a * c);
		if (delta < 0.0001f)
		{
			return false;
		}

		float b0 = (-b - sqrt(delta)) / (2 * a);
		float b1 = (-b + sqrt(delta)) / (2 * a);

		float t0 = std::max(interval.t0, b0);
		float t1 = std::min(interval.t1, b1);

		if (t0 > t1)
		{

			return false;
		}

		if (t0 > 0.0001f && hitInfo.t > t0)
		{
			hitInfo.t = t0;
			hitInfo.p = ogRay.At(hitInfo.t);

			if (t0 - interval.t0 < glm::epsilon<float>())
			{
				hitInfo.n = vec3(0.f, 0.f, 1.f);
			}
			else
			{
				vec3 temp = ray.At(hitInfo.t);

				vec3 nHat(temp.x, temp.y, 0.f);
				nHat = Rinversed * nHat;
				hitInfo.n = nHat;
				hitInfo.n = normalize(hitInfo.n);
				/*float largest = std::max(std::max(hitInfo.n.x, hitInfo.n.y),
					std::max(hitInfo.n.y, hitInfo.n.z));

				hitInfo.n /= largest;*/


			}
			return true;
		}
		if (t1 > 0.0001f && hitInfo.t > t1)
		{
			hitInfo.t = t1;
			hitInfo.p = ogRay.At(hitInfo.t);

			if (t1 - interval.t1 < glm::epsilon<float>())
			{
				hitInfo.n = vec3(0.f, 0.f, -1.f);
			}
			else
			{
				vec3 temp = ray.At(hitInfo.t);

				vec3 nHat(temp.x, temp.y, 0.f);
				nHat = Rinversed * nHat;
				hitInfo.n = -nHat;
				hitInfo.n = normalize(hitInfo.n);
				/*float largest = std::max(std::max(hitInfo.n.x, hitInfo.n.y),
					std::max(hitInfo.n.y, hitInfo.n.z));

				hitInfo.n /= largest;*/
			}
			return true;
		}
	}

	return false;

}
bool TriangleCollisionCheck(vec3 v0, vec3 v1, vec3 v2, vec3 n0
	, vec3 n1, vec3 n2, RayInfo ray, hitRecord& hitInfo)
{
	vec3 e1 = v1 - v0;
	vec3 e2 = v2 - v0;

	vec3 s = ray.origin - v0;

	float d = dot(cross(ray.direction, e2), e1);
	//float t = dot(cross(s, e1), e2);
	//float u = dot(cross(ray.direction, e2), s);
	//float v = dot(cross(s, e1), ray.direction);

	vec3 p = cross(ray.direction, e2);

	if (d < 0.0001f)
		return false;

	float u = dot(p, s) / d;

	if (u < 0 || u > 1)
		return false;

	vec3 q = cross(s, e1);
	float v = dot(ray.direction, q) / d;

	if (v < 0.f || (u + v) > 1.f)
		return false;

	float t = dot(e2, q) / d;

	if (t < 0.0001f)
		return false;

	if (hitInfo.t > t)
	{
		hitInfo.t = t;
		hitInfo.p = ray.At(t);
		vec3 nor = (1 - u - v) * n0 + u * n1 + v * n2;
		hitInfo.n = normalize(nor);
		return true;

	}
	return false;

}


bool Obj::Intersection(hitRecord& record, RayInfo ray)
{
	switch (type)
	{
	case BOX:
		if (BoxCollisionCheck(pos, diagonal, ray, record))
			return true;
		break;
	case SPHERE:
		if (SphereCollisionCheck(center, radius, ray, record, 0.f, 100.f, isTextured))
			return true;
		break;
	case CYLINDER:
		if (CylinderCollisionCheck(center, cylaxis, ray, record))
			return true;
		break;
	case LIGHT:
		if (SphereCollisionCheck(center, radius, ray, record, 0.f, 100.f))
			return true;
		break;


	case MOVING_SPHERE:

		if (MSphereCollisionCheck(center0, center1, radius, time0, time1, ray, record))
			return true;
		break;

	case MESH:
		vec3 v0 = meshdata->vertices[iv.x].pnt;
		vec3 v1 = meshdata->vertices[iv.y].pnt;
		vec3 v2 = meshdata->vertices[iv.z].pnt;
		vec3 n0 = meshdata->vertices[iv.x].nrm;
		vec3 n1 = meshdata->vertices[iv.y].nrm;
		vec3 n2 = meshdata->vertices[iv.z].nrm;

		if (TriangleCollisionCheck(v0, v1, v2, n0, n1, n2, ray, record))
			return true;
		break;
	}
	return false;
}

vec3 Obj::EvalScattering(vec3 N, vec3 wi, vec3 wo, float t, vec3 textureColor)
{
	vec3 m = normalize(wo + wi);
	vec3 Ed;
	if (textureColor.x >= 0.f || textureColor.y >= 0.f || textureColor.z >= 0.f)
		Ed = textureColor / PI;
	else
		Ed = material->Kd / PI;

	float Dval = D(m, N);
	float Gval = G(wi, wo, m, N);

	//float temp = dot(wi, m);
	//temp = normalize(temp);
	vec3 Fval = F(wi, m);

	vec3 ErUp = (Dval * Gval * Fval);
	float ErDown = (4.f * abs(dot(wi, N)) * abs(dot(wo, N)));
	vec3 Er = ErUp / ErDown;

	float firstResult = abs(dot(N, wi));

	m = -normalize(no * wi + ni * wo);
	float wom = dot(wo, m);
	float r = 1.f - (n * n) * (1.f - (wom * wom));
	vec3 At = BeerLaw(t, wo, N);

	vec3 Et;
	if(r < 0.f)
	{
		m = normalize(wo + wi);
		Dval = D(m, N);
		Gval = G(wi, wo, m, N);
		Fval = F(wi, m);

		vec3 EtUp = (Dval * Gval * Fval);
		float EtDown = (4.f * abs(dot(wi, N)) * abs(dot(wo, N)));

		Et = (EtUp / EtDown) * At;
	}
	else
	{
		Dval = D(m, N);
		Gval = G(wi, wo, m, N);
		Fval = 1.f - F(wi, m);

		vec3 EtUp = (Dval * Gval * Fval);
		float EtDown = (abs(dot(wi, N)) * abs(dot(wo, N)));

		float secondUp = abs(dot(wi, m)) * abs(dot(wo, m)) * (no * no);
		float secondDownVal = (no * dot(wi, m) + ni * dot(wo, m));
		float secondDown = secondDownVal * secondDownVal;

		float second = secondUp / secondDown;

		Et = (EtUp / EtDown) * second * At;
	}
	
	vec3 secondResult = (Ed + Er + Et);

	//return (abs(dot(N, wi)) * kd) / PI;
	vec3 result =  firstResult * secondResult;

	return result;
}

float Obj::D(vec3 m, vec3 N)
{
	float mdotN = dot(m, N);

	tantM = sqrt(1.f - mdotN * mdotN) / mdotN;
	//tantV = sqrt(1.f - (dot(m,N) * dot(m,N))) / (dot)

	float firstVal = CharacteristicFactor(mdotN);

	if (firstVal <= 0.f)
		return 0.f;

	float first, second;
	float secondVal = 0.f;
	if (distributor == Distributor::Phong)
	{
		//secondVal = (((material->alpha) + 2.f) / (2.f * PI)) * pow(mdotN, material->alpha);
		first = (material->alpha + 2.f) / (2.f * PI);
		second = pow(mdotN, material->alpha);
		secondVal = first * second;
	}
	else if (distributor == Distributor::Beckman)
	{
		float check = exp(1);

		float secondFirst = 1.0f / (PI * (ab * ab) * pow(mdotN, 4));

		check = check * secondFirst;
		float secondSecond = pow(check, -(tantM * tantM) / (ab * ab));
		secondVal = secondSecond;
	}
	else if (distributor == Distributor::GGX)
	{
		float agSquare = ag * ag;
		float secondFirst = agSquare;
		float secondSecond = (PI * pow(dot(N, m), 4) * pow(((agSquare) + (tantM * tantM)), 2));
		secondVal = secondFirst / secondSecond;
	}

	float result = firstVal * secondVal;

	//if (result > 1.f)
	//	return 1.f;
	//if (result < 0.f)
	//	return 0.f;

	//CheckResultIntegrateZeroToOne(result);
	
	return result;
}

float Obj::G(vec3 wi, vec3 wo, vec3 m, vec3 N)
{
	if (distributor == Distributor::Phong)
	{
		//float a = sqrt((ap / 2) + 1) / tan
		float first = G1(wi, m, N);
		float second = G1(wo, m, N);

		//float result = first * second;

		return first * second;
	}
	if (distributor == Distributor::Beckman)
	{
		float first = G1(wi, m, N);
		float second = G1(wo, m, N);

		return first * second;
	}
	else if (distributor == Distributor::GGX)
	{
		float first = G1(wi, m, N);
		float second = G1(wo, m, N);

		return first * second;
	}
}

float Obj::G1(vec3 v, vec3 m, vec3 N)
{
	float vDotN = dot(v, N);

	if (vDotN > 1.f)
		return 1.f;

	tantV = sqrt(1.0f - (vDotN * vDotN)) / vDotN;

	if (tantV < std::numeric_limits<float>::epsilon() &&
		tantV > -std::numeric_limits<float>::epsilon())
	{
		//check if tantV is 0
		return 1.f;
	}
	//float a = 1 / (ap * tantV);

	float charaval = CharacteristicFactor(dot(v, m) / dot(v, N));

	if (charaval <= 0.f)
		return 0.f;

	float a;

	if(distributor != Distributor::GGX)
	{
		if (distributor == Distributor::Phong)
			a = sqrt((material->alpha / 2.f) + 1.f) / tantV;
		else if (distributor == Distributor::Beckman)
			a = 1 / (ab * tantV);
		

		if (a < 1.6f)
		{
			float upper = 3.535f * a + 2.181f * (a * a);
			float down = 1.0f + 2.276f * a + 2.577f * (a * a);
			float result = upper / down;

			return result * charaval;
		}
		return 1.f * charaval;
	}
	float secondVal = 2.f / (1.f + sqrt(1.f + ((ag * ag) * (tantV * tantV))));
	return charaval * secondVal;

}

vec3 Obj::SampleBrdf(vec3 N, vec3 wo)
{
	float e = myrandom(RNGen);
	float e1, e2;

	e1 = myrandom(RNGen);
	e2 = myrandom(RNGen);

	if (e < pd)
	{
		//diffuse
		return SampleLobe(N, sqrt(e1), 2.f * PI * e2);
	}

	float cosVal;
	if (distributor == Distributor::Phong)
	{
		float power = 1.f / (material->alpha + 1);
		cosVal = pow(e1, power);
	}
	else if (distributor == Distributor::Beckman)
	{
		cosVal = cos(atan(sqrt(-(ab * ab) * log(1.f - e1))));
	}
	else if (distributor == Distributor::GGX)
	{
		cosVal = cos(atan((ag * sqrt(e1)) / (sqrt(1.f - e1))));
	}
	//cosVal is shared btw in both refle, refra.
	vec3 m = SampleLobe(N, cosVal, 2.f * PI * e2);

	if(e < pd + pr)
	{
		vec3 wi = 2 * abs(dot(wo, m)) * m - wo;
		return wi;
	}
	float wom = dot(wo, m);
	float r = 1.f - (n * n) * (1 - (wom * wom));

	if(r < 0.f)
	{
		vec3 wi = 2 * abs(dot(wo, m)) * m - wo;
		return wi;
	}

	float won = dot(wo, N);
	

	float first = n * wom;
	float second = sign(won) * sqrt(r);

	float temp = first - second;
	vec3 third = temp * m;

	vec3 result = third - n * wo;
	return result;
}

float Obj::PdfBrdf(vec3 N, vec3 wi, vec3 wo)
{
	vec3 m = normalize(wo + wi);

	Pd = abs(dot(wi, N)) / PI;
	Pr = D(m, N) * abs(dot(m, N)) * (1.f / (4.f * abs(dot(wi, m))));

	m = -normalize(no * wi + ni * wo);
	float wom = dot(wo, m);
	float r = 1.f - (n * n) * (1.f - (wom * wom));

	if(r < 0.f)
	{
		m = normalize(wo + wi);
		Pt = D(m, N) * abs(dot(m, N)) * (1.f / (4.f * abs(dot(wi, m))));
	}
	else
	{
		float first = D(m, N);
		float second = abs(dot(m, N));

		float thirdUpper = (no * no) * abs(dot(wi, m));
		float thirdDown = (no * dot(wi, m)) + (ni * dot(wo, m));
		float thirdDown2 = thirdDown * thirdDown;

		float third = thirdUpper / thirdDown2;

		Pt = first * second * third;
	}


	return pd * Pd + pr * Pr + pt * Pt;
}

vec3 Obj::F(vec3 wi, vec3 m)
{
	float D = dot(wi, m);

	float last = 1 - abs(D);

	float temp = static_cast<float>(pow(last, 5));

	vec3 temp2 = (1.f - material->Ks) * temp;

	vec3 result = material->Ks + temp2;

	return result;
}



vec3 Obj::SampleLobe(vec3 A, float c, float angle)
{
	float s = sqrt(1 - (c * c));
	vec3 K(s * cos(angle), s * sin(angle), c);

	if (abs(A.z - 1.f) < 0.0001f)
		return K;

	if (abs(A.z + 1.f) < 0.0001f)
		return vec3(K.x, -K.y, -K.z);

	vec3 B = normalize(vec3(-A.y, A.x, 0.f));

	vec3 C = cross(A, B);

	return K.x * B + K.y * C + K.z * A;
}

float Obj::CharacteristicFactor(float val)
{
	if (val > 0.f)
	{
		return 1.f;
	}
	return 0.f;
}

//void Obj::CheckResultIntegrateZeroToOne(vec3& result)
//{
//	if (result.x < 0.f)
//		result.x = 0.f;
//	
//
//	if (result.y < 0.f)
//		result.y = 0.f;
//
//	if (result.z < 0.f)
//		result.z = 0.f;
//
//}
//
//void Obj::CheckResultIntegrateZeroToOne(float& result)
//{
//	if (result > 1.f)
//		result = 1.f;
//	else if (result < 0.f)
//		result = 0.f;
//}

void Obj::CalculateIndexofRefraction(vec3 wo, vec3 N)
{
	float test = dot(wo, N);

	if(test > 0.f)
	{
		ni = 1.f;
		no = material->ior;
		n = ni / no;
	}
	else
	{
		ni = material->ior;
		no = 1.f;
		n = ni / no;
	}

}

Obj::Obj(MeshData* m, Material* b, vec3 temp, MeshType type_)
    : meshdata(m), material(b), type(type_)
{
	float s = length(material->Kd) + length(material->Ks) + length(material->Kt);
	pd = length(material->Kd) / s;
	pr = length(material->Ks) / s;
	pt = length(material->Kt) / s;

    center = temp;

	ab = sqrt(2.f / (material->alpha + 2.f));
	ag = sqrt(2.f / (material->alpha + 2.f));
}
Simulation::Simulation()
{   
    sphMesh = SphMesh();
    boxMesh = BoxMesh();
    cylMesh = CylMesh();
    nav = false;
    spin = 0.0f;
    tilt = 90.0f;
    speed = 0.05f;
    front = 0.1f;
    back = 1000.0f;
    motionkey = 0;
    ambient = vec3(0.2, 0.2, 0.2); 
}

void Simulation::sphere(const vec3 center, const float r, Material* mat, bool isTextured)
{
    vec3 rrr(r,r,r);
    Obj* obj = new Obj(sphMesh, mat, center, SPHERE);
	obj->isTextured = isTextured;
    obj->area = 4*PI*r*r;

	obj->boxMin = center - rrr;
	obj->boxMax = center + rrr;

    objs.push_back(obj);
    if (mat->isLight())
        lights.push_back(obj);

    obj->radius = r;
}

void Simulation::mSphere(const vec3 center0, const vec3 center1, const float r, Material* mat)
{
	vec3 rrr(r, r, r);
	Obj* obj = new Obj(sphMesh, mat, center0, MOVING_SPHERE);

	obj->area = 4 * PI * r * r;
	obj->time0 = 0.f;
	obj->time1 = 1.f;
	obj->center0 = center0;
	obj->center1 = center1;

	obj->boxMin = center0 - rrr;
	obj->boxMax = center1 + rrr;

	objs.push_back(obj);
	if (mat->isLight())
		lights.push_back(obj);

	obj->radius = r;
}

void Simulation::box(const vec3 base, const vec3 diag, Material* mat)
{
    Obj* obj = new Obj(boxMesh, mat, base, BOX);

    obj->pos = base;
    obj->diagonal = diag;

	vec3 point1 = base;
	vec3 point2 = base + diag;

	float xMin = std::min(point1.x, point2.x);
	float yMin = std::min(point1.y, point2.y);
	float zMin = std::min(point1.z, point2.z);
	
	float xMax = std::max(point1.x, point2.x);
	float yMax = std::max(point1.y, point2.y);
	float zMax = std::max(point1.z, point2.z);

	obj->boxMin = vec3(xMin, yMin, zMin);
	obj->boxMax = vec3(xMax, yMax, zMax);
	
    objs.push_back(obj);
    if (mat->isLight())
        lights.push_back(obj);
}


void Simulation::cylinder(const vec3 base, const vec3 axis, const float radius, Material* mat)
{
    vec3 Z(0.0f, 0.0f, 1.0f);
    vec3 C = normalize(axis);
    vec3 B = cross(C,Z);
    if (length(B) <1e-8)
        B = vec3(0,1,0);
    else
        B = normalize(B);
    vec3 A = normalize(cross(B,C));

    mat4 R(A[0], A[1], A[2], 0.0f,
           B[0], B[1], B[2], 0.0f,
           C[0], C[1], C[2], 0.0f,
           0.0f, 0.0f, 0.0f, 1.0f);           

    mat4 m = translate(base)*R*scale(vec3(radius, radius, length(axis)));
    vec3 rrr(radius,radius,radius);
    Obj* obj = new Obj(cylMesh, mat, base, CYLINDER);

	vec3 point1 = base + rrr;
	vec3 point2 = base - rrr;
	vec3 point3 = (base + axis) + rrr;
	vec3 point4 = (base + axis) - rrr;

	float xMin = std::min(std::min(point1.x, point2.x), std::min(point3.x, point4.x));
	float yMin = std::min(std::min(point1.y, point2.y), std::min(point3.y, point4.y));
	float zMin = std::min(std::min(point1.z, point2.z), std::min(point3.z, point4.z));
	float xMax = std::max(std::max(point1.x, point2.x), std::max(point3.x, point4.x));
	float yMax = std::max(std::max(point1.y, point2.y), std::max(point3.y, point4.y));
	float zMax = std::max(std::max(point1.z, point2.z), std::max(point3.z, point4.z));

	obj->boxMin = vec3(xMin, yMin, zMin);
	obj->boxMax = vec3(xMax, yMax, zMax);

    obj->cylaxis = axis;

    objs.push_back(obj);
    if (mat->isLight())
        lights.push_back(obj);
}
