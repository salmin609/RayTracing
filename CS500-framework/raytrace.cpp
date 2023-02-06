/*
 * Sangmin Kim
 * 2022/03/28
 * Raytracing Assignment
 */

#include <vector>
#include "bvh/ray.hpp"


#ifdef _WIN32
 // Includes for Windows
#include <windows.h>
#include <cstdlib>
#include <limits>
#include <crtdbg.h>
#else
 // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "hitRecord.hpp"
#include "Intervals.hpp"
// A good quality *thread-safe* Mersenne Twister random number generator.

// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].
#include "acceleration.h"
#include <random>
std::random_device device2;
std::mt19937_64 RNGen2(device2());
std::uniform_real_distribution<> myrandom2(0.0, 1.0);

#include "Image.h"

Scene::Scene()
{
	realtime = new Simulation();

}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh)
{
	realtime->triangleMesh(mesh);
}

quat Orientation(int i,
	const std::vector<std::string>& strings,
	const std::vector<float>& f)
{
	quat q(1, 0, 0, 0); // Unit quaternion
	while (i < strings.size()) {
		std::string c = strings[i++];
		if (c == "x")
			q *= angleAxis(f[i++] * Radians, Xaxis());
		else if (c == "y")
			q *= angleAxis(f[i++] * Radians, Yaxis());
		else if (c == "z")
			q *= angleAxis(f[i++] * Radians, Zaxis());
		else if (c == "q") {
			q *= quat(f[i + 0], f[i + 1], f[i + 2], f[i + 3]);
			i += 4;
		}
		else if (c == "a") {
			q *= angleAxis(f[i + 0] * Radians, normalize(vec3(f[i + 1], f[i + 2], f[i + 3])));
			i += 4;
		}
	}
	return q;
}

void Scene::InitTexture()
{
	img = new Image();
	img->Load_Image("earthmap.jpg", imgW, imgH, false);
}

void Scene::DoneReading()
{
	std::vector<Obj*> objVector;
	for (int i = 0; i < realtime->triangles.size(); ++i)
	{
		Obj* obj = realtime->triangles[i];

		size_t meshTriangleCount = obj->meshdata->triangles.size();

		for (size_t j = 0; j < meshTriangleCount; ++j)
		{
			ivec3 i0 = obj->meshdata->triangles[j];
			vec3 v0 = obj->meshdata->vertices[i0.x].pnt;
			vec3 v1 = obj->meshdata->vertices[i0.y].pnt;
			vec3 v2 = obj->meshdata->vertices[i0.z].pnt;
			vec3 center = (v0 + v1 + v2) / 3.f;

			float xMin = std::min(std::min(v0.x, v1.x), std::min(v1.x, v2.x));
			float yMin = std::min(std::min(v0.y, v1.y), std::min(v1.y, v2.y));
			float zMin = std::min(std::min(v0.z, v1.z), std::min(v1.z, v2.z));

			float xMax = std::max(std::max(v0.x, v1.x), std::max(v1.x, v2.x));
			float yMax = std::max(std::max(v0.y, v1.y), std::max(v1.y, v2.y));
			float zMax = std::max(std::max(v0.z, v1.z), std::max(v1.z, v2.z));

			vec3 min(xMin, yMin, zMin);
			vec3 max(xMax, yMax, zMax);

			Obj* triangleObj = new Obj(obj->meshdata, obj->material, vec3(0.f, 0.f, 0.f), MESH);

			triangleObj->center = center;
			triangleObj->boxMin = min;
			triangleObj->boxMax = max;
			triangleObj->iv = i0;

			objVector.push_back(triangleObj);
		}
	}

	for (int i = 0; i < realtime->objs.size(); ++i)
	{
		objVector.push_back(realtime->objs[i]);
	}

	MeshData* sampleMesh = realtime->objs[0]->meshdata;
	Material* sampleMat = realtime->objs[0]->material;

	LightObj* light = new LightObj(sampleMesh, sampleMat, vec3(1.9f, 5.f, 2.f), LIGHT);
	objVector.push_back(light);
	lightObjs.push_back(light);
	acceleration = new AccelerationBvh(objVector);
}

void Scene::Command(const std::vector<std::string>& strings,
	const std::vector<float>& f)
{
	if (strings.size() == 0) return;
	std::string c = strings[0];

	if (c == "screen") {
		// syntax: screen width height
		realtime->setScreen(int(f[1]), int(f[2]));
		width = int(f[1]);
		height = int(f[2]);
	}

	else if (c == "camera") {
		// syntax: camera x y z   ry   <orientation spec>
		// Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
		realtime->setCamera(vec3(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
	}

	else if (c == "ambient") {
		// syntax: ambient r g b
		// Sets the ambient color.  Note: This parameter is temporary.
		// It will be ignored once your raytracer becomes capable of
		// accurately *calculating* the true ambient light.
		realtime->setAmbient(vec3(f[1], f[2], f[3]));
	}

	else if (c == "brdf") {
		// syntax: brdf  r g b   r g b  alpha
		// later:  brdf  r g b   r g b  alpha  r g b ior
		// First rgb is Diffuse reflection, second is specular reflection.
		// third is beer's law transmission followed by index of refraction.
		// Creates a Material instance to be picked up by successive shapes
		currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], vec3(f[8], f[9], f[10]), f[11]);
	}

	else if (c == "light") {
		// syntax: light  r g b   
		// The rgb is the emission of the light
		// Creates a Material instance to be picked up by successive shapes
		currentMat = new Light(vec3(f[1], f[2], f[3]));
	}

	else if (c == "sphere") {
		// syntax: sphere x y z   r
		// Creates a Shape instance for a sphere defined by a center and radius
		realtime->sphere(vec3(f[1], f[2], f[3]), f[4], currentMat);
	}
	else if (c == "tsphere") {
		// syntax: sphere x y z   r
		// Creates a Shape instance for a sphere defined by a center and radius
		realtime->sphere(vec3(f[1], f[2], f[3]), f[4], currentMat, true);
	}
	else if (c == "msphere")
	{
		realtime->mSphere(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat);
	}
	else if (c == "box") {
		// syntax: box bx by bz   dx dy dz
		// Creates a Shape instance for a box defined by a corner point and diagonal vector
		realtime->box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat);

	}

	else if (c == "cylinder") {
		// syntax: cylinder bx by bz   ax ay az  r
		// Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
		realtime->cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat);
	}


	else if (c == "mesh") {
		// syntax: mesh   filename   tx ty tz   s   <orientation>
		// Creates many Shape instances (one per triangle) by reading
		// model(s) from filename. All triangles are rotated by a
		// quaternion (qw qx qy qz), uniformly scaled by s, and
		// translated by (tx ty tz) .
		mat4 modelTr = translate(vec3(f[2], f[3], f[4]))
			* scale(vec3(f[5], f[5], f[5]))
			* toMat4(Orientation(6, strings, f));
		ReadAssimpFile(strings[1], modelTr);
	}


	else {
		fprintf(stderr, "\n*********************************************\n");
		fprintf(stderr, "* Unknown command: %s\n", c.c_str());
		fprintf(stderr, "*********************************************\n\n");
	}
}

void Scene::TraceImage(Color* image, const int pass)
{
	//realtime->run();                          // Remove this (realtime stuff)
	float rx = realtime->ry * ((float)width / (float)height);
	vec3 X = rx * transformVector(realtime->ViewQuaternion(), Xaxis());
	vec3 Y = realtime->ry * transformVector(realtime->ViewQuaternion(), Yaxis());
	vec3 Z = transformVector(realtime->ViewQuaternion(), Zaxis());

	vec3 origin = realtime->eye;

	for (int i = 0; i < pass; ++i)
	{
#pragma for OpenMP

		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				float dx = (2 * (static_cast<float>(x) + myrandom2(RNGen2)) / static_cast<float>(width)) - 1;
				float dy = (2 * (static_cast<float>(y) + myrandom2(RNGen2)) / static_cast<float>(height)) - 1;

				vec3 dir = normalize(dx * X + dy * Y - Z);
				float randomTime = myrandom2(RNGen2);
				RayInfo ray(origin, dir, randomTime);

				vec3 result = TracePath(ray);

				if (!isnan(result.r) && !isnan(result.g) && !isnan(result.b) &&
					!isinf(result.r) && !isinf(result.g) && !isinf(result.b))
				{
					image[y * width + x] += result;
				}

			}
		}
	}

	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			image[y * width + x] /= pass;
		}
	}



}


vec3 Scene::SampleLobe(vec3 A, float c, float angle)
{
	float s = sqrt(1 - (c * c));
	vec3 K(s * cos(angle), s * sin(angle), c);

	if (abs(A.z - 1.f) < diff)
		return K;

	if (abs(A.z + 1.f) < diff)
		return vec3(K.x, -K.y, -K.z);

	vec3 B = normalize(vec3(-A.y, A.x, 0.f));

	vec3 C = cross(A, B);

	return K.x * B + K.y * C + K.z * A;
}

float Scene::PdfLight(hitRecord Q)
{
	LightObj* light = lightObjs[0];
	return 1.f / (light->area * lightObjs.size());
}


float Scene::GeometryFactor(hitRecord A, hitRecord B)
{
	vec3 D = A.p - B.p;

	float result = abs((dot(A.n, D) * dot(B.n, D)) / (dot(D, D) * dot(D, D)));

	return result;
}

hitRecord Scene::SampleSphere(vec3 C, float R)
{
	float e1 = myrandom2(RNGen2);
	float e2 = myrandom2(RNGen2);

	float z = 2.f * e1 - 1.f;
	float r = sqrt(1 - z * z);

	float a = 2.f * PI * e2;

	hitRecord result;
	result.n = vec3(r * cos(a), r * sin(a), z);
	result.p = C + (R * result.n);

	return result;
}

hitRecord Scene::SampleLight()
{
	int random = rand() % lightObjs.size();

	LightObj* obj = lightObjs[random];
	hitRecord record = SampleSphere(obj->center, obj->radius);
	record.matKd = lightColor;
	record.isLight = true;

	return record;
}

vec3 Scene::TracePath(RayInfo ray)
{
	vec3 C(0.f, 0.f, 0.f);
	vec3 W(1.f, 1.f, 1.f);
	hitRecord P = acceleration->intersect(ray);

	if (P.hit)
	{
		if (P.isLight)
		{
			return lightColor;
		}
		vec3 N = P.n;
		vec3 wo = -ray.direction;

		double randomVal = myrandom2(RNGen2);
		while (randomVal <= 0.8)
		{
			P.obj->CalculateIndexofRefraction(wo, N);

			bool hitTexture = P.u >= 0.f && P.v >= 0.f;

			vec3 textureColor = vec3(-1.f, -1.f, -1.f);
			if (hitTexture)
			{
				//hitTexture
				int pixX = (int)((P.u * (float)imgW) - 0.5f);
				int pixY = (int)(((1.f - P.v) * (float)imgH) - 0.5f);

				Image::Color_Ub val = img->GetValue(pixX, pixY);

				float r = (float)val.r / 255.f;
				float g = (float)val.g / 255.f;
				float b = (float)val.b / 255.f;

				textureColor = vec3(r, g, b);
			}


			//Explicit
			hitRecord L = SampleLight();
			float p = PdfLight(L) / GeometryFactor(P, L);
			vec3 wi = L.p - P.p;
			wi = normalize(wi);

			RayInfo shadowRay(P.p, wi, 0.f);

			hitRecord I = acceleration->intersect(shadowRay);

			float q = P.obj->PdfBrdf(N, wi, wo) * 0.8f;
			float wMis = p * p / (p * p + q * q);


			if (p > 0 && I.hit && I.isLight)
			{
				vec3 f = P.obj->EvalScattering(N, wi, wo, P.t, textureColor);
				C += W * wMis * (f / p) * lightColor;
			}

			wi = P.obj->SampleBrdf(N, wo);
			wi = normalize(wi);
			vec3 pos = P.p;

			RayInfo newRay(pos, wi, 0.f);

			hitRecord Q = acceleration->intersect(newRay);

			if (!Q.hit)
				break;

			vec3 f = P.obj->EvalScattering(N, wi, wo, P.t, textureColor);
			p = P.obj->PdfBrdf(N, wi, wo) * 0.8f;

			if (p < 0.000001f)
				break;

			vec3 fdp = f / p;

			W *= fdp;

			if (Q.isLight)
			{
				q = PdfLight(Q) / GeometryFactor(P, Q);
				wMis = p * p / (p * p + q * q);

				C += W * wMis * lightColor;

				break;
			}
			//////////////////////////////////////////////////////////////////
			randomVal = myrandom2(RNGen2);

			P = Q;
			N = P.n;
			wo = -wi;
		}
		return C;
	}
	return vec3(0.f, 0.f, 0.f);
}

