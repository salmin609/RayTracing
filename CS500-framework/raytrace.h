#pragma once
////////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////
#include <vector>
#include "RayInfo.hpp"
#include "hitRecord.hpp"
class Image;
class LightObj;
class AccelerationBvh;
class Obj;
class Shape;

const double PI = 3.14159265359;
const float Radians = PI/180.0f;    // Convert degrees to radians

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
 public:
    vec3 Kd, Ks, Kt;
    float alpha, ior;
    unsigned int texid;

    virtual bool isLight() { return false; }

    Material()  : Kd(vec3(1.0, 0.5, 0.0)), Ks(vec3(1,1,1)), alpha(1.0), texid(0), Kt(vec3(1,1,1)), ior(1.f) {}
    Material(const vec3 d, const vec3 s, const float a, const vec3 kt, const float ior_) 
        : Kd(d), Ks(s), alpha(a), texid(0), Kt(kt), ior(ior_) {}
    Material(Material& o) { Kd = o.Kd;  Ks = o.Ks; Kt = o.Kt; alpha = o.alpha;  texid = o.texid; ior = o.ior; }

    

    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (ivec3: consisting of three
// indices into the vertex array).
    
class VertexData
{
 public:
    vec3 pnt;
    vec3 nrm;
    vec2 tex;
    vec3 tan;
    VertexData(const vec3& p, const vec3& n, const vec2& t, const vec3& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

struct MeshData
{
    std::vector<VertexData> vertices;
    std::vector<ivec3> triangles;
    Material *mat;
};

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:

    Light(const vec3 e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }
    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////////////
// Scene
class Simulation;
class Object;
class Scene {
public:


    int width, height;
    Simulation* realtime;         // Remove this (realtime stuff)
    Material* currentMat;
    AccelerationBvh* acceleration;
    Scene();
    void Finit();
	std::vector<Obj*> objs;
    std::vector<LightObj*> lightObjs;
    void DoneReading();
	
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const mat4& M);
    void InitTexture();

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);


    vec3 SampleLobe(vec3 A, float c, float angle);
    /*vec3 SampleBrdf(vec3 N, vec3 wo);
    float PdfBrdf(vec3 N, vec3 wi, vec3 wo);*/
    float PdfLight(hitRecord Q);
    //vec3 EvalScattering(vec3 N, vec3 wi, vec3 kd, vec3 wo, vec3 ks);
    float GeometryFactor(hitRecord A, hitRecord B);
    hitRecord SampleSphere(vec3 C, float R);
    hitRecord SampleLight();

    vec3 TracePath(RayInfo ray);
    //vec3 F(float D, vec3 ks);
    //float D(vec3 m, vec3 N);
    //float G(vec3 wi, vec3 wo, vec3 m, vec3 N);
    //float G1(vec3 v, vec3 m, vec3 N);
    //float CharacteristicFactor(float val);

    
    float diff = 0.0001f;
    vec3 lightColor = vec3(5.f, 5.f, 5.f);
    Image* img;
    int imgW, imgH;
};

